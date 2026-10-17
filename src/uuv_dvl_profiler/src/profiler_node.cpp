#include "rclcpp/rclcpp.hpp"
#include "uuv_msgs/msg/kogger_dvl_vel.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

class DvlProfilerNode : public rclcpp::Node
{
public:
    DvlProfilerNode() : Node("uuv_dvl_profiler")
    {
        raw_sub_ = create_subscription<uuv_msgs::msg::KoggerDvlVel>(
            "/dvl/raw", rclcpp::QoS(10),
            std::bind(&DvlProfilerNode::rawCallback, this, std::placeholders::_1));

        status_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/dvl/status", rclcpp::QoS(10),
            std::bind(&DvlProfilerNode::statusCallback, this, std::placeholders::_1));

        last_msg_time_ = now();
        start_time_ = last_msg_time_;
        RCLCPP_INFO(get_logger(), "DVL profiler started.");
    }

    ~DvlProfilerNode() override { writeReport(); }

private:
    // Online statistics avoid storing every sample in memory.
    struct FieldStats
    {
        double min_val = std::numeric_limits<double>::infinity();
        double max_val = -std::numeric_limits<double>::infinity();
        double sum = 0.0;
        double mean = 0.0;
        double m2 = 0.0;
        uint64_t valid_count = 0;
        uint64_t non_finite_count = 0;
        uint64_t zero_count = 0;

        void update(double value)
        {
            if (!std::isfinite(value)) {
                ++non_finite_count; // Counts NaN and +/- infinity.
                return;
            }

            ++valid_count;
            if (value == 0.0) ++zero_count;
            min_val = std::min(min_val, value);
            max_val = std::max(max_val, value);
            sum += value;

            // Welford's algorithm; m2 / (N - 1) gives sample variance.
            const double delta = value - mean;
            mean += delta / static_cast<double>(valid_count);
            const double delta2 = value - mean;
            m2 += delta * delta2;
        }

        double getMean() const
        {
            return valid_count ? sum / static_cast<double>(valid_count) : 0.0;
        }

        double getStdDev() const
        {
            if (valid_count < 2) return 0.0;
            return std::sqrt(std::max(
                0.0, m2 / static_cast<double>(valid_count - 1)));
        }
    };

    struct AltitudeBinStats
    {
        uint64_t samples = 0;
        FieldStats unc_z;
        FieldStats unc_z1;
        FieldStats unc_z2;
    };

    static std::string hexValue(uint32_t value)
    {
        std::ostringstream out;
        out << "0x" << std::uppercase << std::hex << value;
        return out.str();
    }

    void updateField(const std::string &name, double value)
    {
        field_stats_[name].update(value);
    }

    void rawCallback(const uuv_msgs::msg::KoggerDvlVel::SharedPtr msg)
    {
        try {
            ++sample_count_;

            const rclcpp::Time current_time = now();
            if (sample_count_ > 1) {
                const double dt = (current_time - last_msg_time_).seconds();
                if (std::isfinite(dt) && dt > 0.0) intervals_.push_back(dt);
            }
            last_msg_time_ = current_time;

            // Count occurrences, rather than only retaining unique FLAGS values.
            ++flags_counts_[msg->flags];

            // Names match the KoggerDvlVel.msg fields shown in the project README.
            updateField("device_timestamp_ms", msg->device_timestamp_ms);
            updateField("flags", static_cast<double>(msg->flags));
            updateField("delta_time_s", msg->delta_time);
            updateField("latency_s", msg->latency);

            updateField("vel_x_mps", msg->vel_x);
            updateField("vel_y_mps", msg->vel_y);
            updateField("vel_z_mps", msg->vel_z);
            updateField("vel_z1_mps", msg->vel_z1);
            updateField("vel_z2_mps", msg->vel_z2);

            updateField("unc_x_mps", msg->unc_x);
            updateField("unc_y_mps", msg->unc_y);
            updateField("unc_z_mps", msg->unc_z);
            updateField("unc_z1_mps", msg->unc_z1);
            updateField("unc_z2_mps", msg->unc_z2);

            updateField("dist_z_m", msg->dist_z);
            updateField("dist_z1_m", msg->dist_z1);
            updateField("dist_z2_m", msg->dist_z2);

            // README describes dist_z as height above the seabed.
            const double altitude = static_cast<double>(msg->dist_z);
            if (std::isfinite(altitude) && altitude >= 0.0 &&
                altitude_bin_width_m_ > 0.0) {
                const int bin = static_cast<int>(
                    std::floor(altitude / altitude_bin_width_m_));
                auto &bin_stats = uncertainty_by_altitude_[bin];
                ++bin_stats.samples;
                bin_stats.unc_z.update(msg->unc_z);
                bin_stats.unc_z1.update(msg->unc_z1);
                bin_stats.unc_z2.update(msg->unc_z2);
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Exception in rawCallback: %s", e.what());
        }
    }

    void statusCallback(
        const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
    {
        for (const auto &status : msg->status) {
            last_status_name_ = status.name;
            last_status_message_ = status.message;
            last_status_level_ = status.level;
            ++status_level_counts_[status.level];

            // Preserve counters the driver exposes through DiagnosticStatus.values.
            for (const auto &kv : status.values) {
                diagnostic_values_[status.name + "/" + kv.key] = kv.value;
            }
        }
    }

    void writeFieldTable(std::ofstream &out)
    {
        out << "| Field | Min | Max | Mean | Sample std dev | Valid count | "
               "NaN/Inf count | Zero count |\n";
        out << "|---|---:|---:|---:|---:|---:|---:|---:|\n";
        out << std::setprecision(8);

        for (const auto &[name, st] : field_stats_) {
            out << "| " << name << " | ";
            if (st.valid_count == 0) {
                out << "N/A | N/A | N/A | N/A | 0 | "
                    << st.non_finite_count << " | " << st.zero_count << " |\n";
            } else {
                out << st.min_val << " | " << st.max_val << " | "
                    << st.getMean() << " | " << st.getStdDev() << " | "
                    << st.valid_count << " | " << st.non_finite_count << " | "
                    << st.zero_count << " |\n";
            }
        }
    }

    void writeReport()
    {
        std::ofstream report("dvl_report.md");
        if (!report.is_open()) {
            RCLCPP_ERROR(get_logger(),
                         "Could not open dvl_report.md in the current working directory.");
            return;
        }

        const double elapsed_s = (now() - start_time_).seconds();

        report << "# DVL Profiler Report\n\n";
        report << "## Recording summary\n\n";
        report << "- Raw `/dvl/raw` messages: " << sample_count_ << "\n";
        report << "- Elapsed time (s): " << std::max(0.0, elapsed_s) << "\n";
        if (!intervals_.empty()) {
            const double avg_dt =
                std::accumulate(intervals_.begin(), intervals_.end(), 0.0) /
                static_cast<double>(intervals_.size());
            report << "- Estimated receive frequency (Hz): "
                   << (avg_dt > 0.0 ? 1.0 / avg_dt : 0.0) << "\n";
        } else {
            report << "- Estimated receive frequency (Hz): N/A (insufficient samples)\n";
        }
        report << "- Last driver status name: "
               << (last_status_name_.empty() ? "None" : last_status_name_) << "\n";
        report << "- Last driver status level: " << static_cast<int>(last_status_level_) << "\n";
        report << "- Last driver status message: "
               << (last_status_message_.empty() ? "None" : last_status_message_) << "\n\n";

        report << "## Numeric field statistics\n\n";
        report << "Non-finite count includes NaN and +/- infinity. Zero count is "
                  "a subset of valid finite values.\n\n";
        writeFieldTable(report);

        report << "\n## FLAGS bit patterns\n\n";
        report << "| FLAGS (hex) | Count | Percentage of raw messages |\n";
        report << "|---|---:|---:|\n";
        for (const auto &[flags, count] : flags_counts_) {
            const double pct = sample_count_
                ? 100.0 * static_cast<double>(count) / static_cast<double>(sample_count_)
                : 0.0;
            report << "| `" << hexValue(flags) << "` | " << count
                   << " | " << std::setprecision(5) << pct << "% |\n";
        }
        report << "\nThe supplied protocol notes do not document FLAGS bit meanings; "
                  "patterns are reported without guessing their semantics.\n";

        report << "\n## Uncertainty versus altitude\n\n";
        report << "Altitude proxy: `dist_z` (distance above seabed), bin width: "
               << altitude_bin_width_m_ << " m. This describes association, not causation.\n\n";
        report << "| Altitude bin (m) | Samples | unc_z mean | unc_z std | "
                  "unc_z1 mean | unc_z1 std | unc_z2 mean | unc_z2 std |\n";
        report << "|---|---:|---:|---:|---:|---:|---:|---:|\n";
        for (const auto &[bin, st] : uncertainty_by_altitude_) {
            const double low = bin * altitude_bin_width_m_;
            const double high = low + altitude_bin_width_m_;
            report << "| [" << low << ", " << high << ") | " << st.samples << " | "
                   << (st.unc_z.valid_count ? std::to_string(st.unc_z.getMean()) : "N/A") << " | "
                   << (st.unc_z.valid_count ? std::to_string(st.unc_z.getStdDev()) : "N/A") << " | "
                   << (st.unc_z1.valid_count ? std::to_string(st.unc_z1.getMean()) : "N/A") << " | "
                   << (st.unc_z1.valid_count ? std::to_string(st.unc_z1.getStdDev()) : "N/A") << " | "
                   << (st.unc_z2.valid_count ? std::to_string(st.unc_z2.getMean()) : "N/A") << " | "
                   << (st.unc_z2.valid_count ? std::to_string(st.unc_z2.getStdDev()) : "N/A") << " |\n";
        }

        report << "\n## Driver diagnostics\n\n";
        report << "Only diagnostic key/value pairs actually published by the driver can be "
                  "reported here. This node does not parse serial frames.\n\n";
        if (diagnostic_values_.empty()) {
            report << "No key/value diagnostic counters were received.\n";
        } else {
            report << "| Diagnostic key | Value |\n|---|---|\n";
            for (const auto &[key, value] : diagnostic_values_) {
                report << "| " << key << " | " << value << " |\n";
            }
        }

        report << "\n## Message IDs and versions\n\n";
        report << "The current `KoggerDvlVel` message does not expose the original protocol "
                  "ID, MODE/version, or raw payload. Therefore this profiler cannot independently "
                  "enumerate all IDs/versions. The driver must expose those counters through "
                  "`/dvl/status` key/value entries or a dedicated protocol-statistics topic.\n";

        report << "\n## Z1 / Z2 investigation\n\n";
        report << "- Tracked independently: `vel_z1`, `vel_z2`, `unc_z1`, `unc_z2`, "
                  "`dist_z1`, and `dist_z2`.\n";
        report << "- Compare separate recordings: in air, still tank, known speed, and altitude sweep.\n";
        report << "- These statistics do not establish what Z1/Z2 physically mean; compare recordings "
                  "and ask Kogger for confirmation.\n";

        report << "\n## Test-condition metadata\n\n";
        report << "Run the profiler separately for each rosbag case and label the resulting report "
                  "with the condition. The node cannot infer 'in air', 'bottom lock', or 'known speed' "
                  "from the data unless that metadata is provided.\n";

        report.close();
        RCLCPP_INFO(get_logger(), "Wrote dvl_report.md in the current working directory.");
    }

    rclcpp::Subscription<uuv_msgs::msg::KoggerDvlVel>::SharedPtr raw_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_sub_;

    uint64_t sample_count_ = 0;
    rclcpp::Time start_time_;
    rclcpp::Time last_msg_time_;
    std::vector<double> intervals_;

    std::map<std::string, FieldStats> field_stats_;
    std::map<uint32_t, uint64_t> flags_counts_;
    std::map<int, AltitudeBinStats> uncertainty_by_altitude_;
    std::map<std::string, std::string> diagnostic_values_;
    std::map<uint8_t, uint64_t> status_level_counts_;

    std::string last_status_name_;
    std::string last_status_message_;
    uint8_t last_status_level_ = diagnostic_msgs::msg::DiagnosticStatus::OK;

    // Adjustable in code; 0.1 m is a starting point, not a device requirement.
    double altitude_bin_width_m_ = 0.1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DvlProfilerNode>();
    rclcpp::spin(node);
    node.reset();  // Write the report before ROS shutdown.
    rclcpp::shutdown();
    return 0;
}