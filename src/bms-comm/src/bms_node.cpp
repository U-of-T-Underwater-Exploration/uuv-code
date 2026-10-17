// bms_node.cpp
// ROS2 node that polls a JK-BMS over UART and publishes battery state data.
//
// Topics published:
//   /bms/data         (sensor_msgs/BatteryState)   – pack-level data
//   /bms/temperature  (std_msgs/Float32MultiArray)  – [MOSFET, probe1, probe2] °C
//
// Parameters (set via launch file):
//   serial_port       (string)  default "/dev/ttyS0"
//   baud_rate         (int)     default 115200
//   publish_frequency (double)  default 2.0  [Hz]

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "bms_parser.hpp"

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

// Design capacity of the pack: 2 × 8 Ah in parallel = 16 Ah
static constexpr double DESIGN_CAPACITY_AH = 16.0;

// JK-BMS protocol V2.5 (bms-protocol.pdf), section 4.2 frame format:
//   STX(2) | Length(2) | TermNo(4) | Cmd(1) | Src(1) | Trans(1) |
//   Data(N) | RecordNo(4) | EndMark(1,=0x68) | Checksum(4)
// Length counts every byte except STX, including the checksum.
// Checksum: high 2 bytes 0 (CRC not enabled), low 2 bytes = 16-bit sum of
// every byte from STX up to and including the end marker.

// Command words (section 4.2.4)
static constexpr uint8_t BMS_CMD_ACTIVATE = 0x01;  // Wake a dormant BMS
static constexpr uint8_t BMS_CMD_READ_ALL = 0x06;  // Read the whole ID table

// Frame source (section 4.2.5): 0 BMS, 1 Bluetooth, 2 GPS, 3 PC
static constexpr uint8_t BMS_SRC_PC = 0x03;

// Transport type (section 4.2.6): 0 request, 1 reply, 2 active upload
static constexpr uint8_t BMS_TRANS_REQUEST = 0x00;
static constexpr uint8_t BMS_TRANS_REPLY   = 0x01;

static constexpr uint8_t BMS_END_MARK = 0x68;

// Request frames carry a single data-identifier byte (0x00 = read all).
static constexpr size_t BMS_REQUEST_LEN = 21;

// Byte offsets inside any frame
static constexpr size_t BMS_OFF_CMD   = 8;
static constexpr size_t BMS_OFF_TRANS = 10;

// Smallest valid frame: header through end marker plus checksum, no data.
static constexpr size_t BMS_MIN_FRAME_LEN = 20;

// Maximum expected response size.  A full 24-cell frame is ~320 bytes;
// 512 gives comfortable headroom.
static constexpr size_t READ_BUF_SIZE = 512;

// Per-read() attempt timeout expressed as VTIME units (tenths of a second).
static constexpr uint8_t SERIAL_VTIME = 10;   // 1.0 s

// How many consecutive empty read() calls before giving up.  The protocol
// guarantees a reply within 5 s (section 4.1).
static constexpr int MAX_READ_ATTEMPTS = 5;

// Consecutive failed polls before assuming the BMS has gone dormant and
// re-sending the activation command.
static constexpr int MAX_POLL_FAILURES = 3;

// Build a request frame for the given command word.
static std::array<uint8_t, BMS_REQUEST_LEN> make_request(uint8_t cmd)
{
    std::array<uint8_t, BMS_REQUEST_LEN> f{
        0x4E, 0x57,                     // STX
        0x00, BMS_REQUEST_LEN - 2,      // Length = 19
        0x00, 0x00, 0x00, 0x00,         // Terminal number
        cmd,                            // Command word
        BMS_SRC_PC,                     // Frame source
        BMS_TRANS_REQUEST,              // Transport type
        0x00,                           // Data identifier (0x00 = all)
        0x00, 0x00, 0x00, 0x00,         // Record number
        BMS_END_MARK,                   // End marker
        0x00, 0x00, 0x00, 0x00          // Checksum (filled below)
    };
    const uint16_t sum = computeChecksum(f.data(), BMS_REQUEST_LEN - 4);
    f[BMS_REQUEST_LEN - 2] = static_cast<uint8_t>(sum >> 8);
    f[BMS_REQUEST_LEN - 1] = static_cast<uint8_t>(sum & 0xFF);
    return f;
}

static const auto BMS_ACTIVATE_FRAME = make_request(BMS_CMD_ACTIVATE);
static const auto BMS_READ_ALL_FRAME = make_request(BMS_CMD_READ_ALL);

// ---------------------------------------------------------------------------
// BMSNode
// ---------------------------------------------------------------------------
class BMSNode : public rclcpp::Node
{
public:
    BMSNode()
    : Node("bms_node"), serial_fd_(-1), bms_active_(false), poll_failures_(0)
    {
        // ---- Parameters ----------------------------------------------------
        this->declare_parameter<std::string>("serial_port",       "/dev/ttyS0");
        this->declare_parameter<int>        ("baud_rate",          115200);
        this->declare_parameter<double>     ("publish_frequency",  2.0);

        const std::string port = this->get_parameter("serial_port").as_string();
        const int         baud = this->get_parameter("baud_rate").as_int();
        const double      freq = this->get_parameter("publish_frequency").as_double();

        RCLCPP_INFO(this->get_logger(),
            "BMS node starting – port: %s  baud: %d  freq: %.1f Hz",
            port.c_str(), baud, freq);

        // ---- Publishers (Reliable, depth 10) --------------------------------
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
            "/bms/data", qos);
        temp_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/bms/temperature", qos);

        // ---- Serial port ---------------------------------------------------
        open_serial(port, baud);

        // ---- Polling timer -------------------------------------------------
        if (freq <= 0.0) {
            RCLCPP_FATAL(this->get_logger(), "publish_frequency must be > 0");
            throw std::runtime_error("publish_frequency must be > 0");
        }
        auto period = std::chrono::duration<double>(1.0 / freq);
        timer_ = this->create_wall_timer(
            period, std::bind(&BMSNode::timer_callback, this));
    }

    ~BMSNode()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
    }

private:
    // -----------------------------------------------------------------------
    // open_serial
    //   Opens the serial port and configures it for raw 8N1 communication.
    //   Logs an error but does NOT throw so the node can run and retry.
    // -----------------------------------------------------------------------
    void open_serial(const std::string& port, int baud)
    {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(),
                "Cannot open serial port '%s': %s",
                port.c_str(), std::strerror(errno));
            return;
        }

        struct termios tty{};
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(),
                "tcgetattr failed on '%s': %s", port.c_str(), std::strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return;
        }

        // Baud rate
        speed_t speed;
        switch (baud) {
            case 9600:   speed = B9600;   break;
            case 115200: speed = B115200; break;
            default:
                RCLCPP_WARN(this->get_logger(),
                    "Unsupported baud %d – defaulting to 115200", baud);
                speed = B115200;
                break;
        }
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        // 8N1, no flow control, raw mode
        tty.c_cflag  =  (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag |=  CREAD | CLOCAL;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        tty.c_oflag &= ~(OPOST | ONLCR);

        // Blocking read with 1-second timeout; VMIN=0 allows partial returns.
        tty.c_cc[VTIME] = SERIAL_VTIME;
        tty.c_cc[VMIN]  = 0;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(),
                "tcsetattr failed on '%s': %s", port.c_str(), std::strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return;
        }

        // Flush any stale bytes
        tcflush(serial_fd_, TCIOFLUSH);

        RCLCPP_INFO(this->get_logger(),
            "Serial port '%s' opened at %d baud.", port.c_str(), baud);
    }

    // -----------------------------------------------------------------------
    // transact
    //   Sends a request frame, then accumulates bytes until a complete reply
    //   frame arrives (detected via the declared frame-length field) or until
    //   MAX_READ_ATTEMPTS reads in a row return no data.
    //
    //   The reply must start with 0x4E 0x57, have a valid checksum, echo the
    //   request's command word and carry transport type 1 (reply).
    //
    //   Returns true and fills 'frame' (trimmed to the declared length) on
    //   success.  Returns false and logs the reason on any error.
    // -----------------------------------------------------------------------
    bool transact(const std::array<uint8_t, BMS_REQUEST_LEN>& request,
                  std::vector<uint8_t>& frame)
    {
        const uint8_t cmd = request[BMS_OFF_CMD];

        // Discard stale data before sending the request
        tcflush(serial_fd_, TCIFLUSH);

        // ---- Send request --------------------------------------------------
        ssize_t written = write(serial_fd_, request.data(), request.size());
        if (written != static_cast<ssize_t>(request.size())) {
            RCLCPP_ERROR(this->get_logger(),
                "Serial write failed for cmd 0x%02X (wrote %zd of %zu bytes): %s",
                cmd, written, request.size(), std::strerror(errno));
            return false;
        }

        // ---- Accumulate response -------------------------------------------
        frame.clear();
        frame.reserve(READ_BUF_SIZE);

        uint8_t tmp[READ_BUF_SIZE];
        int attempts = 0;

        while (attempts < MAX_READ_ATTEMPTS) {
            ssize_t n = read(serial_fd_, tmp, sizeof(tmp));
            if (n < 0) {
                RCLCPP_ERROR(this->get_logger(),
                    "Serial read error: %s", std::strerror(errno));
                return false;
            }
            if (n == 0) {
                // Timeout (VTIME expired, no bytes)
                ++attempts;
                continue;
            }

            frame.insert(frame.end(), tmp, tmp + n);
            attempts = 0; // reset on progress

            // Drop any leading noise so the buffer starts at 0x4E 0x57
            size_t start = 0;
            while (start < frame.size() &&
                   !(frame[start] == 0x4E &&
                     (start + 1 == frame.size() || frame[start + 1] == 0x57))) {
                ++start;
            }
            frame.erase(frame.begin(), frame.begin() + start);

            if (frame.size() < 4) {
                continue;
            }

            const size_t declaredEnd =
                2 + static_cast<size_t>((frame[2] << 8) | frame[3]);
            if (declaredEnd < BMS_MIN_FRAME_LEN || declaredEnd > READ_BUF_SIZE) {
                RCLCPP_ERROR(this->get_logger(),
                    "Reply to cmd 0x%02X declares invalid length %zu.",
                    cmd, declaredEnd);
                return false;
            }
            if (frame.size() < declaredEnd) {
                continue;
            }

            // ---- Complete frame: validate it -------------------------------
            frame.resize(declaredEnd);

            const uint16_t expected = static_cast<uint16_t>(
                (frame[declaredEnd - 2] << 8) | frame[declaredEnd - 1]);
            const uint16_t computed = computeChecksum(frame.data(), declaredEnd - 4);
            if (expected != computed) {
                RCLCPP_ERROR(this->get_logger(),
                    "Reply to cmd 0x%02X has bad checksum "
                    "(got 0x%04X, computed 0x%04X).", cmd, expected, computed);
                return false;
            }
            if (frame[BMS_OFF_CMD] != cmd ||
                frame[BMS_OFF_TRANS] != BMS_TRANS_REPLY) {
                RCLCPP_ERROR(this->get_logger(),
                    "Unexpected reply to cmd 0x%02X "
                    "(cmd 0x%02X, transport type %u).",
                    cmd, frame[BMS_OFF_CMD], frame[BMS_OFF_TRANS]);
                return false;
            }
            return true;
        }

        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No reply from BMS to cmd 0x%02X.", cmd);
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "Reply to cmd 0x%02X incomplete after %d read attempts "
                "(%zu bytes received).", cmd, MAX_READ_ATTEMPTS, frame.size());
        }
        return false;
    }

    // -----------------------------------------------------------------------
    // activate
    //   Sends the 0x01 activation command.  Per section 4.2.4 the controller
    //   must receive the BMS's reply before issuing any other command.
    // -----------------------------------------------------------------------
    bool activate()
    {
        std::vector<uint8_t> reply;
        if (!transact(BMS_ACTIVATE_FRAME, reply)) {
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "BMS activated.");
        return true;
    }

    // -----------------------------------------------------------------------
    // timer_callback – called at publish_frequency Hz
    // -----------------------------------------------------------------------
    void timer_callback()
    {
        if (serial_fd_ < 0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Serial port is not open – cannot poll BMS.");
            return;
        }

        // Activate first; polling starts on the next tick, which also keeps
        // the protocol's 100 ms minimum gap between packets (section 4.1).
        if (!bms_active_) {
            bms_active_ = activate();
            poll_failures_ = 0;
            return;
        }

        // Timestamp captured immediately before the UART request so it
        // reflects when the measurement was *requested*, not when we publish.
        auto stamp = this->get_clock()->now();

        std::vector<uint8_t> frame;
        if (!transact(BMS_READ_ALL_FRAME, frame)) {
            // Error already logged inside transact()
            if (++poll_failures_ >= MAX_POLL_FAILURES) {
                RCLCPP_WARN(this->get_logger(),
                    "%d failed polls in a row – re-activating BMS.",
                    poll_failures_);
                bms_active_ = false;
            }
            return;
        }
        poll_failures_ = 0;

        BMSData data;
        if (!parseBMSFrame(frame.data(), frame.size(), data)) {
            RCLCPP_ERROR(this->get_logger(),
                "BMS frame parse failed (frame size: %zu bytes). "
                "Check wiring and baud rate.", frame.size());
            return;
        }

        publish_data(data, stamp);
    }

    // -----------------------------------------------------------------------
    // publish_data
    // -----------------------------------------------------------------------
    void publish_data(const BMSData& data, const rclcpp::Time& stamp)
    {
        // ---- /bms/data  (sensor_msgs/BatteryState) -------------------------
        sensor_msgs::msg::BatteryState bat;
        bat.header.stamp    = stamp;
        bat.header.frame_id = "bms_link";

        bat.voltage          = static_cast<float>(data.voltage);
        bat.current          = static_cast<float>(data.current);

        // percentage: BatteryState convention is 0.0–1.0
        bat.percentage       = static_cast<float>(data.capacityPercentage / 100.0);

        // temperature: MOSFET sensor per requirements
        bat.temperature      = static_cast<float>(data.temperatureMOSFET);

        // charge = remaining capacity [Ah]
        bat.charge           = static_cast<float>(data.capacityAh);

        // capacity / design_capacity = total pack capacity [Ah]
        bat.capacity         = static_cast<float>(DESIGN_CAPACITY_AH);
        bat.design_capacity  = static_cast<float>(DESIGN_CAPACITY_AH);

        // Individual cell voltages
        bat.cell_voltage.assign(data.cellVoltages.begin(), data.cellVoltages.end());

        // Power supply status
        if (data.isCharging) {
            bat.power_supply_status =
                sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        } else if (data.isDischarging) {
            bat.power_supply_status =
                sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        } else {
            bat.power_supply_status =
                sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
        }

        // Health: flag if any alarm is active
        bat.power_supply_health =
            (data.alarmLowCapacity || data.alarmOverTemp || data.alarmOverCurrent)
            ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
            : sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;

        battery_pub_->publish(bat);

        // ---- /bms/temperature  (std_msgs/Float32MultiArray) ----------------
        // Index 0: internal MOSFET temperature
        // Index 1: left external probe
        // Index 2: right external probe
        std_msgs::msg::Float32MultiArray temp;
        temp.data = {
            static_cast<float>(data.temperatureMOSFET),
            static_cast<float>(data.temperatureProbe1),
            static_cast<float>(data.temperatureProbe2)
        };
        temp_pub_->publish(temp);
    }

    // -----------------------------------------------------------------------
    int serial_fd_;
    bool bms_active_;     // true once the BMS has answered an activation
    int  poll_failures_;  // consecutive failed read-all polls
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr    battery_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  temp_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BMSNode>());
    rclcpp::shutdown();
    return 0;
}
