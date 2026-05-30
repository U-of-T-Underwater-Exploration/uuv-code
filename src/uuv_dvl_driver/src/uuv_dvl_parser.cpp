#include <map>
#include <string>
#include <vector>
#include <cstdint>
#include <cstring>
#include <stdexcept>

// Parses ID_DVL_VEL (0x79) payload (68 bytes) and returns it in a dictionary format (std::map)
std::map<std::string, double> parse_dvl_vel(const std::vector<uint8_t>& payload) {
    if (payload.size() < 68) {
        throw std::invalid_argument("Payload size is less than the required 68 bytes for ID_DVL_VEL.");
    }

    std::map<std::string, double> dvl_dict;

    uint32_t flags;
    std::memcpy(&flags, &payload[0], sizeof(uint32_t));
    dvl_dict["FLAGS"] = static_cast<double>(flags);

    uint32_t timestamp;
    std::memcpy(&timestamp, &payload[4], sizeof(uint32_t));
    dvl_dict["TIMESTAMP"] = static_cast<double>(timestamp);

    float delta_time;
    std::memcpy(&delta_time, &payload[8], sizeof(float));
    dvl_dict["DELTA_TIME"] = static_cast<double>(delta_time);

    float latency;
    std::memcpy(&latency, &payload[12], sizeof(float));
    dvl_dict["LATENCY"] = static_cast<double>(latency);

    float velocity_x;
    std::memcpy(&velocity_x, &payload[16], sizeof(float));
    dvl_dict["VELOCITY_X"] = static_cast<double>(velocity_x);

    float velocity_y;
    std::memcpy(&velocity_y, &payload[20], sizeof(float));
    dvl_dict["VELOCITY_Y"] = static_cast<double>(velocity_y);

    float velocity_z;
    std::memcpy(&velocity_z, &payload[24], sizeof(float));
    dvl_dict["VELOCITY_Z"] = static_cast<double>(velocity_z);

    float velocity_z1;
    std::memcpy(&velocity_z1, &payload[28], sizeof(float));
    dvl_dict["VELOCITY_Z1"] = static_cast<double>(velocity_z1);

    float velocity_z2;
    std::memcpy(&velocity_z2, &payload[32], sizeof(float));
    dvl_dict["VELOCITY_Z2"] = static_cast<double>(velocity_z2);

    float uncertainty_x;
    std::memcpy(&uncertainty_x, &payload[36], sizeof(float));
    dvl_dict["UNCERTAINTY_X"] = static_cast<double>(uncertainty_x);

    float uncertainty_y;
    std::memcpy(&uncertainty_y, &payload[40], sizeof(float));
    dvl_dict["UNCERTAINTY_Y"] = static_cast<double>(uncertainty_y);

    float uncertainty_z;
    std::memcpy(&uncertainty_z, &payload[44], sizeof(float));
    dvl_dict["UNCERTAINTY_Z"] = static_cast<double>(uncertainty_z);

    float uncertainty_z1;
    std::memcpy(&uncertainty_z1, &payload[48], sizeof(float));
    dvl_dict["UNCERTAINTY_Z1"] = static_cast<double>(uncertainty_z1);

    float uncertainty_z2;
    std::memcpy(&uncertainty_z2, &payload[52], sizeof(float));
    dvl_dict["UNCERTAINTY_Z2"] = static_cast<double>(uncertainty_z2);

    float distance_z;
    std::memcpy(&distance_z, &payload[56], sizeof(float));
    dvl_dict["DISTANCE_Z"] = static_cast<double>(distance_z);

    float distance_z1;
    std::memcpy(&distance_z1, &payload[60], sizeof(float));
    dvl_dict["DISTANCE_Z1"] = static_cast<double>(distance_z1);

    float distance_z2;
    std::memcpy(&distance_z2, &payload[64], sizeof(float));
    dvl_dict["DISTANCE_Z2"] = static_cast<double>(distance_z2);

    return dvl_dict;
}
