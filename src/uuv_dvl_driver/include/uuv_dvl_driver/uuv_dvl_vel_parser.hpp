#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

// Decode a 68-byte ID_DVL_VEL (0x79) payload.
std::map<std::string, double> parse_dvl_vel(
    const std::vector<uint8_t>& payload);
