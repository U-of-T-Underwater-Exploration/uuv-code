#include <cstddef>    // size_t
#include <cstdint>    // uint8_t, uint32_t
#include <cstring>    // std::memcpy
#include <iostream>   // std::cout
#include <limits>     // std::numeric_limits
#include <map>        // std::map
#include <stdexcept>  // std::invalid_argument
#include <string>     // std::string
#include <vector>     // std::vector

// Compile-time compatibility checks

static_assert(sizeof(float) == 4, "The DVL parser requires a 32-bit float.");

static_assert(std::numeric_limits<float>::is_iec559,
              "The DVL parser requires IEEE 754 floating-point support.");

static_assert(sizeof(uint32_t) == 4, "The DVL parser requires a 32-bit unsigned integer.");

// Internal decoding helpers

namespace {
uint32_t read_u32_le(const std::vector<uint8_t> &payload, std::size_t offset) {
  // Reads 4 bytes from the payload, starting at offset, and interprets them as a little-endian
  // unsigned 32-bit integer.

  uint32_t value = 0;

  for(std::size_t i = 0; i < 4; i++) {
    uint32_t byte = payload[offset + i];

    byte = byte << (8 * i);  // 8 * i because we are shifting bits and each byte is 8 bits.

    value |= byte;  // Bitwise OR operator
  }

  return value;
}

float read_f32_le(const std::vector<uint8_t> &payload, std::size_t offset) {
  uint32_t bits = read_u32_le(payload, offset);

  float value = 0.0f;

  std::memcpy(&value, &bits, sizeof(float));

  return value;
}
}  // namespace

// Main DVL payload parser!

const std::vector<std::string> dvl_vel_keys = { "FLAGS",          "TIMESTAMP",      "DELTA_TIME",
                                                "LATENCY",        "VELOCITY_X",     "VELOCITY_Y",
                                                "VELOCITY_Z",     "VELOCITY_Z1",    "VELOCITY_Z2",
                                                "UNCERTAINTY_X",  "UNCERTAINTY_Y",  "UNCERTAINTY_Z",
                                                "UNCERTAINTY_Z1", "UNCERTAINTY_Z2", "DISTANCE_Z",
                                                "DISTANCE_Z1",    "DISTANCE_Z2" };

std::map<std::string, double> parse_dvl_vel(const std::vector<uint8_t> &payload) {
  // Parses the ID_DVL_VEL (0x79) payload (68 bytes) and returns a std::map with the keys and
  // values.

  // The map stores decoded values as doubles. On IEEE 754 binary64 platforms, double represents all
  // uint32_t values and all values decoded from IEEE 754 binary32 floats exactly.

  if(payload.size() != 68) {
    throw std::invalid_argument(
        "Payload size is not equal to the required 68 bytes for ID_DVL_VEL.");
  }

  std::map<std::string, double> dvl_vel_dict = {};

  for(std::size_t i = 0; i < 2; i++) {
    dvl_vel_dict[dvl_vel_keys[i]] = read_u32_le(payload, 4 * i);
  }

  for(std::size_t i = 2; i < dvl_vel_keys.size(); i++) {
    dvl_vel_dict[dvl_vel_keys[i]] = read_f32_le(payload, 4 * i);
  }

  return dvl_vel_dict;
}
