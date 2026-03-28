#ifndef BMS_PARSER_HPP
#define BMS_PARSER_HPP

#include <vector>
#include <cstdint>

struct BMSData {
    std::vector<double> cellVoltages;
    double maxCellDiff = 0.0;

    // Temperatures
    double temperatureMOSFET = 0.0;
    double temperatureProbe1 = 0.0;
    double temperatureProbe2 = 0.0;

    // Battery Data
    double voltage = 0.0;
    double current = 0.0;
    double capacityPercentage = 0.0;
    double capacityAh = 0.0;

    // Battery Info
    uint16_t cycleCount = 0;
    double cycleCapacityAh = 0.0;
    uint16_t numStrings = 0;

    // Battery States
    bool isCharging = false;
    bool isDischarging = false;
    bool isBalancing = false;

    // Alarms
    bool alarmLowCapacity = false;
    bool alarmOverTemp = false;
    bool alarmOverCurrent = false;
};

// Compute simple checksum: sum of bytes
uint16_t computeChecksum(const uint8_t* data, size_t len);

// Parse the full buffer received from BMS
bool parseBMSFrame(const uint8_t* buffer, size_t length, BMSData& outData);

#endif // BMS_PARSER_HPP