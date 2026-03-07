#ifndef BMS_PARSER_HPP
#define BMS_PARSER_HPP

#include <vector>
#include <cstdint>

struct BMSWarning {
    bool lowCapacity = false;
    bool powerTubeOvertemperature = false;
    bool chargingOvervoltage = false;
    bool dischargingUndervoltage = false;
    bool batteryOverTemperature = false;
    bool chargingOvercurrent = false;
    bool dischargingOvercurrent = false;
    bool cellPressureDifference = false;
    bool overtemperatureAlarmInTheBatteryBox = false;
    bool batteryLowTemperature = false;
    bool cellOvervoltage = false;
    bool cellUndervoltage = false;
    bool alarm309_AProtection = false;
    bool alarm309_BProtection = false;
};

struct BMSStatus {
    bool chargingEnabled = false;
    bool dischargingEnabled = false;
    bool balancingEnabled = false;
    bool batteryConnected = false;
};

struct BMSData {
    uint16_t numberOfCells = 0;
    std::vector<double> cellVoltages;
    double maxCellDiff = 0.0;

    uint16_t temperatureInternal = 0;
    uint16_t temperatureBattery1 = 0;
    uint16_t temperatureBattery2 = 0;

    double voltage = 0.0;
    double current = 0.0;
    int currentRaw = 0;
    double power = 0.0;

    uint8_t remainingBattery = 0;
    uint8_t numberOfNTC = 0;
    uint16_t numberOfBatteryCycles = 0;
    uint32_t batteryCycleCapacityAh = 0;
    uint16_t numberOfStrings = 0;

    BMSWarning warning;
    BMSStatus status;
};

uint16_t chksum(const std::vector<uint8_t>& buffer, size_t len);
BMSData parseBMSFrame(const std::vector<uint8_t>& buffer);

#endif // BMS_PARSER_HPP