#include <iostream>
#include <vector>
#include <cstdint>
#include <algorithm>

#include "bms_parser.hpp"



// Compute simple checksum: sum of bytes
uint16_t chksum(const std::vector<uint8_t>& buffer, size_t len) {
    uint16_t sum = 0;
    for(size_t i=0; i<len; ++i) {
        sum += buffer[i];
    }
    return sum;
}

BMSData parseBMSFrame(const std::vector<uint8_t>& buffer) {
    BMSData bms;

    if(buffer.size() < 10) {
        std::cerr << "Frame too short!" << std::endl;
        return bms;
    }

    // Check header
    if(buffer[0] != 0x4E || buffer[1] != 0x57) {
        std::cerr << "Invalid header!" << std::endl;
        return bms;
    }

    uint16_t dataLen = (buffer[2] << 8) | buffer[3];

    // CRC
    uint16_t computedCRC = chksum(buffer, dataLen);
    uint16_t remoteCRC = (buffer[dataLen] << 8) | buffer[dataLen + 1];

    if(computedCRC != remoteCRC) {
        std::cerr << "CRC check failed!" << std::endl;
    }

    size_t idx = 12; // start after header + length + unknown fields

    // Number of cells
    bms.numberOfCells = buffer[idx] / 3;
    bms.cellVoltages.resize(bms.numberOfCells);
    for(int cell=0; cell < bms.numberOfCells; ++cell) {
        size_t base = idx + 1 + cell*3;
        bms.cellVoltages[cell] = (buffer[base - 1] << 8 | buffer[base]) / 1000.0;
    }
    bms.maxCellDiff = *std::max_element(bms.cellVoltages.begin(), bms.cellVoltages.end()) -
                      *std::min_element(bms.cellVoltages.begin(), bms.cellVoltages.end());

    idx += 1 + bms.numberOfCells * 3;

    // Temperature marker 0x80
    ++idx;
    if(buffer[idx] != 0x80) std::cerr << "Temperature marker incorrect!" << std::endl;
    bms.temperatureInternal = (buffer[idx+1] << 8 | buffer[idx+2]);
    bms.temperatureBattery1 = (buffer[idx+4] << 8 | buffer[idx+5]);
    bms.temperatureBattery2 = (buffer[idx+7] << 8 | buffer[idx+8]);
    idx += 9;

    // Voltage 0x83
    ++idx;
    if(buffer[idx] != 0x83) std::cerr << "Voltage marker incorrect!" << std::endl;
    bms.voltage = (buffer[idx+1] << 8 | buffer[idx+2]) / 100.0;
    idx += 3;

    // Current 0x84
    ++idx;
    if(buffer[idx] != 0x84) std::cerr << "Current marker incorrect!" << std::endl;
    bms.currentRaw = buffer[idx+1] << 8 | buffer[idx+2];
    const int CURRENT_ZERO = 32768;
    if(bms.currentRaw < CURRENT_ZERO)
        bms.current = bms.currentRaw / -100.0;
    else
        bms.current = (bms.currentRaw - CURRENT_ZERO) / 100.0;
    bms.power = bms.current * bms.voltage;
    idx += 3;

    // Remaining battery 0x85
    ++idx;
    if(buffer[idx] != 0x85) std::cerr << "SOC marker incorrect!" << std::endl;
    bms.remainingBattery = buffer[idx+1];
    idx += 2;

    // Number of NTC 0x86
    ++idx;
    if(buffer[idx] != 0x86) std::cerr << "NTC marker incorrect!" << std::endl;
    bms.numberOfNTC = buffer[idx+1];
    idx += 2;

    // Number of battery cycles 0x87
    ++idx;
    if(buffer[idx] != 0x87) std::cerr << "Battery cycles marker incorrect!" << std::endl;
    bms.numberOfBatteryCycles = (buffer[idx+1] << 8 | buffer[idx+2]);
    idx += 3;

    // Battery cycle capacity Ah 0x89
    ++idx;
    if(buffer[idx] != 0x89) std::cerr << "Battery cycle capacity marker incorrect!" << std::endl;
    bms.batteryCycleCapacityAh = (buffer[idx+1] << 24 | buffer[idx+2] << 16 | buffer[idx+3] << 8 | buffer[idx+4]);
    idx += 5;

    // Number of strings 0x8A
    ++idx;
    if(buffer[idx] != 0x8A) std::cerr << "Number of strings marker incorrect!" << std::endl;
    bms.numberOfStrings = (buffer[idx+1] << 8 | buffer[idx+2]);
    idx += 3;

    // Battery warning 0x8B
    ++idx;
    if(buffer[idx] != 0x8B) std::cerr << "Warning marker incorrect!" << std::endl;
    uint16_t warningBits = buffer[idx+1] << 8 | buffer[idx+2];
    bms.warning.lowCapacity = warningBits & 0x0001;
    bms.warning.powerTubeOvertemperature = warningBits & 0x0010;
    bms.warning.chargingOvervoltage = warningBits & 0x0100;
    bms.warning.dischargingUndervoltage = warningBits & 0x1000;
    bms.warning.batteryOverTemperature = warningBits & 0x10000;
    bms.warning.chargingOvercurrent = warningBits & 0x100000;
    bms.warning.dischargingOvercurrent = warningBits & 0x1000000;
    bms.warning.cellPressureDifference = warningBits & 0x10000000;
    // Add other warnings similarly
    idx += 3;

    // Battery status 0x8C
    ++idx;
    if(buffer[idx] != 0x8C) std::cerr << "Status marker incorrect!" << std::endl;
    uint16_t statusBits = buffer[idx+1] << 8 | buffer[idx+2];
    bms.status.chargingEnabled = statusBits & 0x0001;
    bms.status.dischargingEnabled = statusBits & 0x0010;
    bms.status.balancingEnabled = statusBits & 0x0100;
    bms.status.batteryConnected = statusBits & 0x1000;

    return bms;
}

int main() {
    // Example usage
    std::vector<uint8_t> exampleFrame = { 
        0x4E, 0x57, 0x00, 0x13, /* ... rest of frame ... */ 
    };

    BMSData bms = parseBMSFrame(exampleFrame);

    std::cout << "Voltage: " << bms.voltage << "V\n";
    std::cout << "Current: " << bms.current << "A\n";
    std::cout << "SOC: " << (int)bms.remainingBattery << "%\n";

    return 0;
}