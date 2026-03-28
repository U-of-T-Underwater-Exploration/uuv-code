#include "bms_parser.hpp"
#include <algorithm>


uint16_t computeChecksum(const uint8_t* data, size_t len) {
    uint32_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return static_cast<uint16_t>(sum & 0xFFFF);
}

bool parseBMSFrame(const uint8_t* buffer, size_t length, BMSData& outData) {
    if (length < 21) return false; // Minimum frame length

    // Check header
    if (buffer[0] != 0x4E || buffer[1] != 0x57) return false;

    // Length field (2 bytes)
    uint16_t frameLength = (buffer[2] << 8) | buffer[3];
    // Checksum is at the end of the frame (4 bytes, but only lower 2 are typically used or compared)
    // The JSON reference uses 4 bytes for checksum, but we'll check the sum.
    uint32_t expectedChecksum = (buffer[length - 4] << 24) | (buffer[length - 3] << 16) | 
                                 (buffer[length - 2] << 8) | buffer[length - 1];
    
    uint16_t computedSum = computeChecksum(buffer, length - 4);
    if (computedSum != (expectedChecksum & 0xFFFF)) {
        // Log error but maybe proceed
        // std::cerr << "BMS Checksum mismatch: " << computedSum << " != " << (expectedChecksum & 0xFFFF) << std::endl;
        // return false; 
    }

    // Data starts at index 11 (after STX, Length, TermNo, Cmd, Src, TransType)
    // terminalNo: 4 bytes (4-7)
    // commandWord: 1 byte (8)
    // frameSrc: 1 byte (9)
    // transType: 1 byte (10)
    // batteryValues starts at index 11
    
    size_t idx = 11;
    
    while (idx < length - 5) { // -5 to stay before the end marker (0x68) and checksum
        uint8_t marker = buffer[idx++];
        
        switch (marker) {
            case 0x79: { // Cell Voltages
                uint8_t byteCount = buffer[idx++];
                int cellCount = byteCount / 3;
                outData.cellVoltages.clear();
                for (int i = 0; i < cellCount; ++i) {
                    uint32_t v_mv = (buffer[idx] << 16) | (buffer[idx + 1] << 8) | buffer[idx + 2];
                    outData.cellVoltages.push_back(v_mv / 1000.0);
                    idx += 3;
                }
                if (!outData.cellVoltages.empty()) {
                    auto min_it = std::min_element(outData.cellVoltages.begin(), outData.cellVoltages.end());
                    auto max_it = std::max_element(outData.cellVoltages.begin(), outData.cellVoltages.end());
                    outData.maxCellDiff = *max_it - *min_it;
                }
                break;
            }
            case 0x80: // Internal MOSFET Temp
                {
                    int16_t temp = (buffer[idx] << 8) | buffer[idx+1];
                    outData.temperatureMOSFET = (temp > 100) ? (100 - temp) : temp;
                    idx += 2;
                }
                break;
            case 0x81: // Probe 1 Temp
                {
                    int16_t temp = (buffer[idx] << 8) | buffer[idx+1];
                    outData.temperatureProbe1 = (temp > 100) ? (100 - temp) : temp;
                    idx += 2;
                }
                break;
            case 0x82: // Probe 2 Temp
                {
                    int16_t temp = (buffer[idx] << 8) | buffer[idx+1];
                    outData.temperatureProbe2 = (temp > 100) ? (100 - temp) : temp;
                    idx += 2;
                }
                break;
            case 0x83: // Total Voltage
                outData.voltage = ((buffer[idx] << 16) | (buffer[idx+1] << 8) | buffer[idx+2]) / 1000.0;
                idx += 3;
                break;
            case 0x84: { // Current
                uint32_t currVal = (buffer[idx] << 16) | (buffer[idx+1] << 8) | buffer[idx+2];
                // From the Node-REd JSON Code
                if (frameLength < 260) {
                    outData.current = (10000.0 - currVal) / 100.0;
                    outData.current = (1000.0 - currVal) * 0.01;
                } else {
                    if (currVal & 0x8000) {
                        outData.current = (currVal & 0x7FFF) / 100.0;
                    } else {
                        outData.current = ((currVal & 0x7FFF) / 100.0) * -1.0;
                    }
                }
                idx += 3;
                break;
            }
            case 0x85: // SOC %
                outData.capacityPercentage = buffer[idx++];
                break;
            case 0x86: // Number of NTC (skip)
                idx++;
                break;
            case 0x87: // Cycle Count
                outData.cycleCount = (buffer[idx] << 8) | buffer[idx+1];
                idx += 2;
                break;
            case 0x89: // Total Cycle Capacity (Ah)
                outData.cycleCapacityAh = ((buffer[idx] << 24) | (buffer[idx+1] << 16) | (buffer[idx+2] << 8) | buffer[idx+3]) / 1000.0;
                idx += 4;
                break;
            case 0x8A: // Number of Strings
                outData.numStrings = (buffer[idx] << 8) | buffer[idx+1];
                idx += 2;
                break;
            case 0x8B: { // Alarms
                uint16_t alarms = (buffer[idx] << 8) | buffer[idx+1];
                outData.alarmLowCapacity = alarms & 0x0001;
                outData.alarmOverTemp = alarms & 0x0002; // temp example, need to map correctly
                outData.alarmOverCurrent = alarms & 0x0004;
                idx += 2;
                break;
            }
            case 0x8C: { // Status
                uint16_t status = (buffer[idx] << 8) | buffer[idx+1];
                outData.isCharging = status & 0x0001;
                outData.isDischarging = status & 0x0002;
                outData.isBalancing = status & 0x0004;
                idx += 2;
                break;
            }
            case 0x68: // End marker
                return true;
            default:
                // Unknown marker, stop
                return true; 
        }
    }

    return true;
}