// bms_parser.cpp
// Decodes raw UART frames from the JK-BMS (JK-BD6A20S10P / similar) into a
// BMSData struct.  Decoding logic follows the 2025 Summer Node-RED flow as
// the authoritative reference.
//
// Frame layout (JK-BMS "GPS" port, response packet):
//   Byte  0-1  : Start-of-frame magic  0x4E 0x57
//   Byte  2-3  : Frame length (uint16, big-endian) – counts bytes from [2] onward
//   Byte  4-7  : Terminal number (uint32, big-endian)
//   Byte  8    : Command word   (0x06 = read-all)
//   Byte  9    : Frame source   (0x00 = BMS)
//   Byte  10   : Transport type (0x00)
//   Byte  11…N : TLV data records (tag, [length,] value …)
//   Byte  N+1  : End-of-record marker 0x68
//   Byte  N+2  : Record type  (0x00)
//   Byte  N+3-4: Frame counter (uint16)
//   Byte  N+5-8: Checksum     (uint32, big-endian; only lower 16-bits compared)

#include "bms_parser.hpp"
#include <algorithm>
#include <cstring>

// ---------------------------------------------------------------------------
uint16_t computeChecksum(const uint8_t* data, size_t len)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return static_cast<uint16_t>(sum & 0xFFFF);
}

// ---------------------------------------------------------------------------
// Temperature decode helper
//   JK-BMS encodes negative temperatures as (100 + |T|).
//   e.g. -20 °C is stored as 120.  Values ≤ 100 are positive.
// ---------------------------------------------------------------------------
static double decodeTemperature(uint16_t raw)
{
    if (raw > 100) {
        return static_cast<double>(100) - static_cast<double>(raw);
    }
    return static_cast<double>(raw);
}

// ---------------------------------------------------------------------------
bool parseBMSFrame(const uint8_t* buffer, size_t length, BMSData& outData)
{
    // Minimum sanity check: header (2) + length (2) + terminal (4) + cmd (1)
    // + src (1) + trans (1) + end marker (1) + record-type (1) + counter (2)
    // + checksum (4) = 19, but we need at least one data byte, so 21.
    if (length < 21) {
        return false;
    }

    // --- Magic bytes --------------------------------------------------------
    if (buffer[0] != 0x4E || buffer[1] != 0x57) {
        return false;
    }

    // --- Frame length field (bytes 2-3) ------------------------------------
    // According to JK-BMS spec the length field counts from byte[2] to the
    // end of the checksum.  Use it to validate we have the whole frame.
    uint16_t frameLength = static_cast<uint16_t>((buffer[2] << 8) | buffer[3]);
    // Total expected buffer size = 2 (magic) + frameLength
    if (length < static_cast<size_t>(2 + frameLength)) {
        return false;
    }
    // Work only within the declared frame boundary from here on.
    size_t frameEnd = 2 + frameLength; // exclusive upper bound

    // --- Checksum (last 4 bytes of frame; only low 16-bits compared) -------
    // The 4-byte field is: 0x00 0x00 [high-byte] [low-byte]
    uint16_t expectedChecksum =
        static_cast<uint16_t>((buffer[frameEnd - 2] << 8) | buffer[frameEnd - 1]);
    uint16_t computedChecksum = computeChecksum(buffer, frameEnd - 4);
    if (computedChecksum != expectedChecksum) {
        return false;
    }

    // --- TLV data records start at byte 11 ---------------------------------
    // Stop before the end-of-record marker (0x68) + 4-byte tail.
    // The 0x68 marker sits at frameEnd - 5.
    size_t idx = 11;
    size_t dataEnd = frameEnd - 5; // exclusive; 0x68 sits at dataEnd

    while (idx < dataEnd) {
        uint8_t tag = buffer[idx++];
        if (idx > dataEnd) break;

        switch (tag) {

            // ----------------------------------------------------------------
            case 0x79: { // Cell voltages
                // byte: count of data bytes that follow (each cell = 3 bytes)
                uint8_t byteCount = buffer[idx++];
                int cellCount = byteCount / 3;
                outData.cellVoltages.clear();
                outData.cellVoltages.reserve(cellCount);
                for (int i = 0; i < cellCount && (idx + 2) < frameEnd; ++i) {
                    // 3-byte big-endian millivolt value
                    uint32_t mv = (static_cast<uint32_t>(buffer[idx])     << 16)
                                | (static_cast<uint32_t>(buffer[idx + 1]) <<  8)
                                |  static_cast<uint32_t>(buffer[idx + 2]);
                    outData.cellVoltages.push_back(mv / 1000.0);
                    idx += 3;
                }
                if (!outData.cellVoltages.empty()) {
                    auto [mn, mx] = std::minmax_element(
                        outData.cellVoltages.begin(), outData.cellVoltages.end());
                    outData.maxCellDiff = *mx - *mn;
                }
                break;
            }

            // ----------------------------------------------------------------
            case 0x80: { // Internal MOSFET temperature
                uint16_t raw = static_cast<uint16_t>((buffer[idx] << 8) | buffer[idx + 1]);
                outData.temperatureMOSFET = decodeTemperature(raw);
                idx += 2;
                break;
            }

            case 0x81: { // Left external probe temperature
                uint16_t raw = static_cast<uint16_t>((buffer[idx] << 8) | buffer[idx + 1]);
                outData.temperatureProbe1 = decodeTemperature(raw);
                idx += 2;
                break;
            }

            case 0x82: { // Right external probe temperature
                uint16_t raw = static_cast<uint16_t>((buffer[idx] << 8) | buffer[idx + 1]);
                outData.temperatureProbe2 = decodeTemperature(raw);
                idx += 2;
                break;
            }

            // ----------------------------------------------------------------
            case 0x83: { // Total pack voltage
                // 3-byte big-endian, unit = mV
                uint32_t mv = (static_cast<uint32_t>(buffer[idx])     << 16)
                            | (static_cast<uint32_t>(buffer[idx + 1]) <<  8)
                            |  static_cast<uint32_t>(buffer[idx + 2]);
                outData.voltage = mv / 1000.0;
                idx += 3;
                break;
            }

            // ----------------------------------------------------------------
            case 0x84: {
                // Current – 3-byte big-endian, unit = 10 mA (i.e. value/100 = A)
                // Sign encoding depends on frame length (per Node-RED 2025 ref):
                //   frameLength < 260  →  short frame: signed offset from 10000
                //                         current [A] = (10000 – rawValue) / 100
                //   frameLength >= 260 →  long  frame: bit-15 is direction flag
                //                         bit-15 set   → discharging (+A)
                //                         bit-15 clear → charging    (−A)
                uint32_t raw = (static_cast<uint32_t>(buffer[idx])     << 16)
                             | (static_cast<uint32_t>(buffer[idx + 1]) <<  8)
                             |  static_cast<uint32_t>(buffer[idx + 2]);
                if (frameLength < 260) {
                    // Short-frame encoding: 10000 is the zero-current offset
                    outData.current = (10000.0 - static_cast<double>(raw)) / 100.0;
                } else {
                    // Long-frame encoding: sign bit in bit-15
                    if (raw & 0x8000) {
                        outData.current =  static_cast<double>(raw & 0x7FFF) / 100.0;
                    } else {
                        outData.current = -static_cast<double>(raw & 0x7FFF) / 100.0;
                    }
                }
                idx += 3;
                break;
            }

            // ----------------------------------------------------------------
            case 0x85: // State of Charge [%]
                outData.capacityPercentage = static_cast<double>(buffer[idx++]);
                break;

            case 0x86: // Number of NTC temperature sensors (informational, skip)
                idx += 1;
                break;

            case 0x87: // Cycle count (uint16)
                outData.cycleCount =
                    static_cast<uint16_t>((buffer[idx] << 8) | buffer[idx + 1]);
                idx += 2;
                break;

            case 0x88: { // Remaining capacity [mAh] → Ah
                // 4-byte big-endian, unit = mAh
                uint32_t mah = (static_cast<uint32_t>(buffer[idx])     << 24)
                             | (static_cast<uint32_t>(buffer[idx + 1]) << 16)
                             | (static_cast<uint32_t>(buffer[idx + 2]) <<  8)
                             |  static_cast<uint32_t>(buffer[idx + 3]);
                outData.capacityAh = mah / 1000.0;
                idx += 4;
                break;
            }

            case 0x89: { // Total cumulative cycle capacity [mAh] → Ah
                uint32_t mah = (static_cast<uint32_t>(buffer[idx])     << 24)
                             | (static_cast<uint32_t>(buffer[idx + 1]) << 16)
                             | (static_cast<uint32_t>(buffer[idx + 2]) <<  8)
                             |  static_cast<uint32_t>(buffer[idx + 3]);
                outData.cycleCapacityAh = mah / 1000.0;
                idx += 4;
                break;
            }

            case 0x8A: // Number of battery strings (uint16)
                outData.numStrings =
                    static_cast<uint16_t>((buffer[idx] << 8) | buffer[idx + 1]);
                idx += 2;
                break;

            case 0x8B: { // Alarm / warning flags (uint16)
                uint16_t alarms =
                    static_cast<uint16_t>((buffer[idx] << 8) | buffer[idx + 1]);
                // Bit map from JK-BMS documentation:
                //   bit 0 : low SOC warning
                //   bit 3 : cell over-temperature
                //   bit 7 : over-current (discharge)
                //   bit 8 : over-current (charge)
                outData.alarmLowCapacity = (alarms & 0x0001) != 0;
                outData.alarmOverTemp    = (alarms & 0x0008) != 0;
                outData.alarmOverCurrent = (alarms & 0x0080) != 0;
                idx += 2;
                break;
            }

            case 0x8C: { // Status flags (uint16)
                uint16_t status =
                    static_cast<uint16_t>((buffer[idx] << 8) | buffer[idx + 1]);
                // bit 0 : charge MOSFET on
                // bit 1 : discharge MOSFET on
                // bit 2 : balancer active
                outData.isCharging    = (status & 0x0001) != 0;
                outData.isDischarging = (status & 0x0002) != 0;
                outData.isBalancing   = (status & 0x0004) != 0;
                idx += 2;
                break;
            }

            case 0x68: // End-of-record marker – should be caught by loop bound
                return true;

            default:
                // Unknown tag – the JK-BMS may add proprietary tags in newer
                // firmware.  We cannot safely skip an unknown TLV without a
                // length byte, so stop parsing here and return whatever we got.
                return true;
        }
    }

    return true;
}
