#ifndef BMS_PARSER_HPP
#define BMS_PARSER_HPP

#include <vector>
#include <cstdint>

// ---------------------------------------------------------------------------
// BMSData
//   Holds all values decoded from a single JK-BMS response frame.
//   Units:  voltages → V (double),  current → A (positive = discharge),
//           temperatures → °C,  capacity → Ah,  percentage → 0-100 %.
// ---------------------------------------------------------------------------
struct BMSData {
    // ---- Cell-level data ---------------------------------------------------
    std::vector<double> cellVoltages;   // Individual cell voltages [V]
    double maxCellDiff = 0.0;           // max – min cell voltage [V]

    // ---- Temperature sensors -----------------------------------------------
    double temperatureMOSFET = 0.0;    // Internal MOSFET sensor [°C]
    double temperatureProbe1  = 0.0;   // Left  external probe   [°C]
    double temperatureProbe2  = 0.0;   // Right external probe   [°C]

    // ---- Pack-level electrical data ----------------------------------------
    double   voltage            = 0.0; // Total pack voltage       [V]
    double   current            = 0.0; // Pack current (+dis / -chg) [A]
    double   capacityPercentage = 0.0; // State of Charge          [%]
    double   capacityAh         = 0.0; // Remaining capacity       [Ah]

    // ---- Battery info ------------------------------------------------------
    uint16_t cycleCount       = 0;
    double   cycleCapacityAh  = 0.0;
    uint16_t numStrings       = 0;

    // ---- Status flags ------------------------------------------------------
    bool isCharging    = false;
    bool isDischarging = false;
    bool isBalancing   = false;

    // ---- Alarm flags -------------------------------------------------------
    bool alarmLowCapacity = false;
    bool alarmOverTemp    = false;
    bool alarmOverCurrent = false;
};

// ---------------------------------------------------------------------------
// computeChecksum
//   JK-BMS uses a simple 16-bit sum of all bytes up to (not including) the
//   4-byte checksum trailer.
// ---------------------------------------------------------------------------
uint16_t computeChecksum(const uint8_t* data, size_t len);

// ---------------------------------------------------------------------------
// parseBMSFrame
//   Parses a raw byte buffer received from the JK-BMS "GPS" UART port.
//   Returns true and populates outData on success.
//   Returns false if the buffer is too short, the header magic is wrong,
//   or the checksum does not match.
// ---------------------------------------------------------------------------
bool parseBMSFrame(const uint8_t* buffer, size_t length, BMSData& outData);

#endif // BMS_PARSER_HPP
