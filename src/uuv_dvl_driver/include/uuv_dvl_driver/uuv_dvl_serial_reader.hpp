#pragma once
#ifndef __linux__
#error "DvlSerialReader requires Linux"
#endif

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

class DvlSerialReader {
public:
    using FrameCallback = std::function<void(
        uint8_t id,
        uint8_t mode,
        const std::vector<uint8_t>& payload)>;

    explicit DvlSerialReader(const std::string& device);
    ~DvlSerialReader();

    DvlSerialReader(const DvlSerialReader&) = delete;
    DvlSerialReader& operator=(const DvlSerialReader&) = delete;

    void poll(const FrameCallback& on_frame);

private:
    int fd_ = -1;
    std::vector<uint8_t> buffer_;
};
