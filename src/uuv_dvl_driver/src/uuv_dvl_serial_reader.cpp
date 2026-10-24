#include "uuv_dvl_driver/uuv_dvl_serial_reader.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

DvlSerialReader::DvlSerialReader(const std::string &device) {
  fd_ = ::open(device.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);

  if(fd_ < 0) {
    throw std::runtime_error("Cannot open serial device " + device + ": " + std::strerror(errno));
  }

  termios tty{};

  if(tcgetattr(fd_, &tty) != 0) {
    const std::string error = std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcgetattr failed: " + error);
  }

  cfmakeraw(&tty);

  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;

  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CRTSCTS;

  // Kogger Micro DVL UART speed
  if(cfsetispeed(&tty, B921600) != 0 || cfsetospeed(&tty, B921600) != 0 ||
     tcsetattr(fd_, TCSANOW, &tty) != 0) {
    const std::string error = std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("Failed to configure serial port: " + error);
  }
}

DvlSerialReader::~DvlSerialReader() {
  if(fd_ >= 0) { ::close(fd_); }
}

void DvlSerialReader::poll(const FrameCallback &on_frame) {
  uint8_t bytes[1024];

  // Read all bytes currently available.
  while(true) {
    const ssize_t n = ::read(fd_, bytes, sizeof(bytes));

    if(n > 0) {
      buffer_.insert(buffer_.end(), bytes, bytes + n);
    } else if(n == 0) {
      break;
    } else if(errno == EINTR) {
      continue;
    } else if(errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    } else {
      throw std::runtime_error(std::string("Serial read failed: ") + std::strerror(errno));
    }
  }

  // The Kogger SBP frame has 6 header bytes,
  // a variable-length payload, and 2 checksum bytes.
  while(buffer_.size() >= 8) {
    // Locate the synchronization bytes.
    if(buffer_[0] != 0xBB || buffer_[1] != 0x55) {
      buffer_.erase(buffer_.begin());
      continue;
    }

    const uint8_t length = buffer_[5];

    if(length > 128) {
      buffer_.erase(buffer_.begin());
      continue;
    }

    const std::size_t frame_size = 6 + static_cast<std::size_t>(length) + 2;

    // Wait for the rest of the frame.
    if(buffer_.size() < frame_size) { break; }

    // Fletcher-16 over ROUTE, MODE, ID,
    // LENGTH, and PAYLOAD.
    uint8_t check1 = 0;
    uint8_t check2 = 0;

    for(std::size_t i = 2; i < 6 + length; ++i) {
      check1 = static_cast<uint8_t>(check1 + buffer_[i]);
      check2 = static_cast<uint8_t>(check2 + check1);
    }

    if(check1 != buffer_[6 + length] || check2 != buffer_[7 + length]) {
      // Invalid packet: advance one byte and
      // search again for synchronization.
      buffer_.erase(buffer_.begin());
      continue;
    }

    const uint8_t mode = buffer_[3];
    const uint8_t id = buffer_[4];

    std::vector<uint8_t> payload(buffer_.begin() + 6, buffer_.begin() + 6 + length);

    // This complete frame has been consumed.
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);

    on_frame(id, mode, payload);
  }
}
