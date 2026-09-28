#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <iostream>

class MotorDriver {
  
public:
  MotorDriver() : fd_(-1) {}
  ~MotorDriver() { closePort(); }

  bool openPort(const std::string & port_name, speed_t baud_rate = B115200) {
    fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
      return false;
    }

    struct termios options;
    tcgetattr(fd_, &options);

    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcflush(fd_, TCIFLUSH);
    tcsetattr(fd_, TCSANOW, &options);
    fcntl(fd_, F_SETFL, FNDELAY);

    rx_buffer_.clear();
    return true;
  }

  void closePort() {
    if (fd_ != -1) {
      close(fd_);
      fd_ = -1;
    }
    rx_buffer_.clear();
  }

  bool isConnected() const { return fd_ != -1; }

  bool writeString(const std::string & msg) {
    if (fd_ == -1) return false;
    ssize_t bytes_written = write(fd_, msg.c_str(), msg.length());
    return bytes_written == static_cast<ssize_t>(msg.length());
  }

  // Returns a complete line only when a full '\n' packet has arrived.
  // Returns an empty string if the incoming message is still incomplete.
  std::string readLine() {
    if (fd_ == -1) return "";

    // 1. Slurp all bytes currently waiting in the Linux kernel buffer
    char chunk[64];
    ssize_t bytes_read;
    while ((bytes_read = read(fd_, chunk, sizeof(chunk))) > 0) {
      rx_buffer_.append(chunk, bytes_read);
    }

    // 2. Check if a full sentence ending with '\n' exists in the accumulator
    size_t newline_pos = rx_buffer_.find('\n');
    if (newline_pos == std::string::npos) {
      // Incomplete sentence: keep bytes in rx_buffer_ and wait for the rest
      return "";
    }

    // 3. Extract the full sentence up to the newline
    std::string line = rx_buffer_.substr(0, newline_pos);

    // 4. Erase the extracted sentence from the accumulator
    rx_buffer_.erase(0, newline_pos + 1);

    // 5. Clean up any trailing carriage return ('\r')
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    return line;
  }

private:
  int fd_;
  std::string rx_buffer_; // Persistent accumulator memory across read cycles
};

