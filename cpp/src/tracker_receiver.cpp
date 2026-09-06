#include "panda_tracker/tracker_receiver.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace panda_tracker {

TrackerReceiver::TrackerReceiver(
    const std::string& bind_ip,
    const std::string& expected_source_ip,
    std::uint16_t port)
    : expected_source_ip_(expected_source_ip) {
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    throw std::runtime_error(
        std::string("Unable to create tracker socket: ") +
        std::strerror(errno));
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  if (inet_pton(AF_INET, bind_ip.c_str(), &address.sin_addr) != 1) {
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::invalid_argument("Invalid tracker bind IP: " + bind_ip);
  }

  if (bind(
          socket_fd_,
          reinterpret_cast<sockaddr*>(&address),
          sizeof(address)) < 0) {
    const std::string message = std::strerror(errno);
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("Unable to bind tracker socket: " + message);
  }

  const int current_flags = fcntl(socket_fd_, F_GETFL, 0);
  if (current_flags < 0 ||
      fcntl(socket_fd_, F_SETFL, current_flags | O_NONBLOCK) < 0) {
    const std::string message = std::strerror(errno);
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error(
        "Unable to make tracker socket non-blocking: " + message);
  }
}

TrackerReceiver::~TrackerReceiver() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
  }
}

void TrackerReceiver::poll() {
  while (true) {
    std::array<std::uint8_t, 2048> buffer{};
    sockaddr_in source{};
    socklen_t source_size = sizeof(source);
    const ssize_t received = recvfrom(
        socket_fd_,
        buffer.data(),
        buffer.size(),
        0,
        reinterpret_cast<sockaddr*>(&source),
        &source_size);

    if (received < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error(
          std::string("Tracker recvfrom failed: ") + std::strerror(errno));
    }

    char source_text[INET_ADDRSTRLEN]{};
    if (inet_ntop(
            AF_INET,
            &source.sin_addr,
            source_text,
            sizeof(source_text)) == nullptr) {
      ++rejected_packets_;
      continue;
    }

    const std::string source_ip = source_text;
    if (!expected_source_ip_.empty() && source_ip != expected_source_ip_) {
      ++wrong_source_packets_;
      continue;
    }

    TaskPosePacket packet{};
    const DecodeStatus status = decode_task_pose(
        buffer.data(), static_cast<std::size_t>(received), packet);
    if (status != DecodeStatus::kOk) {
      ++rejected_packets_;
      last_decode_error_ = status;
      continue;
    }

    if (snapshot_.available &&
        packet.sequence_id <= snapshot_.packet.sequence_id) {
      ++duplicate_or_old_packets_;
      continue;
    }

    snapshot_.packet = packet;
    snapshot_.arrival = std::chrono::steady_clock::now();
    snapshot_.source_ip = source_ip;
    snapshot_.source_port = ntohs(source.sin_port);
    snapshot_.available = true;
    ++accepted_packets_;
  }
}

TrackerSnapshot TrackerReceiver::latest() const {
  return snapshot_;
}

std::uint64_t TrackerReceiver::accepted_packets() const {
  return accepted_packets_;
}

std::uint64_t TrackerReceiver::rejected_packets() const {
  return rejected_packets_;
}

std::uint64_t TrackerReceiver::duplicate_or_old_packets() const {
  return duplicate_or_old_packets_;
}

std::uint64_t TrackerReceiver::wrong_source_packets() const {
  return wrong_source_packets_;
}

DecodeStatus TrackerReceiver::last_decode_error() const {
  return last_decode_error_;
}

}  // namespace panda_tracker
