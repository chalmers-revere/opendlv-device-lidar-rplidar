/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RPLIDAR_DECODER
#define RPLIDAR_DECODER

#include "opendlv-standard-message-set.hpp"
#include "rplidar-message-set.hpp"

#include <mutex>
#include <sstream>

class RPLidarDecoder {
 public:
  enum RPLidarMessages {
    UNKNOWN     = 0,
    GOT_INFO    = 0x04,
    GOT_HEALTH  = 0x06,
    GOT_SCAN    = 0x81,
  };

  enum RPLidarBytes {
    GET_INFO    = 0x50,
    GET_HEALTH  = 0x52,
    RESET       = 0x40,
    SCAN        = 0x20,
    STOP        = 0x25,
    SYNC_BYTE0  = 0xA5,
    SYNC_BYTE1  = 0x5A,
  };

 private:
  RPLidarDecoder(const RPLidarDecoder &) = delete;
  RPLidarDecoder(RPLidarDecoder &&)      = delete;
  RPLidarDecoder &operator=(const RPLidarDecoder &) = delete;
  RPLidarDecoder &operator=(RPLidarDecoder &&) = delete;

 public:
  RPLidarDecoder() = default;
  ~RPLidarDecoder() = default;

 public:
  size_t decode(const uint8_t *buffer, const size_t size) noexcept;

 private:
  bool parseMessage(const uint8_t *buf, const size_t offset, const size_t sizeOfMessage, RPLidarMessages type) noexcept;
  bool parseScan(const uint8_t *buf, const size_t offset, const size_t length) noexcept;

  opendlv::device::lidar::rplidar::DeviceInfo getDeviceInfo(const uint8_t *buffer, const size_t offset, const size_t sizeOfMessage) noexcept;
  opendlv::device::lidar::rplidar::DeviceHealth getDeviceHealth(const uint8_t *buffer, const size_t offset, const size_t sizeOfMessage) noexcept;

 private:
  bool m_inScanningMode{false};
  uint32_t m_payloadSize{0};
  RPLidarMessages m_nextRPLidarMessage{RPLidarDecoder::UNKNOWN};
  bool m_foundFirstStart{false};
  float m_startAzimuth{0};
  uint32_t m_anglesWritten{0};
  std::stringstream m_angles{};
  std::stringstream m_distances{};

 public:
  RPLidarMessages getLastRPLidarMessage() const noexcept;
  opendlv::device::lidar::rplidar::DeviceInfo getDeviceInfo() const noexcept;
  opendlv::device::lidar::rplidar::DeviceHealth getDeviceHealth() const noexcept;

 private:
  mutable std::mutex m_dataMutex{};
  RPLidarMessages m_lastRPLidarMessage{RPLidarDecoder::UNKNOWN};
  opendlv::device::lidar::rplidar::DeviceInfo m_deviceInfo{};
  opendlv::device::lidar::rplidar::DeviceHealth m_deviceHealth{};
  opendlv::proxy::PointCloudReading m_pointCloudReading{};
};

#endif

