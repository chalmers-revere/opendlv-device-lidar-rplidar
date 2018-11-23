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

#ifndef RPLIDAR
#define RPLIDAR

#include "opendlv-standard-message-set.hpp"
#include "rplidar-message-set.hpp"
#include "serialport.hpp"

#include "rplidar-decoder.hpp"

#include <functional>
#include <memory>
#include <thread>

class RPLidar {
 private:
  RPLidar(const RPLidar &) = delete;
  RPLidar(RPLidar &&)      = delete;
  RPLidar &operator=(const RPLidar &) = delete;
  RPLidar &operator=(RPLidar &&) = delete;

 public:
  RPLidar(const std::string &device) noexcept;
  ~RPLidar();

 public:
  bool isOpen() const noexcept;
  void startScanning(std::function<void(const opendlv::device::lidar::rplidar::DeviceInfo &)> delegateDeviceInfo,
                     std::function<void(const opendlv::device::lidar::rplidar::DeviceHealth &)> delegateDeviceHealth,
                     std::function<void(opendlv::proxy::PointCloudReading pc)> delegateCompleteScan);

 private:
  std::unique_ptr<serial::Serial> m_rplidarDevice{nullptr};
  std::unique_ptr<std::thread> m_readingBytesFromDeviceThread{nullptr};

  RPLidarDecoder m_decoder{};
};

#endif

