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

#include "rplidar.hpp"

RPLidar::RPLidar(const std::string &device) noexcept {
  constexpr const uint32_t BAUDRATE{115200};
  constexpr const uint32_t TIMEOUT{500};
  m_rplidarDevice.reset(new serial::Serial(device, BAUDRATE, serial::Timeout::simpleTimeout(TIMEOUT)));
  if (isOpen()) {
    m_rplidarDevice->setDTR(false);

    m_readingBytesFromDeviceThread.reset(new std::thread(
      [&rplidarDevice = m_rplidarDevice,
       &decoder = m_decoder](){
          const uint16_t BUFFER_SIZE{2048};
          uint8_t *data = new uint8_t[BUFFER_SIZE];
          size_t size{0};
          while (rplidarDevice->isOpen()) {
            if (rplidarDevice->waitReadable()) {
              size_t bytesAvailable{rplidarDevice->available()};
              size_t bytesRead = rplidarDevice->read(data+size, ((BUFFER_SIZE - size) < bytesAvailable) ? (BUFFER_SIZE - size) : bytesAvailable);
              size += bytesRead;

              size_t consumed = decoder.decode(data, size);
              for (size_t i{0}; i < (size - consumed); i++) {
                data[i] = data[i + consumed];
              }

              size -= consumed;
              // If the parser does not work at all, cancel it.
              if (size >= BUFFER_SIZE) {
                break;
              }
            }
          }
          delete [] data;
          data = nullptr;
        }
    ));
  }
}

RPLidar::~RPLidar() {
  if (isOpen()) {
    // Reset device.
    {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      const std::vector<uint8_t> COMMAND_RESET{RPLidarDecoder::SYNC_BYTE0, RPLidarDecoder::RESET};
      m_rplidarDevice->write(COMMAND_RESET);
    }

    m_rplidarDevice->close();
    m_readingBytesFromDeviceThread->join();
  }
  m_rplidarDevice.reset(nullptr);
}

bool RPLidar::isOpen() const noexcept {
  return m_rplidarDevice->isOpen();
}

void RPLidar::startScanning(
    std::function<void(const opendlv::device::lidar::rplidar::DeviceInfo &)> delegateDeviceInfo,
    std::function<void(const opendlv::device::lidar::rplidar::DeviceHealth &)> delegateDeviceHealth,
    std::function<void(opendlv::proxy::PointCloudReading pc)> delegateCompleteScan) {
  // Setup delegates to distribute information.
  m_decoder.setDelegates(delegateDeviceInfo, delegateDeviceHealth, delegateCompleteScan);

  // Reset device.
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const std::vector<uint8_t> COMMAND_RESET{RPLidarDecoder::SYNC_BYTE0, RPLidarDecoder::RESET};
    m_rplidarDevice->write(COMMAND_RESET);
  }

  RPLidarDecoder::RPLidarMessages lastMessage{RPLidarDecoder::UNKNOWN};
  uint8_t attempts{0};

  // Get device info.
  attempts = 4;
  do {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const std::vector<uint8_t> COMMAND_GET_INFO{RPLidarDecoder::SYNC_BYTE0, RPLidarDecoder::GET_INFO};
    m_rplidarDevice->write(COMMAND_GET_INFO);
    lastMessage = m_decoder.getLastRPLidarMessage();
  } while ((lastMessage != RPLidarDecoder::GOT_INFO) && (attempts-- > 0));

  // Get device health.
  attempts = 4;
  do {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const std::vector<uint8_t> COMMAND_GET_HEALTH{RPLidarDecoder::SYNC_BYTE0, RPLidarDecoder::GET_HEALTH};
    m_rplidarDevice->write(COMMAND_GET_HEALTH);
    lastMessage = m_decoder.getLastRPLidarMessage();
  } while ((lastMessage != RPLidarDecoder::GOT_HEALTH) && (attempts-- > 0));

  // Enter scanning mode.
  attempts = 4;
  do {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const std::vector<uint8_t> COMMAND_GET_SCAN{RPLidarDecoder::SYNC_BYTE0, RPLidarDecoder::SCAN};
    m_rplidarDevice->write(COMMAND_GET_SCAN);
    lastMessage = m_decoder.getLastRPLidarMessage();
  } while ((lastMessage != RPLidarDecoder::GOT_SCAN) && (attempts-- > 0));
}

