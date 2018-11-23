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

//http://dai.fmph.uniba.sk/projects/smelyzajko/datasheets/sensors/lidar-protokol_LR001_SLAMTEC_rplidar_protocol_v0.2_en.pdf

#include "cluon-complete.hpp"
#include "rplidar-decoder.hpp"

#include <sstream>
#include <string>

RPLidarDecoder::RPLidarMessages RPLidarDecoder::getLastRPLidarMessage() const noexcept {
  std::lock_guard<std::mutex> lck(m_dataMutex);
  return m_lastRPLidarMessage;
}

opendlv::device::lidar::rplidar::DeviceInfo RPLidarDecoder::getDeviceInfo() const noexcept {
  std::lock_guard<std::mutex> lck(m_dataMutex);
  return m_deviceInfo;
}

opendlv::device::lidar::rplidar::DeviceHealth RPLidarDecoder::getDeviceHealth() const noexcept {
  std::lock_guard<std::mutex> lck(m_dataMutex);
  return m_deviceHealth;
}

size_t RPLidarDecoder::decode(const uint8_t *buffer, const size_t size) noexcept {
  const size_t HEADER_SIZE{7};
  size_t offset{0};
  while (true) {
    if (m_inScanningMode) {
      if (offset + 5 > size) {
        return size;
      }

      if (parseScan(buffer, offset, 5)) {
        offset += 5;
      }
      else {
        offset += 1;
      }
    }
    else {
      // Sanity check whether we consumed all data.
      if ((offset + 6) > size) {
        return size;
      }

      if ( (buffer[offset + 0] == RPLidarBytes::SYNC_BYTE0) &&
           (buffer[offset + 1] == RPLidarBytes::SYNC_BYTE1) ) {
        uint32_t info = ((buffer[offset + 2] & 0xFF)) |
                        ((buffer[offset + 3] & 0xFF) << 8) |
                        ((buffer[offset + 4] & 0xFF) << 16) |
                        ((buffer[offset + 5] & 0xFF) << 24);
        //const uint8_t sendMode = (info >> 30) & 0xFF;
        m_payloadSize = info & 0x3FFFFFFF;
        m_nextRPLidarMessage = static_cast<RPLidarMessages>(buffer[offset + 6]);

        //std::cout << "Packet 0x" << std::hex << +m_nextRPLidarMessage << std::dec << ", length = " << m_payloadSize << ", sendMode = " << +sendMode << std::endl;

        // Do not process further as we need more data.
        if ((offset + HEADER_SIZE + m_payloadSize) > size) {
          return offset;
        }

        // Parse contained message.
        if (parseMessage(buffer, offset + HEADER_SIZE, m_payloadSize, m_nextRPLidarMessage)) {
          offset += HEADER_SIZE + m_payloadSize;
          m_lastRPLidarMessage = m_nextRPLidarMessage;
        }
        else {
          // Parsing failed even though we found the two SYNC bytes; consume them and start over.
          offset += 2;
        }
      }
      else {
        // No SYNC bytes found yet; consume one byte.
        offset++;
      }
    }
  }
  // We should not get here as we will leave the state machine inside
  // the while loop at certain point. If we are still getting here,
  // we simply discard everything and start over.
  return size;
}

bool RPLidarDecoder::parseMessage(const uint8_t *buffer, const size_t offset, const size_t sizeOfMessage, RPLidarMessages type) noexcept {
  if (RPLidarDecoder::GOT_INFO == type) {
    std::lock_guard<std::mutex> lck(m_dataMutex);
    m_deviceInfo = getDeviceInfo(buffer, offset, sizeOfMessage);
    return true;
  }
  else if (RPLidarDecoder::GOT_HEALTH == type) {
    std::lock_guard<std::mutex> lck(m_dataMutex);
    m_deviceHealth = getDeviceHealth(buffer, offset, sizeOfMessage);
    return true;
  }
  else if (RPLidarDecoder::GOT_SCAN == type) {
    if (parseScan(buffer, offset, sizeOfMessage)) {
      m_inScanningMode = true;
      return true;
    }
  }
  return false;
}

bool RPLidarDecoder::parseScan(const uint8_t *buffer, const size_t offset, const size_t length) noexcept {
  if (5 != length) {
    return false;
  }

  uint8_t byte0 = buffer[offset + 0];
  uint8_t byte1 = buffer[offset + 1];

  bool startFlag = (byte0 & 0x1) != 0;
  bool inverseStartFlag = (byte0 & 0x2) != 0;

  // startFlag and inverseStartFlag must always be different.
  if (startFlag == inverseStartFlag) {
    return false;
  }

  // Check bit must always be 1.
  if (0x1 != (byte1 & 0x1)) {
    return false;
  }

  uint8_t quality{byte0 >> 2};
  float angle = ((byte1 & 0xFF) | ((buffer[offset + 2] & 0xFF) << 8)) >> 1;
  angle /= 64.0f;

  float distance = ((buffer[offset + 3] & 0xFF) | ((buffer[offset + 4] & 0xFF) << 8));
  distance /= 4.0f;
  // Turn into m.
  distance /= 1000.0f;

  std::cout << "Scan: start = " << startFlag << ", quality = " << +quality << ", angle = " << angle << ", distance = " << distance << std::endl;

  return true;
}

opendlv::device::lidar::rplidar::DeviceInfo RPLidarDecoder::getDeviceInfo(const uint8_t *buffer, const size_t offset, const size_t sizeOfMessage) noexcept {
  opendlv::device::lidar::rplidar::DeviceInfo data;
  if (20 == sizeOfMessage) {
    uint8_t model{buffer[offset]};
    uint8_t firmware_minor{buffer[offset + 1]};
    uint8_t firmware_major{buffer[offset + 2]};
    uint8_t hardware{buffer[offset + 3]};

    data.model(model)
        .firmware_minor(firmware_minor)
        .firmware_major(firmware_major)
        .hardware(hardware);

    union {
      uint32_t number{0};
      uint8_t array[4];
    } serialNumber;

    for (uint8_t i{0}; i < 16; i+=4) {
      serialNumber.number = 0;
      serialNumber.array[(i+0)%4] = buffer[offset + 4 + (i+0)];
      serialNumber.array[(i+1)%4] = buffer[offset + 4 + (i+1)];
      serialNumber.array[(i+2)%4] = buffer[offset + 4 + (i+2)];
      serialNumber.array[(i+3)%4] = buffer[offset + 4 + (i+3)];
      serialNumber.number = be32toh(serialNumber.number);

      if (0 == i) {
        data.serialNumber0(serialNumber.number);
      }
      else if (4 == i) {
        data.serialNumber1(serialNumber.number);
      }
      else if (8 == i) {
        data.serialNumber2(serialNumber.number);
      }
      else if (12 == i) {
        data.serialNumber3(serialNumber.number);
      }
    }
  }
  return data;
}

opendlv::device::lidar::rplidar::DeviceHealth RPLidarDecoder::getDeviceHealth(const uint8_t *buffer, const size_t offset, const size_t sizeOfMessage) noexcept {
  opendlv::device::lidar::rplidar::DeviceHealth data;
  if (3 == sizeOfMessage) {
    uint8_t status{buffer[offset]};

    data.status(status);

    union {
      uint16_t number{0};
      uint8_t array[2];
    } error_code;

    error_code.array[0] = buffer[offset + 1 + 0];
    error_code.array[1] = buffer[offset + 1 + 1];
    error_code.number = be16toh(error_code.number);

    data.error_code(error_code.number);
  }
  return data;
}

