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

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"

#include "rplidar-decoder.hpp"

#include <vector>

const std::vector<uint8_t> RCV_INFO_BYTES {
  0xAB, 0xa5, 0x5a, 0x14, 0x0, 0x0, 0x0, 0x4, 0x0,
  0xf, 0x1, 0x0, 0xff, 0xef, 0xe7, 0xf2, 0xc1,
  0xe2, 0x9b, 0xf3, 0xc0, 0xe3, 0x9e, 0xf6, 0xe,
  0x3c, 0x49, 0x36
};

TEST_CASE("Test DeviceInfo.") {
  RPLidarDecoder decoder;
  decoder.decode(RCV_INFO_BYTES.data(), RCV_INFO_BYTES.size());
  {
    REQUIRE(0 == decoder.getDeviceInfo().model());
    REQUIRE(1 == decoder.getDeviceInfo().firmware_major());
    REQUIRE(15 == decoder.getDeviceInfo().firmware_minor());
    REQUIRE(0 == decoder.getDeviceInfo().hardware());
    REQUIRE(0xffefe7f2 == decoder.getDeviceInfo().serialNumber0());
  }
}

