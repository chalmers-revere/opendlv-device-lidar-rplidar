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

#include "cluon-complete.hpp"

#include "rplidar.hpp"

#include <cstdint>
#include <iostream>

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ( (0 == commandlineArguments.count("cid")) ||
       (0 == commandlineArguments.count("device")) ) {
    std::cerr << argv[0] << " connects to an RPlidar device to provide opendlv.proxy.PointCloudReading messages." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --device=<serial port to open> [--id=<ID>] [--verbose]" << std::endl;
    std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
    std::cerr << "         --device: serial port where the RPlidar is attached to" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --device=/dev/ttyUSB0 --verbose" << std::endl;
  }
  else {
    const std::string DEVICE{commandlineArguments["device"]};
//    const uint32_t ID{static_cast<uint32_t>((commandlineArguments.count("id") != 0) ? std::stoi(commandlineArguments["id"]) : 0)};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};

    RPLidar rplidar(DEVICE, VERBOSE);
    if (rplidar.isOpen()) {
      rplidar.startScanning();

      // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
      cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

      // Endless loop; end the program by pressing Ctrl-C.
      while (od4.isRunning()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
    retCode = 0;
  }
  return retCode;
}

