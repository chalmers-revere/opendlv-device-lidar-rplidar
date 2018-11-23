#!/bin/sh

VERSION=$1

cat <<EOF >/tmp/multi.yml
image: chalmersrevere/opendlv-device-lidar-rplidar-multi:$VERSION
manifests:	
  - image: chalmersrevere/opendlv-device-lidar-rplidar-amd64:$VERSION
    platform:
      architecture: amd64
      os: linux
  - image: chalmersrevere/opendlv-device-lidar-rplidar-armhf:$VERSION
    platform:
      architecture: arm
      os: linux
  - image: chalmersrevere/opendlv-device-lidar-rplidar-aarch64:$VERSION
    platform:
      architecture: arm64
      os: linux
EOF
manifest-tool-linux-amd64 push from-spec /tmp/multi.yml
