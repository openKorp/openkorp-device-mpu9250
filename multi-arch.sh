#!/bin/sh

VERSION=$1

docker build -t openkorp/openkorp-device-mpu2950-aarch64:$VERSION -f Dockerfile.aarch64 . &
docker build -t openkorp/openkorp-device-mpu2950-amd64:$VERSION -f Dockerfile.amd64 . &
docker build -t openkorp/openkorp-device-mpu2950-armhf:$VERSION -f Dockerfile.armhf .

docker push openkorp/openkorp-device-mpu2950-aarch64:$VERSION 
docker push openkorp/openkorp-device-mpu2950-amd64:$VERSION 
docker push openkorp/openkorp-device-mpu2950-armhf:$VERSION 

cat <<EOF >/tmp/multi.yml
image: openkorp/openkorp-device-mpu2950-multi:$VERSION
manifests:  
  - image: openkorp/openkorp-device-mpu2950-amd64:$VERSION
    platform:
      architecture: amd64
      os: linux
  - image: openkorp/openkorp-device-mpu2950-armhf:$VERSION
    platform:
      architecture: arm
      os: linux
  - image: openkorp/openkorp-device-mpu2950-aarch64:$VERSION
    platform:
      architecture: arm64
      os: linux
EOF
manifest-tool-linux-amd64 push from-spec /tmp/multi.yml