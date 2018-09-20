#!/bin/sh

VERSION=$1
PROJECTNAME=${PWD##*/}
ORGANISATION=openkorp

docker build -t $ORGANISATION/$PROJECTNAME-armhf:$VERSION -f Dockerfile.armhf . &
docker build -t $ORGANISATION/$PROJECTNAME-aarch64:$VERSION -f Dockerfile.aarch64 . &
docker build -t $ORGANISATION/$PROJECTNAME-amd64:$VERSION -f Dockerfile.amd64 . &

wait

docker push $ORGANISATION/$PROJECTNAME-armhf:$VERSION &
docker push $ORGANISATION/$PROJECTNAME-aarch64:$VERSION & 
docker push $ORGANISATION/$PROJECTNAME-amd64:$VERSION  &

wait 

cat <<EOF >/tmp/multi.yml
image: $ORGANISATION/$PROJECTNAME-multi:$VERSION
manifests:  
  - image: $ORGANISATION/$PROJECTNAME-amd64:$VERSION
    platform:
      architecture: amd64
      os: linux
  - image: $ORGANISATION/$PROJECTNAME-armhf:$VERSION
    platform:
      architecture: arm
      os: linux
  - image: $ORGANISATION/$PROJECTNAME-aarch64:$VERSION
    platform:
      architecture: arm64
      os: linux
EOF
manifest-tool-linux-amd64 push from-spec /tmp/multi.yml