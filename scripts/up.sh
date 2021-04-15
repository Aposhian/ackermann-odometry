#!/usr/bin/env sh
xhost +
docker-compose -f docker-compose.yml -f ./private/docker-compose.override.yml "$@" up
