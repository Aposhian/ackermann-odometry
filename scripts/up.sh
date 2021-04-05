#!/usr/bin/env sh
docker-compose -f docker-compose.yml -f ./private/docker-compose.override.yml "$@" up
