#!/bin/bash

# Required for graceful shutdown, because it expects a CTRL+C from interactive terminal
docker exec ros2-apparatus-recording pkill -SIGINT -f hector_recorder
docker-compose down