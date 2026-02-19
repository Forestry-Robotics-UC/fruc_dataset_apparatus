#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$SCRIPT_DIR/docker"

# Required for graceful shutdown, because it expects a CTRL+C from interactive terminal
podman exec $(podman ps -q -f name=recording) pkill -SIGINT -f hector_recorder
podman-compose down