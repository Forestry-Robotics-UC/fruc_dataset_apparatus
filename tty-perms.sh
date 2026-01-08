#!/bin/bash

# Since running script as root, we can get away with preparing serial device permissions for containers to use them?
echo "Setting permissions for /dev/ttyUSB0"
chmod 777 /dev/ttyUSB0
echo "Setting permissions for /dev/ttyACM1"
chmod 777 /dev/ttyACM1