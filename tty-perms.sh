#!/bin/bash

# Since running script as root, we can get away with preparing serial device permissions for containers to use them?
chmod 777 /dev/ttyUSB0
chmod 777 /dev/ttyACM1