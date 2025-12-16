#! /bin/bash

STEAMOS=0
STEAMOS_READONLY=0

# Test for SteamOS and disable readonly mode if we're running on it
if command -v steamos-readonly >& /dev/null
then
	# Test if SteamOS readonly mode is enabled
	if sudo steamos-readonly status | grep 'enabled'
	then
		STEAMOS_READONLY=1
	fi

	echo "Disabling steamOS readonly mode"
	STEAMOS=1
	sudo steamos-readonly disable
fi

#Create config directory for the rules
mkdir config

#Download udev rules from the official librealsense github
echo "Downloading udev rules files from librealsense github"
wget -q https://raw.githubusercontent.com/realsenseai/librealsense/refs/heads/master/config/99-realsense-libusb.rules -O config/99-realsense-libusb.rules
wget -q https://raw.githubusercontent.com/realsenseai/librealsense/refs/heads/master/config/99-realsense-d4xx-mipi-dfu.rules -O config/99-realsense-d4xx-mipi-dfu.rules

# Copy files to udev rules directory (/etc/udev/rules.d/)
echo "Installing udev rules on /etc/udev/rules.d/"
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo cp config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/

#Delete config folder
rm -r config/

# Reload the rules
echo "Reloading udevadm"
sudo udevadm control --reload-rules
sudo udevadm trigger

#Enable again the steamOS readonly mode
if [ "$STEAMOS" = 1 ] ; then
	if [ "$STEAMOS_READONLY" = 1 ] ; then
		echo "Enabling steamos readonly mode"
		sudo steamos-readonly enable
	fi
fi
