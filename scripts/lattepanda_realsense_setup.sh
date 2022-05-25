#!/usr/bin/bash
set -e

wget https://github.com/IntelRealSense/librealsense/raw/master/config/99-realsense-libusb.rules

sed -i 's|, GROUP:="plugdev"||g' 99-realsense-libusb.rules
sed -i 's|, GROUP="plugdev"||g' 99-realsense-libusb.rules

sudo install -Dm 644 99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules
rm 99-realsense-libusb.rules

read -p 'Refresh udev rules? Make sure that any Realsense camera is not attached (Y/n): ' choice
if [ "$choice" != "n" ]; then
    sudo udevadm control --reload-rules && sudo udevadm trigger
fi

# alternative to the install command
#wget https://github.com/IntelRealSense/librealsense/raw/master/scripts/setup_udev_rules.sh
#sed -i 's,cp config/99,mv 99,' setup_udev_rules.sh
#sudo bash ./setup_udev_rules.sh
#rm setup_udev_rules.sh
