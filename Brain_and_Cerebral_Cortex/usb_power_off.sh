#!/bin/sh
sudo echo '1-1' |sudo tee /sys/bus/usb/drivers/usb/unbind
sudo /opt/vc/bin/tvservice -o

