#!/bin/sh
sudo echo '1-1' |sudo tee /sys/bus/usb/drivers/usb/bind
sudo /opt/vc/bin/tvservice -p
