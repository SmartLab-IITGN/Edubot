#!/bin/bash
  
echo "remap the Arduino and RPLIDAR Connections"
echo "rplidar usb connection as /dev/rplidar , check it using the command : ls -l /dev|grep ttyUSB"
echo "Arduino usb connection as /dev/arduino_mega , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy edubot.rules to  /etc/udev/rules.d/"
echo "`rospack find edubot_base`/scripts/edubot.rules"
sudo cp `rospack find edubot_base`/scripts/edubot.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "