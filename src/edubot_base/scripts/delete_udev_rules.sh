#!/bin/bash

echo "delete remap of Serial ports to rplidar & arduino_mega"
echo "sudo rm   /etc/udev/rules.d/edubot.rules"
sudo rm   /etc/udev/rules.d/edubot.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
