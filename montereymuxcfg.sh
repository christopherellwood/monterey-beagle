#!/bin/sh
cat /sys/devices/bone_capemgr.*/slots
echo am33xx_pwm > /sys/devices/bone_capemgr.8/slots
echo bone_six_pwm > /sys/devices/bone_capemgr.8/slots
echo DM-GPIO-Test > /sys/devices/bone_capemgr.8/slots
echo BB-SPIDEV1 > /sys/devices/bone_capemgr.8/slots
cat /sys/devices/bone_capemgr.8/slots
