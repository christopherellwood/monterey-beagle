Timer.c, Timer.h are from http://www.teuniz.net/Timer_code/
i2c-dev.h is from the Linux Foundation

To compile the beaglebone code in windows:

Install Eclipse Kepler
Install Linaro arm-linux-gnueabi-2012.04
Install Cmake
Install Mingw

Update path. Go to:
My Computer->properties->Advanced System Settings
Add this to the "path" box:
C:\MinGW\bin;C:\MinGW\MSYS\1.0\local\bin;C:\MinGW\MSYS\1.0\bin;C:\Program Files (x86)\CMake 2.8\bin;C:\Program Files (x86)\Linaro\gcc-linaro-arm-linux-gnueabi-2012.04\bin

This part is fraught with difficulty. If it doesn't work, you have to make sure you did the previous steps.
Run build.bat from command line

Load Eclipse and import project

To setup beaglebone:
Capemgr is needed, which means angstrom or debian+kernel 3.8
For debian, which ships on beaglebone rev C:
apt-get update
apt-cache search linux-image | grep 3.8.13-bone
apt-get install linux-image-3.8.13-boneX

Then copy the custom dtb overlays
cp bone_six_pwm-00A0.dtbo /lib/firmware
cp DM-GPIO-Test.dtbo /lib/firmware

Then edit the device tree load options
vi /boot/uEnv.txt

Uncomment:
cape_disable=capemgr.disable_partno=BB-BONELT-HDMI,BB-BONELT-HDMIN

Add line:
cape_enable=capemgr.enable_partno=am33xx_pwm,BB-SPIDEV1,BB-I2C1 drm.debug=7

Edit the custom cape loader workaround file
vi /etc/default/capemgr
CAPE=bone_six_pwm,DM-GPIO-Test

Reboot and run cat /sys/devices/bone_capemgr*/slots should read
 0: 54:PF---
 1: 55:PF---
 2: 56:PF---
 3: 57:PF---
 4: ff:P-O-L Bone-LT-eMMC-2G,00A0,Texas Instrument,BB-BONE-EMMC-2G
 5: ff:P-O-- Bone-Black-HDMI,00A0,Texas Instrument,BB-BONELT-HDMI
 6: ff:P-O-- Bone-Black-HDMIN,00A0,Texas Instrument,BB-BONELT-HDMIN
 7: ff:P-O-L Override Board Name,00A0,Override Manuf,am33xx_pwm
 8: ff:P-O-L Override Board Name,00A0,Override Manuf,BB-SPIDEV1
 9: ff:P-O-L Override Board Name,00A0,Override Manuf,BB-I2C1
10: ff:P-O-L Override Board Name,00A0,Override Manuf,bone_six_pwm
11: ff:P-O-L Override Board Name,00A0,Override Manuf,DM-GPIO-Test

To run the program, load PWM_Pins.cfg and Disc_Pins.cfg into the same folder as the the monterey-beagle executable and ./monterey-beagle.