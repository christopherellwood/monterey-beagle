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


To setup beaglebone

cp bone_six_pwm-00A0.dtbo /lib/firmware
cp DM-GPIO-Test.dtbo /lib/firmware

$ mount /dev/mmcblk0p1 /mnt/card
$ vi uEnv.txt

Make it say the following:

optargs=quiet capemgr.disable_partno=BB-BONELT-HDMI,BB-BONELT-HDMIN capemgr.enable_partno=am33xx_pwm,bone_six_pwm,DM-GPIO-Test,BB-SPIDEV1,BB-I2C1 drm.debug=7


[ESC]:wq[ENTER]

$ umount /mnt/card

To run the program, load PWM_Pins.cfg and Disc_Pins.cfg into the same folder as the the monterey-beagle executable and ./monterey-beagle.