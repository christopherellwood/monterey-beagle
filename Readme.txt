To setup compiler

Install Eclipse Kepler
Install Linaro arm-linux-gnueabi-2012.04
Install Cmake
Install Mingw
My Computer->properties->Advanced System Settings
C:\MinGW\bin;C:\MinGW\MSYS\1.0\local\bin;C:\MinGW\MSYS\1.0\bin;C:\Program Files (x86)\CMake 2.8\bin;C:\Program Files (x86)\Linaro\gcc-linaro-arm-linux-gnueabi-2012.04\bin

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
