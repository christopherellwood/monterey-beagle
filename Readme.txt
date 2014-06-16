cp bone_six_pwm-00A0.dtbo /lib/firmware
cp DM-GPIO-Test.dtbo /lib/firmware

$ mount /dev/mmcblk0p1 /mnt/card
$ vi uEnv.txt

Make it say the following:

optargs=quiet capemgr.disable_partno=BB-BONELT-HDMI,BB-BONELT-HDMIN capemgr.enable_partno=am33xx_pwm,bone_six_pwm,DM-GPIO-Test,BB-SPIDEV1,BB-I2C1 drm.debug=7


[ESC]:wq[ENTER]

$ umount /mnt/card
