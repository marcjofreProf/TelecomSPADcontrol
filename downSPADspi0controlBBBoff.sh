#!/bin/bash
#Geiger signal control disable
cd /sys/class/pwm/pwmchip7/pwm-7\:0
sudo sh -c "echo '0' >> ./enable"

#DC bias control disable
cd /home/debian/Scripts/TelecomSPADcontrol 
python3 ./pythonSPADspiControlBBBdownOFF.py


