#!/bin/bash
# Script to turn off DC bias, and gating if previously enabled

# To run the script
# debian@beaglebone:~/Scripts/TelecomSPADcontrol$ ./downSPADspi0controlBBBoff.sh 

#Gating signal control disable
cd /sys/class/pwm/pwmchip7/pwm-7\:0
sudo sh -c "echo '0' >> ./enable"
# Re-confirm order
sudo sh -c "echo '0' >> ./enable"

#DC bias control disable
cd /home/debian/Scripts/TelecomSPADcontrol 
python3 ./pythonSPADspiControlBBBdownOFF.py


