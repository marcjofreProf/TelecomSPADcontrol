#!/bin/bash
# Script to turn off DC bias, and gating if previously enabled

# To run the script
# debian@beaglebone:~/Scripts/TelecomSPADcontrol$ ./downSPADspi0controlBBBoff.sh 

# Gating signal control disable
cd /sys/class/pwm/pwmchip7/pwm-7\:0
sudo config-pin P8_19 pwm
# Always at least set a period and duty cycle so that the pwm can be disabled
sudo sh -c "echo '50' >> ./period"
sudo sh -c "echo '5' >> ./duty_cycle"
sudo sh -c "echo '0' >> ./enable"
# Re-confirm order
sudo sh -c "echo '0' >> ./enable"

# DC diode Schottky bias
cd /sys/class/pwm/pwmchip1/pwm-1\:0
sudo config-pin P9_22 pwm
# Always at least set a period and duty cycle so that the pwm can be disabled
sudo sh -c "echo '1000000000' >> ./period"
sudo sh -c "echo '1000000000' >> ./duty_cycle"
sudo sh -c "echo '0' >> ./enable"
# Re-confirm order
sudo sh -c "echo '0' >> ./enable"

# DC bias control disable
cd /home/debian/Scripts/TelecomSPADcontrol 
python3 ./pythonSPADspiControlBBBdownOFF.py


