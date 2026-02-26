#!/bin/bash
# Script to turn off DC bias, and gating if previously enabled

# To run the script
# debian@beaglebone:~/Scripts/TelecomSPADcontrol/ManualControl$ ./downSPADspi0controlBBBoff.sh 

# Geiger signal control disable
cd /sys/class/pwm/pwmchip7/pwm-7\:0
sudo config-pin P8_19 pwm
# Always at least set a period and duty cycle so that the pwm can be disabled
sudo sh -c "echo '2560' >> ./period"
sudo sh -c "echo '120' >> ./duty_cycle"
sudo sh -c "echo '0' >> ./enable"
# Re-confirm order
sudo sh -c "echo '0' >> ./enable"

# DC bias control disable
cd /home/debian/Scripts/TelecomSPADcontrol/ManualControl
python3 ./pythonSPADspiControlBBBdownOFF.py

echo "DC bias voltage ramp down done"

