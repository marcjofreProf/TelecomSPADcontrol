#!/bin/bash
# Scrip to directly gate the system
sudo config-pin P8_19 pwm

cd /sys/class/pwm/pwmchip7/pwm-7\:0
sudo sh -c "echo '1000000' >> ./period"
sudo sh -c "echo '500000' >> ./duty_cycle"
sudo sh -c "echo '1' >> ./enable"
# Re-confirm order
sudo sh -c "echo '1' >> ./enable"
echo "Gating ON"
