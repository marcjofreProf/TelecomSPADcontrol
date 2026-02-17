#!/bin/bash
# Script to turn off the modulated Telecom LD on pin P9.14
cd /sys/class/pwm/pwmchip4/pwm-4\:0
sudo config-pin P9_14 pwm
# Always at least set a period and duty cycle so that the pwm can be disabled
sudo sh -c "echo '50000' >> ./period"
sudo sh -c "echo '40000' >> ./duty_cycle"
sudo sh -c "echo '0' >> ./enable"
# Re- confirm order
sudo sh -c "echo '0' >> ./enable"
