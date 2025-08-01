# Script to modulate and LED for testing purposes with the PWM
cd /sys/class/pwm/pwmchip4/pwm-4\:0
sudo config-pin P9_14 pwm
sudo sh -c "echo '1000000' >> ./period"
sudo sh -c "echo '100000' >> ./duty_cycle"
sudo sh -c "echo '1' >> ./enable"
