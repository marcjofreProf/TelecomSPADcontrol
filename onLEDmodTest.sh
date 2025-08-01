# Script to modulate and LED for testing purposes with the PWM
cd /sys/class/pwm/pwmchip4/pwm-4\:0
sudo sh -c "echo '10000' >> ./period"
sudo sh -c "echo '1000' >> ./duty_cycle" (with respect the period)
sudo sh -c "echo '1' >> ./enable"
