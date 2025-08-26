# Script to modulate a Telecom LD for testing purposes with the PWM
cd /sys/class/pwm/pwmchip4/pwm-4\:0
sudo config-pin P9_14 pwm
sudo sh -c "echo '100' >> ./period"
sudo sh -c "echo '50' >> ./duty_cycle"
sudo sh -c "echo '1' >> ./enable"
# Re-confirm order
sudo sh -c "echo '1' >> ./enable"
