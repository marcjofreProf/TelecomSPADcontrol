# Script to turn off the modulated Telecom LD on pin P9.14
cd /sys/class/pwm/pwmchip4/pwm-4\:0
sudo sh -c "echo '0' >> ./enable"
# Re- confirm order
sudo sh -c "echo '0' >> ./enable"
