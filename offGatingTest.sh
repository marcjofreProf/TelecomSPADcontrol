# Script to disable gating directly

cd /sys/class/pwm/pwmchip7/pwm-7\:0
sudo config-pin P8_19 pwm
# Always at least set a period and duty cycle so that the pwm can be disabled
sudo sh -c "echo '1000000' >> ./period"
sudo sh -c "echo '500000' >> ./duty_cycle"
sudo sh -c "echo '0' >> ./enable"
# Re-confirm order
sudo sh -c "echo '0' >> ./enable"
