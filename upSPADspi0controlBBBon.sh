#!/bin/bash

# Execute on a BeagleBone Black with no overlays loaded.
# These commands require the default pin configuration in order to work.
# Geiger signal control
cd /sys/class/pwm/pwmchip7/pwm-7\:0
sudo config-pin p8_19 pwm
sudo sh -c "echo '1000' >> ./period"
sudo sh -c "echo '10' >> ./duty_cycle"
sudo sh -c "echo '1' >> ./enable"
# Re-confirm order
sudo sh -c "echo '1' >> ./enable"

# DC bias control 
cd /home/debian/Scripts/TelecomSPADcontrol
# For SPI1, /dev/spidev1.#
#
# sudo config-pin p9_17 spi_cs
# sudo config-pin p9_18 spi
# sudo config-pin p9_21 spi
# sudo config-pin p9_22 spi_sclk
 
# For SPI0, /dev/spidev2.#
#
sudo config-pin p9_28 spi_cs
sudo config-pin p9_29 spi
sudo config-pin p9_30 spi
sudo config-pin p9_31 spi_sclk

python3 ./pythonSPADspiControlBBBupON.py


