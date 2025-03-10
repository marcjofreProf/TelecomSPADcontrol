#!/bin/bash

# Execute on a BeagleBone Black with no overlays loaded.
# These commands require the default pin configuration in order to work.
 
# For SPI1, /dev/spidev1.#
#
sudo config-pin p9_17 spi_cs
sudo config-pin p9_18 spi
sudo config-pin p9_21 spi
sudo config-pin p9_22 spi_sclk
 
# For SPI0, /dev/spidev2.#
#
sudo config-pin p9_28 spi_cs
sudo config-pin p9_29 spi
sudo config-pin p9_30 spi
sudo config-pin p9_31 spi_sclk

python3 ./pythonSPADspiControlBBBupON.py


