#!/bin/bash

# Define the SPI device (SPI bus 1, Chip Select 0)
SPI_DEV="/dev/spidev1.0"

# SPI mode, bits per word, and speed (in Hz)
MODE=0
BITS=8
SPEED=500000  # 500kHz

# Data to send (hex bytes)
DATA_TO_SEND="\xAA\x55\xFF\x00"

# Set SPI mode, bits, and speed
echo $MODE > /sys/class/spidev/spidev1.0/mode
echo $BITS > /sys/class/spidev/spidev1.0/bits_per_word
echo $SPEED > /sys/class/spidev/spidev1.0/max_speed_hz

# Send and receive data using the spidev_test utility
echo -n -e $DATA_TO_SEND | spidev_test -D $SPI_DEV -s $SPEED -b $BITS -v
