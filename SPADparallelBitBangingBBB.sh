#!/bin/bash

# Define the GPIO pins to be used
GPIO_PINS=(30 31 32 33 34 35 36 37)

# Function to export GPIO pins
export_gpio() {
    for pin in "${GPIO_PINS[@]}"; do
        if [ ! -d /sys/class/gpio/gpio$pin ]; then
            echo $pin > /sys/class/gpio/export
        fi
        echo out > /sys/class/gpio/gpio$pin/direction
    done
}

# Function to unexport GPIO pins
unexport_gpio() {
    for pin in "${GPIO_PINS[@]}"; do
        if [ -d /sys/class/gpio/gpio$pin ]; then
            echo $pin > /sys/class/gpio/unexport
        fi
    done
}

# Function to write data to the GPIO pins
write_data() {
    local data=$1
    for i in "${!GPIO_PINS[@]}"; do
        local pin=${GPIO_PINS[$i]}
        local bit=$((data >> i & 1))
        echo $bit > /sys/class/gpio/gpio$pin/value
    done
}

# Main script
if [ $# -ne 1 ]; then
    echo "Usage: $0 <8-bit data>"
    exit 1
fi

DATA=$1

# Validate input
if [[ $DATA =~ ^[0-9]+$ ]] && [ $DATA -ge 0 ] && [ $DATA -le 255 ]; then
    export_gpio
    write_data $DATA
    echo "Data $DATA written to GPIO pins."
    unexport_gpio
else
    echo "Invalid input. Please provide an 8-bit number (0-255)."
    exit 1
fi
