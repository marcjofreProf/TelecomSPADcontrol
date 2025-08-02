#!/usr/bin/env python3

# Simple Script to set a secific voltage value 
import spidev, time, tqdm
import numpy as np

# Initialize SPI
spi = spidev.SpiDev()

# Open SPI bus (SPI1, CS0)
spi.open(1, 0)  # Use (0, 0) for SPI0

# Set SPI mode (CPOL and CPHA)
# Mode 0: CPOL=0, CPHA=0
# Mode 1: CPOL=0, CPHA=1
# Mode 2: CPOL=1, CPHA=0
# Mode 3: CPOL=1, CPHA=1
spi.mode = 0  # Change this to 0, 1, 2, or 3 as needed

# Set max speed (in Hz)
spi.max_speed_hz = 5000  # 500 Hz

# Set bits per word (default is 8)
spi.bits_per_word = 8

# Set the clock polarity (CPOL) and clock phase (CPHA) explicitly
# This is equivalent to setting the mode, but you can also do it individually
#spi.cpol = 0  # Clock polarity: 0 (idle low) or 1 (idle high)
#spi.cpha = 0  # Clock phase: 0 (sample on first edge) or 1 (sample on second edge)

# Set the SPI loopback mode (for testing)
spi.loop = False  # Set to True for loopback testing

# Set the SPI three-wire mode (half-duplex)
spi.threewire = False  # Set to True for three-wire mode


# Initial starting voltage set, Perform SPI transfer
tx_data = [0xFF]  # Data to send
rx_data = spi.xfer(tx_data)  # Send and receive data
time.sleep(1) # Sleep for 1 second


# Compute operational values
MAX_VOLTAGE_RATE = 2.0  # Maximum voltage ramp rate 2 V/s.

DESIRED_VOLTAGE = 85  #Notice that this value is approximated
MAX_VALUE = 88.7
MIN_VALUE = 39.5
RATIO = (MAX_VALUE - MIN_VALUE) / 255
INT_VALUE = int((DESIRED_VOLTAGE - MIN_VALUE) / RATIO)
INVERTED_RATIO = 256 - INT_VALUE

MIN_VOLTAGE_VALUE = float(0xFF)  # Lowest DAC value (assuming 8-bit DAC control)
MAX_VOLTAGE_VALUE = float(INVERTED_RATIO)  # Maximum DAC value

# Compute voltage step size
step_range_conversion_byte=(90.0-40.0)/255.0 #around 0.24 V per bit. sweepeable voltage range with MAX1932 evaluation kit
voltage_range = abs(MAX_VOLTAGE_VALUE - MIN_VOLTAGE_VALUE)
decimation_step_factor=10.0
voltage_step_count=int(np.round(voltage_range/step_range_conversion_byte)/decimation_step_factor)

step_time = (step_range_conversion_byte/MAX_VOLTAGE_RATE)*decimation_step_factor  # Time delay per step to maintain ramp rate
values_apply=np.linspace(int(np.round(MIN_VOLTAGE_VALUE)),int(np.round(MAX_VOLTAGE_VALUE)),voltage_step_count,dtype=int)

# Convert the voltage values to np.uint8 (clamped between 0 and 255)
#values_apply = np.clip(np.round(values_apply), 0, 255).astype(np.uint8)

# Checks
#print(values_apply)

# Function to send SPI data
def send_spi(specific_value):
    tx_data = [int(specific_value)] # Data to send
    rx_data = spi.xfer(tx_data) # Send and receive data
    # Print received data
    # print("sent data: ", tx_data)
    # print("Received data: ", rx_data)

# Ramp voltage up from MIN_VALUE to MAX_VALUE
for value in tqdm.tqdm(range(0,len(values_apply),1)):
    #clear_output(wait=True)
    send_spi(values_apply[value])
    time.sleep(step_time)  # Control ramp speed

# Close SPI
spi.close()

print("Telecom SPAD voltage ramp completed.")
