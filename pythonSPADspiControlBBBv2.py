import spidev

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

# Perform SPI transfer
tx_data = [0xAF]  # Data to send
rx_data = spi.xfer(tx_data)  # Send and receive data

# Print received data
#print("Sent data:", tx_data)
#print("Received data:", rx_data)

# Close SPI
spi.close()
