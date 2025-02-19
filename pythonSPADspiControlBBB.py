import spidev

spi = spidev.SpiDev()
spi.open(1, 0)  # Bus 1, Device 0
spi.mode = 0    # Set SPI mode 0 (Idle LOW)
spi.bits_per_word = 8
spi.max_speed_hz = 50000

data = [0x11]  # Send 0xAA (10101010)

response = spi.xfer2(data)


#print("Received:", response)


spi.close()
