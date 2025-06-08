# SPDX-License-Identifier: MIT
# PCA9546 I2C Scanner rewritten in Python with periphery and custom NAU7802 driver

import time
from periphery import I2C

# Constants
PCAADDR = 0x70  # I2C mux address
NAU7802_I2C_ADDR = 0x2A  # Default address of NAU7802

# NAU7802 Register Addresses
NAU7802_REG_CTRL2 = 0x01
NAU7802_REG_ADC = 0x12

# Custom NAU7802 class
class NAU7802:
    def __init__(self, i2c_dev, address=NAU7802_I2C_ADDR):
        self.i2c_dev = i2c_dev
        self.address = address

    def read_register(self, reg):
        msg = [I2C.Message([reg], read=False), I2C.Message([0x00], read=True)]
        self.i2c_dev.transfer(self.address, msg)
        return msg[1].data[0]

    def write_register(self, reg, value):
        msg = [I2C.Message([reg, value], read=False)]
        self.i2c_dev.transfer(self.address, msg)

    def available(self):
        # Check if data is ready in the ADC register
        adc_val = self.read_register(NAU7802_REG_ADC)
        return adc_val != 0

    def read_adc(self):
        # Read 24-bit ADC value
        msg = [I2C.Message([NAU7802_REG_ADC], read=False), I2C.Message([0x00, 0x00, 0x00], read=True)]
        self.i2c_dev.transfer(self.address, msg)
        adc_bytes = msg[1].data
        value = (adc_bytes[0] << 16) | (adc_bytes[1] << 8) | adc_bytes[2]
        if value & 0x800000:  # Handle sign extension for 24-bit values
            value -= 1 << 24
        return value

# I2C setup
i2c_dev = I2C("/dev/i2c-3")
nau1 = NAU7802(i2c_dev)
nau2 = NAU7802(i2c_dev)

def nauselect(nau, channel):
    ctrl2 = nau.read_register(NAU7802_REG_CTRL2)
    ctrl2 &= 0b01111111
    ctrl2 |= (channel << 7)
    nau.write_register(NAU7802_REG_CTRL2, ctrl2)
    print(f"NAU select channel: {channel}")

def pcaselect(channel):
    if channel > 3:
        return
    data = [1 << channel]
    i2c_dev.transfer(PCAADDR, [I2C.Message(data)])
    print(f"PCA select port: {channel}")

def scan_i2c_bus():
    print("Scanning I2C bus...")
    for addr in range(1, 127):
        if addr == PCAADDR:
            continue
        try:
            msg = I2C.Message([], read=False)
            i2c_dev.transfer(addr, [msg])
            print(f"Found I2C device at address 0x{addr:02X}")
        except IOError:
            pass

def initialize_nau(nau, id):
    print(f"Initializing NAU7802: {id}")
    nauselect(nau, 0)
    print(f"NAU{id} initialized on channel 0")

def read_nau(nau, channel, id):
    nauselect(nau, channel)
    if nau.available():
        val = nau.read_adc()
        print(f"Read NAU{id}, channel {channel}: {val}")
    else:
        print(f"No data available for NAU{id}, channel {channel}")

def setup():
    print("Starting setup...")
    for port in range(4):
        pcaselect(port)
        print(f"Scanning PCA port {port}")
        scan_i2c_bus()

    pcaselect(0)
    initialize_nau(nau1, "1")

    pcaselect(1)
    initialize_nau(nau2, "2")

    print("Setup complete.")

def loop():
    while True:
        time.sleep(0.5)
        pcaselect(0)
        read_nau(nau1, 0, "1")

        time.sleep(0.5)
        read_nau(nau1, 1, "1")

        time.sleep(0.5)
        pcaselect(1)
        read_nau(nau2, 0, "2")

if __name__ == "__main__":
    setup()
    loop()

