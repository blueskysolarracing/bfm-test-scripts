from adc78h89 import *
from periphery import SPI, GPIO, PWM
import time
from time import sleep

class Counter:
    NUM_SAMPLES = 2
    cbuf = [0]*NUM_SAMPLES
    avg = 0
    prev_val = False
    time_since_last_hit = 0

    def __init__(self):
        # Initialize SPI and ADC
        # self.spi = SPI('/dev/spidev0.0', 3, 1e6)
        # self.adc = ADC78H89(self.spi)

        # Initialize GPIO
        #gpio1 = GPIO('/dev/gpiochip4', 24, 'in') # B-PIN BROKEN
        self.gpio1 = GPIO('/dev/gpiochip4', 25, 'in') # A-PIN OK

        self.time_since_last_hit = time.time()


    def push_cbuf(self, val):
        self.avg -= self.cbuf[0]
        for i in range(self.NUM_SAMPLES - 1):
            self.cbuf[i] = self.cbuf[i + 1]

        self.cbuf[self.NUM_SAMPLES - 1] = val/self.NUM_SAMPLES
        self.avg += self.cbuf[self.NUM_SAMPLES - 1]

    def hit(self):
        time_delta = time.time() - self.time_since_last_hit
        self.time_since_last_hit = time.time()
        self.push_cbuf(60 / time_delta) # convert sec/rot -> rot/min

    def run(self):
        cur_val = self.gpio1.read()
        if cur_val and cur_val != self.prev_val:
            self.hit()

        self.prev_val = cur_val  
