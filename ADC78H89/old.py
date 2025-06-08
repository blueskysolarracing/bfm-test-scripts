from adc78h89 import *
from periphery import SPI, GPIO 
from time import sleep
import json

spi = SPI('/dev/spidev1.0', 3, 1e6)
adc78h89 = ADC78H89(spi, 3.3)
cs = GPIO('/dev/gpiochip3', 10, 'out')

array_cs = GPIO('/dev/gpiochip5', 20, 'out')
motor_cs = GPIO('/dev/gpiochip5', 19, 'out')
battery_cs = GPIO('/dev/gpiochip5', 21, 'out')
psm_cs = GPIO('/dev/gpiochip4', 26, 'out')

array_cs.write(True)
battery_cs.write(True)
motor_cs.write(True)
psm_cs.write(True)

cs.write(True)  

print(adc.sample_all())
cs.write(False)
