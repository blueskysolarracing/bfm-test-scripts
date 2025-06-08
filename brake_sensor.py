from time import sleep
from periphery import GPIO

sensor = GPIO('/dev/gpiochip6', 20, 'in')

while(True):
    value = sensor.read()
    print(value)
    
