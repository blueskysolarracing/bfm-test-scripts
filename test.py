from adc78h89 import *
from mcp4161 import *
from sn74hcs137 import *
from periphery import SPI, GPIO, PWM
import can
import time
from time import sleep

print("Setting up drivers..")

'''
bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000)

while True:
    msg = can.Message(
        arbitration_id=0xC0FFEE,
        data=[0, 25, 0, 1, 3, 1, 4, 1],
        is_extended_id=True
    )
    try:
        bus.send(msg)
        print(f"Message sent on {bus.channel_info}")
    except can.CanError:
        print("Message NOT sent")

'''

spi = SPI('/dev/spidev1.0', 3, 1e6)
pot = MCP4161(spi)
# Acel_Cs
accel = GPIO('/dev/gpiochip2', 26, 'out')
# Regen_CS
regen = GPIO('/dev/gpiochip5', 28, 'out')

gpios = [
        GPIO('/dev/gpiochip4', 2, 'out'),
        GPIO('/dev/gpiochip3', 26, 'out'),
        GPIO('/dev/gpiochip3', 28, 'out'),
        #GPIO('/dev/gpiochip0', 11, 'out'),
        #GPIO('/dev/gpiochip3', 22, 'out'),
        #GPIO('/dev/gpiochip0', 10, 'out'),
        #GPIO('/dev/gpiochip4', 17, 'out'),

        #GPIO('/dev/gpiochip3', 25, 'out'),
        #GPIO('/dev/gpiochip3', 24, 'out'),
        #GPIO('/dev/gpiochip3', 23, 'out'),

        # Horn
        #GPIO('/dev/gpiochip3', 5, 'out'),
        #GPIO('/dev/gpiochip2', 30, 'out'),
        #GPIO('/dev/gpiochip2', 31, 'out'),

        # Misc Module
        #GPIO('/dev/gpiochip0', 16, 'out'),
        #GPIO('/dev/gpiochip0', 22, 'out'),
        #GPIO('/dev/gpiochip0', 23, 'out'),
        #GPIO('/dev/gpiochip0', 19, 'out'),
        #GPIO('/dev/gpiochip6', 15, 'out'),
        #GPIO('/dev/gpiochip2', 7, 'out'),
        #GPIO('/dev/gpiochip4', 16, 'out'),

        # Relay Module
        #GPIO('/dev/gpiochip6', 18, 'out'),
        #GPIO('/dev/gpiochip6', 14, 'out'),
        #GPIO('/dev/gpiochip6', 13, 'out'),
        #GPIO('/dev/gpiochip6', 21, 'out'),
        #GPIO('/dev/gpiochip6', 20, 'out'),
        #GPIO('/dev/gpiochip6', 19, 'out'),
        #GPIO('/dev/gpiochip4', 2, 'out'),

        #GPIO('/dev/gpiochip4', 18, 'out'),
        #GPIO('/dev/gpiochip1', 4, 'out'),
        #GPIO('/dev/gpiochip1', 22, 'out'),
        #GPIO('/dev/gpiochip4', 22, 'out'),

        # Toradex Module
        #GPIO('/dev/gpiochip5', 25, 'out'),
        #GPIO('/dev/gpiochip4', 0, 'out')
]

regen.write(False)

while(1):

    for gpio in gpios:
        print("ON")
        gpio.write(True)

    #accel.write(False)
        time.sleep(5)

        print("OFF")
        gpio.write(False)

    #data_out = [0xaa, 0xbb, 0xcc, 0xd]
    #data_in = spi.transfer(data_out)

    #pot.set_step(200)
    #accel.write(True)
    #print("all on")

    #for gpio in gpios:
        #gpio.write(False)

    #accel.write(False)
    #time.sleep(2)
    #pot.set_step(100)
    #accel.write(True)
    #print("all off")

'''
spi = SPI('/dev/spidev0.0', 3, 1e6)
adc = ADC78H89(spi)
# Pin 13 on Ixora X27 header is GPIO 1
gpio1 = GPIO('/dev/gpiochip0', 8, 'out')
gpio2 = GPIO('/dev/gpiochip0', 7, 'out')
gpio3 = GPIO('/dev/gpiochip0', 6, 'out')
gpio4 = GPIO('/dev/gpiochip0', 5, 'out')

pwm = PWM(0, 0)

print("Periodically printing")

while True:
    gpio4.write(True)
    gpio3.write(True)
    gpio2.write(True)
    gpio1.write(False)
    print(adc.sample_all())
    gpio1.write(True)
    gpio4.write(False)
    gpio3.write(False)
    gpio2.write(False)
    time.sleep(1)
'''
