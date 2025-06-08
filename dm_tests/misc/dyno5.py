#import threading
#from adc78h89 import *
#import logging
#import datetime
#from periphery import Serial, GPIO, SPI
#from time import sleep
#from motor_test import M2096
#from unittest.mock import MagicMock
#from abc import ABC, abstractmethod
#from dataclasses import dataclass, field
#from enum import Enum, StrEnum
#from typing import ClassVar
#from warnings import warn

#from SpeedCounter import Counter
#from unittest.mock import MagicMock
#from time import sleep
#from periphery import GPIO, SPI

import ina_class as ina

'''
Notes:
    Do we need to handle case when regen and acceleration are set?
'''

def write_psm():
    #while True:
    current = ina.read_current()
    voltage = ina.read_voltage()
    out = f"motor_psm_current {current}\n"
    #serial.write(out.encode())
    print(out)
    out = f"motor_psm_voltage {voltage}\n"
    #serial.write(out.encode())
    print(out)
    #sleep(psm_period)

if __name__ == "__main__":
    #dyno_state = DynoState()
    #logging.basicConfig(
    #    format='%(asctime)s %(message)s',
    #    handlers=[
    #        logging.FileHandler(f"{get_datetime()}.log"),
    #        logging.StreamHandler()
    #    ]
    #)
    #logger = logging.getLogger()
    #logger.setLevel(logging.DEBUG)

    #hall_effect_period = 0.1
    psm_period = 1

    #serial = Serial("/dev/ttyLP2", 115200)
    #adc = ADC78H89(SPI('/dev/spidev1.0', 3, 1e6)) 

    #ls = GPIO('/dev/gpiochip4', 2, 'out')
    #hs = GPIO('/dev/gpiochip3', 26, 'out')
    #pc = GPIO('/dev/gpiochip3', 28, 'out')
    #null_gpio = MagicMock()
    #cs_expander_gpio = GPIO('/dev/gpiochip3', 5, 'out')
    #cs_display_gpio = GPIO('/dev/gpiochip3', 6, 'out')
    #vfm_rst = GPIO('/dev/gpiochip6', 17, 'out')
    #cs_display_gpio.write(True)

    '''
    mc = M2096(
        SPI('/dev/spidev2.0', 3, 1e6),
        GPIO('/dev/gpiochip5', 29, 'out'),
        GPIO('/dev/gpiochip5', 28, 'out'),
        GPIO('/dev/gpiochip6', 14, 'out', inverted=True),
        GPIO('/dev/gpiochip6', 13, 'out'),
        GPIO('/dev/gpiochip6', 11, 'out'),
        GPIO('/dev/gpiochip6', 12, 'out'),
        GPIO('/dev/gpiochip6', 10, 'out'),
        MagicMock(),
    )
    '''

    while True:
        #read()
        #write_hall_effect()
        write_psm()
        #hall_effect()

    #t1 = threading.Thread(target=read, name='t1')
    #t2 = threading.Thread(target=write_hall_effect, name='t2')
    #t3 = threading.Thread(target=write_psm, name='t3')
    #t4 = threading.Thread(target=hall_effect, name='t4')

    #try: 
    #    t1.start()
    #    t2.start()
    #    t3.start()
    #    t4.start()
    #except KeyboardInterrupt:
       # t1.join()
       # t2.join()
       # t3.join()
       # t4.join()

       # serial.close()






