from iclib.ltc6810 import LTC6810
from periphery import SPI, GPIO
import time
import random
from time import sleep

spi = SPI('/dev/spidev2.0', 3, 3.25e6)
bms = LTC6810(spi)
cs = GPIO('/dev/gpiochip0', 27, 'out')
cs.write(True)

while True:
    for i in range(16):

        cs.write(False)
        sid = bms.RDSID(i)
        cs.write(True)
        print(sid)

        sleep(0.001)
        cs.write(False)
        bms.ADCV(LTC6810.CHMode.M14000, False, 0b000, i)
        cs.write(True)
        sleep(0.001)
        cs.write(False)
        tvA = bms.RDCVA(i)
        cs.write(True)
        sleep(0.001)
        cs.write(False)
        tvB = bms.RDCVB(i)
        cs.write(True)
        sleep(0.001)

        print(f"{random.randrange(1,10)}: {tvA.C1V}, {tvA.C2V}, {tvA.C3V}, {tvB.C4V}, {tvB.C5V}, {tvB.C6V}")
    time.sleep(0.5)
