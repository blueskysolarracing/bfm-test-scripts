from dataclasses import dataclass
from enum import IntEnum
from logging import getLogger
from math import inf
from time import sleep, time
from typing import ClassVar
from warnings import warn
from unittest.mock import MagicMock

from periphery import GPIO, SPI
#from iclib.mcp23s17 import MCP23S17, Port, Register
from periphery import GPIO, SPI
from unittest.mock import MagicMock
from time import sleep



class Direction(IntEnum):
    FORWARD = 0
    BACKWARD = 1


@dataclass
class M2096:
    @staticmethod
    def __position_potentiometer(
            gpio,
            spi: SPI,
            position: float,
            eeprom: bool,
    ) -> None:
        if not 0 <= position <= 1:
            raise ValueError('position not between 0 and 1')

        raw_data = round(256 * position)

        if eeprom:
            raw_data |= 1 << 13

        data = [raw_data >> 8, raw_data & ((1 << 8) - 1)]

        gpio.write(False)
        spi.transfer(data)
        gpio.write(True)

    main_switch_timeout: ClassVar[float] = 5
    variable_field_magnet_switch_timeout: ClassVar[float] = 0.2
    revolution_timeout: ClassVar[float] = 5
    potentiometer_spi: SPI
    accel_potentiometer_cs: GPIO
    regen_potentiometer_cs: GPIO
    main_switch_gpio: GPIO
    forward_or_reverse_switch_gpio: GPIO
    power_or_economical_switch_gpio: GPIO
    variable_field_magnet_up_switch_gpio: GPIO
    variable_field_magnet_down_switch_gpio: GPIO
    revolution_gpio: GPIO

    def __post_init__(self) -> None:
        self.accelerate(0, True)
        self.accelerate(0)
        self.regenerate(0, True)
        self.regenerate(0)
        self.main_switch_gpio.write(False)
        self.forward_or_reverse_switch_gpio.write(False)
        self.power_or_economical_switch_gpio.write(False)
        self.variable_field_magnet_up_switch_gpio.write(False)
        self.variable_field_magnet_down_switch_gpio.write(False)

    @property
    def revolution_period(self) -> float:
        if not self.revolution_gpio.poll(self.revolution_timeout):
            return inf

        timestamp = time()

        if not self.revolution_gpio.poll(self.revolution_timeout):
            return inf

        return 2 * (time() - timestamp)

    @property
    def status(self) -> bool:
        return self.main_switch_gpio.read()

    def accelerate(self, acceleration: float, eeprom: bool = False) -> None:
        self.__position_potentiometer(
            self.accel_potentiometer_cs,
            self.potentiometer_spi,
            acceleration,
            eeprom,
        )

    def regenerate(self, regeneration: float, eeprom: bool = False) -> None:
        self.__position_potentiometer(
            self.regen_potentiometer_cs,
            self.potentiometer_spi,
            regeneration,
            eeprom,
        )

    def state(self, status: bool) -> None:
        wait = status and not self.status

        self.main_switch_gpio.write(status)

        if wait:
            sleep(self.main_switch_timeout)

    def direct(self, direction: Direction) -> None:
        self.forward_or_reverse_switch_gpio.write(bool(direction))

    def economize(self, mode: bool) -> None:
        self.power_or_economical_switch_gpio.write(mode)

    def variable_field_magnet_up(self) -> None:
        self.variable_field_magnet_up_switch_gpio.write(True)
        sleep(self.variable_field_magnet_switch_timeout)
        self.variable_field_magnet_up_switch_gpio.write(False)

    def variable_field_magnet_down(self) -> None:
        self.variable_field_magnet_down_switch_gpio.write(True)
        sleep(self.variable_field_magnet_switch_timeout)
        self.variable_field_magnet_down_switch_gpio.write(False)


if __name__ == "__main__":
    ls = GPIO('/dev/gpiochip4', 2, 'out')
    hs = GPIO('/dev/gpiochip3', 26, 'out')
    pc = GPIO('/dev/gpiochip3', 28, 'out')
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


    def read_encoder(mcp):
        testval = 0
        prevtestval = 0
        encAState = False
        encBState = False
        vals = [0,0,0,0,0,0,0,0]
        while True:
            #for i in range(8):
            #    vals[i] = mcp.read_bit(Port.PORTA, Register.GPIO, i)
            encA = mcp.read_bit(Port.PORTA, Register.GPIO, 2)
            encB = mcp.read_bit(Port.PORTA, Register.GPIO, 3)
            #print(encA)

            if prevtestval != testval:
                prevtestval = testval
                print(testval)

            if encA == True and encB == True:
                continue

            #if encA == False and encB == False:
            #    continue

            if encAState == True and encB == False:
                encAState = False
                return 1

            if encBState == True and encA == False:
                encBState = False
                return -1

            if encA == False and encB == True:
                encAState = True

            if encA == True and encB == False:
                encBState == True




    null_gpio = MagicMock()
    cs_expander_gpio = GPIO('/dev/gpiochip3', 5, 'out')
    cs_display_gpio = GPIO('/dev/gpiochip3', 6, 'out')
    vfm_rst = GPIO('/dev/gpiochip6', 17, 'out')
    cs_display_gpio.write(True)

    spi = SPI('/dev/spidev0.0', 3, 1e6)
    #mcp = MCP23S17(null_gpio, null_gpio, null_gpio, null_gpio, null_gpio, null_gpio, spi, cs_expander_gpio)


    ls.write(False)
    hs.write(False)
    pc.write(False)


    ls.write(True)
    pc.write(True)
    #sleep(3)
    hs.write(True)
    pc.write(False)

    print('starting...')

    mc.accelerate(0.0)

    mc.state(False)
    sleep(2)
    mc.state(True)
    mc.accelerate(0.3)
    sleep(5)


    vfm_rst.write(False)
    sleep(1)
    vfm_rst.write(True)
    '''
    print('50')
    mc.accelerate(0.0)
    sleep(2)

    val = 0.0
    print("SPIN")
    while True:
        out = read_encoder(mcp)
        if out == 1:
            val += 0.004
        elif out == -1:
            val -= 0.02
        val = max(0, min(0.6, val))
        print(val)
        if val >= 0.5:
            break

        mc.accelerate(val)
    sleep(2)
    '''

    print("up")
    mc.variable_field_magnet_up()
    sleep(3)
    print("up")
    mc.variable_field_magnet_up()
    sleep(3)
    print("up")
    mc.variable_field_magnet_up()
    sleep(3)

    print("down")
    mc.variable_field_magnet_down()
    sleep(3)
    print("down")
    mc.variable_field_magnet_down()
    sleep(3)
    print("down")
    mc.variable_field_magnet_down()
    sleep(3)

    print("stop")
    mc.accelerate(0)
    mc.regenerate(0)
    mc.state(False)

    ls.write(False)
    hs.write(False)
    pc.write(False)
