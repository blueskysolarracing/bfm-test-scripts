from dataclasses import dataclass
from typing import ClassVar
from math import floor
from warnings import warn
from enum import auto, Enum, IntEnum
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from unittest.mock import MagicMock
from periphery import SPI, GPIO
import freetype
from time import sleep

led_pin = GPIO('/dev/gpiochip1', 10, 'out')

while True:
    led_pin.write(True)
    sleep(1)
    led_pin.write(False)
    sleep(1)
