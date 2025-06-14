"""This module implements the ADC78H89 driver."""

from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import ClassVar
from warnings import warn

from periphery import SPI


class InputChannel(IntEnum):
    """The enum class for input channels.

    Refer to Table 3, ADC78H89 Datasheet, Page 13.
    """

    AIN1: int = 0b000
    """The first (default) input channel."""
    AIN2: int = 0b001
    """The second input channel."""
    AIN3: int = 0b010
    """The third input channel."""
    AIN4: int = 0b011
    """The fourth input channel."""
    AIN5: int = 0b100
    """The fifth input channel."""
    AIN6: int = 0b101
    """The sixth input channel."""
    AIN7: int = 0b110
    """The seventh input channel."""
    GROUND: int = 0b111
    """The ground input channel."""

    @property
    def ADD2(self) -> bool:
        """Get ``ADD2`` of the input channel.

        :return: ``ADD2``.
        """
        return bool(self & 0b100)

    @property
    def ADD1(self) -> bool:
        """Get ``ADD1`` of the input channel.

        :return: ``ADD1``.
        """
        return bool(self & 0b010)

    @property
    def ADD0(self) -> bool:
        """Get ``ADD0`` of the input channel.

        :return: ``ADD0``.
        """
        return bool(self & 0b001)


@dataclass
class ADC78H89:
    """The class for Texas Instruments ADC78H89 7-Channel, 500 KSPS,
    12-Bit A/D Converter.
    """

    SPI_MODE: ClassVar[int] = 0b11
    """The supported spi mode."""
    MIN_SPI_MAX_SPEED: ClassVar[float] = 5e4
    """The supported minimum spi maximum speed."""
    MAX_SPI_MAX_SPEED: ClassVar[float] = 8e6
    """The supported maximum spi maximum speed."""
    SPI_BIT_ORDER: ClassVar[str] = 'msb'
    """The supported spi bit order."""
    SPI_WORD_BIT_COUNT: ClassVar[int] = 8
    """The supported spi number of bits per word."""
    INPUT_CHANNEL_BITS_OFFSET: ClassVar[int] = 3
    """The input channel bits offset for control register bits."""
    DIVISOR: ClassVar[int] = 4096
    """The lsb width for ADC78H89."""
    DEFAULT_INPUT_CHANNEL: ClassVar[InputChannel] = InputChannel(0)
    """The default input channel."""
    spi: SPI
    """The SPI for the ADC device."""
    reference_voltage: float
    """The reference voltage value (in volts)."""

    def __post_init__(self) -> None:
        if self.spi.mode != self.SPI_MODE:
            raise ValueError('unsupported spi mode')
        elif not (
                self.MIN_SPI_MAX_SPEED
                <= self.spi.max_speed
                <= self.MAX_SPI_MAX_SPEED
        ):
            raise ValueError('unsupported spi maximum speed')
        elif self.spi.bit_order != self.SPI_BIT_ORDER:
            raise ValueError('unsupported spi bit order')
        elif self.spi.bits_per_word != self.SPI_WORD_BIT_COUNT:
            raise ValueError('unsupported spi number of bits per word')

        if self.spi.extra_flags:
            warn(f'unknown spi extra flags {self.spi.extra_flags}')

    def sample(self, *input_channels: InputChannel) -> list[float]:
        """Sample the voltages of the input channels.

        :param input_channels: The input channels.
        :return: The sampled voltages.
        """
        transmitted_data_bytes = []

        for input_channel in input_channels:
            transmitted_data_bytes.append(
                input_channel << self.INPUT_CHANNEL_BITS_OFFSET,
            )
            transmitted_data_bytes.append(0)

        print("Transmitted data bytes: " + '[{}]'.format(', '.join(hex(x) for x in transmitted_data_bytes)))
        received_data_bytes = self.spi.transfer(transmitted_data_bytes)
        voltages = []
        for i in range(0, len(received_data_bytes), 2):
            data_byte = (
                received_data_bytes[i]
                << self.SPI_WORD_BIT_COUNT
                | received_data_bytes[i + 1]
            )

            voltages.append(
                self.reference_voltage * data_byte / self.DIVISOR,
            )

        return voltages

    def sample_all(self) -> dict[InputChannel, float]:
        """Sample the voltages of all input channels.

        :return: The sampled voltages.
        """
        self.sample(InputChannel.GROUND)

        voltages = deque(self.sample(*InputChannel))

        voltages.rotate(-1)

        return dict(zip(InputChannel, voltages))

from periphery import SPI, GPIO 
from time import sleep

array_cs = GPIO('/dev/gpiochip5', 20, 'out')
motor_cs = GPIO('/dev/gpiochip5', 19, 'out')
battery_cs = GPIO('/dev/gpiochip5', 21, 'out')
psm_cs = GPIO('/dev/gpiochip4', 26, 'out')

array_cs.write(True)
battery_cs.write(True)
motor_cs.write(True)
psm_cs.write(True)

# Use spidev1.0 when on BFM and spidev0.0 when on ixora
# Use GPIO3.10 when on BFM and GPIO3.05 when on ixora

spi = SPI('/dev/spidev2.0', 3, 1e6)
#spi = SPI('/dev/spidev0.0', 3, 1e6)
adc78h89 = ADC78H89(spi, 3.3)

cs = GPIO('/dev/gpiochip3', 10, 'out')
#cs = GPIO('/dev/gpiochip3', 5, 'out')
cs.write(True)  

# TOR_AMBIENT_TEMP=AIN1 (index0)
# FOOT_PEDAL_BUFFER=AIN2 (index 1) 
# 12V_SENSE=AIN3 (index2)
#AIN4/5 are not connected (indexes 3/4)
# BUFFER_THERM_1=AIN6 (index5)
# BUFFER_THERM_2=AIN7 (index6)
# Ground = index7 

#for ain in InputChannel:
#    cs.write(False)
#    voltage= adc78h89.sample(ain)
#    cs.write(True)
#    print(voltage)
#    sleep(0.01)

while True:
    data = [0xAA, 0x55]
    cs.write(False)
    sleep(0.01)
    #res = spi.transfer(data)
    res = adc78h89.sample_all()
    sleep(0.01)
    cs.write(True)
    print(res)
    sleep(1)

