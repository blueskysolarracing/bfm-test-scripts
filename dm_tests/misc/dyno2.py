import threading
import logging
import datetime
from periphery import Serial, GPIO, SPI
from time import sleep
from motor_test import M2096
from unittest.mock import MagicMock
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, StrEnum
from typing import ClassVar
from warnings import warn

def twos_complement(value: int, bit_count: int) -> int:
    if value & (1 << (bit_count - 1)):
        value -= (1 << bit_count)

    return value


class ReadOrWriteType(StrEnum):
    READ = 'R'
    WRITE = 'W'
    READ_OR_WRITE = 'R/W'


class Register(Enum):
    CONFIG = 0x00, 'Configuration', 16
    ADC_CONFIG = 0x01, 'ADC Configuration', 16
    SHUNT_CAL = 0x02, 'Shunt Calibration', 16
    SHUNT_TEMPCO = 0x03, 'Shunt Temperature Coefficient', 16
    VSHUNT = 0x04, 'Shunt Voltage Measurement', 24
    VBUS = 0x05, 'Bus Voltage Measurement', 24
    DIETEMP = 0x06, 'Temperature Measurement', 16
    CURRENT = 0x07, 'Current Result', 24
    POWER = 0x08, 'Power Result', 24
    ENERGY = 0x09, 'Energy Result', 40
    CHARGE = 0x0A, 'Charge Result', 40
    DIAG_ALRT = 0x0B, 'Diagnostic Flags and Alert', 16
    SOVL = 0x0C, 'Shunt Overvoltage Threshold', 16
    SUVL = 0x0D, 'Shunt Undervoltage Threshold', 16
    BOVL = 0x0E, 'Bus Overvoltage Threshold', 16
    BUVL = 0x0F, 'Bus Undervoltage Threshold', 16
    TEMP_LIMIT = 0x10, 'Temperature Over-Limit Threshold', 16
    PWR_LIMIT = 0x11, 'Power Over-Limit Threshold', 16
    MANUFACTURING_ID = 0x3E, 'Manufacturer ID', 16
    DEVICE_ID = 0x3F, 'Device ID', 16

    def __init__(self, address: int, name: str, size: int) -> None:
        self.address = address
        self.__name = name
        self.size = size

    @property
    def name(self) -> str:
        return self.__name

    @property
    def acronym(self) -> str:
        return super().name


class CONFIGRegisterField(Enum):
    RST = range(15, 16), ReadOrWriteType.READ_OR_WRITE, 0
    RSTACC = range(14, 15), ReadOrWriteType.READ_OR_WRITE, 0
    CONVDLY = range(6, 14), ReadOrWriteType.READ_OR_WRITE, 0
    TEMPCOMP = range(5, 6), ReadOrWriteType.READ_OR_WRITE, 0
    ADCRANGE = range(4, 5), ReadOrWriteType.READ_OR_WRITE, 0
    RESERVED = range(4), ReadOrWriteType.READ, 0

    def __init__(
            self,
            bits: range,
            type_: ReadOrWriteType,
            reset: int,
    ) -> None:
        self.bits = bits
        self.type = type_
        self.reset = reset

    @property
    def bit(self) -> int:
        if len(self.bits) != 1:
            raise ValueError('no or multiple associated bits')

        return self.bits[0]

    @property
    def field(self) -> str:
        return self.name


@dataclass
class SPIFrame(ABC):
    READ_OR_WRITE_TYPE: ClassVar[ReadOrWriteType]
    register_: Register  # appended underscore due to weird bug in dataclass

    @property
    def read_or_write_bit(self) -> bool:
        return self.READ_OR_WRITE_TYPE == ReadOrWriteType.READ

    @property
    def control_byte(self) -> int:
        return (self.register_.address << 2) | self.read_or_write_bit

    @property
    def data_byte_count(self) -> int:
        return self.register_.size // INA229.SPI_WORD_BIT_COUNT

    @property
    def transmitted_data_byte_count(self) -> int:
        return 1 + self.data_byte_count

    @property
    @abstractmethod
    def data_bytes(self) -> list[int]:
        pass

    @property
    def transmitted_data_bytes(self) -> list[int]:
        return [self.control_byte, *self.data_bytes]

    def parse_received_data_bytes(self, data_bytes: list[int]) -> int:
        assert len(data_bytes) == self.transmitted_data_byte_count
        assert not data_bytes[0]

        parsed_data = 0

        for data_byte in data_bytes[-self.data_byte_count:]:
            parsed_data <<= INA229.SPI_WORD_BIT_COUNT
            parsed_data |= data_byte

        return parsed_data


@dataclass
class SPIReadFrame(SPIFrame):
    READ_OR_WRITE_TYPE: ClassVar[ReadOrWriteType] = ReadOrWriteType.READ

    @property
    def data_bytes(self) -> list[int]:
        return [0] * self.data_byte_count


@dataclass
class SPIWriteFrame(SPIFrame):
    READ_OR_WRITE_TYPE: ClassVar[ReadOrWriteType] = ReadOrWriteType.WRITE
    data: int

    def __post_init__(self) -> None:
        assert self.register_.size == 16

    @property
    def data_bytes(self) -> list[int]:
        return [
            self.data >> INA229.SPI_WORD_BIT_COUNT,
            self.data & ((1 << INA229.SPI_WORD_BIT_COUNT) - 1),
        ]


@dataclass
class INA229:
    """A Python driver for Texas Instruments INA229 85-V, 20-Bit,
    Ultra-Precise Power/Energy/Charge Monitor With SPI Interface
    Expander with Serial Interface.
    """

    SPI_MODE: ClassVar[int] = 0b01
    """The supported spi modes."""
    MAX_SPI_MAX_SPEED: ClassVar[float] = 10e6
    """The supported maximum spi maximum speed."""
    SPI_BIT_ORDER: ClassVar[str] = 'msb'
    """The supported spi bit order."""
    SPI_WORD_BIT_COUNT: ClassVar[int] = 8
    """The supported spi number of bits per word."""
    R_SHUNT: float
    """The resistance value of the external shunt used to develop the
    differential voltage across the IN+ and IN- pins."""
    alert_gpio: GPIO
    """The alert GPIO."""
    spi: SPI
    """The SPI."""
    _ADCRANGE: bool = field(default=False, init=False)
    _SHUNT_CAL: int = field(default=0x1000, init=False)

    def __post_init__(self) -> None:
        if self.spi.mode != self.SPI_MODE:
            raise ValueError('unsupported spi mode')
        elif self.spi.max_speed > self.MAX_SPI_MAX_SPEED:
            raise ValueError('unsupported spi maximum speed')
        elif self.spi.bit_order != self.SPI_BIT_ORDER:
            raise ValueError('unsupported spi bit order')
        elif self.spi.bits_per_word != self.SPI_WORD_BIT_COUNT:
            raise ValueError('unsupported spi number of bits per word')

        if self.spi.extra_flags:
            warn(f'unknown spi extra flags {self.spi.extra_flags}')

    # Semantic

    def spi_communicate(self, *spi_frames: SPIFrame) -> list[int]:
        transmitted_data_bytes = []

        for spi_frame in spi_frames:
            transmitted_data_bytes.extend(spi_frame.transmitted_data_bytes)

        received_data_bytes = self.spi.transfer(transmitted_data_bytes)

        assert isinstance(received_data_bytes, list)
        assert len(transmitted_data_bytes) == len(received_data_bytes)

        parsed_received_data_bytes = []
        begin = 0

        for spi_frame in spi_frames:
            end = begin + spi_frame.transmitted_data_byte_count

            parsed_received_data_bytes.append(
                spi_frame.parse_received_data_bytes(
                    received_data_bytes[begin:end],
                ),
            )

            begin = end

        assert len(parsed_received_data_bytes) == len(spi_frames)

        return parsed_received_data_bytes

    def read(self, register: Register) -> int:
        return self.spi_communicate(SPIReadFrame(register))[0]

    def write(self, register: Register, data: int) -> int:
        return self.spi_communicate(SPIWriteFrame(register, data))[0]

    def reset(self) -> None:
        data = self.read(Register.CONFIG)
        data |= 1 << CONFIGRegisterField.RST.bit

        self.write(Register.CONFIG, data)

    @property
    def shunt_voltage(self) -> float:
        if self.ADCRANGE:
            conversion_factor = 78.125
        else:
            conversion_factor = 312.5

        return (
            conversion_factor
            * twos_complement(self.read(Register.VSHUNT) >> 4, 20)
            / 1e9
        )

    @property
    def bus_voltage(self) -> float:
        return (
            195.3125e-6
            * twos_complement(self.read(Register.VBUS) >> 4, 20) * 12.9
        )

    @property
    def temperature(self) -> float:
        return (
            7.8125
            * twos_complement(self.read(Register.DIETEMP), 16)
            / 1e3
        )

    @property
    def current(self) -> float:
        return (
            self.CURRENT_LSB
            * twos_complement(self.read(Register.CURRENT) >> 4, 20)
        )

    @property
    def power(self) -> float:
        return 3.2 * self.CURRENT_LSB * self.read(Register.POWER)

    @property
    def energy(self) -> float:
        return 16 * 3.2 * self.CURRENT_LSB * self.read(Register.ENERGY)

    @property
    def charge(self) -> float:
        return (
            self.CURRENT_LSB
            * twos_complement(self.read(Register.CHARGE), 40)
        )

    # Non-semantic

    @property
    def ADCRANGE(self) -> bool:
        assert (
            self._ADCRANGE
            == bool(
                (
                    self.read(Register.CONFIG)
                    & (1 << CONFIGRegisterField.ADCRANGE.bit)
                ),
            )
        )

        return self._ADCRANGE

    @ADCRANGE.setter
    def ADCRANGE(self, value: bool) -> None:
        CONFIG = self.read(Register.CONFIG)

        if bool(CONFIG & (1 << CONFIGRegisterField.ADCRANGE.bit)) != value:
            CONFIG ^= 1 << CONFIGRegisterField.ADCRANGE.bit

            self.write(Register.CONFIG, CONFIG)

    @property
    def SHUNT_CAL(self) -> int:
        assert self._SHUNT_CAL == self.read(Register.SHUNT_CAL)

        return self._SHUNT_CAL

    @SHUNT_CAL.setter
    def SHUNT_CAL(self, value: int) -> None:
        self.write(Register.SHUNT_CAL, value)

    @property
    def CURRENT_LSB(self) -> float:
        SHUNT_CAL = self.SHUNT_CAL

        if self.ADCRANGE:
            SHUNT_CAL *= 4

        return SHUNT_CAL / (13107.2 * 1e6 * self.R_SHUNT)


'''
Notes:
    Do we need to handle case when regen and acceleration are set?
'''

class DynoState:
    motor_state: bool
    acceleration: float
    regen: float

def read():
    while True:
        # Read up to 128 bytes with 500ms timeout
        buf = serial.read(128, 0.5)

        # Read
        if buf:
            string = buf.decode("utf-8")
            if not string:
                continue
            logger.debug(string)
            parsed = string.split()
            key = parsed[0]
            if len(parsed) == 2:
                val = parsed[1]

            match key:
                case "get_acceleration":
                    serial.write(f"{dyno_state.acceleration}\n")

                case "set_acceleration":
                    dyno_state.acceleration = max(0, min(1, float(val)))
                    mc.accelerate(dyno_state.acceleration)
                    logger.info(f"Acclerating to: {dyno_state.acceleration}")

                case "get_motor_state":
                    if dyno_state.motor_state:
                        serial.write(b"motor_state on\n")
                    else:
                        serial.write(b"motor_state off\n")

                case "toggle_motor_state":
                    dyno_state.motor_state = not dyno_state.motor_state
                    mc.state(dyno_state.motor_state)
                    if dyno_state.motor_state:
                        serial.write(b"motor_state on\n")
                    else:
                        serial.write(b"motor_state on\n")

                case "set_motor_state":
                    if val.lower() == "on":
                        dyno_state.motor_state = True
                        mc.state(dyno_state.motor_state)
                        serial.write(b"motor_state on\n")
                    elif val.lower() == "off":
                        dyno_state.motor_state = False
                        mc.state(dyno_state.motor_state)
                        serial.write(b"motor_state off\n")

                case "set_vfm":
                    if val.lower() == "up":
                        mc.variable_field_magnet_up()
                    elif val.upper() == "down":
                        mc.variable_field_magnet_down()

                case "set_regen":
                    dyno_state.regen = max(0, min(1, float(val)))
                    mc.regen(dyno_state.regen)
                    logger.info(f"Regenerating to: {dyno_state.regen}")

                case "get_regen":
                    serial.write(f"{dyno_state.regen}\n")

        sleep(1)

def get_datetime():
    return datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")

def write_hal_effect():
    while True:
        hal_val = 0
        out = f"hal_effect {hal_val}\n"
        serial.write(out.encode())
        sleep(hal_effect_period)

def write_psm():
    while True:
        motor_cs.write(False)
        current = ina.current
        print(current)
        motor_cs.write(True)
        motor_cs.write(False)
        voltage = ina.bus_voltage
        print(voltage)
        motor_cs.write(True)
        out = f"motor_psm_current {current}\n"
        serial.write(out.encode())
        out = f"motor_psm_voltage {voltage}\n"
        serial.write(out.encode())
        sleep(psm_period)

if __name__ == "__main__":
    dyno_state = DynoState()
    logging.basicConfig(
        format='%(asctime)s %(message)s',
        handlers=[
            logging.FileHandler(f"{get_datetime()}.log"),
            logging.StreamHandler()
        ]
    )
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)

    hal_effect_period = 3
    psm_period = 1

    serial = Serial("/dev/ttyLP2", 115200)
    ina = INA229(0.002, MagicMock(), SPI('/dev/spidev1.0', 1, 1e6))
    array_cs = GPIO('/dev/gpiochip5', 20, 'out')
    motor_cs = GPIO('/dev/gpiochip5', 19, 'out')
    battery_cs = GPIO('/dev/gpiochip5', 21, 'out')
    psm_cs = GPIO('/dev/gpiochip4', 26, 'out')
    array_cs.write(True)
    battery_cs.write(True)
    motor_cs.write(True)
    psm_cs.write(True)


    #ls = GPIO('/dev/gpiochip4', 2, 'out')
    #hs = GPIO('/dev/gpiochip3', 26, 'out')
    pc = GPIO('/dev/gpiochip3', 28, 'out')
    null_gpio = MagicMock()
    cs_expander_gpio = GPIO('/dev/gpiochip3', 5, 'out')
    cs_display_gpio = GPIO('/dev/gpiochip3', 6, 'out')
    vfm_rst = GPIO('/dev/gpiochip6', 17, 'out')
    cs_display_gpio.write(True)

    mc = M2096(
        SPI('/dev/spidev1.0', 3, 1e6),
        GPIO('/dev/gpiochip5', 29, 'out'),
        GPIO('/dev/gpiochip5', 28, 'out'),
        GPIO('/dev/gpiochip6', 14, 'out', inverted=True),
        GPIO('/dev/gpiochip6', 13, 'out'),
        GPIO('/dev/gpiochip6', 11, 'out'),
        GPIO('/dev/gpiochip6', 12, 'out'),
        GPIO('/dev/gpiochip6', 10, 'out'),
        MagicMock(),
    )

    t1 = threading.Thread(target=read, name='t1')
    t2 = threading.Thread(target=write_hal_effect, name='t2')
    t3 = threading.Thread(target=write_psm, name='t3')

    
    try:
        t1.start()
        t2.start()
        t3.start()
    except KeyboardInterrupt:
        t1.join()
        t2.join()
        t3.join()

        serial.close()
