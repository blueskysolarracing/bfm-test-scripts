"""This module implements the ADC78H89 driver."""

import os
from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import ClassVar
from warnings import warn

from periphery import SPI
from can import Bus, BusABC


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

from abc import ABC
from dataclasses import dataclass
from math import copysign
from struct import pack, unpack
from typing import ClassVar

from can import BusABC, Message


@dataclass
class MotorControlBroadcastMessage(ABC):
    MESSAGE_IDENTIFIER: ClassVar[int]
    FORMAT: ClassVar[str]


@dataclass
class IdentificationInformation(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x00
    FORMAT = '<II'
    prohelion_id: int
    serial_number: int


@dataclass
class StatusInformation(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x01
    FORMAT = '<HHHBB'
    limit_flags: int
    error_flags: int
    active_motor: int
    transmit_error_count: int
    receive_error_count: int


@dataclass
class BusMeasurement(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x02
    FORMAT = '<ff'
    bus_voltage: float
    bus_current: float


@dataclass
class VelocityMeasurement(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x03
    FORMAT = '<ff'
    motor_velocity: float
    vehicle_velocity: float


@dataclass
class PhaseCurrentMeasurement(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x04
    FORMAT = '<ff'
    motor_velocity: float
    phase_c_current: float


@dataclass
class MotorVoltageVectorMeasurement(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x05
    FORMAT = '<ff'
    Vq: float
    Vd: float


@dataclass
class MotorCurrentVectorMeasurement(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x06
    FORMAT = '<ff'
    Iq: float
    Id: float


@dataclass
class MotorBackEMFMeasurementPrediction(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x07
    FORMAT = '<ff'
    BEMFq: float
    BEMFd: float


@dataclass
class VoltageRailMeasurement15V(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x08
    FORMAT = '<ff'
    reserved: float
    supply_15v: float


@dataclass
class VoltageRailMeasurement3_3VAnd1_9V(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x09
    FORMAT = '<ff'
    supply_1_9v: float
    supply_3_3v: float


@dataclass
class Reserved0(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x0A
    FORMAT = '<ff'
    reserved_2: float
    reserved_1: float


@dataclass
class HeatSinkAndMotorTemperatureMeasurement(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x0B
    FORMAT = '<ff'
    motor_temp: float
    heat_sink_temp: float


@dataclass
class DSPBoardTemperatureMeasurement(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x0C
    FORMAT = '<ff'
    dsp_board_temp: float
    reserved: float


@dataclass
class Reserved1(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x0D
    FORMAT = '<ff'
    reserved_2: float
    reserved_1: float


@dataclass
class OdometerAndBusAmpHoursMeasurement(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x0E
    FORMAT = '<ff'
    odometer: float
    dc_bus_amphours: float


@dataclass
class SlipSpeedMeasurement(MotorControlBroadcastMessage):
    MESSAGE_IDENTIFIER = 0x17
    FORMAT = '<ff'
    reserved: float
    slip_speed: float


@dataclass
class WaveSculptor22:
    CAN_BUS_BITRATES: ClassVar[tuple[int, ...]] = (
        1000000,
        500000,
        250000,
        125000,
        100000,
        50000,
    )
    can_bus: BusABC
    driver_controls_base_address: int
    motor_controller_base_address: int

    def __post_init__(self) -> None:
        if self.driver_controls_base_address not in range(1 << 12):
            raise ValueError('invalid driver controls base address')

    def _send(
            self,
            message_identifier: int,
            data: bytes,
            timeout: float | None = None,
            base_address: int | None = None,
    ) -> None:
        if len(data) != 8:
            raise ValueError('data is not 8 bytes')

        if base_address is None:
            base_address = self.driver_controls_base_address

        arbitration_id = base_address + message_identifier
        message = Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
        )

        self.can_bus.send(message, timeout)

    # Drive Commands

    def motor_drive(
            self,
            motor_current: float,
            motor_velocity: float,
            timeout: float | None = None,
    ) -> None:
        """Send the Motor Drive Command.

        :param motor_current: The ``Motor Current`` variable of the
                              percentage type.
        :param motor_velocity: The ``Motor Velocity`` variable of the
                               rpm type.
        :return: ``None``.
        """
        self._send(0x1, pack('<ff', motor_velocity, motor_current), timeout)

    def motor_power(
            self,
            bus_current: float,
            timeout: float | None = None,
    ) -> None:
        self._send(0x2, pack('<ff', 0, bus_current), timeout)

    def reset(self, timeout: float | None = None) -> None:
        self._send(0x3, pack('<ff', 0, 0), timeout)

    UNOBTAINABLE_VELOCITY: ClassVar[float] = 20000

    def motor_drive_torque_control_mode(
            self,
            motor_current: float,
            timeout: float | None = None,
    ) -> None:
        motor_velocity = copysign(self.UNOBTAINABLE_VELOCITY, motor_current)

        self.motor_drive(motor_current, motor_velocity, timeout)

    def motor_drive_velocity_control_mode(
            self,
            motor_velocity: float,
            motor_current: float = 1,
            timeout: float | None = None,
    ) -> None:
        self.motor_drive(motor_current, motor_velocity, timeout)

    # Motor Control Broadcast Messages

    MOTOR_CONTROL_BROADCAST_MESSAGE_TYPES: ClassVar[
            tuple[type[MotorControlBroadcastMessage], ...]
    ] = (
        IdentificationInformation,
        StatusInformation,
        BusMeasurement,
        VelocityMeasurement,
        PhaseCurrentMeasurement,
        MotorVoltageVectorMeasurement,
        MotorCurrentVectorMeasurement,
        MotorBackEMFMeasurementPrediction,
        VoltageRailMeasurement15V,
        VoltageRailMeasurement3_3VAnd1_9V,
        Reserved0,
        HeatSinkAndMotorTemperatureMeasurement,
        DSPBoardTemperatureMeasurement,
        Reserved1,
        OdometerAndBusAmpHoursMeasurement,
        SlipSpeedMeasurement,
    )

    def parse(self, message: Message) -> MotorControlBroadcastMessage | None:
        device_identifier = message.arbitration_id >> 5

        if self.motor_controller_base_address != device_identifier << 5:
            return None

        message_identifier = message.arbitration_id & ((1 << 5) - 1)
        broadcast_message = None

        for type_ in self.MOTOR_CONTROL_BROADCAST_MESSAGE_TYPES:
            if message_identifier == type_.MESSAGE_IDENTIFIER:
                broadcast_message = type_(*unpack(type_.FORMAT, message.data))

                break

        return broadcast_message

    # Configuration Commands

    CONFIGURATION_ACCESS_KEY: ClassVar[bytes] = b'ACTMOT'

    def active_motor_change(
            self,
            active_motor: int,
            timeout: float | None = None,
    ) -> None:
        if active_motor not in range(10):
            raise ValueError('invalid active motor')

        self._send(
            0x12,
            pack('<6sH', self.CONFIGURATION_ACCESS_KEY, active_motor),
            timeout,
            self.motor_controller_base_address,
        )

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

CAN_BUS_CHANNEL: str = 'can0'
CAN_BUS_BITRATE: int = 500000

os.system(f'ip link set {CAN_BUS_CHANNEL} down')
os.system(f'ip link set {CAN_BUS_CHANNEL} up type can bitrate {CAN_BUS_BITRATE}')

CAN_BUS: BusABC = Bus(channel=CAN_BUS_CHANNEL, interface='socketcan')
REVOLUTION_BASE_ADDRESS: int = 0x500
WAVESCULPTOR22_BASE_ADDRESS: int = 0x400

motor = WaveSculptor22(
    CAN_BUS,
    REVOLUTION_BASE_ADDRESS,
    WAVESCULPTOR22_BASE_ADDRESS,
)

motor.reset()

while True:
    data = [0xAA, 0x55]
    cs.write(False)
    sleep(0.1)
    #res = spi.transfer(data)
    res = adc78h89.sample(InputChannel.AIN2)
    motor.motor_drive_torque_control_mode(1)
    #motor.motor_drive_torque_control_mode(res[0] > 0.7)
    sleep(0.1)
    motor.motor_drive_torque_control_mode(0)
    cs.write(True)
    print(res[0])
    #sleep(2)

