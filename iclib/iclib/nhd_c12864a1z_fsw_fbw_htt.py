from dataclasses import dataclass
from math import floor
from typing import ClassVar
from warnings import warn

from freetype import Face  # type: ignore[import-untyped]
from periphery import GPIO, SPI


@dataclass
class NHDC12864A1ZFSWFBWHTT:
    """A Python driver for Newhaven Display Intl
    NHD-C12864A1Z-FSW-FBW-HTT COG (Chip-On-Glass) Liquid Crystal Display
    Module
    """

    SPI_MODE: ClassVar[int] = 0b11
    """The supported spi modes."""
    MIN_SPI_MAX_SPEED: ClassVar[float] = 5e4
    """The supported minimum spi maximum speed."""
    MAX_SPI_MAX_SPEED: ClassVar[float] = 30e6
    """The supported maximum spi maximum speed."""
    SPI_BIT_ORDER: ClassVar[str] = 'msb'
    """The supported spi bit order."""

    WIDTH: ClassVar[int] = 128
    """The number of pixels by width."""
    HEIGHT: ClassVar[int] = 64
    """The number of pixels by height."""
    BASE_PAGE: ClassVar[int] = 0xB0
    """The address of the first page (of 8)."""
    DISPLAY_START_ADDRESS: ClassVar[int] = 0x40
    """The address of the display."""
    DISPLAY_OFF: ClassVar[int] = 0xAE
    """The command to turn the display off."""
    DISPLAY_ON: ClassVar[int] = 0xAF
    """The command to turn the display on."""
    TURN_POINTS_ON: ClassVar[int] = 0xA5
    """The command to confirm the writes on the display."""
    REVERT_NORMAL: ClassVar[int] = 0xA4
    """The command to show the result on the display."""

    spi: SPI
    """The SPI for the display device."""
    a0_pin: GPIO
    """The mode select pin for the display device."""
    reset_pin: GPIO
    """The reset pin (active low) for the display device."""

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

        if self.spi.extra_flags:
            warn(f'unknown spi extra flags {self.spi.extra_flags}')

        self.framebuffer = [0x0 for i in range(64 * 16)]
        self.face = None
        self.font_width = -1
        self.font_height = -1

    def reset(self) -> None:
        """Resets everything in the display.

        :return: ``None``.
        """
        self.reset_pin.write(False)
        self.reset_pin.write(True)

    def configure(self) -> None:
        """Configures the diplay for normal operation.

        :return: ``None``.
        """
        self.reset()
        self.a0_pin.write(False)
        self.spi.transfer(
            [
                0xA0,              # ADC select.
                self.DISPLAY_OFF,  # Display OFF.
                0xC8,              # COM direction scan.
                0xA2,              # LCD bias set.
                0x2F,              # Power Control set.
                0x26,              # Resistor Ratio Set.
                0x81,              # Electronic Volume Command (set contrast).
                0x11,              # Electronic Volume value (contrast value).
                self.DISPLAY_ON,   # Display ON
            ]
        )

    def clear_screen(self) -> None:
        """Clears the framebuffer and the display.

        :return: ``None``.
        """
        for i in range(len(self.framebuffer)):
            self.framebuffer[i] = 0x00

        self.display()

    def display(self) -> None:
        """Writes what is in the local framebuffer to the display
        memory.

        :return: ``None``.
        """
        index = 0
        # Write LCD pixel data
        page = self.BASE_PAGE
        self.spi.transfer([self.DISPLAY_OFF, self.DISPLAY_START_ADDRESS])

        for i in range(8):
            self.spi.transfer([page, 0x10, 0x00])
            self.a0_pin.write(True)
            self.spi.transfer(
                [
                    self.framebuffer[i + index * self.WIDTH]
                    for i in range(self.WIDTH)
                ],
            )
            self.a0_pin.write(False)
            page += 1
            index += 1

        self.spi.transfer(
            [
                self.DISPLAY_ON,
                self.TURN_POINTS_ON,
                self.REVERT_NORMAL,
            ],
        )

    def framebuffer_offset(self, x: int, y: int) -> int:
        """Returns the flattened index in the framebuffer given an ``x``
        and ``y`` coordinate.

        :param x: The ``x`` coordinate.
        :param y: The ``y`` coordinate.
        :return: The framebuffer offset.
        """
        return x + 128 * floor(y / 8)

    def page_offset(self, x: int, y: int) -> int:
        """Returns the page ``(1-8)`` given a coordinate on the display.

        :param x: The ``x`` coordinate.
        :param y: The ``y`` coordinate.
        :return: The page offset.
        """
        return floor(y / 8)

    def write_pixel(self, x: int, y: int) -> None:
        """Turn on pixel at ``(x, y)`` in the framebuffer. This is does
        not immediately update the display.

        :param x: The ``x`` coordinate.
        :param y: The ``y`` coordinate.
        :return: ``None``.
        """
        i = self.framebuffer_offset(x, y)
        self.framebuffer[i] = self.framebuffer[i] | (1 << (y % 8))

    def write_pixel_immediate(self, x: int, y: int) -> None:
        """Write to framebuffer and update display.

        :param x: The ``x`` coordinate.
        :param y: The ``y`` coordinate.
        :return: ``None``.
        """
        i = self.framebuffer_offset(x, y)
        page = self.BASE_PAGE + self.page_offset(x, y)
        self.write_pixel(x, y)

        self.spi.transfer([page, 0x10, 0x00])
        self.a0_pin.write(True)
        self.spi.transfer([self.framebuffer[i]])
        self.a0_pin.write(False)

    def clear_pixel(self, x: int, y: int) -> None:
        """Turn off pixel at ``(x, y)`` in the framebuffer.

        This is does not immediately update the display.

        :param x: The ``x`` coordinate.
        :param y: The ``y`` coordinate.
        :return: ``None``.
        """
        i = self.framebuffer_offset(x, y)
        self.framebuffer[i] = self.framebuffer[i] & ~(1 << (y % 8))

    def clear_pixel_immediate(self, x: int, y: int) -> None:
        """Write to framebuffer and update display.

        :param x: The ``x`` coordinate.
        :param y: The ``y`` coordinate.
        :return: ``None``.
        """
        i = self.framebuffer_offset(x, y)
        page = self.BASE_PAGE + self.page_offset(x, y)
        self.clear_pixel(x, y)

        self.spi.transfer([page, 0x10, 0x00])
        self.a0_pin.write(True)
        self.spi.transfer([self.framebuffer[i]])
        self.a0_pin.write(False)

    def draw_fill_rect(self, x: int, y: int, width: int, height: int) -> None:
        """Draw a filled rectangle.

        :param x: The ``x`` coordinate.
        :param y: The ``y`` coordinate.
        :param width: The width.
        :param height: The height.
        :return: ``None``
        """
        if (
                not self.pixel_in_bounds(x, y)
                or not self.pixel_in_bounds(x + width - 1, y + height - 1)
        ):
            return

        for row in range(height):
            for col in range(width):
                self.write_pixel(x + col, y + row)

    def draw_rect(self, x: int, y: int, width: int, height: int) -> None:
        """Draw a hollow rectangle.

        :param x: The ``x`` coordinate.
        :param y: The ``y`` coordinate.
        :param width: The width.
        :param height: The height.
        :return: ``None``
        """
        if (
                not self.pixel_in_bounds(x, y) or
                not self.pixel_in_bounds(x + width - 1, y + height - 1)
        ):
            return

        for row in range(height):
            self.write_pixel(x, y + row)
            self.write_pixel(x + width - 1, y + row)

        for col in range(width):
            self.write_pixel(x + col, y)
            self.write_pixel(x + col, y + height - 1)

    def set_font(self, filename: str) -> None:
        """Set the font for drawing letters.

        :param filename: The ``.ttf`` file to set.
        :return: ``None``.
        """
        self.face = Face(filename)

    def pixel_in_bounds(self, x: int, y: int) -> bool:
        """Checks if the x and y coordinate is
        within the bounds of the display resolution.
        """
        return x >= 0 and x <= self.WIDTH and y >= 0 and y <= self.HEIGHT

    def set_size(self, width: int, height: int) -> None:
        """Set the size of the letters.

        :param width: The width.
        :param height: The height.
        :return: ``None``.
        """
        if self.face is None:
            return

        # freetype uses 26.6 scaling, so the first 6 lsb are for decimals.
        self.face.set_char_size(width << 6, height << 6)
        self.font_width = width
        self.font_height = height

    def draw_letter(self, letter: str, x: int, y: int) -> None:
        """Draw the letter at position ``(x, y)``.

        :param x: The ``x`` coordinate.
        :param y: The ``y`` coordinate.
        :return: ``None``.
        """
        if (
            self.face is None
            or (
                not self.pixel_in_bounds(
                    x + self.font_width,
                    y + self.font_height,
                )
            )
        ):
            return

        self.face.load_char(letter)
        bitmap = self.face.glyph.bitmap

        for row in range(bitmap.rows):
            for col in range(bitmap.width):
                if x + col >= self.WIDTH or y + row >= self.HEIGHT:
                    continue

                if bitmap.buffer[row * bitmap.pitch + col]:
                    self.write_pixel(x + col, y + row)
                else:
                    self.clear_pixel(x + col, y + row)

    def draw_word(self, word: str, x: int, y: int) -> None:
        """Draws the word while wrapping if offscreen.

        :param word: The word.
        :param x: The ``x`` coordinate.
        :param y: The ``y`` coordinate.
        :return: ``None``
        """
        if (
                self.face is None
                or (
                    not self.pixel_in_bounds(
                        x + self.font_width,
                        y + self.font_height,
                    )
                )
        ):
            return

        x_off = x
        y_off = y

        for letter in word:
            if y_off + self.font_height >= self.HEIGHT:
                break

            if x_off + self.font_width >= self.WIDTH:
                x_off = x
                y_off += self.font_height

            self.draw_letter(letter, x_off, y_off)
            x_off += self.font_width
