from iclib.mcp23s17 import MCP23S17, Port, Register
from iclib.nhd_c12864a1z_fsw_fbw_htt import NHDC12864A1ZFSWFBWHTT
from periphery import GPIO, SPI
from unittest.mock import MagicMock
from time import sleep

null_gpio = MagicMock()
cs_expander_gpio = GPIO('/dev/gpiochip3', 5, 'out')
cs_display_gpio = GPIO('/dev/gpiochip3', 6, 'out')
cs_display_gpio.write(True)
cs_expander_gpio.write(False)

spi = SPI('/dev/spidev0.0', 3, 1e5)
mcp = MCP23S17(null_gpio, null_gpio, null_gpio, spi)
#A0 = CUSTOM_GPIO_A0(mcp, cs_expander_gpio, Port.PORTB, Register.GPIO)
#NOT_RESET = CUSTOM_GPIO_RESET(mcp, cs_expander_gpio, Port.PORTB, Register.GPIO)
#display = NHDC12864A1ZFSWFBWHTT(spi=spi, a0_pin=A0, reset_pin=NOT_RESET)


testval = 0
prevtestval = 0
encAState = False
encBState = False
vals = [0,0,0,0,0,0,0,0]
while True:
    cs_expander_gpio.write(False)
    for i in range(8):
        vals[i] = mcp.read_bit(Port.PORTA, Register.GPIO, i)
    print(vals)
    cs_expander_gpio.write(True) 
    #battery = mcp.read_bit(Port.PORTA, Register.GPIO, 0)
    #encB = mcp.read_bit(Port.PORTA, Register.GPIO, 4)
    
    #print(battery)
    sleep(0.5)
    #print(encB)

    #print(encA)
    '''
    if prevtestval != testval:
        prevtestval = testval
        print(testval)

    if encA == True and encB == True:
        continue

    #if encA == False and encB == False:
    #    continue

    if encAState == True and encB == False:
        encAState = False
        print("HitA")

    if encBState == True and encA == False:
        encBState = False
        print("HitB")

    if encA == False and encB == True:
        encAState = True

    if encA == True and encB == False:
        encBState == True
    '''
