from iclib.adc78h89 import ADC78H89, InputChannel
#from adc78h89_local import ADC78H89, InputChannel
from periphery import SPI, GPIO 
from time import sleep
import json

spi = SPI('/dev/spidev1.0', 3, 1e6)
adc78h89 = ADC78H89(spi, 3.3)
cs = GPIO('/dev/gpiochip3', 10, 'out')

cs.write(True)  

# TOR_AMBIENT_TEMP=AIN1 (index0)
# FOOT_PEDAL_BUFFER=AIN2 (index 1) 
# 12V_SENSE=AIN3 (index2)
#AIN4/5 are not connected (indexes 3/4)
# BUFFER_THERM_1=AIN6 (index5)
# BUFFER_THERM_2=AIN7 (index6)
# Ground = index7 

cs.write(False)
voltages = adc78h89.sample_all()
print(voltages)
cs.write(True)

while True:
    cs.write(False)
    sleep(5)
    #voltage = adc78h89.sample(InputChannel.AIN2)
    voltages = adc78h89.sample_all()
    cs.write(True)
    #print(voltage)
    print(voltages)
    print(json.dumps(voltages, indent=4, sort_keys=True))
    #voltages = adc78h89.sample_all()
    #cs.write(True)
    #print(f"GROUND={voltages[InputChannel.GROUND]}")
    #print(f"TOR_AMBIENT_TEMP={voltages[InputChannel.AIN1]}")
    #print(f"FOOT_PEDAL_BUFFER={voltages[InputChannel.AIN2]}")
    #print(f"12V_SENSE={voltages[InputChannel.AIN3]}")
    #print(f"BUFFER_THERM_1={voltages[InputChannel.AIN6]}")
    #print(f"BUFFER_THERM_2={voltages[InputChannel.AIN7]}")
    print()
    sleep(2)
