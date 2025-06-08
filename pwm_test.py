from periphery import SPI, GPIO, PWM
from time import sleep

pwm = PWM(0, 0)
pwm2 = PWM(1, 0)
pwm3 = PWM(2, 0)
pwm4 = PWM(3, 0)

dc = 0.1
f = 1e3

pwm.frequency = f
pwm.duty_cycle = dc
pwm.enable()

pwm2.frequency = f
pwm2.duty_cycle = dc
pwm2.enable()

pwm3.frequency = f
pwm3.duty_cycle = dc
pwm3.enable()

pwm4.frequency = f
pwm4.duty_cycle = dc
pwm4.enable()

for i in range (1, 10):
    dc = 0.8 / i
    pwm.duty_cycle = dc
    pwm2.duty_cycle = dc
    pwm3.duty_cycle = dc
    pwm4.duty_cycle = dc
    sleep(1)


sleep(1)
pwm.disable()
pwm2.disable()
pwm3.disable()
pwm4.disable()

pwm.close()
pwm2.close()
pwm3.close()
pwm4.close()
