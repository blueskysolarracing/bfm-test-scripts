from periphery import GPIO
from iclib.a1230 import A1230

a = GPIO('/dev/gpiochip4', 25, 'in')
b = GPIO('/dev/gpiochip4', 24, 'in')

# he = A1230(a, b)

print(a.read(), b.read())
