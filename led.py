from os import environ
from time import sleep

from periphery import GPIO

value = True
brake_test = GPIO('/dev/gpiochip6', 20, 'in')

while True:
  ll = brake_test.read()
  print(ll)
  sleep(1) 
