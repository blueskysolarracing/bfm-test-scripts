from periphery import GPIO
import time

direction = GPIO('/dev/gpiochip1', 13, 'out', inverted=False)
end = GPIO('/dev/gpiochip0', 1, 'in', inverted=False)
encoder_a = GPIO('/dev/gpiochip0', 0, 'in', edge='both', inverted=False)
encoder_b = GPIO('/dev/gpiochip1', 12, 'in', edge='both', inverted=False)
enable = GPIO('/dev/gpiochip0', 8, 'out', inverted=False)

position = 0
prev_dir = False

def go_to_endpoint(dir):
  global position
  global prev_dir

  direction.write(dir)
  enable_status = False
  enable.write(enable_status)
  counter_a = 0
  counter_b = 0

  start_time = time.time()

  while not end.read():
    enable_status = not enable_status
    enable.write(enable_status)

    if encoder_a.poll(0):
      encoder_a.read_event()
      counter_a += 1
    if encoder_b.poll(0):
      encoder_b.read_event()
      counter_b += 1

    time.sleep(0.00005)

  enable_status = False
  enable.write(enable_status)

  end_time = time.time()
  print(f"counter_a: {counter_a} counter_b: {counter_b} time: {(end_tme - start_time):5f}s")

  prev_dir = dir
  if dir:
    position = -1 # TODO: find max
  else:
    position = 0

def increment(dir):
  global position
  global prev_dir

  direction.write(dir)
  enable_status = False
  enable.write(enable_status)

  count = 0
  iteration = 1
  if dir != prev_dir:
    iteration = 2

  while not end.read() and count < iteration:
    enable_status = not enable_status
    enable.write(enable_status)
    if encoder_a.poll(0):
      encoder_a.read_event()
      count += 1
      continue
    
    time.sleep(0.00005)

  enable_status = False
  enable.write(enable_status)

  prev_dir = dir
  if dir:
    position += 1
  else:
    position -= 1
  
while True:
  user_input = input("input: ")

  if user_input == 'w':
    go_to_endpoint(True)
  elif user_input == 's':
    go_to_endpoint(False)
  elif user_input == 'u':
    increment(True)
  elif user_input == 'j':
    increment(False)

  print(f"position={position} prev_dir={prev_dir}")
