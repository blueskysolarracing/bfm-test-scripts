from time import sleep
from periphery import PWM, GPIO

gpio3 = GPIO('/dev/gpiochip0', 6, 'out')
vfm_reset = GPIO('/dev/gpiochip6', 17, 'out')
vfm_stall = GPIO('/dev/gpiochip0', 1, 'in')
vfm_enc_a = GPIO('/dev/gpiochip0', 0, 'in')
vfm_enc_b = GPIO('/dev/gpiochip1', 12, 'in')
vfm_dir  = GPIO('/dev/gpiochip1', 13, 'out')
vfm_en = GPIO('/dev/gpiochip0', 8, 'out')

vfm_en.write(True)
sleep(0.1)
vfm_dir.write(False)
while True:
    a = vfm_enc_a.read()
    b = vfm_enc_b.read()
    print(f"a: {a} b: {b}")
    current_state = vfm_stall.read()
    if (current_state):
        vfm_dir.write(True)
        sleep(0.2)
        vfm_en.write(False)
        sleep(0.01)
        vfm_en.write(True)
        sleep(0.01)
        vfm_dir.write(True)
        break
while True:
    a = vfm_enc_a.read()
    b = vfm_enc_b.read()
    print(f"a: {a} b: {b}")
    current_state = vfm_stall.read()
    if (current_state):
        vfm_dir.write(False)
        sleep(0.2)
        vfm_en.write(False)
        sleep(0.01)
        vfm_en.write(True)
        sleep(0.01)
        vfm_dir.write(False)
        break
vfm_en.write(False)

'''
while True:
    current_state = vfm_stall.read()
    print(current_state)
    if (current_state):
        vfm_en.write(False)
        break


    
'''
