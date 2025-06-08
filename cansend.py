

from time import sleep


"""
This example shows how sending a single message works.
"""

import can
import os

br = 500000

mc_arb_id = 0x490

bfm_arb_id = 0x082

os.system('ip link set can0 down')
os.system(f'ip link set can0 type can bitrate {br}')
os.system('ip link set can0 type can restart-ms 100')
#os.system('ip link set can0 type can presume-ack on')
#os.system('ip link set can0 type can one-shot on')
#os.system('ip link set can0 down')
os.system('ip link set can0 up')

def send_one(bus, cmd, data):
    """Sends a single message."""

    msg = can.Message(
        arbitration_id=cmd, data=data, is_extended_id=False, is_remote_frame=False
    )

    #try:
    bus.send(msg)
    print(f"Message sent on {bus.channel_info},  ")
    print(cmd)
    #except Exception as e:
     #   print("Message NOT sent")
      #  print(e)

    return 0

def recieve(bus):
    msg = bus.recv(1)

    print(msg)

    return 0


if __name__ == "__main__":
    bus = can.Bus(interface='socketcan', channel='can0', bitrate=br)

    #data = []

    #send_one(bus, 0x503, data)

    #sleep(0.2)

    data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f]
    #dataf=[0x3f, 0x00, 0x00, 0x00]

    #send_one(bus, 0x502, data)

    #data=[0x00, 0x40, 0x9c, 0x46, 0xcd, 0xcc, 0x4c, 0x3e]

    #send_one(bus, 0x501, data)

    #sleep(0.1)
    while True:
        data=[0x00, 0x40, 0x9c, 0x46, 0x00, 0x00, 0x00, 0x3f]
        #dataf = [0x3e, 0x99, 0x99, 0x9a, 0x46, 0x9c, 0x40, 0x00]

        #send_one(bus, 0x501, data)
        #sleep(0.1)

        send_one(bus, 0x500, data)
        sleep(1)

        #recieve(bus)
        #sleep(0.1)

        #recieve(bus)
        #sleep()
        print("\n")
        #send_one(bus)
        #sleep(5)

'''
import os
import time

while(1):
    os.system('cansend can0 01F#1122334455667788')
    print("sending packet")
    time.sleep(1)
'''
