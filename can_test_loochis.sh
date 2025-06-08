#!/bin/bash
ip link set can0 down
ip link set can0 type can bitrate 500000
ip link set can0 type can restart-ms 100
ip link set can0 up
candump can0
