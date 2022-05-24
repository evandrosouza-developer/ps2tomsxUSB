#!/bin/bash
#Script to connect and run on Black Magic Probe
#Please change line 5 (target extend-remote) to your own Black Magic Probe.
#I REALLY recommend you to use the "by-id", because reconnection sometimes changes the ttyACM0 to else other one (ttyACM1 an so on).

arm-none-eabi-gdb "$1"
target extended-remote /dev/serial/by-id/usb-Black_Sphere_Technologies_Black_Magic_Probe__STLINK_V2__v1.7.1-275-gb076d5c_E1E1ADE6-if00
monitor swdp_scan
attach 1

