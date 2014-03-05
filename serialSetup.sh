#!/bin/bash
# Sets the serial port for 1Mbit baud rate

sudo echo "Setting the serial port for Dynamixels"  | sudo tee /dev/kmsg
sudo stty -F /dev/ttyAMA0 1000000
