python_dynamixels
=================

'''
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/

Use it at your own risk. Assume that I have no idea what I am doing.

Low level script to control AX 12 servos from Robotis. Based on original script
by Inxfergy. Found here:
http://forums.trossenrobotics.com/tutorials/how-to-diy-128/controlling-ax-12-servos-3275/
The scipt was slightly modified to fix errors and
ported to allow running on Raspberry pi.

The script assumes that it is connected to the servo through a buffer
such as a 74HC125. The communication direction is controlled through
GPIO pin 8.  A schematic for the circuit can be found on page 8 of
the AX12 manual here:
http://www.trossenrobotics.com/images/productdownloads/AX-12%28English%29.pdf

If a Raspberry Pi is being used then the startup config needs to be modified to
set the UART crystal to allow 1Mbps transfer. The TTY attached to the com
port needs to be disabled as well. Make sure to set the baud rate of
the TTY to 1Mbps.  A BASH script is included to set the TTY baud rate.  This 
page has a great deal of information on what else needs to be done to disconnect
the serial port from the console.  I also used it when first unraveling
the AX12 protocol.
http://www.oppedijk.com/robotics/control-dynamixel-with-raspberrypi

-Jesse Merritt
'''
