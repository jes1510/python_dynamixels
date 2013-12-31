'''
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/

Use it at your own risk. Assume that I have no idea what I am doing.

Low level script to control AX 12 servos from Robotis.  Based on original script
by Michael Ferguson.  Found here:
http://forums.trossenrobotics.com/tutorials/how-to-diy-128/controlling-ax-12-servos-3275/
The scipt was slightly modified to fix errors and
ported to allow running on Raspberry pi.  

The script assumes that it is connected to the servo through a buffer
such as a 74HC125.  The communication direction is controlled through
a GPIO pin.

If a Raspberry Pi is being used then the startup config needs to be modified to
set the UART crystal to allow 1Mbps transfer.  The TTY attached to the com
port needs to be disabled as well.  Make sure to set the baud rate of
the TTY to 1Mbps
-Jesse Merritt
'''

import serial                   # we need to import the pySerial stuff to use
import time						# time module for sleeps and such
import RPi.GPIO as GPIO			# GPIO control module

# important AX-12 constants
AX_WRITE_DATA = 3
AX_READ_DATA = 2			# Corrected to 02.  Was 04

s = serial.Serial()         # create a serial port object
s.baudrate = 1000000        # baud rate, in bits/second
s.port = "/dev/ttyAMA0"     # this is whatever port your are using
s.open()

GPIO.setmode(GPIO.BCM)	# Set GPIO mode
tx = GPIO.LOW			# Low for TX mode
rx = GPIO.HIGH			# High for RX mode
directionPin = 18		# GPIO pin connected to Enable pins on buffer
GPIO.setup(directionPin, GPIO.OUT)	# Configure pin for output

# Error lookup dictionary for bit masking
dictErrors = {	1 : "Input Voltage",
				2 : "Angle Limit",
				4 : "Overheating",
				8 :	"Range",
				16 : "Checksum", 
				32 : "Overload",
				64 : "Instruction"
					}

# Custom error class to report AX servo errors
class axError(Exception) :		
	pass

def direction(d):	
	'''	Set direction of data.  Either rx or tx'''						

	GPIO.output(directionPin, d)	# set the pin
	time.sleep(0.05)	# sleep for 50mS to allow things to settle.  Decreases checksum errors

	

def setReg(index, reg,values):		# Corrected first arg to 'index'
	''' Set register values'''
	direction(tx)				# set the direction to be transmit
	length = 3 + len(values)	# configure length
	checksum = 255-((index+length+AX_WRITE_DATA+reg+sum(values))%256)    # calculate checksum, same as ~(sum(data))  
	s.write(chr(0xFF)+chr(0xFF)+chr(index)+chr(length)+chr(AX_WRITE_DATA)+chr(reg))	# Write the first part of the protocol
	for val in values:		# actually writes the data payload
		s.write(chr(val))	# chr(val) sends the binary data rather than ASCII
	s.write(chr(checksum))	# write the checksum
	direction(rx)			# Switch back to RX mode

def getReg(index, regstart, rlength):
	direction(tx)		# Set to TX mode
	s.flushInput() 	  	# flush any garbage in the buffer
	checksum = 255 - ((6 + index + regstart + rlength)%256)	# calculate the checksum	
	outData = chr(0xFF)+chr(0xFF)+chr(index)+chr(0x04)+chr(AX_READ_DATA)+chr(regstart)+chr(rlength)+chr(checksum)	# build a string with the first part
	s.write(outData)	# write it out of the serial port
	vals = list()	# build empty list
	direction(rx)	# configure for RX mode
	h1 = s.read()   # 0xff, not used for anything
	h2 = s.read()   # 0xff, not used for anything
	origin = s.read()   # Origin ID
	length = ord(s.read()) - 1	# Length of data payload
	error = ord(s.read())   # read the error data, was being tossed   
	while length > 0:		# read the data payload in a while loop
		vals.append(ord(s.read()))	# append the payload to the list
		length = length - 1			# decrement the counter

	if error != 0 :			# If the error is not 0 then bail with an error code
		e = "Error from servo: " + dictErrors[error] + ' (code  ' + hex(error) + ')'	# build an error string with the AX error code
		raise axError(e)	# raise the error

	
	if rlength == 1:	# If it's just a single byte then return it
		return vals[0]
	return vals			# It's more than one byte so return the list
	
def setposition(index, position) :
	setReg(index,30,((position%256),(position>>8)))  # this will move servo 1 to centered position (512)


if __name__ == '__main__' :		# Running as a standalone, loop and print temperature of servo 1
	while True :
		print "position", getReg(1, 36, 2)				# get the current position
		setposition(1, 0)
		print "temp:", getReg(1,43,1)               # get the temperature
		
		
		time.sleep(3)
		print "position", getReg(1, 36, 2)				# get the current position
		setposition(1, 1023)
		
		time.sleep(3)
