'''
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more detailport.

You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/

Use it at your own risk. Assume that I have no idea what I am doing.

Low level script to control AX 12 servos from Robotiport.  Based on original script
by Michael Ferguson.  Found here:
http://forumport.trossenroboticport.com/tutorials/how-to-diy-128/controlling-ax-12-servos-3275/
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
AX_SYNC_WRITE = 0x83
AX_RESET = 6
AX_ACTION = 5
AX_REG_WRITE = 4
AX_WRITE_DATA = 3
AX_READ_DATA = 2			# Corrected to 02.  Was 04
AX_PING = 1
BROADCASTID = 0xFE

port = serial.Serial()         # create a serial port object
port.baudrate = 1000000        # baud rate, in bits/second
port.port = "/dev/ttyAMA0"     # this is whatever port your are using
port.timeout = 0.5
port.open()

GPIO.setwarnings(False)
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
class axError(Exception) :	pass	
	
# Unspecified Error
class generalError(Exception) :	pass	
	
# Servo timeout
class timeoutError(Exception) :	pass
	

def direction(d):	
	'''	Set direction of data.  Either rx or tx'''						

	GPIO.output(directionPin, d)	# set the pin
	#time.sleep(0.000050)	# sleep for 50uS to allow things to settle.  Decreases checksum errors
	time.sleep(0.0005)

	
def ping(index) :
	direction(tx)		# Set to TX mode
	port.flushInput() 	  	# flush any garbage in the buffer
	checksum = 255 - ((index + AX_PING + 2)%256)	# calculate the checksum	
	outData = chr(0xFF)+chr(0xFF)+chr(index)+chr(0x02)+chr(AX_PING)+chr(checksum)	# build a string with the first part
	port.write(outData)	# write it out of the serial port	
	getStatus(index)
	
def getStatus(index) :
	vals = list()	# build empty list
	direction(rx)	# configure for RX mode
	try :
		h1 = port.read()   # 0xff, used to make sure the servo responds	
		assert ord(h1) == 0xFF # Make sure the header byte is right
	
	except :
		e = "Timeout on servo " + str(index)
		raise timeoutError(e)	# raise a timeout error
	
	try :
		h2 = port.read()   # 0xff, not used for anything
		origin = port.read()   # Origin ID
		length = ord(port.read()) - 1	# Length of data payload
		error = ord(port.read())   # read the error data, was being tossed   
		while length > 0:		# read the data payload in a while loop
			vals.append(ord(port.read()))	# append the payload to the list
			length = length - 1			# decrement the counter

		if error != 0 :			# If the error is not 0 then bail with an error code
			e = "Error from servo: " + dictErrors[error] + ' (code  ' + hex(error) + ')'	# build an error string with the AX error code
			raise axError(e)	# raise the error

		
		#if rlength == 1:	# If it's just a single byte then return it
		#	return vals[0]
		return vals			# It's more than one byte so return the list
		
	except Exception, detail:
		raise axError(detail)

def reset(index) :
	'''
	Reset servo to factory default settings.
	THIS WILL DESTROY ALL SETTINGS ON THE SERVO!!!!
	'''
	direction(tx)				# set the direction to be transmit
	length = 2	# configure length
	checksum = 255-((index+length+AX_RESET)%256)    # calculate checksum, same as ~(sum(data))  
	port.write(chr(0xFF)+chr(0xFF)+chr(index)+chr(length)+chr(AX_RESET))	# Write the first part of the protocol
	port.write(chr(checksum))	# write the checksum
	direction(rx)			# Switch back to RX mode		

def sync_write(indexes, reg, values):
	direction(tx)
	length = (len(values[0]) + 1) * len(indexes) + 4
	indexSum = sum(indexes)	
	valuesSum = 0
	for i in range(0, len(values)) :
		valuesSum += sum(values[i])		
		
	checksum = 255-((indexSum+length+AX_SYNC_WRITE+reg+valuesSum)%256)    # calculate checksum, same as ~(sum(data))  
	port.write((chr(0xFF)+chr(0xFF)+chr(BROADCASTID)+chr(length)+chr(AX_REG_WRITE)+chr(reg) + chr(len(values[0]))))

	for i in range(0, len(indexes)):	
		port.write(chr(indexes[i]))
		for val in values[i] :
			port.write(chr(val))
			
	port.write(chr(checksum))


def reg_write(index, reg, values) :
	''' Set register values but don't act on them'''
	direction(tx)				# set the direction to be transmit
	length = 3 + len(values)	# configure length
	checksum = 255-((index+length+AX_REG_WRITE+reg+sum(values))%256)    # calculate checksum, same as ~(sum(data))  
	port.write(chr(0xFF)+chr(0xFF)+chr(index)+chr(length)+chr(AX_REG_WRITE)+chr(reg))	# Write the first part of the protocol
	for val in values:		# actually writes the data payload
		port.write(chr(val))	# chr(val) sends the binary data rather than ASCII
	port.write(chr(checksum))	# write the checksum
	direction(rx)			# Switch back to RX mode
	
def action(index) :
	''' Act on set values'''
	direction(tx)				# set the direction to be transmit
	length = 2			# configure length
	checksum = 255-((index+length+AX_ACTION)%256)    # calculate checksum, same as ~(sum(data))  
	port.write(chr(0xFF)+chr(0xFF)+chr(index)+chr(length)+chr(AX_ACTION))	# Write the first part of the protocol	
	port.write(chr(checksum))	# write the checksum
	direction(rx)			# Switch back to RX mode
	
def setReg(index, reg,values):		# Corrected first arg to 'index'
	''' Set register values'''
	direction(tx)				# set the direction to be transmit
	length = 3 + len(values)	# configure length
	checksum = 255-((index+length+AX_WRITE_DATA+reg+sum(values))%256)    # calculate checksum, same as ~(sum(data))  
	port.write(chr(0xFF)+chr(0xFF)+chr(index)+chr(length)+chr(AX_WRITE_DATA)+chr(reg))	# Write the first part of the protocol
	for val in values:		# actually writes the data payload
		port.write(chr(val))	# chr(val) sends the binary data rather than ASCII
	port.write(chr(checksum))	# write the checksum
	direction(rx)			# Switch back to RX mode

def getReg(index, regstart, rlength):
	direction(tx)		# Set to TX mode
	port.flushInput() 	  	# flush any garbage in the buffer
	checksum = 255 - ((6 + index + regstart + rlength)%256)	# calculate the checksum	
	outData = chr(0xFF)+chr(0xFF)+chr(index)+chr(0x04)+chr(AX_READ_DATA)+chr(regstart)+chr(rlength)+chr(checksum)	# build a string with the first part
	port.write(outData)	# write it out of the serial port
	vals = list()	# build empty list
	direction(rx)	# configure for RX mode
	
	try :
		h1 = port.read()   # 0xff, used to make sure the servo responds	
		assert ord(h1) == 0xFF # Make sure the header byte is right
	
	except :
		e = "Timeout on servo " + str(index)
		raise timeoutError(e)	# raise a timeout error
	
	try :
		h2 = port.read()   # 0xff, not used for anything
		origin = port.read()   # Origin ID
		length = ord(port.read()) - 1	# Length of data payload
		error = ord(port.read())   # read the error data, was being tossed   
		while length > 0:		# read the data payload in a while loop
			vals.append(ord(port.read()))	# append the payload to the list
			length = length - 1			# decrement the counter

		if error != 0 :			# If the error is not 0 then bail with an error code
			e = "Error from servo: " + dictErrors[error] + ' (code  ' + hex(error) + ')'	# build an error string with the AX error code
			raise axError(e)	# raise the error

		
		if rlength == 1:	# If it's just a single byte then return it
			return vals[0]
		return vals			# It's more than one byte so return the list
		
	except Exception, detail:
		raise axError(detail)
		
def getPose2(indexes) :
	''' Steps through the Servos in in the list a gets their positions.
	Returns a dictionary of positions with the servo ID's as keys
	'''
	pose = {}
	for i in indexes :
		raw = getReg(i, 36, 2)[0:2]		
		pose[i] = (raw[0] | (raw[1] << 8))
	return pose

def getPose(indexes) :
	''' Steps through the Servos in in the list a gets their positions.
	Returns a list of positions that correspond to the servos'''
	pose = []
	for i in indexes :
		raw = getReg(i, 36, 2)[0:2]		
		pose.append(raw[0] | (raw[1] << 8))
	return pose
			
def setPositionDelayed(index, position) :
	reg_write(index, ((position%256, position>>8)))

def groupMove2(servoDict) :
	''' Move a group of servos to specified positions.  Uses Dictionary'''
	for i in servoDict.keys() :
		reg_write(i, 30, ((servoDict[i]%256, servoDict[i]>>8)))	
			
	action(BROADCASTID)	
	
def groupMove(indexes, positions) :
	''' Move a group of servos to specified positions'''
	for i in range(0, len(indexes)):	
		reg_write(indexes[i], 30, ((positions[i]%256, positions[i]>>8)))
			
	action(BROADCASTID)
	
def setposition(index, position) :
	setReg(index,30,((position%256),(position>>8)))  # Moves servo to specified position

def learnServos(minValue=1, maxValue=32, timeout=0.25, verbose=False) :
	oldTimeout = port.timeout	# Save the original timeout
	port.timeout = timeout		# set timeout to something fast
	servoList = []					# Init an empty list
	for i in range(minValue, maxValue + 1) :	# loop through possible servos
		try :
			temp = ping(i)	
			time.sleep(.1)		
			servoList.append(i)					# It no errors happened so we assume it's a good ID 
			if verbose: print "Found servo #" + str(i)
			
		except Exception, detail:	
			if verbose : print "Error testing #" + str(i) + ': ' + str(detail)
			pass
			
	port.timeout = oldTimeout
	return servoList

if __name__ == '__main__' :		# Running as a standalone, loop and print temperature of servo 1
	while True :
		print "position", getReg(1, 36, 2)				# get the current position
		setposition(1, 0)
		print "temp:", getReg(1,43,1)               # get the temperature
		
		
		time.sleep(3)
		print "position", getReg(1, 36, 2)				# get the current position
		setposition(1, 1023)
		
		time.sleep(3)
