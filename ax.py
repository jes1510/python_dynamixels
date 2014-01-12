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
disableDirPin = False

connectedServos = []

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
					
	if not disableDirPin :
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
	'''
	Synchronized write for multiple servos
	'''
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
	''' 
	Steps through the Servos in in the list a gets their positions.
	Returns a dictionary of positions with the servo ID's as keys
	'''
	pose = {}			# Empty dictionary to hold the positions
	
	for i in indexes :		
		raw = getReg(i, 36, 2)[0:2]			# read the register on the servo
		pose[i] = (raw[0] | (raw[1] << 8))	# add it to the dictionary
	return pose	

def getPose(indexes) :
	''' 
	Steps through the Servos in in the list a gets their positions.
	Returns a list of positions that correspond to the servos
	'''
	pose = []			# empty list to hold the pose
	for i in indexes :
		raw = getReg(i, 36, 2)[0:2]				# Read the current position
		pose.append(raw[0] | (raw[1] << 8))		# Append it to the list
	return pose		


def groupMove2(servoDict) :
	''' Move a group of servos to specified positions.  Uses Dictionary'''
	for i in servoDict.keys() :		# step through the keys
		if Arguments.verbose : print "Group Move2", i , "to Pos:", servoDict[i]	
		reg_write(i, 30, ((servoDict[i]%256, servoDict[i]>>8)))	 # Write the data to the servo, using reg_write
			
	action(BROADCASTID)		# tell all servos to move to the written positions
	
def groupMove(indexes, positions) :
	''' Move a group of servos to specified positions'''
	for i in range(0, len(indexes)):
		if Arguments.verbose : print "Group Move", indexes[i], "to Pos:", positions[i]	
		reg_write(indexes[i], 30, ((positions[i]%256, positions[i]>>8)))
			
	action(BROADCASTID)		# tell all servos to perform the action
	
def relax(indexes) :		
	'''
	Turn off toque for list of servos
	'''
	for i in indexes:	# Step through the servos
		if Arguments.verbose : print "Relaxing servo", i		
		setReg(i, 24, [0])  # Turn off torque
		
def setposition(index, position) :
	setReg(index,30,((position%256),(position>>8)))  # Moves servo to specified position

def learnServos(minValue=1, maxValue=32, timeout=0.25, verbose=False) :
	'''
	Step through the possible servos and ping them
	Add the found servos to a list and return it
	'''
	oldTimeout = port.timeout		# Save the original timeout
	port.timeout = timeout			# set timeout to something fast
	servoList = []					# Init an empty list
	for i in range(minValue, maxValue + 1) :	# loop through possible servos
		try :
			temp = ping(i)			# Request a pingt packet				
			servoList.append(i)		# No errors happened so we assume it's a good ID 
			if verbose: print "Found servo #" + str(i)
			time.sleep(.1)			# A wee bit of sleep to keep from pounding the bus
			
		except Exception, detail:	
			if verbose : print "Error pinging servo #" + str(i) + ': ' + str(detail)
			pass
			
	port.timeout = oldTimeout		# set the timeout to the original
	return servoList
	
def playPose() :
	'''
	Open a file and move the servos to specified positions in a group move
	'''
	infile=open(Arguments.playpose, 'r')	# Open the file 
	poseDict = {}							# Dictionary to hold poses and positions
	if Arguments.verbose : print "Reading pose from", Arguments.playpose
	for line in infile.readlines() :		# Read the file and step through it
		servo = int(line.split(':')[0])		# Servo is first
		position = int(line.split(':')[1])	# Position is second
		poseDict[servo]=position			# add the servo to the Dictionary

	groupMove2(poseDict)
		
	

def writePose() :
	'''
	Read the servos and save the positions to a file
	'''	
	of = open(Arguments.savepose, 'w')		# open the output file
	pose = getPose2(connectedServos)		# get the positions
	if Arguments.verbose : 
		print "Servo Positions"
		print "---------------"
	for key in  pose.keys():				# step through the keys, writing to the file
		if Arguments.verbose : print "Servo " + str(key), pose[key]
		of.write(str(key) + ':' + str(pose[key]) + '\n')	# Write to the file
	
	if Arguments.verbose :
		print "Wrote pose to " + Arguments.savepose
		print 	
	
	of.close()		# close the file
	
def processArgs() :
	'''
	Process the arguments after parsing
	'''
	global connectedServos
	
	if Arguments.port :			# specify a different serial port
		port.port = Arguments.port	
		
	if Arguments.nodir :			# allow the DIR pin to be disabled		
		global disableDirPin
		disableDirPin = True
		
	if Arguments.baud :
		port.baudrate = int(Arguments.baud)
	
	if Arguments.timeout :
		port.timeout = int(Arguments.timeout)
	
	if Arguments.learn :		# Learn the connected servos	
		connectedServos = learnServos(int(Arguments.servomin), int(Arguments.servomax), verbose=Arguments.verbose)
	
	if Arguments.servos :		# Servos were specified in arguments
		connectedServos = map(int, Arguments.servos.split(','))		
		
	if Arguments.savepose :		# Write the current pose to a file
		writePose()	
		
	if Arguments.playpose :		# Play an old pose from a file
		playPose()
	
	if Arguments.relax :		# Relax all the servos (for posing)
		relax(connectedServos) 
		

		
	
	
def parseArgs() :
	'''
	Parse the command line arguments
	'''
	parser = argparse.ArgumentParser(description="Rudementary command parser for manipulating AX12 servos")
	parser.add_argument('--servomin', default=0, action='store', help='Specify minimum servo')
	parser.add_argument('--servomax', default=32, action='store', help='Specify maximum servo')
	parser.add_argument('--learn', 	action='store_true', help='Automatically discover attached servos')
	parser.add_argument('--verbose', action='store_true', help='Show output in verbose mode')
	parser.add_argument('--savepose', action='store', help='Read the servos and save the positions to a file')
	parser.add_argument('--servos', action='store', help='Specify particular servos.  Lists must be comma seperated with no spaces')
	parser.add_argument('--playpose', action='store', help='Move the servos to positions specified in pose')
	parser.add_argument('--relax', action='store_true', help='Relax the servos')
	parser.add_argument('--nodir', action='store_true', help='Disable use of direction pin')
	parser.add_argument('--port', action='store', help="Specify different serial port ie '/dev/ttyACM0")
	parser.add_argument('--baud', action='store', help="Specify baud rate for the serial port")
	parser.add_argument('--timeout', action='store', help="Specify timeout for the serial port")	
	parser.parse_args(namespace=Arguments)	

		
	
	
class Arguments(object) :	# dummy class for command line arguments
	pass
	
if __name__ == '__main__' :		# Running as a standalone, loop and print temperature of servo 1
	import argparse	
	parseArgs()	
	processArgs()
	
	
