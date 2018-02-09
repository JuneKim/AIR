#!/usr/bin/env python
# -*- coding : uft8 -*-

import serial
import sys
import socket
import time
import datetime
import struct
import timeit
import asyncore
import SocketServer
import threading

MSP_BASIC="\x24\x4d\x3c\x00"
MSP_IDT = MSP_BASIC + "\x64\x64"
MSP_STATUS = MSP_BASIC + "\x63\x63"
MSP_RAW_IMU = MSP_BASIC + "\x66\x66"
MSP_SERVO = MSP_BASIC + "\x67\x67"
MSP_MOTOR = MSP_BASIC + "\x68\x68"
MSP_RC = MSP_BASIC + "\x69\x69"
MSP_RAW_GPS = MSP_BASIC + "\x6A\x6A"
MSP_COMP_GPS = MSP_BASIC + "\x71\x71"
MSP_ATTITUDE = MSP_BASIC + "\x72\x72"
MSP_ALTITUDE = MSP_BASIC + "\x73\x73"
MSP_BAT = MSP_BASIC + "\x74\x74"
MSP_SET_RC = MSP_BASIC + "\xC8\xC8"

CURRENT = MSP_STATUS



class asulejr(object):
	def __init__(self, port):

		self.port = port
		self.ser = serial.Serial()
		self.ser.port = port
		self.ser.baudrate = 115200
		self.ser.bytesize = serial.EIGHTBITS
		self.ser.parity = serial.PARITY_NONE
		self.ser.stopbits = serial.STOPBITS_ONE
		self.ser.timeout = 0
		self.ser.xonxoff = False
		self.ser.rtscts = False
		self.ser.dsrdtr = False
		self.ser.wirteTimeout = 2

		try:
			self.ser.open()
		except Exception, e:
			print("Error to open serial port: " + str(e))
			exit()

		self.cmd2code = {
			'MSP_IDENT' : 100,
			'MSP_STATUS' : 101,
			'MSP_RAW_IMU' : 102,
			'MSP_SERVO' : 103,
			'MSP_MOTOR' : 104,
			'MSP_RC' : 105,
			'MSP_RAW_GPS' : 106,
			'MSP_COMP_GPS' : 107,
			'MSP_ATTITUDE' : 108,
			'MSP_ALTITUDE' : 109,
			'MSP_ANALOG' : 110,
			'MSP_RC_TUNNING' : 111,
			'MSP_PID' : 112,
			'MSP_BOX' : 113,
			'MSP_MISC' : 114,
			'MSP_MOTOR_PINS' : 115,
			'MSP_BOXNAMES' : 116,
			'MSP_PIDNAMES' : 117,
			'MSP_WP' : 118,
			'MSP_BOXIDS' : 119,

			'MSP_SET_RAW_RC' : 200,
			'MSP_SET_RAW_GPS' : 201,
			'MSP_SET_PID' : 202,
			'MSP_SET_BOX' : 203,
			'MSP_SET_RC_TUNNING' : 204,
			'MSP_ACC_CALIBRATION' : 205,
			'MSP_MAG_CALIBRATION' : 206,
			'MSP_SET_MISC' : 207,
			'MSP_RESET_CONF' : 208,
			'MSP_SET_WP' : 209,
			'MSP_SWITCH_RC_SERIAL' : 210,
			'MSP_IS_SERIAL' : 211,
			'MSP_DEBUG' : 254,
		}

		self.latitude = 0.0
		self.longitude = 0.0
		self.altitude = -0
		self.heading = -0
		self.timestamp = -0
		self.gpsString = -0
		self.numSats = -0
		self.accuracy = -1
		self.beginFlag = 0
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.pitch = 0
		self.yaw = 0
		self.thorttle = 0
		self.angx = 0.0
		self.angy = 0.0
		self.m1 = 0
		self.m2 = 0
		self.m3 = 0
		self.m4 = 0
		self.message = ""
		self.ax = 0
		self.ay = 0
		self.az = 0
		self.gx = 0
		self.gy = 0
		self.gz = 0
		self.magx = 0
		self.magy = 0
		self.magz = 0
		self.elapsed = 0
		self.flytime = 0
		self.numOfValues = 0
		self.precision = 3
		self.rcData = [1500, 1500, 1500, 1500]

		self.loopThread = threading.Thread(taret=self.loop)
		if self.ser.isOpen():
			time.sleep(5)
			self.loopThread.start()

	def stop(self):
		self.started = False

	def littleEndian(self, data):
		length = len(data)
		realData = ""
		for x in range(0, length/2):
			realData += data[length - 2 - (2*x) : length - (2*x)]
			x += 1
		intVal = self.twosComp(realData)
		return intVal
	
	def twosComp(self, hexValue):
		firstVal = int(hexValue[:1], 16)
		if firstVal >= 8:
			bValue = in(int(hexValue, 16))
			bValue = bValue[2:]
			newBinary = []
			length = len(bValue)
			index = bValue.rfind('1')
			for x in range(0, index + 1):
				if x == index:
					newBinary.append(bValue[index:])
				elif bValue[x:x+1] == '1':
					newBinary.append('0')
				elif bValue[x:x+1] == '0':
					newBinary.append('1')
				x += 1
			newBinary = ''.join(newBinary)
			finalVal = -int(newBinary, 2)
			return finalVal
		else:
			return int(hexValue, 16)

	def sendData(self, data_length, code, data):
		checksum = 0
		total_data = ['$', 'M', '<', data_length, code] + data
		for i in struct.pack('<2B%dh' % len(data), *total_data[3:len(total_data)]):
			checksum = checksum ^ ord(i)
		total_data.append(checksum)

		try:
			b = None
			b = self.ser.write(struct.pack('<3c2B%dhB' % len(data), *total_data))
		except Exception, ex:
			print 'send data error'
			print(ex)
		return b

	

	def start(self, port):
		if ser.isOpen():
			time.sleep(15)
			try:
				ser.flushInput()
				ser.flushOutput()
				ser.write(CURRENT)
				time.sleep(1)
				numOfLines=0
				while True:
					response = self.ser.readLine()

					print(response.encode("hex"))
					numOfLines += 1

					if (numOfLines > 0):
						break

				ser.close()



	def stop(self):

	def askRC(self):
		self.ser.flushInput()
		self.ser.falusOutput()
		self.ser.write(self.MSP_RC)
		response = self.ser.readline()
		if str(response) == "":
			return
		else:
			msp_hex = response.encode("hex")

			if msp_hex[10:14] = "":
				print("poll unavailble")
			else:
				self.roll = float(self.littleEndian(msp_hex[10:14]))

			if msp_hex[14:18] = "":
				print("pitch unavailale")
			else:
				self.pitch = float(self.littelEndian(msp_hex[14:18]))

			if msp_hex[18:22] == "":
				print("yaw unavailable")
			else:
				self.yaw = float(self.littleEndian(msp_hex[18:22]))

			if msp_hex[22:26] == "":
				print("throttle unavailable")
			else:
				self.throttle = float(self.llittelEndian(msp_hex[22:26]))

	def setRC(self):
		self.sendData(8, self.cmd2code[MSP_SET_RAW_RC], self.rcData)
		time.sleep(self.timeMSP)

	def loop(self):
		try:
			while self.started:
				if self.SET_RC:
					self.setRC()
				if self.ATT:
					self.askATT()
				if self.ALT:
					self.askALT()
				if self.RC:
					self.askRC()
				if self.MOT:
					self.askMOTOR()
				if self.RAW:
					self.askRAW()
				if self.SCK:
					self.getUDP()
			self.ser.close()
			file.close()
		except Exception, e1:
			print("Error on main: " + str(e1))

