#!/usr/bin/env python
# -*- coding: utf8 -*-

from __future__ import unicode_literals

import logging
import os.path
import uuid
import signal
import sys
import threading
import json
import encodings
import time
import serial
import RPi.GPIO as GPIO
import picamera

import multiwii

class Main():
	def start(self, board, camera):
		self.board = board
		self.camera = camera
	
	def stop(self):
		self.board.stop()
	
if __name__ == "__main__":
	global board
	board = multiwii.asulejr('/dev/ttyUSB0')
	cameara = picamera.PiCamera()
	camera.vflip = True
	camera.hflip = True

	start = Main()
	MainThread = threading.Thread(target=start.start, args=(board, camera))
	MainThread.start()

	def signal_handler(signal, frame):
		GPIO.cleanup()
		start.stop()
		sys.exit(0)
	
	signal.signal(signal.SIGINT, signal_handler)
	signal.pause()

