# -*- coding: utf-8 -*-
########################################
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2025/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import time
import numpy as np

"""
GPIO mode options: BOARD/BCM
# Raspberry Pi 3/4: BOARD pin numbers
# Raspberry Pi 5:   BCM (Broadcom System-On-Chip (SOC)) pin numbers
# triggerPin = 18 (BOARD) and 24 (BCM)
#    echoPin = 24 (BOARD) and  8 (BCM)
"""

GAIN = 343.0/2.0

############################################
# Ultrasonic Class for Raspberry Pi 4 or 5
############################################
class Ultrasonic:
	########################################
	# Constructor
	########################################
	def __init__(self, triggerPin=None, echoPin=None, minRange=0.0, maxRange=4.0):
		
		"""
		Initializes the ultrasonic sensor with given trigger and echo pins.
		Configures GPIO settings and set sensor limits

		Args:
			triggerPin (int): GPIO pin number for the trigger signal.
			echoPin (int): GPIO pin number for the echo signal.
			minRange (float): Minimum measurable distance in meters.
			maxRange (float): Maximum measurable distance in meters.
		"""
		
		# limites do sensor
		self.minRange = minRange
		self.maxRange = maxRange

		# Detectar a versao da Raspberry
		self.rpi_version = self.detect_rpi_version()
		print(f"[INFO] Detected Raspberry Pi {self.rpi_version}")

		# Raspberry Pi 5
		if self.rpi_version == 5:
			import lgpio as GPIO
			self.GPIO = GPIO
			self.handleChip = GPIO.gpiochip_open(0)
			self.triggerPin = triggerPin if triggerPin is not None else 24
			self.echoPin = echoPin if echoPin is not None else 8
			GPIO.gpio_claim_output(self.handleChip, self.triggerPin)
			GPIO.gpio_claim_input(self.handleChip, self.echoPin)

		# Raspberry Pi 3/4
		else:
			import RPi.GPIO as GPIO
			self.GPIO = GPIO
			self.triggerPin = triggerPin if triggerPin is not None else 18
			self.echoPin = echoPin if echoPin is not None else 24
			GPIO.setwarnings(False)
			GPIO.setmode(GPIO.BOARD)
			GPIO.setup(self.triggerPin, GPIO.OUT)
			GPIO.setup(self.echoPin, GPIO.IN)

	##############################################
	# Raspberry Pi version detector
	##############################################
	def detect_rpi_version(self):
		try:
			with open('/proc/device-tree/model') as f:
				model = f.read()
				if 'Raspberry Pi 5' in model:
					return 5
				else:
					return 4
		except:
			return 4  # Default caso nÃ£o consiga detectar

	########################################
	# Funcao para medir distancia
	########################################
	def getDistance(self):
		
		# Raspberry Pi 5
		if self.rpi_version == 5:
			# set trigger to HIGH
			self.GPIO.gpio_write(self.handleChip, self.triggerPin, 1)
			time.sleep(0.0001)
			self.GPIO.gpio_write(self.handleChip, self.triggerPin, 0)

			# save startTime
			startTime = time.time()
			while self.GPIO.gpio_read(self.handleChip, self.echoPin) == 0:
				startTime = time.time()

			# save time of arrival
			stopTime = time.time()
			while self.GPIO.gpio_read(self.handleChip, self.echoPin) == 1:
				stopTime = time.time()

		# Raspberry Pi 3/4
		else:
			# set trigger to HIGH
			self.GPIO.output(self.triggerPin, True)
			time.sleep(0.0001)
			self.GPIO.output(self.triggerPin, False)

			# save startTime
			startTime = time.time()
			while self.GPIO.input(self.echoPin) == 0:
				startTime = time.time()

			# save time of arrival
			stopTime = time.time()
			while self.GPIO.input(self.echoPin) == 1:
				stopTime = time.time()

		# time difference between start and arrival
		timeElapsed = stopTime - startTime
		
		# calculate distance (in meters)
		distance = GAIN * timeElapsed

		# Returns: distance (float): The measured distance in meters, constrained by min/max range
		return self.saturate(distance)

	########################################
	# Saturacao dos valores
	########################################
	def saturate(self, distance):
		return np.clip(distance, self.minRange, self.maxRange)			

	########################################
	# Limpeza dos pinos
	########################################
	def cleanup(self):
		# Closes the GPIO connection.
		if self.rpi_version == 5:
			self.GPIO.gpiochip_close(self.handleChip)
		else:
			self.GPIO.cleanup()

	########################################
	# Destrutor
	########################################
	def __del__(self):
		#Ensure cleanup is called when object is deleted
		self.cleanup()

################################################################################
# Teste rapido
if __name__ == '__main__':
	
	import matplotlib.pyplot as plt
	plt.ion()
	fig = plt.figure(1)

	ts = []
	dist = []
	m = 20

	# cria o ultrasom
	us = Ultrasonic()

	# testa leitura
	t0 = time.time()
	while (time.time() - t0) <= 20.0:
		
		dist.append(us.getDistance())
		ts.append(time.time() - t0)
		
		plt.clf()
		plt.plot(ts[-m:], dist[-m:], 'r')
		plt.xlabel('Time [s]')
		plt.ylabel('Distance [m]')
		plt.ylim([us.minRange, us.maxRange])
		plt.pause(0.1)
		plt.show()	
		
		print(f"Distance: {dist[-1]:.2f} [m]")
		
	plt.ioff()
