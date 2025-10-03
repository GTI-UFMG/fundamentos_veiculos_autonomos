# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2025/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
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
TRIGGER_PIN = 24
ECHO_PIN = 8

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
		
		# ultima leitura valida
		self.measured = 0.0

		# Detectar a versao da Raspberry
		self.rpi_version = self.detect_rpi_version()
		print(f"[INFO] Detected Raspberry Pi {self.rpi_version}")

		# Raspberry Pi 5
		if self.rpi_version == 5:
			import lgpio as GPIO
			self.GPIO = GPIO
			self.handleChip = GPIO.gpiochip_open(0)
			self.triggerPin = triggerPin if triggerPin is not None else TRIGGER_PIN
			self.echoPin = echoPin if echoPin is not None else ECHO_PIN
			GPIO.gpio_claim_output(self.handleChip, self.triggerPin)
			GPIO.gpio_claim_input(self.handleChip, self.echoPin)
			
			# funcao de leitura pra raspberry pi 5
			self.read_func = lambda: self.GPIO.gpio_read(self.handleChip, self.echoPin)

		# Raspberry Pi 3/4
		else:
			import RPi.GPIO as GPIO
			self.GPIO = GPIO
			self.triggerPin = triggerPin if triggerPin is not None else TRIGGER_PIN
			self.echoPin = echoPin if echoPin is not None else ECHO_PIN
			GPIO.setwarnings(True)
			GPIO.setmode(GPIO.BCM)
			GPIO.setup(self.triggerPin, GPIO.OUT)
			GPIO.setup(self.echoPin, GPIO.IN)
			
			# funcao de leitura pra raspberry pi 4
			self.read_func = lambda: self.GPIO.input(self.echoPin)

	##############################################
	# Raspberry Pi version detector
	##############################################
	def detect_rpi_version(self):
		try:
			with open('/proc/device-tree/model') as f:
				return 5 if 'Raspberry Pi 5' in f.read() else 4
		except:
			return 4  # Default caso nao consiga detectar

	########################################
	# mede os pulsos com timeout
	########################################
	def _measure_pulse(self, level, timeout_s=0.03):
		"""Espera por level (0/1) com timeout; retorna (ok, t)."""
		deadline = time.time() + timeout_s
		while self.read_func() != level:
			if time.time() > deadline:
				return False, None
		# sucesso	
		return True, time.time()
        
	########################################
	# Funcao para medir distancia
	########################################
	def getDistance(self):
		
		# Raspberry Pi 5
		if self.rpi_version == 5:
			# set trigger to HIGH
			self.GPIO.gpio_write(self.handleChip, self.triggerPin, 1)
			time.sleep(0.00002)  # 20 us
			self.GPIO.gpio_write(self.handleChip, self.triggerPin, 0)

		# Raspberry Pi 3/4
		else:
			# set trigger to HIGH
			self.GPIO.output(self.triggerPin, True)
			time.sleep(0.00002)  # 20 us
			self.GPIO.output(self.triggerPin, False)

		# 1) esperar o inicio do eco (subir para 1). Timeout evita travar.
		ok1, startTime = self._measure_pulse(1)

		# 2) medir duracao do pulso alto (ate cair para 0). Outro timeout.
		ok2, stopTime = self._measure_pulse(0)
		
		# forca uma pequena espera para evitar erro de GPIO
		time.sleep(0.01)
		
		if (not ok1) or (not ok2):
			return self.measured #self.maxRange
		else:
			# time difference between start and arrival
			timeElapsed = stopTime - startTime
			
			# calculate distance (in meters)
			distance = GAIN * timeElapsed
			self.measured = np.clip(distance, self.minRange, self.maxRange)
			
			# Returns: distance (float): The measured distance in meters, constrained by min/max range
			return self.measured

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
	def close(self):
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
	m = 50

	# cria o ultrasom
	us = Ultrasonic()

	# testa leitura
	t0 = time.time()
	while (time.time() - t0) <= 20.0:
		
		dist.append(us.getDistance())
		ts.append(time.time() - t0)
		
		if len(dist) % 10 == 0:
			plt.clf()
			plt.plot(ts[-m:], dist[-m:], 'r')
			plt.xlabel('Time [s]')
			plt.ylabel('Distance [m]')
			plt.ylim([us.minRange-0.2, us.maxRange+0.2])
			plt.pause(0.1)
			plt.show()
		
		print(f"Distance: {dist[-1]:.2f} [m]", flush=True)
		
	plt.ioff()
	us.close()
