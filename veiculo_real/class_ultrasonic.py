# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import time
import numpy as np
import threading
import re
import subprocess

########################################
# Globais
########################################
"""
GPIO mode options: BOARD/BCM
# Raspberry Pi 3/4: BOARD pin numbers
# Raspberry Pi 5:   BCM (Broadcom System-On-Chip (SOC)) pin numbers
# trigger_pin = 18 (BOARD) and 24 (BCM)
#    echo_pin = 24 (BOARD) and  8 (BCM)
"""
GAIN = 343.0/2.0
TRIGGER_PIN = 24
ECHO_PIN = 8
SAMPLE_TIME = 0.1  # 100 ms -> 10 Hz
SENSOR_TIMEOUT  = 0.30   	# tempo maximo sem medida [s]

############################################
# Ultrasonic Class for Raspberry Pi 4 or 5
############################################
class Ultrasonic:
	########################################
	# Constructor
	########################################
	def __init__(self, trigger_pin=None, echo_pin=None, min_range=0.0, max_range=4.0):
		
		"""
		Initializes the ultrasonic sensor with given trigger and echo pins.
		Configures GPIO settings and set sensor limits

		Args:
			trigger_pin (int): GPIO pin number for the trigger signal.
			echo_pin (int): GPIO pin number for the echo signal.
			min_range (float): Minimum measurable distance in meters.
			max_range (float): Maximum measurable distance in meters.
		"""
		
		# limites do sensor
		self.min_range = min_range
		self.max_range = max_range
		
		# ultima leitura valida
		self.dist = 0.0
		self.valid = False
		self.last_measurement = time.monotonic()

		# Detectar a versao da Raspberry
		self.rpi_version = self.detect_rpi_version()
		#print(f"[INFO] Detected Raspberry Pi {self.rpi_version}")

		# Raspberry Pi 5
		if self.rpi_version == 5:
			import lgpio as GPIO
			self.GPIO = GPIO
			# detecta o chip
			chip_id = self._find_gpiochip()
			self.handle_chip = self.GPIO.gpiochip_open(chip_id)
			self.trigger_pin = trigger_pin if trigger_pin is not None else TRIGGER_PIN
			self.echo_pin = echo_pin if echo_pin is not None else ECHO_PIN
			self.GPIO.gpio_claim_output(self.handle_chip, self.trigger_pin)
			self.GPIO.gpio_claim_input(self.handle_chip, self.echo_pin)
			
			# funcao de leitura pra raspberry pi 5
			self.read_func = lambda: self.GPIO.gpio_read(self.handle_chip, self.echo_pin)

		# Raspberry Pi 3/4
		else:
			import RPi.GPIO as GPIO
			self.GPIO = GPIO
			self.trigger_pin = trigger_pin if trigger_pin is not None else TRIGGER_PIN
			self.echo_pin = echo_pin if echo_pin is not None else ECHO_PIN
			self.GPIO.setwarnings(False)
			self.GPIO.setmode(GPIO.BCM)
			self.GPIO.setup(self.trigger_pin, GPIO.OUT)
			self.GPIO.setup(self.echo_pin, GPIO.IN)
			
			# funcao de leitura pra raspberry pi 4
			self.read_func = lambda: self.GPIO.input(self.echo_pin)
		
		# lock de secao critica
		self.lock = threading.Lock()
		self.stop = threading.Event()
		# thread de leitura
		self.thread = threading.Thread(target=self._read, daemon=True)
		self.thread.start()

	##############################################
	# Raspberry Pi version detector
	##############################################
	def detect_rpi_version(self):
		try:
			with open('/proc/device-tree/model') as f:
				return 5 if 'Raspberry Pi 5' in f.read() else 4
		except OSError:
			return 4  # Default caso nao consiga detectar
	
	##############################################
	# Detecta gpiochip
	##############################################
	def _find_gpiochip(self):
		try:
			out = subprocess.check_output(["gpiodetect"], text=True)
			for line in out.splitlines():
				if "pinctrl-rp1" in line:
					match = re.match(r"gpiochip(\d+)", line)

					if match:
						return int(match.group(1))

		except (OSError, subprocess.SubprocessError):
			pass

		raise RuntimeError("GPIO chip principal da Raspberry Pi 5 nao encontrado.")

	########################################
	# thread de leitura continua
	########################################
	def _read(self):
		# inicializacao
		time.sleep(0.1)

		# loop de leitura
		while not self.stop.is_set():
			d = self.get_measure()
			if d is not None:
				with self.lock:
					self.dist = d
					self.last_measurement = time.monotonic()
					self.valid = True

			self.stop.wait(SAMPLE_TIME)
	
	########################################
	# Funcao para medir distancia
	########################################			
	def get_distance(self):
		with self.lock:
			# medida eh valida?
			if (time.monotonic() - self.last_measurement) > SENSOR_TIMEOUT:
				self.valid = False

			return self.dist, self.valid
	
	########################################
	# escreve no pino de trigger
	def _set_trigger(self, state):
		# Raspberry Pi 5
		if self.rpi_version == 5:
			self.GPIO.gpio_write(self.handle_chip, self.trigger_pin, 1 if state else 0)
		# Raspberry Pi 3/4
		else:
			self.GPIO.output(self.trigger_pin, True if state else False)
			
	########################################
	# Funcao para medir distancia
	########################################
	def get_measure(self):

		# envia pulso de trigger
		self._set_trigger(True)
		time.sleep(0.00002)
		self._set_trigger(False)

		# espera inicio do eco
		ok1, start_time = self._measure_pulse(1)

		# mede duracao do eco
		ok2, stop_time = self._measure_pulse(0)

		# deu tudo certo?
		if not ok1 or not ok2:
			return None

		# calcula distancia
		time_elapsed = stop_time - start_time
		distance = GAIN * time_elapsed
		
		if not (self.min_range <= distance <= self.max_range):
			return None

		return distance

	########################################
	# mede os pulsos com timeout
	########################################
	def _measure_pulse(self, level, timeout_s=0.03):
		"""Espera por level (0/1) com timeout; retorna (ok, t)."""
		deadline = time.monotonic() + timeout_s
		while self.read_func() != level:
			if time.monotonic() > deadline:
				return False, None
		# sucesso	
		return True, time.monotonic()
		
	########################################
	# Limpeza dos pinos
	########################################
	def cleanup(self):
		# Closes the GPIO connection.
		if self.rpi_version == 5:
			self.GPIO.gpiochip_close(self.handle_chip)
		else:
			self.GPIO.cleanup(self.trigger_pin)
			self.GPIO.cleanup(self.echo_pin)

	########################################
	# Destrutor
	########################################
	def close(self):
		# termina a thread
		self.stop.set()
		self.thread.join()
		
		#Ensure cleanup is called when object is deleted
		self.cleanup()

########################################################################
# Teste rapido
########################################################################
if __name__ == '__main__':

	import matplotlib.pyplot as plt
	plt.ion()
	fig = plt.figure(figsize=(8, 4))

	# parametros do teste
	TEST_TIME = 30.0
	PLOT_INTERVAL = 2.0
	PLOT_WINDOW = 5.0

	# dados
	ts = []
	dist = []
	valids = []

	# cria o ultrasom
	us = Ultrasonic()

	try:
		t0 = time.monotonic()
		last_plot = t0

		while (time.monotonic() - t0) <= TEST_TIME:

			# tempo atual
			now = time.monotonic()
			t = now - t0

			# le sensor
			distance, valid = us.get_distance()

			# salva dados
			ts.append(t)
			dist.append(distance)
			valids.append(valid)

			# atualiza grafico a cada 5 segundos
			if (now - last_plot) >= PLOT_INTERVAL:

				# converte para arrays
				t_array = np.array(ts)
				d_array = np.array(dist)
				v_array = np.array(valids)

				# seleciona ultimos 10 segundos
				mask = t_array >= (t - PLOT_WINDOW)

				t_plot = t_array[mask]
				d_plot = d_array[mask]
				v_plot = v_array[mask]

				# apaga
				plt.clf()

				# sinal continuo
				plt.plot(t_plot, d_plot, 'b-', label='Distancia')

				# destaca medidas invalidas
				plt.scatter(t_plot[~v_plot], d_plot[~v_plot], color='red', label='Medida invalida', zorder=3)

				plt.xlabel('Time [s]')
				plt.ylabel('Distance [m]')

				plt.ylim([us.min_range - 0.2, us.max_range + 0.2])

				plt.grid()
				plt.legend()
				plt.show(block=False)
				plt.pause(0.1)

				last_plot = now

			time.sleep(SAMPLE_TIME)

	finally:
		print('Terminou...')
		us.close()
