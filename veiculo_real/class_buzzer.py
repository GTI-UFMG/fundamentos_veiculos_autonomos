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
import threading
import re
import subprocess

########################################
# Globais
########################################
BUZZER_PIN = 17

########################################
# classe para sons de indicacao
########################################
class Buzzer:
	########################################
	# construtor
	def __init__(self):
		
		# Detectar a versao da Raspberry
		self.rpi_version = self.detect_rpi_version()
		#print(f"[INFO] Detected Raspberry Pi {self.rpi_version}")
		
		# Raspberry Pi 5
		if self.rpi_version == 5:
			import lgpio as GPIO
			self.GPIO = GPIO
			# abre o chip GPIO existente
			chip_id = self._find_gpiochip()
			self.chip = self.GPIO.gpiochip_open(chip_id)
			# configura pino como saida, inicialmente desligado
			self.GPIO.gpio_claim_output(self.chip, BUZZER_PIN, 0)

		# Raspberry Pi 3/4
		else:
			import RPi.GPIO as GPIO
			self.GPIO = GPIO
			self.GPIO.setmode(GPIO.BCM)
			# configura pino como saida, inicialmente desligado
			self.GPIO.setup(BUZZER_PIN, GPIO.OUT, initial=GPIO.LOW)
		
		# garante apenas uma thread de beep por vez
		self.lock = threading.Lock()
		self.thread = None

		# indica que objeto ainda esta ativo
		self.closed = False
		
	##############################################
	# Raspberry Pi version detector
	def detect_rpi_version(self):
		try:
			with open('/proc/device-tree/model') as f:
				return 5 if 'Raspberry Pi 5' in f.read() else 4
		except OSError:
			return 4  # Default caso nao consiga detectar
	
	##############################################
	# detecta gpio
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
	# liga/desliga buzzer
	def _set_buzzer(self, state):
		# Raspberry Pi 5
		if self.rpi_version == 5:
			self.GPIO.gpio_write(self.chip, BUZZER_PIN, 1 if state else 0)

		# Raspberry Pi 3/4
		else:
			self.GPIO.output(BUZZER_PIN, self.GPIO.HIGH if state else self.GPIO.LOW)
	
	########################################
	# executa um beep
	def _beep(self, duration):
		try:
			self._set_buzzer(True)
			time.sleep(duration)

		finally:
			# garante buzzer desligado mesmo em caso de erro
			self._set_buzzer(False)
	
	########################################
	# dispara um beep
	def beep(self, duration=0.1, block=False):

		duration = max(0.0, float(duration))

		if self.closed:
			return False

		with self.lock:
			# se ja existe um beep em andamento, ignora a nova solicitacao
			if (self.thread is not None and self.thread.is_alive()):
				return False

			# cria uma nova thread apenas para este beep
			self.thread = threading.Thread(target=self._beep, args=(duration,), daemon=True)
			self.thread.start()
			thread = self.thread

		# opcionalmente espera o beep terminar
		if block:
			thread.join()

		return True
			
	########################################
	# musiquinha final
	def victory_tune(self):

		pattern = [0.1, 0.1, 0.1, 0.3, 0.1, 0.5]
		for duration in pattern:
			# block=True garante sequencia ordenada
			self.beep(duration=duration, block=True)
			# pequena pausa entre os beeps
			time.sleep(0.05)
			
	########################################
	# Limpeza dos pinos
	def cleanup(self):
		try:
			# garante buzzer desligado
			self._set_buzzer(False)
		except Exception:
			pass

		# Raspberry Pi 5
		if self.rpi_version == 5:
			try:
				self.GPIO.gpiochip_close(self.chip)
			except Exception:
				pass

		# Raspberry Pi 3/4
		else:
			try:
				self.GPIO.cleanup(BUZZER_PIN)
			except Exception:
				pass
			
	########################################
	# Desliga e fecha
	def close(self):
		if self.closed:
			return
		self.closed = True

		# espera eventual beep atual terminar
		thread = self.thread
		if (thread is not None and thread.is_alive()):
			thread.join()

		# garante desligamento do buzzer
		self.cleanup()

########################################
# main test
########################################
if __name__=="__main__":
	
	bz = Buzzer()

	try:
		# beep assincrono
		bz.beep(duration=0.3, block=False)
		time.sleep(1.0)

		# beep bloqueante
		bz.beep(duration=0.5, block=True)
		time.sleep(0.5)

		# sequencia final
		bz.victory_tune()

	except KeyboardInterrupt:
		pass

	finally:
		bz.close()
