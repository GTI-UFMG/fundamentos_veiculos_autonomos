# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075):
# Fundamentos de Veiculos Autonomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import numpy as np
from serial.tools import list_ports
import serial
import threading
import time

########################################
# Globais
########################################
BAUDRATE        = 115200
TIMEOUT         = 0.2
REDUCAO_EIXO    = 7.80
RAIO_RODA       = 0.08
SENSOR_TIMEOUT  = 0.30


########################################
# Classe para leitura do Arduino Nano
#
# A thread consome continuamente a serial e
# armazena os ultimos valores recebidos.
#
# Formato esperado:
# RPM,RC_DIRECAO,RC_ACELERADOR,SEL_DIRECAO,SEL_TRACAO
#
# Selecao:
# 0 = RC
# 1 = AUTO
########################################
class Encoder:

	########################################
	# construtor
	########################################
	def __init__(self):

		port = self.find_arduino()
		if not port:
			raise RuntimeError("Encoder (arduino nano) nao encontrado!")

		self.ser = serial.Serial(port, BAUDRATE, timeout=TIMEOUT)

		# Aguarda reinicializacao do Nano ao abrir a serial
		time.sleep(1.5)
		self.ser.reset_input_buffer()

		# Ultimos valores validos recebidos
		self.vel = 0.0
		self.rc_direcao = 0
		self.rc_acelerador = 0
		self.sel_direcao = 0
		self.sel_tracao = 0

		# Estado da comunicacao
		self.valid = False
		self.last_measurement = 0.0

		# Sincronizacao entre thread e getters
		self.lock = threading.Lock()

		# Controle da thread
		self.running = True
		self.thread = threading.Thread(
			target=self._serial_loop,
			daemon=True
		)
		self.thread.start()


	########################################
	# thread de leitura da serial
	########################################
	def _serial_loop(self):

		while self.running:

			try:
				line = self.ser.readline()

			except (OSError, serial.SerialException):
				with self.lock:
					self.valid = False
				break

			# Timeout normal da serial: nenhuma linha chegou
			if not line:
				with self.lock:
					if (
						self.last_measurement == 0.0 or
						(time.monotonic() - self.last_measurement) > SENSOR_TIMEOUT
					):
						self.valid = False
				continue

			try:
				texto = line.decode("utf-8").strip()
				campos = texto.split(",")

				# RPM,RC_DIR,RC_ACEL,SEL_DIR,SEL_TRACAO
				if len(campos) != 5:
					continue

				rpm_motor = float(campos[0])
				rc_direcao = int(campos[1])
				rc_acelerador = int(campos[2])
				sel_direcao = int(campos[3])
				sel_tracao = int(campos[4])

				if not np.isfinite(rpm_motor):
					continue

				# Garante que os seletores tenham estados validos
				if sel_direcao not in (0, 1) or sel_tracao not in (0, 1):
					continue

				# RPM do motor -> RPM da roda
				rpm_roda = rpm_motor / REDUCAO_EIXO

				# RPM da roda -> velocidade linear [m/s]
				vel = RAIO_RODA * (np.pi / 30.0) * rpm_roda

				# Atualiza o conjunto inteiro de forma atomica
				with self.lock:
					self.vel = vel
					self.rc_direcao = rc_direcao
					self.rc_acelerador = rc_acelerador
					self.sel_direcao = sel_direcao
					self.sel_tracao = sel_tracao
					self.last_measurement = time.monotonic()
					self.valid = True

			except (ValueError, UnicodeDecodeError):
				# Ignora linha incompleta/corrompida e preserva
				# o ultimo conjunto valido recebido
				continue


	########################################
	# retorna velocidade mais recente
	########################################
	def get_vel(self):

		with self.lock:
			vel = self.vel
			valid = self.valid

			if (
				self.last_measurement == 0.0 or
				(time.monotonic() - self.last_measurement) > SENSOR_TIMEOUT
			):
				valid = False

		return vel, valid


	########################################
	# retorna modo de controle
	#
	# retorno:
	# sel_direcao, sel_tracao
	#
	# 0 = RC
	# 1 = AUTO
	########################################
	def get_control_mode(self):

		with self.lock:
			return self.sel_direcao, self.sel_tracao


	########################################
	# retorna comandos mais recentes do radio
	#
	# valores em microssegundos
	########################################
	def get_rc(self):

		with self.lock:
			return self.rc_direcao, self.rc_acelerador


	########################################
	# retorna todos os dados
	# util para diagnostico/testes
	########################################
	def get_data(self):

		with self.lock:
			vel = self.vel
			rc_direcao = self.rc_direcao
			rc_acelerador = self.rc_acelerador
			sel_direcao = self.sel_direcao
			sel_tracao = self.sel_tracao
			valid = self.valid

			if (
				self.last_measurement == 0.0 or
				(time.monotonic() - self.last_measurement) > SENSOR_TIMEOUT
			):
				valid = False

		return (
			vel,
			rc_direcao,
			rc_acelerador,
			sel_direcao,
			sel_tracao,
			valid
		)


	########################################
	# detecta automaticamente a porta
	########################################
	def find_arduino(self):

		for p in list_ports.comports():

			desc = p.description.lower()
			hwid = p.hwid.lower()

			if (
				"arduino" in desc or
				"wch" in desc or
				"1a86" in hwid or
				"ftdi" in desc or
				"0403" in hwid
			):
				return p.device

		return None


	########################################
	# fecha comunicacao serial e thread
	########################################
	def close(self):

		self.running = False

		try:
			if self.thread.is_alive():
				self.thread.join(timeout=TIMEOUT + 0.1)
		except RuntimeError:
			pass

		try:
			if self.ser and self.ser.is_open:
				self.ser.close()
		except (OSError, serial.SerialException):
			pass


########################################
# main test
########################################
if __name__ == "__main__":

	enc = Encoder()

	print()
	print("Teste: odometria + receptor RC + chaves")
	print("Chaves: 0 = RC | 1 = AUTO")
	print("Ctrl+C para sair")
	print()

	try:
		while True:

			(
				vel,
				rc_dir,
				rc_acel,
				sel_dir,
				sel_tracao,
				valid
			) = enc.get_data()

			modo_dir = "AUTO" if sel_dir else "RC"
			modo_tracao = "AUTO" if sel_tracao else "RC"
			status = "OK" if valid else "INVALIDA"

			print(
				f"Vel = {vel:7.3f} m/s | "
				f"RC Dir = {rc_dir:4d} us | "
				f"RC Acel = {rc_acel:4d} us | "
				f"Direcao = {modo_dir:4s} | "
				f"Tracao = {modo_tracao:4s} | "
				f"{status}"
			)

			time.sleep(0.1)

	except KeyboardInterrupt:
		print("\nTeste encerrado.")

	finally:
		enc.close()
