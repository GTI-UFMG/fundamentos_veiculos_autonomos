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
# Calibracao do receptor RC [us]
########################################

# Direcao
RC_DIR_MIN      = 1170
RC_DIR_CENTER   = 1490
RC_DIR_MAX      = 1820

# Acelerador / freio
RC_ACEL_MIN     = 1210
RC_ACEL_CENTER  = 1450
RC_ACEL_MAX     = 1780

# Zona morta do comando normalizado
# +/- 5% em torno do centro
RC_DEADZONE     = 0.05


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
	# normaliza um canal RC para [-1, +1]
	#
	# minimum -> -1
	# center  ->  0
	# maximum -> +1
	#
	# Aplica zona morta em torno de zero
	# e reescala o restante para manter
	# toda a faixa [-1, +1].
	########################################
	def normalize_rc(self, value, minimum, center, maximum):

		# Normalizacao em dois trechos
		if value <= center:
			u = (value - center) / (center - minimum)
		else:
			u = (value - center) / (maximum - center)

		# Limita a faixa
		u = float(np.clip(u, -1.0, 1.0))

		# Zona morta
		if abs(u) <= RC_DEADZONE:
			return 0.0

		# Reescala o restante para evitar salto
		# na saida da zona morta
		if u > 0.0:
			u = (u - RC_DEADZONE) / (1.0 - RC_DEADZONE)
		else:
			u = (u + RC_DEADZONE) / (1.0 - RC_DEADZONE)

		return float(np.clip(u, -1.0, 1.0))


	########################################
	# retorna comandos normalizados do radio
	#
	# retorno:
	# direcao, acelerador
	#
	# -1.0 = minimo
	#  0.0 = centro
	# +1.0 = maximo
	########################################
	def get_rc(self):

		with self.lock:
			rc_direcao = self.rc_direcao
			rc_acelerador = self.rc_acelerador

		direcao = self.normalize_rc(
			rc_direcao,
			RC_DIR_MIN,
			RC_DIR_CENTER,
			RC_DIR_MAX
		)

		acelerador = self.normalize_rc(
			rc_acelerador,
			RC_ACEL_MIN,
			RC_ACEL_CENTER,
			RC_ACEL_MAX
		)

		return direcao, acelerador


	########################################
	# retorna todos os dados crus
	# util para diagnostico/testes
	#
	# RC permanece em microssegundos aqui.
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
	print("RC normalizado: -1.0 a +1.0")
	print("Ctrl+C para sair")
	print()

	try:
		while True:

			(
				vel,
				rc_dir_raw,
				rc_acel_raw,
				sel_dir,
				sel_tracao,
				valid
			) = enc.get_data()

			# Valores normalizados
			rc_dir, rc_acel = enc.get_rc()

			modo_dir = "AUTO" if sel_dir else "RC"
			modo_tracao = "AUTO" if sel_tracao else "RC"
			status = "OK" if valid else "INVALIDA"

			print(
				f"Vel = {vel:.1f} m/s | "
				f"RC Dir = {rc_dir:+.2f} | " #({rc_dir_raw:4d} us) | "
				f"RC Acel = {rc_acel:+.2f} | " #({rc_acel_raw:4d} us) | "
				f"Direcao = {modo_dir:4s} | "
				f"Tracao = {modo_tracao:4s} | "
				f"{status}",
				flush=True
			)

			time.sleep(0.1)

	except KeyboardInterrupt:
		print("\nTeste encerrado.")

	finally:
		enc.close()
