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
import time

########################################
# Globais
########################################
BAUDRATE 		= 115200
TIMEOUT  		= 0.2		# s (evita travar se nada chegar)
REDUCAO_EIXO 	= 7.80 		# motivo: motor -> roda
RAIO_RODA    	= 0.08		# m (raio efetivo)
SENSOR_TIMEOUT  = 0.30   	# tempo maximo sem medida [s]

########################################
# classe para ler velocidade do robo
########################################
class Encoder:
	########################################
	# construtor
	def __init__(self):
		
		# procura porta do arduino
		port = self.find_arduino()
		if not port:
			raise RuntimeError("Encoder (arduino nano) nao encontrado!")
		print("\033[32mEncoder conectado em: %s\033[0m" % port)
		
		# abrindo porta serial
		self.ser = serial.Serial(port, BAUDRATE, timeout=TIMEOUT)
		time.sleep(1.5)
		self.ser.reset_input_buffer()
		
		# velocidade inicial nula
		self.vel = 0.0
		self.valid = True
		
		# Instante da ultima medida valida
		self.last_measurement = time.monotonic()
		
	########################################
	# le velocidade
	def get_vel(self):
		
		try:
			# Le todas as mensagens disponíveis, mantendo a mais recente
			line = None
			while self.ser.in_waiting:
				nova_linha = self.ser.readline()
				if nova_linha:
					line = nova_linha
		except (OSError, serial.SerialException):
			self.valid = False
			return self.vel, self.valid
		
		# --------------------------------
		# Houve uma nova medida
		# --------------------------------
		if line is not None:
			try:
				rpm_motor = float(line.decode('utf-8').strip())
						
				if np.isfinite(rpm_motor):
					# RPM do motor -> RPM da roda
					rpm_roda = rpm_motor / REDUCAO_EIXO

					# RPM da roda -> velocidade linear [m/s]
					self.vel = (RAIO_RODA *	(np.pi/30.0) * rpm_roda)
					
					# atualiza timestamp de medicao valida
					self.last_measurement = (time.monotonic())
					self.valid = True
					
			except (ValueError, UnicodeDecodeError):
				pass
							
		# --------------------------------
		# Verifica perda de comunicação
		# --------------------------------
		if (time.monotonic() - self.last_measurement) > SENSOR_TIMEOUT:
			self.valid = False
			
		return self.vel, self.valid
	
	########################################
	# detecta automaticamente a porta do arduino
	def find_arduino(self):
		"""
		Procura um Arduino Nano na lista de portas seriais.
		Retorna o device (ex.: '/dev/ttyUSB0') ou None.
		"""
		for p in list_ports.comports():
			desc = p.description.lower()
			hwid = p.hwid.lower()
			# palavras que costumam aparecer em Nanos
			if ('arduino' in desc or 'wch' in desc or '1a86' in hwid or
				'ftdi' in desc or '0403' in hwid):
				return p.device
		return None
	
	########################################
	# fecha comunicacao serial
	def close(self):
		try:
			if self.ser and self.ser.is_open:
				self.ser.close()
		except (OSError, serial.SerialException):
			pass
			
########################################
# main test
########################################
if __name__=="__main__":
	
	# cria encoder
	enc = Encoder()
	
	# funcao para testar o encoder por 15s
	t0 = time.monotonic()
	while (time.monotonic() - t0) <= 15.0:
		vel, valid = enc.get_vel()

		if valid:
			print(f"Vel = {vel:.1f} m/s")
		else:
			print(f"Vel = {vel:.1f} m/s \033[31m[INVALIDA]\033[0m")
			
		time.sleep(0.1)
	
	# fecha odometro
	enc.close()
