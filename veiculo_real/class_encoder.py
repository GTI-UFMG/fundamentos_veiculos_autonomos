# -*- coding: utf-8 -*-
########################################
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2025/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import numpy as np
from serial.tools import list_ports
import serial

########################################
# Globais
BAUDRATE = 115200
TIMEOUT  = 0.2			# s (evita travar se nada chegar)
REDUCAO_EIXO = 7.80 	# motivo: motor -> roda
RAIO_RODA    = 0.08		# m (raio efetivo)

########################################
# detecta automaticamente a porta do arduino
def find_arduino():
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
# classe para ler velocidade do robo
########################################
class Encoder:
	########################################
	# construtor
	def __init__(self):
		
		# procura porta do arduino
		port = find_arduino()
		if not port:
			raise RuntimeError("Arduino Nano não encontrado!")
		print("Encoder conectando em", port)
		
		# abrindo porta serial
		self.ser = serial.Serial(port, BAUDRATE, timeout=TIMEOUT)
		
		# velocidade inicial nula
		self.vel = 0.0
		
	########################################
	# lê velocidade
	def getVel(self):
		
		# le a linha mais recente
		while self.ser.in_waiting:
			line = self.ser.readline()
		
		# recebe o RPM do eixo do motor pela serial (o mais recente)
		rpm = float(line.decode('utf-8').strip())
		
		if (not np.isnan(rpm)) and (not np.isinf(rpm)):
			# redução do eixo do motor para a roda
			rpm = rpm/REDUCAO_EIXO
		
			# converte velocidade de rpm para m/s
			self.vel = RAIO_RODA*(np.pi/30.0)*rpm
		
		return self.vel
	
	########################################
	# destrutor
	def __del__(self):
		try:
			if self.ser and self.ser.is_open:
				self.ser.close()
		except Exception:
			pass
			
########################################
# main test
########################################
if __name__=="__main__":
	
	import time
	
	# cria encoder
	enc = Encoder()
	
	# função para testar o encoder por 10s
	t0 = time.time()
	while (time.time() - t0) <= 10.0:
		print(f"Velocidade = {enc.getVel():.2f} m/s")
		time.sleep(0.1)
