# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import serial
import numpy as np
import time

DEVICE = '/dev/ttyUSB0'
BAUDRATE = 57600

REDUCAO_EIXO = 7.8
RAIO_RODA = 0.08
MEAN_FILTER = 4

########################################
# classe para ler velocidade do robo
########################################
class Encoder:
	########################################
	# construtor
	def __init__(self):
		
		# abrindo porta serial
		self.ser = serial.Serial(DEVICE, BAUDRATE)
		
		# pega o primeiro valor de velocidade
		self.vel = 0.0
		self.vant = [0.0] * MEAN_FILTER
		for i in range(MEAN_FILTER):
			self.vel = self.getVel()
	
	########################################
	# filtro da media para o sinal de velocidade
	def filter(self, v):
		self.vant = self.vant[:-1]
		self.vant.insert(0, v)
		return sum(self.vant)/MEAN_FILTER
		
	########################################
	# lê velocidade
	def getVel(self):
		
		# apaga os dados que já chegaram para pegar o mais recente
		self.ser.flushInput()
		
		# recebe o RPM do eixo do motor pela serial
		rpm = float(self.ser.readline().decode('utf-8').strip())
		
		# redução do eixo do motor para a roda
		rpm = rpm/REDUCAO_EIXO
		
		# converte velocidade de rpm para m/s
		vel = RAIO_RODA*(np.pi/30.0)*rpm
		if not np.isnan(vel):
			self.vel = vel
		
		self.vel = self.filter(self.vel)
		return self.vel
	
	########################################
	# função para testar o encoder por 10s
	def test(self, total_time=10.0):
		ti = float(time.time())
		while (float(time.time()) - ti) <= total_time:
			print('Velocidade = %.1f m/s' % self.getVel())
	
	########################################
	# destrutor
	def __del__(self):
		# Feche a porta serial quando terminar
		self.ser.close()

########################################
#enc = Encoder()
#enc.test()
