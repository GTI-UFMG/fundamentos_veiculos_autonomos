# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import time
import numpy as np
import class_encoder
import class_servos
import class_camera

########################################
# GLOBAIS
########################################
# parametros do carro
CAR = {
		'VELMAX'	: 5.0,				# m/s
		'ACCELMAX'	: 0.25, 			# m/s^2
		'STEERMAX'	: np.deg2rad(20.0),	# deg
		'MASS'		: 6.35,				# kg
		'L'			: 0.36,				# distancia entre os eixos das rodas
	}

# parametro de filtragem
ALFA = 0.3

########################################
# Carrinho
########################################
class Car:	
	########################################
	# construtor
	def __init__(self):
		
		# tempo
		self.t = 0.0
		
		# tempo de amostragem
		self.dt = 0.0
		
		# velocidade de comando
		self.vref = 0.0
		
		# comando de aceleracao
		self.u = 0.0
		
		# comando de esterçamento
		self.st = 0.0
		
		# odometro da roda
		self.odometer = class_encoder.Encoder()
		
		# atuadores de esterçamento e aceleração
		self.atuador = class_servos.Servos()
		
		# camera
		self.cam = class_camera.Camera()
				
		print('Car ok!')
	
	########################################
	# get states
	def getStates(self):
		self.p = self.getPos()
		self.th = self.getYaw()
		self.v, self.w = self.getVel()
		self.t = self.getTime() - self.tinit
				
		return self.p, self.v, self.th, self.w, self.t
	
	########################################
	# comeca a missao
	def startMission(self):
		
		# tempo inicial
		self.tinit = self.getTime()
		
		# estados iniciais
		self.getStates()
		
		# comeca parado
		self.setU(0.0)
		self.setSteer(0.0)
		
		# salva trajetoria
		self.saveTraj()
		
	########################################
	# termina a missao
	def stopMission(self):	
		None
	
	########################################
	def step(self):
		
		# tempo anterior
		t0 = self.t
		
		# condicoes iniciais
		self.getStates()
		
		# atualiza amostragem
		self.dt = self.t - t0
		
		# salva trajetoria
		self.saveTraj()
		
	########################################
	# salva a trajetoria
	def saveTraj(self):
		
		# dados
		data = {	't'     : self.t, 
					'p'     : self.p, 
					'v'     : self.v,
					'vref'  : self.vref,
					'th'    : self.th,
					'w'     : self.w,
					'u'     : self.u}
				
		# se ja iniciou as trajetorias
		try:
			self.traj.append(data)
		# se for a primeira vez
		except:
			self.traj = [data]
			
	########################################
	# retorna tempo do sistema
	def getTime(self):
		return float(time.time())
					
	########################################
	# retorna posicao do carro - sem GPS
	def getPos(self):
		return np.array((0.0, 0.0))			
				
	########################################
	# retorna yaw - sem bussola
	def getYaw(self):
		return 0.0
		
	########################################
	# retorna velocidades linear e angular
	def getVel(self):
		
		# lê velocidade do encoder
		v = self.odometer.getVel()
		
		# sem IMU, calular artificialmente
		#w = 0.0
		w = v*np.tan(self.st)/CAR['L']

		return v, w
					
	########################################
	# seta torque do veiculo
	def setVel(self, vref):
		
		Kp = 0.7
		Kd = 0.1
		
		# referencia de velocidade
		self.vref = np.clip(vref, 0.0, CAR['VELMAX'])
		
		# controle de velocidade
		u = Kp*(self.vref - self.v) + Kd*(0.0 - self.u)
		self.setU(u)
				
	########################################
	# seta torque dos motores do veiculo
	def setU(self, u):
		
		# limita aceleracao
		self.u = np.clip(u, -CAR['ACCELMAX'], CAR['ACCELMAX'])
		
		# seta aceleracao (normalizada entre -1 e 1)
		self.atuador.setU(self.u/CAR['ACCELMAX'])

	########################################
	# seta steer do veiculo
	def setSteer(self, st):	
		
		# limita angulo de esterçamento
		self.st = np.clip(st, -CAR['STEERMAX'], CAR['STEERMAX'])
		
		# atua no volante
		self.atuador.setSteer(self.st)
	
	########################################
	# get image data
	def getImage(self):
		# pega imagem
		return self.cam.getImage()
		
	########################################
	# termina a classe
	def __del__(self):
		
		time.sleep(1.0)
		
		print ('Programa terminado!')
			
	########################################
	# termina a classe
	def __exit__(self):
		self.stopMission()
		self.__del__()
