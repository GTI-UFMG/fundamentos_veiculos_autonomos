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
		'ACCELMAX'	: 0.25, 				# 
		'STEERMAX'	: np.deg2rad(20.0),	# deg
		'MASS'		: 5.20,				# kg
		'L'			: 0.36,				# distancia entre os eixos das rodas
		'RMOTOR'	: 10, 				# resitencia do motor
		'KVMOTOR'	: 2100,
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
		self.dt = 0.1
		
		# velocidade de comando
		self.vref = 0.0
		self.v = 0.0
		self.a = 0.0
		
		# variaveis calculadas (sem medição)
		self.th = 0.0
		self.w = 0.0
		self.p = np.array((0.0, 0.0))
		
		# comando de aceleracao
		self.u = 0.0
		self.pwm = 0.0
		
		# comando de esterçamento
		self.st = 0.0
		
		# atuadores de esterçamento e aceleração
		self.atuador = class_servos.Servos()
		print('Servos ok...', flush=True)
		
		# camera
		self.cam = class_camera.Camera()
		print('Camera ok...', flush=True)
		
		# odometro da roda
		self.odometer = class_encoder.Encoder()
		print('Odometria ok...', flush=True)
		
		print('Carro pronto!', flush=True)
	
	########################################
	# get states
	def getStates(self):
		
		# velocidade 
		self.v_ant = self.v
		self.v, self.w = self.getVel()
		
		# aceleracao
		self.a = self.getAccel()
		
		# orientacao
		self.th = self.getYaw()
		
		# posicao
		self.p = self.getPos()
		
		# tempo
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
		
		# seta orientacao da camera
		self.setPanTilt()
		
		# salva trajetoria
		self.saveTraj()
		
	########################################
	# termina a missao
	def stopMission(self):
		
		# termina parado
		self.setU(0.0)
		self.setSteer(0.0)
	
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
					'u'     : self.u,
					'a'     : self.a}
				
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
		
		x = self.p[0] + self.v*np.cos(self.th)*self.dt
		y = self.p[1] + self.v*np.sin(self.th)*self.dt
		
		return np.array((x, y))			
				
	########################################
	# retorna yaw - sem bussola
	def getYaw(self):
		
		th = self.th + self.w*self.dt
		
		while th < 0.0:
			th += 2.0*np.pi
		while th > 2.0*np.pi:
			th -= 2.0*np.pi
		
		return th
		
	########################################
	# retorna velocidades linear e angular
	def getVel(self):
		
		# lê velocidade do encoder
		v = self.odometer.getVel()

		# sem IMU, calcular artificialmente
		w = v*np.tan(self.st)/CAR['L']

		return v, w
		
	########################################
	# retorna velocidades linear e angular
	def getAccel(self):
		BETA = 0.05
		a = (self.v - self.v_ant)/self.dt
		# filtro
		a = BETA*a + (1.0-BETA)*self.a
		return a
					
	########################################
	# seta torque do veiculo
	def setVel(self, vref):
		
		Kp = 0.1
		Kd = 0*0.1
		
		# referencia de velocidade
		vref = np.clip(vref, 0.0, CAR['VELMAX'])
		
		# filtragem
		self.vref = ALFA*vref + (1.0-ALFA)*self.vref
		
		# controle de velocidade
		u = Kp*(self.vref - self.v) + Kd*(0.0 - self.u)
		
		# seta o comando do servos
		self.setU(u)
	
	########################################
	# seta torque dos motores do veiculo
	def setU(self, u):
		
		REDUCAO_EIXO_MOTOR = 2.5
		RAIO_RODA = class_encoder.RAIO_RODA
		REDUCAO_EIXO = class_encoder.REDUCAO_EIXO
		
		# limita aceleracao
		u = np.clip(u, -CAR['ACCELMAX'], CAR['ACCELMAX'])
		
		# Calcula rotacao do eixo do motor
		rpm = self.v/(RAIO_RODA*np.pi/30.0)
		rpm *= class_encoder.REDUCAO_EIXO
		omega = (2.0*np.pi/60.0)*REDUCAO_EIXO_MOTOR*rpm
		
		# limita para nao ser zero
		omega = max(omega, 1000.0)
		
		# calcula PWM
		alpha = 0.1
		self.pwm = alpha*(CAR['RMOTOR']*CAR['KVMOTOR']*u)/omega
		self.pwm = np.clip(self.pwm, np.deg2rad(0.0), np.deg2rad(90.0))
		
		# seta PWM
		self.atuador.setU(self.pwm)

	########################################
	# seta steer do veiculo
	def setSteer(self, st):	
		
		# limita angulo de esterçamento
		self.st = np.clip(st, -CAR['STEERMAX'], CAR['STEERMAX'])
		
		# atua no volante
		self.atuador.setSteer(self.st)
	
	########################################
	# seta orientacao da camera
	def setPanTilt(self, pan=np.deg2rad(0.0), tilt=np.deg2rad(-35.0)):
		self.atuador.setPan(pan)
		self.atuador.setTilt(tilt)
		
	########################################
	# get image data
	def getImage(self, gray=False):
		# pega imagem
		return self.cam.getImage(gray)
		
	########################################
	# termina a classe
	def __del__(self):
		self.stopMission()
		
		time.sleep(1.0)
		
		print ('Programa terminado!')
			
	########################################
	# termina a classe
	def __exit__(self):
		self.__del__()
