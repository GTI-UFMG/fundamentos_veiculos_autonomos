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
import time, os
from datetime import datetime
import class_encoder
import class_servos
import class_filter
import class_ultrasonic

########################################
# GLOBAIS
########################################
# parametros do carro
CAR = {
		'VELMAX'	: 3.0,				# m/s
		'ACCELMAX'	: 1.0, 				# m/s^2
		'STEERMAX'	: np.deg2rad(20.0),	# deg
		'MASS'		: 5.16,				# kg
		'L'			: 0.36,				# distancia entre os eixos das rodas
		'RW' 		: 0.08,				# raio da roda [m]
		'MI' 		: 0.04,				# constante de friccao
		'GRAV'   	: 9.81, 			# gravidade [m/s^2]
	}

########################################
# Carrinho
########################################
class Car:	
	########################################
	# construtor
	def __init__(self, parameters):
		
		self.parameters = parameters
		
		# inicializa sensores
		self.initSensors()
		
		# tempo
		self.t = 0.0
		# tempo de amostragem
		self.dt = 0.05
		
		# velocidade de comando
		self.vref = 0.0
		self.v = 0.0
		self.a = 0.0
		
		# variaveis calculadas (sem medição)
		self.p = np.zeros(2)
		self.th = 0.0
		self.w = 0.0
		
		# comando de aceleracao
		self.u = 0.0
		# comando de esterçamento
		self.st = 0.0
		
		# filtros dos sinais
		self.v_filt    = class_filter.MovingAverage(n=15)
		self.a_filt    = class_filter.MovingAverage(n=30)
		self.vref_filt = class_filter.MovingAverage(n=50)
		self.w_filt    = class_filter.MovingAverage(n=20)
		
		# logs de salvamento
		self.logfile = parameters['logfile']
		# Nome do diretorio com o timestamp
		self.logfile += datetime.now().strftime("%Y%m%d_%H%M%S") + "/"
		# Crie a pasta se ela não existir
		os.makedirs(self.logfile, exist_ok=True)
		
		print('Carro pronto!', flush=True)
		
	########################################
	# inicializa sensores e atuadores
	def initSensors(self):
		
		# atuadores de esterçamento, aceleração e ultrasom/camera
		self.atuador = class_servos.Servos(ultrasonic=self.parameters['ultrasonic_steering'])
		print('Servos ok...', flush=True)
		
		# odometro da roda
		self.odometer = class_encoder.Encoder()
		print('Odometria ok...', flush=True)

		# camera
		if self.parameters['camera']:
			import class_camera
			self.cam = class_camera.Camera()
			print('Camera ok...', flush=True)
		else:
			print('Not using camera...', flush=True)
			
		# ultrasom
		self.us = class_ultrasonic.Ultrasonic()
		print('Ultrasom ok...', flush=True)
		
		print('##############################')

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
				
		return self.p, self.v, self.a, self.th, self.w, self.t
	
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
		
		# termina parado
		self.setU(-CAR['ACCELMAX'])
		self.setSteer(0.0)
		time.sleep(2.0)
	
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
		with self.lock:
			data = {	't'     : self.t, 
						'p'     : self.p, 
						'v'     : self.v,
						'a'		: self.a,
						'vref'  : self.vref,
						'th'    : self.th,
						'w'     : self.w,
						'u'     : self.u,
					}
				
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
		yaw = self.th + self.w*self.dt
		
		while yaw < 0.0:
			yaw += 2.0*np.pi
		while yaw > 2.0*np.pi:
			yaw -= 2.0*np.pi
		
		return yaw
		
	########################################
	# retorna velocidades linear e angular
	def getVel(self):
		
		# lê velocidade do encoder
		v = self.odometer.getVel()
		vf = self.v_filt.filter(v)
		
		# velocidade angular sem IMU, calculada artificialmente
		w = (v/CAR['L'])*np.tan(self.st)
		wf = self.w_filt.filter(w)
		
		return  vf, wf
	
	########################################
	# retorna aceleracao
	def getAccel(self):
		
		if self.dt == 0.0:
			return 0.0
		
		# aceleracao sem IMU, calculada artificialmente
		a = (self.v - self.v_ant)/self.dt
		af = self.a_filt.filter(a)
		
		return af
					
	########################################
	# seta torque do veiculo
	def setVel(self, vref):
		
		# ganhos
		Kp = 3.5
		Kd = 2.5
		
		# referencia de velocidade
		self.vref = self.vref_filt.filter(vref)
		self.vref = np.clip(self.vref, 0.0, CAR['VELMAX'])
		
		# controle de velocidade
		du = Kp*(self.vref - self.v) + Kd*(-self.a)
		u = self.u + du*self.dt
		self.setU(u)
	
	########################################
	# seta torque dos motores do veiculo
	def setU(self, u):
		
		# limita aceleracao
		self.u = np.clip(u, -CAR['ACCELMAX'], CAR['ACCELMAX'])
		
		# medida de segurança
		if self.v > CAR['VELMAX']:
			self.u = 0.0
			
		# controlador linearizante
		F = CAR['MASS']*self.u
		
		# torque de referencia
		T = CAR['RW']*np.sum(F)
		
		# impede que o carro se movimente para tras
		if (np.sign(self.v) < 0.0) and (np.sign(u) < 0.0):
			T = 0.0
		
		# seta o torque
		self.atuador.setTorque(T, dt=self.dt)

	########################################
	# seta steer do veiculo
	def setSteer(self, st):
		
		# limita angulo de esterçamento
		self.st = np.clip(st, -CAR['STEERMAX'], CAR['STEERMAX'])
		
		# atua no volante
		self.atuador.setSteer(self.st)
		
	########################################
	# get image data
	def getImage(self, gray=False):
		return self.cam.getImage(gray)
	
	########################################
	# save traj
	def save(self):
		filename = self.logfile + ('car%d.npz') % self.id
		data = [traj for traj in self.traj]
		np.savez(filename, data=data)
		
	########################################
	# load traj
	def load(self):
		filename = self.logfile + ('car%d.npz') % self.id
		data = np.load(filename, allow_pickle=True)
		self.traj = data['data']
		
	########################################
	# termina a classe
	def close(self):
		self.stopMission()
		print ('Program finished!')
