# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2025/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
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
		
		# variaveis calculadas (sem medicao)
		self.p = np.zeros(2)
		self.th = 0.0
		self.w = 0.0
		
		# comando de aceleracao
		self.u = 0.0
		# comando de estercamento
		self.st = 0.0
		
		# filtros dos sinais
		self.v_filt    = class_filter.MovingAverage(n=15)
		self.a_filt    = class_filter.MovingAverage(n=30)
		self.vref_filt = class_filter.MovingAverage(n=80)
		self.w_filt    = class_filter.MovingAverage(n=20)
		
		# logs de salvamento
		if self.parameters['save']:
			self.logfile = parameters['logfile']
			# Nome do diretorio com o timestamp
			self.logfile += datetime.now().strftime("%Y%m%d_%H%M%S") + "/"
			# Crie a pasta se ela nao existir
			os.makedirs(self.logfile, exist_ok=True)
		
		print('Carro pronto!', flush=True)
		
	########################################
	# inicializa sensores e atuadores
	def initSensors(self):
		
		# atuadores de estercamento, aceleracao e ultrasom/camera
		self.atuador = class_servos.Servos(ultrasonic=self.parameters['ultrasonic_steering'])
		print("\033[32mServos ok...\033[0m", flush=True)
		
		# odometro da roda
		self.odometer = class_encoder.Encoder()
		print("\033[32mOdometria ok...\033[0m", flush=True)

		# camera
		if self.parameters['camera']:
			import class_camera
			self.cam = class_camera.Camera()
			print("\033[32mCamera ok...\033[0m", flush=True)
		else:
			print("\033[31mNot using camera...\033[0m", flush=True)
			
		# ultrasom
		self.us = class_ultrasonic.Ultrasonic()
		print("\033[32mUltrasom ok...\033[0m", flush=True)
		
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
		
		# le velocidade do encoder
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
		Kp = 3.0
		Kd = 1.0
		
		# referencia de velocidade
		self.vref = self.vref_filt.filter(vref)
		self.vref = np.clip(self.vref, 0.0, CAR['VELMAX'])
		
		# controle de velocidade
		u = Kp*(self.vref - self.v) + Kd*(-self.a)
		self.setU(u)
	
	########################################
	# seta torque dos motores do veiculo
	def setU(self, u):
		
		# limita aceleracao
		self.u = np.clip(u, -CAR['ACCELMAX'], CAR['ACCELMAX'])
		
		# medida de seguranca
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
		
		# limita angulo de estercamento
		self.st = np.clip(st, -CAR['STEERMAX'], CAR['STEERMAX'])
		
		# atua no volante
		self.atuador.setSteer(self.st)
		
	########################################
	# get image data
	def getImage(self, gray=False):
		return self.cam.getImage(gray)
		
	########################################
	# get ultrasonic distance
	def getDistance(self, max_dist=4.0):
		return np.min([self.us.getDistance(), max_dist])
	
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
		
		self.odometer.close()
		self.atuador.close()
		self.us.close()
		if self.parameters['camera']:
			self.cam.close()
		
		print ('Program finished!')
		
########################################
# main teste
########################################
if __name__ == "__main__":
	
	# Globais
	parameters = {	
				'ts'					: 30.0,		# tempo da execucao
				'save'					: False,	# salvar trajetoria
				'logfile'				: 'logs/',	# log file
				'camera'				: False,	# usar camera
				'ultrasonic_steering' 	: True,		# mover ultrasom com estercamento
			}
	
	# cria comunicacao com o carrinho
	car = Car(parameters)
	car.startMission()
	
	# testa leitura
	t0 = time.time()
	while (time.time() - t0) <= 20.0:
		
		t = time.time() - t0
		
		# le ultrasom
		dist = car.getDistance()
		print(f"Distance: {dist:.2f} [m]")
		
		# seta torque do motor
		if dist > 0.30:
			car.setU(0.5*np.sin(0.5*t))
		else:
			car.setU(-1.0)
			
		# seta estercamento junto com ultrasom
		car.setSteer(np.deg2rad(20.0)*np.sin(0.5*t))
				
		time.sleep(.1)

	# salva os dados coletados
	if parameters['save']:
		car.save()
		
	car.close()
