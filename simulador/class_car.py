# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2024/1
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import sys, os
sys.path.append("coppeliasim_zmqremoteapi/")
from coppeliasim_zmqremoteapi_client import *
import numpy as np
import time

########################################
# GLOBAIS
########################################
# parametros do carro
CAR = {
		'VELMAX'	: 5.0,				# m/s
		'ACCELMAX'	: 1.0, 				# m/s^2
		'STEERMAX'	: np.deg2rad(20.0),	# deg
		'MASS'		: 6.3,				# kg
		'L'			: 0.302,			# distancia entre os eixos das rodas
		'RW' 		: 0.08,				# raio da roda [m]
		'MI' 		: 0.03,				# constante de friccao
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
		
		# id do carro no comboio
		self.id = parameters['car_id']
		
		# inicia simulador
		self.initCoppeliaSim()
		
		# tempo
		self.t = 0.0
		# tempo de amostragem
		self.dt = 0.0
		
		# velocidade de comando
		self.vref = 0.0
		self.v = 0.0
		
		# comando de aceleracao
		self.u = 0.0
		
		# comando de esterçamento
		self.st = 0.0

		# filtros dos sinais
		self.v_filt    = Filter(alpha=0.6)
		self.a_filt    = Filter(alpha=0.2)
		self.vref_filt = Filter(alpha=0.2)
		self.w_filt    = Filter(alpha=0.5)
		
		# logs de salvamento
		self.logfile = parameters['logfile']
		# Crie a pasta se ela não existir
		os.makedirs(self.logfile, exist_ok=True)
		
		print('Carro %d pronto!' % self.id, flush=True)
		
	########################################
	# inicializa interacao com o Coppelia -- Connect to CoppeliaSim
	def initCoppeliaSim(self):
			
		# Cria o cliente
		RemoteAPIClient().getObject('sim').stopSimulation()
		self.client = RemoteAPIClient()
		self.sim = self.client.getObject('sim')
		
		car_name = '/Car'#_%d' % self.id
		
		# car
		self.robot = self.sim.getObject(car_name)
		if self.robot == -1:
			print ('Remote API function call returned with error code (robot): ', -1)
			
		# motors
		self.motorL = self.sim.getObject(car_name+'/joint_motor_L')
		if self.motorL == -1:
			print ('Remote API function call returned with error code (motorL): ', -1)
			
		self.motorR = self.sim.getObject(car_name+'/joint_motor_R')
		if self.motorR == -1:
			print ('Remote API function call returned with error code (motorR): ', -1)
			
		# steering
		self.steerL = self.sim.getObject(car_name+'/joint_steer_L')
		if self.steerL == -1:
			print ('Remote API function call returned with error code (steerL): ', -1)
		
		self.steerR = self.sim.getObject(car_name+'/joint_steer_R')
		if self.steerR == -1:
			print ('Remote API function call returned with error code (steerR): ', -1)
		
		# camera
		self.cam = self.sim.getObject(car_name+'/Vision_sensor')
		if self.cam == -1:
			print ('Remote API function call returned with error code: ', -1)
			
	
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
		
		# sicronizado com o simulador
		self.client.setStepping(True)
		
		# comeca a simulacao
		self.sim.startSimulation()
		
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
		self.setU(-CAR['ACCELMAX'])
		self.setSteer(0.0)

		# stop simulador
		self.sim.stopSimulation()
	
	########################################
	def step(self):
		
		# passo de simulacao
		self.client.step()
		
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
					'u'     : self.u}
				
		# se ja iniciou as trajetorias
		try:
			self.traj.append(data)
		# se for a primeira vez
		except:
			self.traj = [data]
			
	########################################
	# retorna tempo da simulacao no Coppelia
	def getTime(self):
		while True:
			t = self.sim.getSimulationTime()
			if (t != -1.0): # Em caso de não retornar um erro
				return t
					
	########################################
	# retorna posicao do carro
	def getPos(self):
		while True:
			pos = self.sim.getObjectPosition(self.robot, -1)
			if (pos != -1):
				return np.array((pos[0], pos[1]))			
				
	########################################
	# retorna yaw
	def getYaw(self):
		while True:		
			q = self.sim.getObjectQuaternion(self.robot,-1)
			if (q != -1):
				break
	
		# quaternion to roll-pitch-yaw
		yaw = self.quaternion_to_yaw(q)
		yaw -= np.pi
		while yaw < 0.0:
			yaw += 2.0*np.pi
		while yaw > 2.0*np.pi:
			yaw -= 2.0*np.pi
		
		return yaw
		
	########################################
	def quaternion_to_yaw(self, q):
		
		qx, qy, qz, qw = q
		
		# Ensure the quaternion is normalized
		norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
		qx /= norm
		qy /= norm
		qz /= norm
		qw /= norm

		# Calculate the yaw angle (rotation about the Z-axis)
		yaw = np.arctan2(2 * (qx * qy + qw * qz), qw**2 + qx**2 - qy**2 - qz**2)

		return yaw
		
	########################################
	# retorna velocidades linear e angular
	def getVel(self):
		
		while True:		
			lin, ang = self.sim.getObjectVelocity(self.robot)
			if (lin != -1): # CHECAR ERRO
				break
		
		# velocidade linear
		v = self.v_filt.filter(np.linalg.norm(lin))
		
		# velocidade angular
		w = self.w_filt.filter(ang[2])
		
		return  v, w
	
	########################################
	# retorna aceleracao
	def getAccel(self):
		
		if self.dt == 0.0:
			return 0.0
			
		a = (self.v - self.v_ant)/self.dt
		# filtro
		a = self.a_filt.filter(a)
		
		return a
					
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
		
		# controlador linearizante
		F = []
		s = np.tanh(10.0*self.v)
		F.append(s*CAR['MASS']*CAR['GRAV']*CAR['MI'])
		F.append(CAR['MASS']*self.u)
		
		# torque de referencia
		GAMMA = 0.63 # reducao interna do eixo
		T = GAMMA*CAR['RW']*np.sum(F)
		
		# impede que o carro se movimente para tras
		if (s < 0) and (np.sign(u) < 0.0):
			T = 0.0
		
		# atua
		for m in [self.motorL, self.motorR]:		
			# Set the velocity to some large number with the correct sign, because v-rep is weird like that
			while True:
				status = self.sim.setJointTargetVelocity(m, np.sign(T)*CAR['VELMAX'])
				if status == 1:
					break
			# Apply the desired torques to the joints
			while True:
				status = self.sim.setJointForce(m, np.abs(T))
				if status == 1:
					break

	########################################
	# seta steer do veiculo
	def setSteer(self, st):
		
		# distancia entre rodas
		width = 0.108

		st = np.clip(st, -CAR['STEERMAX'], CAR['STEERMAX'])
		if np.tan(st) == 0:
			stL = stR = 0.0
		else:
			stL = np.arctan(CAR['L'] / ( width + CAR['L'] / np.tan(st)))
			stR = np.arctan(CAR['L'] / (-width + CAR['L'] / np.tan(st)))			
		
		# Set steering command
		while True:
			status = self.sim.setJointTargetPosition(self.steerL, stL)
			if status == 1:
				break
		while True:
			status = self.sim.setJointTargetPosition(self.steerR, stR)
			if status == 1:
				break
	
	########################################
	# seta orientacao da camera
	def setPanTilt(self, pan=np.deg2rad(0.0), tilt=np.deg2rad(-35.0)):
		return
			
	########################################
	# get image data
	def getImage(self, gray=False):
			
		while True:
			image, resolution = self.sim.getVisionSensorImg(self.cam)
			if image != -1:
				break
		# trata imagem		
		img = np.frombuffer(image, dtype=np.uint8)
		img.resize([resolution[1], resolution[0],3])
		return img
		
	########################################
	# save traj
	def save(self, log):
		filename = log + ('car%d.npz') % self.id
		data = [traj for traj in self.traj]
		np.savez(filename, data=data)
		
	########################################
	# load traj
	def load(self, log):
		filename = log + ('car%d.npz') % self.id
		data = np.load(filename, allow_pickle=True)
		self.traj = data['data']
		
	########################################
	# termina a classe
	def __del__(self):
		# fecha simulador
		self.stopMission()
		
		print ('Programa terminado!')
			
	########################################
	# termina a classe
	def __exit__(self):
		for _ in range(2):
			time.sleep(.1)
			self.stopMission()
		self.__del__()

########################################
# filtro da media móvel
########################################
class Filter:
	########################################
	# construtor
	def __init__(self, alpha=0.5):
		self.alpha = alpha
		self.alpha = np.clip(self.alpha, 0.0, 1.0)
		self.m = 0.0
	
	########################################
	def filter(self, m):
		try:
			self.m = self.alpha*m + (1.0-self.alpha)*self.m
		except:
			print('erro...')
			self.m = m
		
		return self.m

	########################################
	# termina a classe
	def __del__(self):
		None
