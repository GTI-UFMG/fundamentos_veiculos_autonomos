# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2024/1
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import sys
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
		'ACCELMAX'	: 0.25, 			# m/s^2
		'STEERMAX'	: np.deg2rad(20.0),	# deg
		'MASS'		: 6.35,				# kg
		'L'			: 0.302,			# distancia entre os eixos das rodas
	}

# parametro de filtragem
ALFA = 0.3

########################################
# Carrinho
########################################
class CarCoppelia:	
	########################################
	# construtor
	def __init__(self):
		
		# tempo
		self.t = 0.0
		
		# velocidade de comando
		self.vref = 0.0
		self.v = 0.0
		
		# comando de aceleracao
		self.u = 0.0
		
		# tempo de amostragem
		self.dt = 0.0
		
		# inicia conexao
		self.initCoppeliaSim()
		
	########################################
	# inicializa interacao com o Coppelia -- Connect to CoppeliaSim
	def initCoppeliaSim(self):
			
		# Cria o cliente
		self.client = RemoteAPIClient()
		self.sim = self.client.getObject('sim')
		self.sim.stopSimulation()
				
		# car
		self.robot = self.sim.getObject('/Car')
		if self.robot == -1:
			print ('Remote API function call returned with error code (robot): ', -1)
			
		# motors
		self.motorL = self.sim.getObject('/joint_motor_L')
		if self.motorL == -1:
			print ('Remote API function call returned with error code (motorL): ', -1)
			
		self.motorR = self.sim.getObject('/joint_motor_R')
		if self.motorR == -1:
			print ('Remote API function call returned with error code (motorR): ', -1)
			
		# steering
		self.steerL = self.sim.getObject('/joint_steer_L')
		if self.steerL == -1:
			print ('Remote API function call returned with error code (steerL): ', -1)
		
		self.steerR = self.sim.getObject('/joint_steer_R')
		if self.steerR == -1:
			print ('Remote API function call returned with error code (steerR): ', -1)
		
		# camera
		self.cam = self.sim.getObject('/Vision_sensor')
		if self.cam == -1:
			print ('Remote API function call returned with error code: ', -1)
			
		print('Car ready!')
	
	########################################
	# get states
	def getStates(self):
		self.p = self.getPos()
		self.th = self.getYaw()
		self.v_ant = self.v
		self.v, self.w = self.getVel()
		self.t = self.getTime() - self.tinit
				
		return self.p, self.v, self.th, self.w, self.t
	
	########################################
	# comeca a missao
	def startMission(self):
		
		# comeca a simulacao
		self.sim.startSimulation()
		
		# sicronizado com o simulador
		self.client.setStepping(True)
		
		# tempo inicial
		self.tinit = self.getTime()
		
		# estados iniciais
		self.getStates()
		
		# comeca parado
		self.setU(0.0)
		
		# salva trajetoria
		self.saveTraj()
		
	########################################
	# termina a missao
	def stopMission(self):	
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
		v = np.linalg.norm(lin)
		try:
			v = ALFA*v + (1.0-ALFA)*self.v
		except: None
		
		# velocidade angular
		w = ang[2]
		try:
			w = ALFA*w + (1.0-ALFA)*self.w
		except: None
		
		return  v, w
	
	########################################
	# retorna aceleracao
	def getAccel(self):
		BETA = 0.05
		a = (self.v - self.v_ant)/self.dt
		# filtro
		a = BETA*a + (1.0-BETA)*self.a
		return a
					
	########################################
	# seta torque do veiculo
	def setVel(self, vref):
		
		# ganhos
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
		
		# sinal da velocidade
		su = np.sign(u)*CAR['VELMAX']
		
		# impede que o carro se movimente para tras
		if (self.v < 0.05) and (su < 0.0):
			self.u = 0.0
		
		# atua
		for m in [self.motorL, self.motorR]:
					
			# Set the velocity to some large number with the correct sign, because v-rep is weird like that
			while True:
				status = self.sim.setJointTargetVelocity(m, su)
				if status == 1:
					break
			
			# Apply the desired torques to the joints
			while True:
				status = self.sim.setJointForce(m, np.abs(self.u))
				if status == 1:
					break

	########################################
	# seta steer do veiculo
	def setSteer(self, st):
		
		width = 0.108

		st = np.clip(st, -CAR['STEERMAX'], CAR['STEERMAX'])
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
	def getImage(self):
			
		while True:
			image, resolution = self.sim.getVisionSensorImg(self.cam)
			if image != -1:
				break
		# trata imagem		
		img = np.frombuffer(image, dtype=np.uint8)
		img.resize([resolution[1], resolution[0],3])
		return img
		
	########################################
	# termina a classe
	def __del__(self):
		# fecha simulador
		self.stopMission()
		
		print ('Program finished!')
			
	########################################
	# termina a classe
	def __exit__(self):
		for _ in range(2):
			time.sleep(.1)
			self.stopMission()
		self.__del__()
