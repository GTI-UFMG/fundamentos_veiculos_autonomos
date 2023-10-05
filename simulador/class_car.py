# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import sys
sys.path.append("coppelia/")
import sim
import time
import numpy as np
import math

########################################
# GLOBAIS
########################################
# parametros do carro
CAR = {
		'VELMAX'	: 5.0,		# m/s
		'ACCELMAX'	: 0.25, # m/s^2
		'STEERMAX'	: np.deg2rad(20.0),		# deg
	}

PORT = 19997	# communication port

ALFA = 0.3

########################################
# Manta
########################################
class CarCoppelia:	
	########################################
	# construtor
	def __init__(self):
		
		# tempo
		self.t = 0.0
		
		# velocidade de comando
		self.vref = 0.0
		
		# comando de aceleracao
		self.u = 0.0
		
		# tempo de amostragem
		self.dt = 0.0
		
		# inicia conexao
		self.initCoppeliaSim()
		
	########################################
	# inicializa interacao com o Coppelia -- Connect to CoppeliaSim
	def initCoppeliaSim(self):
		
		# o primeiro cria o cliente
		sim.simxFinish(-1) # just in case, close all opened connections
		self.clientID = sim.simxStart('127.0.0.1', PORT, True, True, 5000, 10)
		if self.clientID == -1:
			print ('Failed connecting to remote API server')

		# car
		err, self.robot = sim.simxGetObjectHandle(self.clientID, 'Car', sim.simx_opmode_oneshot_wait)
		if err != sim.simx_return_ok:
			print ('Remote API function call returned with error code (robot): ', err)
			
		# motors
		err, self.motorL = sim.simxGetObjectHandle(self.clientID, 'joint_motor_L', sim.simx_opmode_oneshot_wait)
		if err != sim.simx_return_ok:
			print ('Remote API function call returned with error code (motorL): ', err)
			
		err, self.motorR = sim.simxGetObjectHandle(self.clientID, 'joint_motor_R', sim.simx_opmode_oneshot_wait)
		if err != sim.simx_return_ok:
			print ('Remote API function call returned with error code (motorR): ', err)
		
		# camera
		err, self.cam = sim.simxGetObjectHandle(self.clientID, "Vision_sensor", sim.simx_opmode_oneshot_wait)
		if err != sim.simx_return_ok:
			print ('Remote API function call returned with error code: ', err)
		
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
	
		# comeca a simulacao
		sim.simxStartSimulation(self.clientID, sim.simx_opmode_blocking)
		
		# tempo inicial
		self.tinit = self.getTime()
		
		# sicronizado com o simulador
		sim.simxSynchronous(self.clientID, True)
	
	########################################
	# termina a missao
	def stopMission(self):		
		# termina a simulacao
		sim.simxStopSimulation(self.clientID, sim.simx_opmode_blocking)
	
	########################################
	def step(self):
		
		sim.simxSynchronousTrigger(self.clientID)
		
		# tempo anterior
		t0 = self.t
		
		# condicoes iniciais
		self.getStates()
		
		# atualiza amostragem
		self.dt = self.t - t0
			
	########################################
	# retorna tempo da simulacao no Coppelia
	def getTime(self):
		while True:
			err, t = sim.simxGetFloatSignal(self.clientID, 'mySimulationTime', sim.simx_opmode_streaming + 10)
			if (err == sim.simx_return_ok):
				return t
    
	########################################
	# retorna posicao do carro
	# -- Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls)
	def getPos(self):
		while True:
			err, pos = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming + 10)
			if (err == sim.simx_return_ok):
				return np.array((pos[0], pos[1]))
				
	########################################
	# retorna yaw
	def getYaw(self):
		while True:
			err, q = sim.simxGetObjectQuaternion(self.clientID, self.robot, -1, sim.simx_opmode_streaming + 10)
			if (err == sim.simx_return_ok):
				roll, pitch, yaw = self.quaternion_to_euler(q)
				#print(np.rad2deg([roll, pitch, yaw]))
				
				try:
					yaw = ALFA*yaw + (1.0-ALFA)*self.th
				except: None
				
				return yaw
	
	########################################
	def quaternion_to_euler(self, q):
		# Extract quaternion components
		w, x, y, z = q

		# Calculate roll (x-axis rotation)
		sinr_cosp = 2.0 * (w * x + y * z)
		cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
		roll = np.arctan2(sinr_cosp, cosr_cosp)

		# Calculate pitch (y-axis rotation)
		sinp = 2.0 * (w * y - z * x)
		if np.abs(sinp) >= 1:
			pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
		else:
			pitch = np.arcsin(sinp)

		# Calculate yaw (z-axis rotation)
		siny_cosp = 2.0 * (w * z + x * y)
		cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
		yaw = np.arctan2(siny_cosp, cosy_cosp)
		
		yaw = -yaw - np.pi/2
		while yaw < 0.0:
			yaw += 2.0*np.pi
		while yaw > 2.0*np.pi:
			yaw -= 2.0*np.pi

		return roll, pitch, yaw
		
	########################################
	# retorna velocidades linear e angular
	def getVel(self):
		while True:
			err, lin, ang = sim.simxGetObjectVelocity(self.clientID, self.robot, sim.simx_opmode_streaming + 10)
			if (err == sim.simx_return_ok):
				v = np.linalg.norm(lin)
				w = ang[2]
				
				try:
					w = ALFA*w + (1.0-ALFA)*self.w
				except: None
				
				return  v, w
	
	########################################
	# seta torque do veiculo
	def setVel(self, vref):
		
		Kp = 1.0
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
		
		# atua
		for m in [self.motorL, self.motorR]:
			# Set the velocity to some large number with the correct sign, because v-rep is weird like that
			while True:
				err = sim.simxSetJointTargetVelocity(self.clientID, m, np.sign(u)*CAR['VELMAX'], sim.simx_opmode_oneshot_wait)
				if (err == sim.simx_return_ok):
					break
					
			# Apply the desired torques to the joints
			while True:
				err = sim.simxSetJointForce(self.clientID, m, np.abs(self.u), sim.simx_opmode_oneshot_wait)
				if (err == sim.simx_return_ok):
					break
		
	########################################
	# seta steer do veiculo
	def setSteer(self, st):	
		
		st = np.clip(st, -CAR['STEERMAX'], CAR['STEERMAX'])
		
		st = (st + CAR['STEERMAX'])/CAR['STEERMAX']
		st *= 500.0
		
		while True:
			err,_,_,_,_=sim.simxCallScriptFunction(self.clientID,'control_truck',sim.sim_scripttype_childscript,'setSteer',[],[st],[],bytearray(),sim.simx_opmode_oneshot_wait)
			if (err == sim.simx_return_ok):
				break
	
	########################################
	# get image data
	def getImage(self):
		
		while True:
			err, resolution, image = sim.simxGetVisionSensorImage(self.clientID, self.cam, 0, sim.simx_opmode_streaming)
			if err == sim.simx_return_ok:
				break
				
		img = np.array(image,dtype=np.uint8)
		img.resize([resolution[1],resolution[0],3])
		return img
		
	########################################
	# termina a classe
	def __del__(self):
		
		time.sleep(.5)
		
		# fecha simulador
		sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot_wait)
		sim.simxFinish(-1)
		print ('Program ended')
			
	########################################
	# termina a classe
	def __exit__(self):
		self.stopMission()
		self.__del__()
