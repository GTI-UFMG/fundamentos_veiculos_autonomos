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
sys.path.append("coppeliasim_zmqremoteapi/")

import sim
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
	}

# parametro de filtragem
ALFA = 0.3

# communication port
PORT = 19997

########################################
# Carrinho
########################################
class CarCoppelia:	
	########################################
	# construtor
	def __init__(self, mode = 'zmq'): # mode = legacy or zmq
		
		# modo de comunicacao
		self.mode = mode
		
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
		
		#-----------------------------------
		if self.mode == 'legacy':
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

		#-----------------------------------
		if self.mode == 'zmq':
			
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
			
			# camera
			self.cam = self.sim.getObject('/Vision_sensor')
			if self.cam == -1:
				print ('Remote API function call returned with error code: ', -1)
			
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
		
		#-----------------------------------
		if self.mode == 'legacy':
			# comeca a simulacao
			sim.simxStartSimulation(self.clientID, sim.simx_opmode_blocking)
			
			# sicronizado com o simulador
			sim.simxSynchronous(self.clientID, True)
		
		#-----------------------------------
		if self.mode == 'zmq':
			# comeca a simulacao
			self.sim.startSimulation()
			
			# sicronizado com o simulador
			self.client.setStepping(True)
		
		#-----------------------------------
		# tempo inicial
		self.tinit = self.getTime()
		
		# estados iniciais
		self.getStates()
		
		# comeca parado
		self.setU(0.0)
		
	########################################
	# termina a missao
	def stopMission(self):	
		#-----------------------------------
		if self.mode == 'legacy':
			# comeca a simulacao
			sim.simxStopSimulation(self.clientID, sim.simx_opmode_blocking)
		
		#-----------------------------------
		if self.mode == 'zmq':
			self.sim.stopSimulation()
	
	########################################
	def step(self):
		
		#-----------------------------------
		if self.mode == 'legacy':
			sim.simxSynchronousTrigger(self.clientID)
		
		#-----------------------------------
		if self.mode == 'zmq':
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
			#-----------------------------------
			if self.mode == 'legacy':
				err, t = sim.simxGetFloatSignal(self.clientID, 'mySimulationTime', sim.simx_opmode_streaming + 10)
				if (err == sim.simx_return_ok):
					return t
			
			#-----------------------------------
			if self.mode == 'zmq':
				t = self.sim.getSimulationTime()
				if (t != -1.0): # Em caso de não retornar um erro
					return t
					
	########################################
	# retorna posicao do carro
	def getPos(self):
		while True:
			#-----------------------------------
			if self.mode == 'legacy':
				err, pos = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming + 10)
				if (err == sim.simx_return_ok):
					return np.array((pos[0], pos[1]))
			
			#-----------------------------------	
			if self.mode == 'zmq':
				pos = self.sim.getObjectPosition(self.robot, -1)
				if (pos != -1):
					return np.array((pos[0], pos[1]))			
				
	########################################
	# retorna yaw
	def getYaw(self):
		while True:
			#-----------------------------------
			if self.mode == 'legacy':
				err, q = sim.simxGetObjectQuaternion(self.clientID, self.robot, -1, sim.simx_opmode_streaming + 10)
				if (err == sim.simx_return_ok):
					break
			
			#-----------------------------------	
			if self.mode == 'zmq':
				q = self.sim.getObjectQuaternion(self.robot,-1)
				if (q != -1):
					break
		
		#-----------------------------------
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
			#-----------------------------------
			if self.mode == 'legacy':
				err, lin, ang = sim.simxGetObjectVelocity(self.clientID, self.robot, sim.simx_opmode_streaming + 10)
				if (err == sim.simx_return_ok):
					break
			
			#-----------------------------------
			if self.mode == 'zmq':
				lin, ang = self.sim.getObjectVelocity(self.robot)
				if (lin != -1): # CHECAR ERRO
					break
					
		#-----------------------------------
		if np.linalg.norm(lin) > 0.01:
			# angulo de translacao
			ps = np.arctan2(lin[1], lin[0])
			while ps < 0.0:
				ps += 2.0*np.pi
			while ps > 2.0*np.pi:
				ps -= 2.0*np.pi
			
			# diferenca entre translacao e orientacao
			if np.abs(ps - self.th) > np.pi/2:
				v = -np.linalg.norm(lin)
			else:
				v = np.linalg.norm(lin)
		else:
			v = 0.0
			
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
			
			#-----------------------------------
			if self.mode == 'legacy':				
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
						
			#-----------------------------------	
			if self.mode == 'zmq':
				# Set the velocity to some large number with the correct sign, because v-rep is weird like that
				while True:
					status = self.sim.setJointTargetVelocity(m, np.sign(u)*CAR['VELMAX'])
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
		
		st = np.clip(st, -CAR['STEERMAX'], CAR['STEERMAX'])
		
		st = (st + CAR['STEERMAX'])/CAR['STEERMAX']
		st *= 500.0
		
		#-----------------------------------
		if self.mode == 'legacy':
			while True:
				err,_,_,_,_= sim.simxCallScriptFunction(self.clientID,'control_truck',sim.sim_scripttype_childscript,'setSteer',[],[st],[],bytearray(),sim.simx_opmode_oneshot_wait)
				if (err == sim.simx_return_ok):
					break
		
		#-----------------------------------
		if self.mode == 'zmq':
			while True:
				SteerScript = self.sim.getScript(self.sim.scripttype_childscript,self.sim.getObject("/Car/control_truck"),'/Car/control_truck')
				_,_,_,err=self.sim.callScriptFunction('setSteer',SteerScript,[],[st],[],None)
				if err == '': # Caso receba a string vazia, quer dizer que a funcao retornou certo
					break		
	
	########################################
	# get image data
	def getImage(self):
		
		#-----------------------------------
		if self.mode == 'legacy':
			while True:
				err, resolution, image = sim.simxGetVisionSensorImage(self.clientID, self.cam, 0, sim.simx_opmode_streaming)
				if err == sim.simx_return_ok:
					break
			# trata imagem		
			img = np.array(image,dtype=np.uint8)
			
		#-----------------------------------		
		if self.mode == 'zmq':
			while True:
				image, resolution = self.sim.getVisionSensorImg(self.cam)
				if image != -1:
					break
			# trata imagem		
			img = np.frombuffer(image, dtype=np.uint8)
			
		#-----------------------------------
		img.resize([resolution[1], resolution[0],3])
		return img
		
	########################################
	# termina a classe
	def __del__(self):
		
		time.sleep(.5)
		
		# fecha simulador
		#-----------------------------------
		if self.mode == 'legacy':
			sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot_wait)
			sim.simxFinish(-1)
		
		#-----------------------------------
		if self.mode == 'zmq':
			self.sim.stopSimulation()
		
		print ('Programa terminado!')
			
	########################################
	# termina a classe
	def __exit__(self):
		self.stopMission()
		self.__del__()
