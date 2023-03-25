# -*- coding: utf-8 -*-
# using coppelia
import sys
sys.path.append("coppelia/")
import sim
import time
import numpy as np

########################################
# GLOBAIS
########################################
# parametros do carro
CAR = {
		'VELMAX'		: 1.0,		# m/s
		'STEERMAX'		: 30.0,		# deg
	}

PORT = 19997	# communication port

########################################
# Manta
########################################
class CarCoppelia:	
	########################################
	# construtor
	def __init__(self):
		
		# tempo
		self.t = 0.0
		
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
	
	########################################
	# termina a missao
	def stopMission(self):		
		# termina a simulacao
		sim.simxStopSimulation(self.clientID, sim.simx_opmode_blocking)
	
	########################################
	def step(self):
		
		# condicoes iniciais
		self.getStates()
			
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
		return np.array((0,0))
		while True:
			err, pos = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming + 10)
			if (err == sim.simx_return_ok):
				return np.array((pos[0], pos[1]))
				
	########################################
	# retorna yaw
	def getYaw(self):
		return 0.0
		while True:
			err, ang = sim.simxGetObjectOrientation(self.clientID, self.robot, -1, sim.simx_opmode_streaming + 10)
			if (err == sim.simx_return_ok):
				return ang[2] + np.pi/2.0
		
	########################################
	# retorna velocidades linear e angular
	def getVel(self):
		
		return 0,0
		
		while True:
			err, lin, ang = sim.simxGetObjectVelocity(self.clientID, self.robot, sim.simx_opmode_streaming + 10)
			if (err == sim.simx_return_ok):
				v = lin[0]*np.cos(self.th) + lin[1]*np.sin(self.th)
				return  v, ang[2] #lin, ang
	
	
	########################################
	# seta torque do veiculo
	def setVel(self, v):
		
		v = np.clip(v, 0.0, CAR['VELMAX'])
		v = v/CAR['VELMAX']
		v *= 1000.0
		
		while True:
			err,_,_,_,_=sim.simxCallScriptFunction(self.clientID,'control_truck',sim.sim_scripttype_childscript,'setVel',[],[v],[],bytearray(),sim.simx_opmode_oneshot_wait)
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
		self.__del__()
