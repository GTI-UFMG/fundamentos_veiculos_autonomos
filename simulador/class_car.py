# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2026/1
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
import class_filter

########################################
# GLOBAIS
########################################
# parametros do carro
CAR = {
		'VELMAX'	: 1.5,				# m/s
		'ACCELMAX'	: 1.0, 				# m/s^2
		'STEERMAX'	: np.deg2rad(20.0),	# deg
		'MASS'		: 6.3,				# kg
		'L'			: 0.302,			# distancia entre os eixos das rodas
		'RW' 		: 0.08,				# raio da roda [m]
		'MI' 		: 0.05,				# constante de friccao
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
		
		# inicia simulador
		self.init_coppelia_sim()
		
		# tempo
		self.t = 0.0
		# tempo de amostragem
		self.dt = 0.0
		
		# velocidade de comando
		self.vref = 0.0
		self.v = 0.0
		
		# marcha
		self.gear = 1   # +1 forward, -1 reverse
		
		# comando de aceleracao
		self.u = 0.0
		
		# comando de esterçamento
		self.st = 0.0

		# filtros dos sinais
		self.v_filt    = class_filter.AlphaFilter(alpha=0.6)
		self.a_filt    = class_filter.AlphaFilter(alpha=0.2)
		self.vref_filt = class_filter.AlphaFilter(alpha=0.2)
		self.w_filt    = class_filter.AlphaFilter(alpha=0.5)
		
		# logs de salvamento
		self.logfile = parameters['logfile']
		# Crie a pasta se ela não existir
		os.makedirs(self.logfile, exist_ok=True)
		
		print("\033[33m##############################\033[0m", flush=True)
		print("\033[33mCarro pronto!\033[0m", flush=True)
		print("\033[33m##############################\033[0m", flush=True)
		
	########################################
	# inicializa interacao com o Coppelia -- Connect to CoppeliaSim
	def init_coppelia_sim(self):
			
		# Cria o cliente
		RemoteAPIClient().getObject('sim').stopSimulation()
		self.client = RemoteAPIClient()
		self.sim = self.client.getObject('sim')
		
		car_name = '/Car'
		
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
			print('Erro: câmera não encontrada')
			
		# depois de self.cam = ...
		self.ultra = self.sim.getObject('/Car/ultra_front')  # ajuste o nome igual ao da cena
		if self.ultra == -1:
			print('Erro: ultrassônico não encontrado')
	
	########################################
	# get states
	def get_states(self):

		# velocidade 
		self.v_ant = self.v
		self.v, self.w = self.get_vel()

		# aceleracao
		self.a = self.get_accel()

		# orientacao
		self.th = self.get_yaw()
		
		# posicao
		self.p = self.get_pos()
		
		# tempo
		self.t = self.get_time() - self.tinit
				
		return self.p, self.v, self.a, self.th, self.w, self.t
	
	########################################
	# comeca a missao
	def start_mission(self):
		
		# sicronizado com o simulador
		self.client.setStepping(True)
		
		# comeca a simulacao
		self.sim.startSimulation()
		
		# tempo inicial
		self.tinit = self.get_time()
		
		# estados iniciais
		self.get_states()
		
		# comeca na marcha para frente
		self.set_forward()

		# comeca parado
		self.set_u(0.0)
		self.set_steer(0.0)
		
		# seta orientacao da camera
		self.set_pan_tilt()
		
		# salva trajetoria
		self.save_traj()
	
	########################################
	def step(self):
		
		# passo de simulacao
		self.client.step()
		
		# tempo anterior
		t0 = self.t
		
		# condicoes iniciais
		self.get_states()
		
		# atualiza amostragem
		self.dt = self.t - t0
		
		# salva trajetoria
		self.save_traj()
		
	########################################
	# salva a trajetoria
	def save_traj(self):
		
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
	def get_time(self):
		#while True:
		t = self.sim.getSimulationTime()
		if (t != -1.0): # Em caso de não retornar um erro
			return t
					
	########################################
	# retorna posicao do carro
	def get_pos(self):
		while True:
			pos = self.sim.getObjectPosition(self.robot, -1)
			if (pos != -1):
				return np.array((pos[0], pos[1]))			
				
	########################################
	# retorna yaw
	def get_yaw(self):
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
	def get_vel(self):

		while True:
			lin, ang = self.sim.getObjectVelocity(self.robot)
			if lin != -1:
				break

		# velocidade longitudinal
		v = self.gear * np.linalg.norm(lin)

		# filtros
		v = self.v_filt.filter(v)
		w = self.w_filt.filter(ang[2])

		return v, w
	
	########################################
	# retorna aceleracao
	def get_accel(self):
		
		if self.dt == 0.0:
			return 0.0
			
		a = (self.v - self.v_ant)/self.dt
		# filtro
		af = self.a_filt.filter(a)
		
		return af
		
	########################################
	# seta referencia de controle
	def set_ref(self, vref):

		self.vref = self.vref_filt.filter(vref)
		self.vref = np.clip(self.vref, -CAR['VELMAX'], CAR['VELMAX'])

		# troca para re
		if (self.vref < 0.0) and (self.gear == 1):
			self.set_reverse()

		# troca para frente
		elif (self.vref > 0.0) and (self.gear == -1):
			self.set_forward()

		return self.vref
					
	########################################
	# seta torque do veiculo
	def set_vel(self, vref):
		
		# ganhos
		Kp = 3.5
		Kd = 2.5
		
		# define referencia e marcha
		self.set_ref(vref)
		
		# controla magnitude da velocidade
		vref_abs = abs(self.vref)
		v_abs = abs(self.v)

		# aceleracao da magnitude
		a_abs = np.sign(self.v) * self.a

		# controle PD
		du = Kp*(vref_abs - v_abs) - Kd*a_abs
		u = self.u + du*self.dt
		self.set_u(u)
	
	########################################
	# seta torque dos motores do veiculo
	def set_u(self, u):

		# limita aceleracao
		self.u = np.clip(u, -CAR['ACCELMAX'], CAR['ACCELMAX'])
		
		# magnitude da velocidade
		v_abs = abs(self.v)

		# compensacao da forca de atrito
		s = np.tanh(10.0*v_abs)
		F_friction = s*CAR['MASS']*CAR['GRAV']*CAR['MI']

		# forca longitudinal
		F = F_friction + CAR['MASS']*self.u
		
		# torque
		GAMMA = 0.63
		T = GAMMA*CAR['RW']*F

		# aplica o sentido da marcha
		T = self.gear*T

		# atua
		for m in [self.motorL, self.motorR]:
			# Set the velocity to some large number with the correct sign, because v-rep is weird like that
			self.sim.setJointTargetVelocity(m, np.sign(T)*CAR['VELMAX'])
			# Apply the desired torques to the joints
			self.sim.setJointForce(m, abs(T))
			
	########################################
	# vai para frente
	def set_forward(self):

		# se ja esta para frente, nao faz nada
		if self.gear == 1:
			return

		# freia ate parar
		while abs(self.v) > 0.1:
			self.set_u(-CAR['ACCELMAX'])
			self.step()

		# troca a marcha
		self.gear = 1
		
	########################################
	# coloca re
	def set_reverse(self):

		# se ja esta de re, nao faz nada
		if self.gear == -1:
			return

		# freia ate parar
		while abs(self.v) > 0.1:
			self.set_u(-CAR['ACCELMAX'])
			self.step()

		# troca a marcha
		self.gear = -1
		
	########################################
	# seta steer do veiculo
	def set_steer(self, st):
		
		# distancia entre rodas
		width = 0.108
		
		self.st = np.clip(st, -CAR['STEERMAX'], CAR['STEERMAX'])
		st = self.st
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
	def set_pan_tilt(self, pan=np.deg2rad(0.0), tilt=np.deg2rad(-35.0)):
		return
			
	########################################
	# get image data
	def get_image(self, gray=False):
			
		while True:
			image, resolution = self.sim.getVisionSensorImg(self.cam)
			if image != -1:
				break
		# trata imagem		
		img = np.frombuffer(image, dtype=np.uint8)
		img.resize([resolution[1], resolution[0],3])
		return img
		
	########################################
	# get ultrasonic distance
	def get_distance(self, max_dist=4.0):
		# CoppeliaSim retorna: hit, dist, point(list3), obj, normal(list3)
		hit, dist, p, obj, _n = self.sim.readProximitySensor(self.ultra)
		if hit:
			dist = min(float(dist), max_dist)
		else:
			dist = max_dist
		valid = True # sempre eh valido
		return dist, valid
		
	########################################
	# save traj
	def save(self, log):
		filename = log + 'car.npz'
		data = [traj for traj in self.traj]
		np.savez(filename, data=data)
		
	########################################
	# load traj
	def load(self, log):
		filename = log + 'car.npz'
		data = np.load(filename, allow_pickle=True)
		self.traj = data['data']
	
	########################################
	# termina a missao
	def stop_mission(self):
		
		# termina parado
		self.set_u(-CAR['ACCELMAX'])
		self.set_steer(0.0)
		
		# espera ate parar
		while abs(self.v) > 0.1:
			self.step()

		# stop simulador
		self.sim.stopSimulation()
		
	########################################
	# fecha tudo
	def close(self):
		self.stop_mission()
		print("\033[33m##############################\033[0m", flush=True)
		print("\033[33mMissao terminada!\033[0m", flush=True)
		print("\033[33m##############################\033[0m", flush=True)
