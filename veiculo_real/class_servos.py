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
from adafruit_servokit import ServoKit

# Canais de entradas dos servos
SERVO_STEERING = 0
SERVO_THROTTLE  = 1
SERVO_PAN  = 4
SERVO_TILT = 5

MAX_STERRING_ANGLE  = 20.0
ZERO_STERRING_ANGLE = 100.0
GAIN_STERRING_ANGLE = 50.0/MAX_STERRING_ANGLE

ZERO_THROTTLE_ANGLE = 95.0
GAIN_THROTTLE_ANGLE = 0.2*90.0

########################################
# Adafruit 16-channel servo driver
########################################
class Servos:
	########################################
	# construtor
	def __init__(self, steering=0.0, throttle=0.0):
		
		# Set channels to the number of servo channels on your kit.
		# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
		self.kit = ServoKit(channels=16)
		
		# ajuste fino dos servos
		self.setTrim()
		
		# inicializa o esterçamento
		self.setSteer(steering)
		
		# inicializa traçao
		self.setU(throttle)
		
		# marcha re
		self.reverse = False
	
	########################################
	# seta o ajuste fino dos servos (em radianos)
	def setTrim(self, steer=0.0, throttle=0.0, pan=0.0, tilt=0.0):
		
		# trim do esterçamento
		self.trim_steer = np.clip(steer, -np.deg2rad(10.0), np.deg2rad(10.0))
		
		# trim da tração
		self.trim_throttle = np.clip(throttle, -np.deg2rad(20.0), np.deg2rad(20.0))
		
		# trim da camera
		self.trim_pan = np.clip(pan, -np.deg2rad(20.0), np.deg2rad(20.0))
		self.trim_tilt = np.clip(tilt, -np.deg2rad(20.0), np.deg2rad(20.0))
	
	########################################
	# seta torque do motor
	def setU(self, u):
		
		# converte para graus
		u = np.rad2deg(u + self.trim_throttle)
		
		# satura angulo de estercamento
		u = np.clip(u, -90.0, 90.0)
		
		# aplica calibracao devido a reduções do eixo
		u += 90.0
		
		# envia comando
		u = np.clip(u, 0.0, 180.0)
		self.kit.servo[SERVO_THROTTLE].angle = u
		
		return
	
	########################################
	# seta steer do veiculo (st in rad)
	def setSteer(self, st):
		
		# converte para graus
		st = np.rad2deg(st + self.trim_steer)
		
		# satura angulo de estercamento
		st = np.clip(st, -MAX_STERRING_ANGLE, MAX_STERRING_ANGLE)
		
		# aplica calibracao devido a reduções do eixo
		st = GAIN_STERRING_ANGLE*st + ZERO_STERRING_ANGLE
		
		# envia comando
		st = np.clip(st, 0.0, 180.0)
		self.kit.servo[SERVO_STEERING].angle = st
		
		return
	
	########################################
	# mode de marcha ré
	def backward(self):
		
		self.kit.servo[SERVO_THROTTLE].angle = ZERO_STERRING_ANGLE - 20.0
		time.sleep(0.08)
		self.kit.servo[SERVO_THROTTLE].angle = ZERO_STERRING_ANGLE
		time.sleep(0.08)
		self.kit.servo[SERVO_THROTTLE].angle = ZERO_STERRING_ANGLE - 20.0
		time.sleep(0.08)
		self.kit.servo[SERVO_THROTTLE].angle = ZERO_STERRING_ANGLE
		
	########################################
	# angulo de pan da camera entre [-90, 90]
	def setPan(self, ang):
			
		# converte para graus
		ang = np.rad2deg(ang + self.trim_pan)
		
		# angulo centrado em zero
		ang += 90.0        
		
		# envia comando
		ang = np.clip(ang, 0.0, 180.0)
		self.kit.servo[SERVO_PAN].angle = ang
		
		return

	########################################
	# angulo de tilt da camera entre [-40, 40]
	def setTilt(self, ang):
		
		# converte para graus
		ang = np.rad2deg(ang + self.trim_tilt)
		
		# limite fisico por causa do fio da camera
		ang = np.clip(ang, -50.0, 50.0)
		
		# angulo centrado em zero
		ang += 90.0        
		
		# envia comando
		ang = np.clip(ang, 0.0, 180.0)
		self.kit.servo[SERVO_TILT].angle = ang
		
		return
	
	########################################
	# função para testar os servos por 10s
	def test(self, total_time=10.0):
		dt = 0.1
		for t in np.linspace(0.0, total_time, int(total_time/dt)):
			print(t)
			# testa servos
			self.setSteer(np.deg2rad(MAX_STERRING_ANGLE)*np.sin(0.1*t))
			self.setU(np.deg2rad(90.0)*np.sin(0.1*t))
			self.setPan(np.deg2rad(90.0)*np.sin(0.1*t))
			self.setTilt(np.deg2rad(50.0)*np.sin(0.1*t))
			
			time.sleep(dt)
	
	########################################
	# destrutor
	def __del__(self):
		self.kit.servo[SERVO_STEERING].angle = None
		self.kit.servo[SERVO_THROTTLE].angle = None
		self.kit.servo[SERVO_PAN].angle = None
		self.kit.servo[SERVO_TILT].angle = None

########################################
'''ser = Servos()
ser.setTrim(throttle=np.deg2rad(10.0))
#ser.test()

for i in range(100):
	ser.setU(np.deg2rad(10))
	time.sleep(1.0)
	print(i)
'''
