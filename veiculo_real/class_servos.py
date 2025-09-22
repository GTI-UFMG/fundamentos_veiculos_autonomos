# -*- coding: utf-8 -*-
########################################
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2025/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
from adafruit_servokit import ServoKit
import time
import numpy as np
import threading
import class_filter

# Canais de entradas dos servos
SERVO_STEERING  	= 0
SERVO_THROTTLE  	= 1
SERVO_ULTRASONIC	= 8

ZERO_STERRING_ANGLE = 100.0
MAX_STERRING_ANGLE  = 20.0
GAIN_STERRING_ANGLE = 50.0/MAX_STERRING_ANGLE

ZERO_THROTTLE_ANGLE = 95.0
GAIN_TORQUE = 0.4

########################################
# Adafruit 16-channel servo driver
########################################
class Servos:
	########################################
	# construtor
	def __init__(self, steering=0.0, throttle=0.0, use_thread=True):
		
		# Set channels to the number of servo channels on your kit.
		# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
		self.kit = ServoKit(channels=16)
		
		# periodo de atuacao
		self.dt = 0.03
		
		# derivada do pwm
		self.dth_pwm = 0.0
		
		# ajuste fino dos servos
		self.setTrim()
		
		# cria filtro de estercamento
		self.st_filt = class_filter.MovingAverage(n=10, initial=ZERO_STERRING_ANGLE)
		
		# lock de secao critica
		self.lock = threading.Lock()
		# thread que promove a atuacao suave
		self.thread = threading.Thread(target=self.actuator)
		
		# inicializa o estercamento
		self.setSteer(steering)
		
		# inicializa tracao
		self.setPWM(throttle)
		
		# dispara thread
		if use_thread:
			self.thread.start()
	
	########################################
	# seta o ajuste fino dos servos (em radianos)
	def setTrim(self, steer=np.deg2rad(0.0), throttle=np.deg2rad(0.0), pan=np.deg2rad(0.0)):
		
		# trim do esterc§amento
		self.trim_steer = np.clip(steer, -np.deg2rad(10.0), np.deg2rad(10.0))
		
		# trim da tracao
		self.trim_throttle = np.clip(throttle, -np.deg2rad(20.0), np.deg2rad(20.0))
		
		# trim do ultrasom
		self.trim_pan = np.clip(pan, -np.deg2rad(90.0), np.deg2rad(90.0))
		
	########################################
	# essa funcao garante a atuacao suave e periodica com self.dt
	def actuator(self):
		
		# pwm inicial
		th_pwm = 0.0
		
		while True:
			# comecou novo ciclo
			start_time = time.time()
			
			# envia comando de estercamento
			with self.lock:
				# filtra para nao dar tranco na direcao
				st_pwm = self.st_filt.filter(self.st_pwm)
			st_pwm = np.clip(st_pwm, 0.0, 180.0)
			self.kit.servo[SERVO_STEERING].angle = st_pwm
			
			# envia comando de tracao
			with self.lock:
				# integra pwm
				dt = self.dt
				th_pwm += self.dth_pwm*dt
			th_pwm = np.clip(th_pwm, np.deg2rad(0.0), np.deg2rad(90.0))
			self.setPWM(th_pwm)
			
			# espera terminar o periodo
			elapsed_time = time.time() - start_time
			time.sleep(np.max([0.0, dt-elapsed_time]))
	
	########################################
	# seta torque do motor em N.m
	def setTorque(self, T, dt=0.03):
		
		# transforma torque para rad
		with self.lock:
			self.dth_pwm = GAIN_TORQUE*T
			self.dt = dt
		
	########################################
	# seta PWM do motor
	def setPWM(self, pwm):
		
		# converte para graus
		pwm = np.rad2deg(pwm + self.trim_throttle)
		
		# aplica calibracao devido a reducoes do eixo
		pwm += ZERO_THROTTLE_ANGLE
		
		# envia comando
		self.pwm = np.clip(pwm, 0.0, 180.0)
		self.kit.servo[SERVO_THROTTLE].angle = self.pwm
	
	########################################
	# seta steer do veiculo (st in rad)
	def setSteer(self, st, ultrasonic=True):
		
		# roda o ultrasom conforme o estercamento (em RAD)
		if ultrasonic:
			self.setPan(st)
			
		# converte para graus
		st = np.rad2deg(st + self.trim_steer)
		
		# satura angulo de estercamento
		trim_deg = np.rad2deg(self.trim_steer)
		st = np.clip(st, -MAX_STERRING_ANGLE + trim_deg, MAX_STERRING_ANGLE + trim_deg)
		
		# aplica calibracao devido a reducoes do eixo
		with self.lock:
			self.st_pwm = GAIN_STERRING_ANGLE*st + ZERO_STERRING_ANGLE
	
	########################################
	# angulo de pan do ultrasom
	def setPan(self, ang):
		
		# converte para graus
		ang = np.rad2deg(ang + self.trim_pan)
		
		# angulo centrado em zero
		ang += 90.0        
		
		# envia comando
		ang = np.clip(ang, 0.0, 180.0)
		self.kit.servo[SERVO_ULTRASONIC].angle = ang
		
	########################################
	# mode de marcha re
	def backward(self):
		self.kit.servo[SERVO_THROTTLE].angle = 0.8*ZERO_THROTTLE_ANGLE
		time.sleep(0.1)
		self.kit.servo[SERVO_THROTTLE].angle = ZERO_THROTTLE_ANGLE
		time.sleep(0.1)
		self.kit.servo[SERVO_THROTTLE].angle = 0.8*ZERO_THROTTLE_ANGLE
		time.sleep(0.1)
		self.kit.servo[SERVO_THROTTLE].angle = ZERO_THROTTLE_ANGLE
	
	########################################
	# destrutor
	def __del__(self):
		# termina de mover os servos e o esc
		time.sleep(1.0)
	
	########################################
	# termina a classe
	def __exit__(self):
		self.setSteer(0.0)
		self.setTorque(0.0)
		self.__del__()

########################################
# main teste
########################################
if __name__ == "__main__":
	
	# cria servos
	ser = Servos(use_thread=True)
	print('Servos ok...')
	
	# testa servos
	t0 = time.time()
	while (time.time() - t0) <= 10.0:
		t = time.time() - t0
		print(f"Tempo = {t:.2f} s")
		
		ser.setSteer(np.deg2rad(MAX_STERRING_ANGLE)*np.sin(0.5*t), ultrasonic=True)
		ser.setTorque(0*np.deg2rad(90.0)*np.sin(0.1*t))
		time.sleep(ser.dt)
		
	# calibracao do ESC
	#ser.kit.servo[SERVO_THROTTLE].angle = 90 #90 (neutro), 180 (maximo), 0 (minimo)
