# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
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
	def __init__(self, steering=0.0, throttle=0.0, velmax=1.5, dt=0.03, ultrasonic=True):
		
		# lock de secao critica
		self.lock = threading.Lock()
		
		# Set channels to the number of servo channels on your kit.
		# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
		self.kit = ServoKit(channels=16)
		
		# periodo de atuacao
		if dt <= 0.0:
			raise ValueError("dt deve ser maior que zero.")
		self.dt = float(dt)
		
		# derivada do pwm
		self.dth_pwm = 0.0
		
		# ajuste fino dos servos
		self.set_trim()
		
		# define se o ultrasom vai se mover junto com o estercamento
		self.ultrasonic = ultrasonic
		
		# definir limite pratico de velocidade do carro
		self.velmax = np.clip(abs(float(velmax)), 0.3, 1.5)
		#v =  0.092294 PWM - 8.9370 (curva interpolada com experimentos)
		self.max_pwm_throttle = (self.velmax + 8.9370)/0.092294
		self.min_pwm_throttle = 2.0*ZERO_THROTTLE_ANGLE - self.max_pwm_throttle
		
		# cria filtro de estercamento
		self.st_filt = class_filter.MovingAverage(n=10, initial=steering)
		# inicializa o estercamento
		self.set_steer(steering)
		
		# inicializa tracao
		self.th_pwm = np.clip(float(throttle), 0.0, np.deg2rad(90.0))
		self._set_pwm(self.th_pwm)
		
		##################################
		# evento para matar a thread
		self.stop = threading.Event()
		# thread que promove a atuacao suave
		self.thread = threading.Thread(target=self.actuator, daemon=True)
		# dispara thread
		self.thread.start()
		
	########################################
	# essa eh a thread que garante a atuacao suave e periodica com self.dt
	def actuator(self):
		
		# loop principal
		while not self.stop.is_set():
			
			# comecou novo ciclo
			start_time = time.monotonic()
			
			with self.lock:
				# envia comando de estercamento
				self.kit.servo[SERVO_STEERING].angle = self.st_pwm
			
				# envia comando de pan da camera/ultrasom
				self.kit.servo[SERVO_ULTRASONIC].angle = self.pan_pwm
			
				# envia comando de tracao (integra pwm)
				self.th_pwm += self.dth_pwm * self.dt
			
				# limita tracao com anti-windup
				self.th_pwm = np.clip(self.th_pwm, np.deg2rad(0.0), np.deg2rad(90.0))
				th_pwm = self.th_pwm
			
			# seta commando
			self._set_pwm(th_pwm)
			
			# espera terminar o periodo
			elapsed_time = time.monotonic() - start_time
			self.stop.wait(max(0.0, self.dt - elapsed_time))
	
	########################################
	# seta PWM do motor
	def _set_pwm(self, pwm):
		
		# converte para graus
		pwm = np.rad2deg(pwm + self.trim_throttle)
		
		# aplica calibracao devido a reducoes do eixo
		pwm += ZERO_THROTTLE_ANGLE
		
		# envia comando
		self.pwm = np.clip(pwm, self.min_pwm_throttle, self.max_pwm_throttle)
		self.kit.servo[SERVO_THROTTLE].angle = self.pwm
		
	########################################
	# emula comando de torque por variacao do throttle
	# T eh um comando de pseudo-torque, nao torque medido em N.m
	def set_torque(self, T):
		
		# transforma torque para rad
		with self.lock:
			self.dth_pwm = GAIN_TORQUE*T
	
	########################################
	# seta steer do veiculo (st in rad)		
	def set_steer(self, st):

		# limita comando em radianos
		st = np.clip(st, -np.deg2rad(MAX_STERRING_ANGLE), np.deg2rad(MAX_STERRING_ANGLE))

		# suaviza comando de esterçamento
		st = self.st_filt.filter(st)

		# camera/ultrassom acompanha o esterçamento
		if self.ultrasonic:
			self._set_pan(st)
		else:
			self._set_pan(0.0)

		# aplica trim e converte para graus
		st_deg = np.rad2deg(st + self.trim_steer)

		with self.lock:
			self.st_pwm = (GAIN_STERRING_ANGLE * st_deg	+ ZERO_STERRING_ANGLE)
	
	########################################
	# angulo de pan da camera/ultrasom (por enquanto nao deve ser operado externamente)
	def _set_pan(self, ang):
			
		ang = np.clip(ang, -np.deg2rad(90.0), np.deg2rad(90.0))

		# angulo centrado em zero
		pan_pwm = np.rad2deg(ang + self.trim_pan) + 90.0

		with self.lock:
			self.pan_pwm = np.clip(pan_pwm, 0.0, 180.0)
        
	########################################
	# seta o ajuste fino dos servos (em radianos)
	def set_trim(self, steer=np.deg2rad(0.0), throttle=np.deg2rad(0.0), pan=np.deg2rad(0.0)):
		
		# trim do estercamento
		self.trim_steer = np.clip(steer, -np.deg2rad(10.0), np.deg2rad(10.0))
		
		# trim da tracao
		self.trim_throttle = np.clip(throttle, -np.deg2rad(20.0), np.deg2rad(20.0))
		
		# trim do ultrasom
		self.trim_pan = np.clip(pan, -np.deg2rad(20.0), np.deg2rad(20.0))
		
	########################################
	# mode de marcha re
	def _backward(self):
		time.sleep(0.5)
		# da toquinos para tras
		for _ in range(4):
			self.kit.servo[SERVO_THROTTLE].angle = 0.5*ZERO_THROTTLE_ANGLE
			time.sleep(0.1)
			self.kit.servo[SERVO_THROTTLE].angle = ZERO_THROTTLE_ANGLE
			time.sleep(0.1)
	
	########################################
	# fecha comando dos servos
	def close(self):
		# para integracao de tracao
		with self.lock:
			self.dth_pwm = 0.0

		# centraliza direcao
		self.set_steer(0.0)

		# sinaliza encerramento da thread
		self.stop.set()

		# espera thread terminar
		if self.thread.is_alive():
			self.thread.join()

		# coloca ESC em neutro
		self.th_pwm = 0.0
		self._set_pwm(self.th_pwm)
		
########################################
# main teste
########################################
if __name__ == "__main__":
	
	# calibracao do ESC
	#ser.kit.servo[SERVO_THROTTLE].angle = 90 #90 (neutro), 180 (maximo), 0 (minimo)
	
	# cria servos
	ser = Servos(ultrasonic=True)
	print('Servos ok...')
	
	try:
		# testa servos
		t0 = time.monotonic()
		while (time.monotonic() - t0) <= 10.0:
			t = time.monotonic() - t0
			print(f"Tempo = {t:.2f} s", flush=True)
			
			# seta estercamento junto com ultrasom
			ser.set_steer(np.deg2rad(MAX_STERRING_ANGLE)*np.sin(0.5*t))
			# seta torque do motor
			ser.set_torque(0.2*np.sin(0.5*t))
			# espera
			time.sleep(ser.dt)
			
	finally:
		ser.close()
