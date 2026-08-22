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
import class_buzzer
import class_filter
import class_ultrasonic
import class_imu

########################################
# GLOBAIS
########################################
# parametros do carro
CAR = {
		'VELMAX'	: 3.0,				# m/s
		'ACCELMAX'	: 1.0, 				# m/s^2
		'STEERMAX'	: np.deg2rad(20.0),	# rad
		'MASS'		: 5.16,				# kg
		'L'			: 0.36,				# distancia entre os eixos das rodas
		'RW' 		: 0.08,				# raio da roda [m]
		'MI' 		: 0.04,				# constante de friccao
		'GRAV'   	: 9.81, 			# gravidade [m/s^2]
		'PERIOD'	: 50.0,				# periodo de amostragem dos sensores
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
		self.init_sensors()
		
		# tempo
		self.t = 0.0
		# tempo de amostragem preterido
		self.sample_rate = 1.0/CAR['PERIOD']
		# tempo de amostragem real medido
		self.dt = 1.0/CAR['PERIOD']
		
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
		
		# botao de emergencia
		self.emergencia = True
		
		# filtros dos sinais
		self.v_filt    = class_filter.MovingAverage(n=20)
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
		
		print('##############################')
		print("\033[32mCarro pronto!\033[0m", flush=True)
		print('##############################')
		
	########################################
	# inicializa sensores e atuadores
	def init_sensors(self):
		
		# atuadores de estercamento, aceleracao e ultrasom/camera
		self.atuador = class_servos.Servos(ultrasonic=self.parameters['ultrasonic_steering'])
		print("\033[32mServos ok...\033[0m", flush=True)
		
		# odometro da roda
		self.odometer = class_encoder.Encoder()
		print("\033[32mOdometria ok...\033[0m", flush=True)
		
		# imu
		self.imu = class_imu.IMU()
		print("\033[32mIMU ok...\033[0m", flush=True)

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
		
		# buzzer de sinalizacao
		self.bz = class_buzzer.Buzzer()
		print("\033[32mBuzzer ok...\033[0m", flush=True)

	########################################
	# comeca a missao
	def start_mission(self):
		
		# desliga a emergencia
		self.emergencia = False
		
		# tempo inicial
		self.tinit = self.get_time()
		
		# estados iniciais
		self.get_states()
		
		# comeca parado
		self.set_u(0.0)
		self.set_steer(0.0)
		
		# salva trajetoria
		self.save_traj()
		
		# aviso sonoro de inicio
		self.bz.beep([0.2] * 3)
		time.sleep(1.0)
		
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
	# passo para atualizar sensores
	def step(self):
		# tempo anterior
		t0 = self.t
		
		# espera o periodo de delta t
		elapsed_time = self.get_time() - self.tinit - t0
		time.sleep(np.max([0.0, self.sample_rate - elapsed_time]))
		
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
	def get_time(self):
		return float(time.time())
					
	########################################
	# retorna posicao do carro - sem GPS
	def get_pos(self):
		x = self.p[0] + self.v*np.cos(self.th)*self.dt
		y = self.p[1] + self.v*np.sin(self.th)*self.dt
		return np.array((x, y))			
				
	########################################
	# retorna yaw - sem bussola
	def get_yaw(self):
		yaw = self.th + self.w*self.dt
		
		while yaw < 0.0:
			yaw += 2.0*np.pi
		while yaw > 2.0*np.pi:
			yaw -= 2.0*np.pi
		
		return yaw
		
	########################################
	# retorna velocidades linear e angular
	def get_vel(self):

		# le velocidade do encoder
		v, valid = self.odometer.get_vel()

		# somente atualiza velocidade se a medida for valida
		if valid:
			vf = self.v_filt.filter(v)
		else:
			vf = self.v

		# velocidade angular calculada pelo modelo cinematico
		w = (vf / CAR['L']) * np.tan(self.st)
		wf = self.w_filt.filter(w)

		return vf, wf
	
	########################################
	# retorna aceleracao
	def get_accel(self):
		
		if self.dt == 0.0:
			return 0.0
		
		# aceleracao sem IMU, calculada artificialmente
		a = (self.v - self.v_ant)/self.dt
		af = self.a_filt.filter(a)
		
		return af
	
	########################################
	# seta referencia de controle
	def set_ref(self, vref):
		# em caso de emergencia, pare
		if self.emergencia:
			self.vref = 0.0
		else:		
			# referencia filtrada de velocidade
			self.vref = self.vref_filt.filter(vref)
			self.vref = np.clip(self.vref, 0.0, CAR['VELMAX'])
			
		return self.vref
			
	########################################
	# seta torque do veiculo
	def set_vel(self, vref):
		
		# ganhos
		Kp = 0.4
		Kd = 0.2
		
		# seta referencia
		self.set_ref(vref)
		
		# controle de velocidade
		u = Kp*(self.vref - self.v) + Kd*(-self.a)
		self.set_u(u)
	
	########################################
	# seta torque dos motores do veiculo
	def set_u(self, u):
		
		# em caso de emergencia, desacelere no maximo
		if self.emergencia:
			u = -CAR['ACCELMAX']
		
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
		if (self.v < 0.1) and (u < 0.0):
			T = 0.0
		
		# seta o torque
		self.atuador.set_torque(T)

	########################################
	# seta steer do veiculo
	def set_steer(self, st):
		# emergencia
		if self.emergencia:
			st = 0.0
		
		# limita angulo de estercamento
		self.st = np.clip(st, -CAR['STEERMAX'], CAR['STEERMAX'])
		
		# atua no volante
		self.atuador.set_steer(self.st)
		
	########################################
	# get image data
	def get_image(self, gray=False):
		return self.cam.get_image(gray)
		
	########################################
	# get ultrasonic distance
	def get_distance(self, max_dist=4.0, d_min=0.3):
		
		# captura a distancia
		d, valid = self.us.get_distance()
		d = np.min([d, max_dist])

		# toca o buzzer se muito proximo ou invalido
		if self.parameters['us_buzzer']:
			if not valid:
				self.bz.beep(0.1)
			elif d <= d_min:
				self.bz.beep(d/3.0)
				
		# retorna distancia
		return d , valid
	
	########################################
	# save traj
	def save(self):
		filename = self.logfile + 'card.npz'
		data = [traj for traj in self.traj]
		np.savez(filename, data=data)
		
	########################################
	# load traj
	def load(self):
		filename = self.logfile + 'car.npz'
		data = np.load(filename, allow_pickle=True)
		self.traj = data['data']
	
	########################################
	# termina a missao
	def stop_mission(self):
		
		# aperta a emergencia
		self.emergencia = True
		
		# termina parado
		self.set_u(-CAR['ACCELMAX'])
		self.set_steer(0.0)
		
		# espera ate parar
		while self.v > 0.1:
			self.step()
			time.sleep(0.1)
		
		# sinaliza fim
		time.sleep(1.0)
		self.bz.victory_tune()
		
	########################################
	# termina a classe
	def close(self):
		# para o carrinho
		self.stop_mission()
		
		# fecha tudo
		self.bz.close()
		self.odometer.close()
		self.atuador.close()
		self.us.close()
		self.imu.close()
		if self.parameters['camera']:
			self.cam.close()
			
		print ("\033[32mMissao terminada!\033[0m")
		
########################################
# main teste
########################################
if __name__ == "__main__":
	
	# Globais
	parameters = {	
				'ts'					: 15.0,		# tempo da execucao
				'save'					: False,	# salvar trajetoria
				'logfile'				: 'logs/',	# log file
				'camera'				: False,	# usar camera
				'ultrasonic_steering' 	: True,		# mover ultrasom com estercamento
				'us_buzzer'				: True,	# aviso sonoro para objetos proximos
				}
	
	# cria comunicacao com o carrinho
	car = Car(parameters)
	car.start_mission()
	
	# testa leitura
	t0 = time.time()
	while (time.time() - t0) <= parameters['ts']:
		t = time.time() - t0
		
		# le sensores
		car.step()
		
		# le ultrasom
		dist, valid = car.get_distance()
		# seta torque do motor
		if valid and dist > 0.10:
			car.set_vel(0.7)
		else:
			car.set_vel(0.0)
		#
		print(f"Distance: {dist:.2f} [m]")
			
		# seta estercamento junto com ultrasom
		car.set_steer(np.deg2rad(20.0)*np.sin(0.5*t))

	# salva os dados coletados
	if parameters['save']:
		car.save()
		
	car.close()
