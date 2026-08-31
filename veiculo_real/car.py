# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import numpy as np
import time, os
from datetime import datetime
try:
	from . import encoder, servos, buzzer, ultrasonic, imu, filter
except ImportError:
	import encoder, servos, buzzer, ultrasonic, imu, filter


# QUESTAO DA ORIENTACAO DO CARRINHO NA FUSAO DE POSICAO (COMECA SEMPRE PARA O LESTE)
# VERIFICAR SINAL DO GZ DA IMU
# VERIFICAR EIXO X DA ACELERACAO DA IMU
# VERIFICAR HEADING DA BUSSOLA COM NORTE MAGNETICO (yaw_geo = np.pi/2 - yaw_mag	)

########################################
# GLOBAIS
########################################
# parametros do carro
CAR = {
		'VELMAX'	: 1.5,				# m/s
		'ACCELMAX'	: 1.0, 				# m/s^2
		'STEERMAX'	: np.deg2rad(20.0),	# rad
		'MASS'		: 5.16,				# kg
		'L'			: 0.36,				# distancia entre os eixos das rodas
		'RW' 		: 0.08,				# raio da roda [m]
		'MI' 		: 0.04,				# constante de friccao
		'GRAV'   	: 9.81, 			# gravidade [m/s^2]
		'PERIOD'	: 50.0,				# periodo de amostragem dos sensores
	}
	
MACS_CARS = {
	'verde':    '2c:cf:67:1c:29:4a',
	'vermelho': 'd8:3a:dd:f1:8a:4f',
	'roxo':     '2c:cf:67:1c:29:07'
}

COLORS = {
	'verde':    '\033[32m',
	'vermelho': '\033[31m',
	'roxo':     '\033[35m'
}

RESET = '\033[0m'

########################################
# Carrinho
########################################
class Car:	
	########################################
	# construtor
	def __init__(self, parameters):
		
		self.parameters = parameters
		
		# detecta carrinho pronto
		self.color = self.get_car_color()
		
		# inicializa sensores
		self.init_sensors()
		
		# tempo
		self.t = 0.0
		# tempo de amostragem preterido
		self.sample_rate = 1.0/CAR['PERIOD']
		# tempo de amostragem real medido
		self.dt = 1.0/CAR['PERIOD']
		
		# velocidade de referencia
		self.vref = 0.0
		
		# variaveis calculadas
		self.p = np.zeros(2)
		self.p_gps = None
		self.th = 0.0
		self.w = 0.0
		self.v = 0.0
		self.a = 0.0
		
		# comando de aceleracao
		self.u = 0.0
		# comando de estercamento
		self.st = 0.0
		
		# botao de emergencia
		self.emergencia = True
		
		# marcha atual
		self.gear = self.atuador.get_gear()
		
		# filtros dos sinais
		self.v_filt    = filter.MovingAverage(n=30)
		self.a_filt    = filter.MovingAverage(n=30)
		self.vref_filt = filter.MovingAverage(n=100)
		self.w_filt    = filter.MovingAverage(n=20)
		
		# logs de salvamento
		if self.parameters['save']:
			# logs de salvamento
			timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
			self.logfile = os.path.join(parameters['logfile'], timestamp)
			# cria a pasta do experimento
			os.makedirs(self.logfile, exist_ok=True)
		
	########################################
	# inicializa sensores e atuadores
	def init_sensors(self):
		
		# atuadores de estercamento, aceleracao e ultrasom/camera	
		try:
			self.atuador = servos.Servos(ultrasonic=self.parameters['ultrasonic_steering'])
		except Exception as e:
			print(f"\033[31mErro nos servos: {e}\033[0m", flush=True)
			raise
		
		# odometro da roda
		try:
			self.odometer = encoder.Encoder()
		except Exception as e:
			print(f"\033[31mErro no velocimetro: {e}\033[0m", flush=True)
			raise
		
		# imu
		try:
			self.imu = imu.IMU()
		except Exception as e:
			print(f"\033[31mErro na IMU: {e}\033[0m", flush=True)
			raise
		
		# ultrasom
		try:
			self.us = ultrasonic.Ultrasonic()
		except Exception as e:
			print(f"\033[31mErro no ultrasom: {e}\033[0m", flush=True)
			raise
		
		# buzzer de sinalizacao
		try:
			self.bz = buzzer.Buzzer()
		except Exception as e:
			print(f"\033[31mErro no buzzer: {e}\033[0m", flush=True)
			raise
		
		# camera
		if self.parameters['camera']:
			try:
				from . import camera
				self.cam = camera.Camera()
			except Exception as e:
				print(f"\033[31mErro na camera: {e}\033[0m", flush=True)
				raise
		else:
			print("\033[33mCamera desativada.\033[0m", flush=True)
			
		# GPS opcional (via celular android)
		try:
			from . import gps
			self.gps = gps.GPS()

			if self.gps.is_available():
				print("\033[32mGPS disponivel.\033[0m", flush=True)
			else:
				# sem GPS, use apenas odometria
				self.gps = None
				print("\033[33mGPS nao disponivel.\033[0m", flush=True)
		except Exception as e:
			self.gps = None
			print(f"\033[33mGPS nao disponivel: {e}\033[0m", flush=True)
			
		# carro pronto
		if self.color is not None:
			print(f"{COLORS[self.color]}##############################{RESET}", flush=True)
			print(f"{COLORS[self.color]}Carro {self.color.upper()} pronto!{RESET}", flush=True)
			print(f"{COLORS[self.color]}##############################{RESET}", flush=True)
		else:
			print("\033[33m##############################\033[0m", flush=True)
			print("\033[33mCarro desconhecido pronto!\033[0m", flush=True)
			print("\033[33m##############################\033[0m", flush=True)

	########################################
	# comeca a missao
	def start_mission(self):
		
		# desliga a emergencia
		self.emergencia = False
		
		# define origem do GPS, se disponivel
		if self.gps is not None:
			if self.gps.set_origin():
				print("\033[32mOrigem GPS definida.\033[0m", flush=True)
				
				# heading inicial pela bussola
				_, _, yaw_mag = self.imu.get_euler(degrees=False)
				if yaw_mag is not None:
					self.th = yaw_mag
					print("\033[32mHeading inicial definido pela bussola.\033[0m", flush=True)
			else:
				print("\033[33mNao foi possivel definir origem GPS.\033[0m", flush=True)
				
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
		
		# se esta dando re, avise
		if self.gear == servos.Gear.REVERSE:
			self.bz.beep(0.3, silence=0.5)
		
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
		return float(time.monotonic())
		
	########################################
	# retorna posicao estimada do carro
	def get_pos(self):

		# predicao pelo modelo cinematico
		x = self.p[0] + self.v*np.cos(self.th)*self.dt
		y = self.p[1] + self.v*np.sin(self.th)*self.dt
		p = np.array((x, y))

		# correcao com GPS, se disponivel
		if self.gps is not None:
			position = self.gps.get_position()
			accuracy = self.gps.get_accuracy()

			# se posicao eh nova
			if position is not None and self.gps.new_position:
				p_gps = self.gps.get_xy(position)

				if p_gps is not None:
					self.p_gps = np.array(p_gps)

					# fusao sensorial simples
					K = 0.1
					p = (1.0 - K)*p + K*self.p_gps

		# retorna posicao
		return p
		
	########################################
	# retorna yaw (usa bussola se tiver GPS)
	def get_yaw(self):

		# predicao pela integracao da velocidade angular
		yaw = self.th + self.w*self.dt

		# corrige com bussola somente se houver GPS
		if self.gps is not None:
			_, _, yaw_mag = self.imu.get_euler(degrees=False)

			if yaw_mag is not None:
				K = 0.05
				# erro angular corretamente embrulhado
				error = np.arctan2(np.sin(yaw_mag - yaw), np.cos(yaw_mag - yaw))
				yaw += K*error

		# mantem entre 0 e 2*pi
		yaw = yaw % (2.0*np.pi)

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

		# velocidade angular pelo modelo cinematico
		w_model = (vf / CAR['L']) * np.tan(self.st)

		# velocidade angular medida pela IMU
		_, _, gz = self.imu.get_gyro()
		w_imu = np.deg2rad(gz)

		# fusao modelo + IMU
		K = 0.8
		w = (1.0 - K)*w_model + K*w_imu

		# filtra velocidade angular
		wf = self.w_filt.filter(w)

		return vf, wf
	
	########################################
	# retorna aceleracao
	def get_accel(self):

		if self.dt == 0.0:
			return 0.0

		# aceleracao pelo encoder
		a_model = (self.v - self.v_ant)/self.dt

		# aceleracao medida pela IMU
		a_x, _, _ = self.imu.get_accel()

		# fusao sensorial
		K = 0.2
		a = (1.0 - K)*a_model + K*a_x

		# filtra
		af = self.a_filt.filter(a)

		return af
	
	########################################
	# seta referencia de controle
	def set_ref(self, vref):
		# em caso de emergencia, pare
		if self.emergencia:
			self.vref = 0.0
			return self.vref
			
		# referencia filtrada de velocidade
		self.vref = self.vref_filt.filter(vref)
		self.vref = np.clip(self.vref, -CAR['VELMAX'], CAR['VELMAX'])
		
		# se eh para dar re e estou indo para frente
		if (self.vref < 0.0) and (self.gear == servos.Gear.FORWARD):
			self.set_reverse()
		
		# se eh para ir para frente e estou dando re
		elif (self.vref > 0.0) and (self.gear == servos.Gear.REVERSE):
			self.set_forward()
			
		return self.vref
			
	########################################
	# seta torque do veiculo
	def set_vel(self, vref):
		
		# ganhos
		Kp = 0.4
		Kd = 0.2

		# define referencia e marcha
		self.set_ref(vref)

		# controla magnitude da velocidade
		vref_abs = abs(self.vref)
		v_abs = abs(self.v)

		# aceleracao da magnitude
		a_abs = np.sign(self.v) * self.a

		# controle PD
		u = Kp*(vref_abs - v_abs) - Kd*a_abs
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
		if np.abs(self.v) > CAR['VELMAX']:
			self.u = 0.0
			
		# controlador linearizante
		F = CAR['MASS']*self.u
		
		# torque de referencia
		T = CAR['RW']*np.sum(F)
		
		# seta o torque
		self.atuador.set_torque(T)

	########################################
	# coloca re
	def set_reverse(self):
		self.atuador.set_reverse()
		self.gear = self.atuador.get_gear()
		
	########################################
	# vai pra frente
	def set_forward(self):
		self.atuador.set_forward()
		self.gear = self.atuador.get_gear()
		
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
	# save traj em csv		
	def save(self):
		filename = os.path.join(self.logfile, 'car.csv')

		data = np.array([
			[
				traj['t'],
				traj['p'][0],
				traj['p'][1],
				traj['v'],
				traj['a'],
				traj['vref'],
				traj['th'],
				traj['w'],
				traj['u']
			]
			for traj in self.traj
		])

		header = 't,x,y,v,a,vref,th,w,u'

		np.savetxt(filename, data, delimiter=',', header=header,  comments='')
	
	########################################
	# termina a missao
	def stop_mission(self):
		
		# aperta a emergencia
		self.emergencia = True
		
		# termina parado
		self.set_u(-CAR['ACCELMAX'])
		self.set_steer(0.0)
		
		# espera ate parar
		while abs(self.v) > 0.1:
			self.step()
			time.sleep(0.1)
		
		# sinaliza fim
		time.sleep(1.0)
		self.bz.victory_tune()
		
	########################################
	# qual eh o carrinho?
	def get_car_color(self):

		# procura os MACs das interfaces de rede
		for interface in os.listdir('/sys/class/net'):
			path = f'/sys/class/net/{interface}/address'
			try:
				with open(path, 'r') as f:
					mac = f.read().strip().lower()
			except OSError:
				continue
			# procura o MAC conhecido
			for color, known_mac in MACS_CARS.items():
				if mac == known_mac.lower():
					return color
		
		return None
		
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
		if self.gps is not None:
			self.gps.close()
			
		# acabou
		if self.color is not None:
			print(f"{COLORS[self.color]}##############################{RESET}", flush=True)
			print(f"{COLORS[self.color]}Missao terminada!{RESET}", flush=True)
			print(f"{COLORS[self.color]}##############################{RESET}", flush=True)
		else:
			print("\033[33m##############################\033[0m", flush=True)
			print("\033[33mMissao terminada!\033[0m", flush=True)
			print("\033[33m##############################\033[0m", flush=True)
		
########################################
# main teste
########################################
if __name__ == "__main__":
	
	# Globais
	parameters = {	
				'ts'					: 30.0,		# tempo da execucao
				'save'					: False,	# salvar trajetoria
				'logfile'				: 'logs/',	# log file
				'camera'				: False,	# usar camera
				'ultrasonic_steering' 	: True,		# mover ultrasom com estercamento
				'us_buzzer'				: True,	# aviso sonoro para objetos proximos
				}
	
	# cria comunicacao com o carrinho
	car = Car(parameters)
	
	try:
		car.start_mission()
		
		# testa leitura
		t0 = time.monotonic()
		while (time.monotonic() - t0) <= parameters['ts']:
			t = time.monotonic() - t0
			
			# le sensores
			car.step()
			
			# le ultrasom
			dist, valid = car.get_distance()
			# seta torque do motor
			if valid and dist > 0.10:
				if t < parameters['ts']/2:
					car.set_vel(0.7)
				else:
					car.set_vel(-0.7)
			else:
				car.set_vel(0.0)
			#
			print(
				f"Vel: {car.v:+.2f} m/s | "
				f"Ref: {car.vref:+.2f} m/s | "
				f"Marcha: {car.gear.value}"
			)
				
			# seta estercamento junto com ultrasom
			car.set_steer(np.deg2rad(20.0)*np.sin(0.5*t))

		# salva os dados coletados
		if parameters['save']:
			car.save()
		
	finally:
		car.close()
