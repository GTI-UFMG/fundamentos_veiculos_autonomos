# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import class_car
import numpy as np
import matplotlib.pyplot as plt
import threading
import time

MAIN_VEL = 0.7 # m/s
refste = np.deg2rad(0.0)
frame = None
stop_event = threading.Event()

########################################
# thread de controle de velocidade
def control_func():
	
	global car
	global refste
	
	while not stop_event.is_set():
		# le sensores
		car.step()
		
		# seta direcao
		car.set_steer(refste)
		
		# atua se nao houver colisao
		dist, valid = car.get_distance()
		#
		if (not valid) or (dist < 0.20):
			print(f"Colisao: distance {dist:.2f} [m]")
			car.set_vel(0.0)
		else:
			car.set_vel(MAIN_VEL)
		
########################################
# thread de visao
def vision_func():
	
	global car
	global refste
	global frame
	
	# pega resolucao da imagem
	W, H = car.cam.get_resolution()
	
	# loop principal
	while not stop_event.is_set():
		# pega image
		frame = car.get_image(gray=True)
		
		# detecta aruco
		frame, point = car.cam.detect_aruco(frame, aruco_id=23)
		
		# nao vi nada, continua
		if point is None:
			continue
			
		# estercamento aponta para o aruco
		cx = point[0] - W/2
		refste = -np.deg2rad(20.0*cx/(W/2))

########################################
# main program
########################################
if __name__ == "__main__":
	
	# Globais
	parameters = {	
				'ts'					: 20.0, 	# tempo da execucao
				'save'					: True,		# salva dados da trajetoria
				'logfile'				: 'logs/',	# log file
				'camera'				: False,	# habilitar camera e thread de visao
				'ultrasonic_steering' 	: False,		# mover ultrasom com estercamento
				'us_buzzer'				: False,		# aviso sonoro para objetos proximos
			}
	
	# cria comunicacao com o carrinho
	car = class_car.Car(parameters)
	thread_control = None
	thread_vision = None

	try:
		car.start_mission()

		# disparar threads
		thread_control = threading.Thread(target=control_func, daemon=True)
		thread_control.start()
		
		if parameters['camera']:
			thread_vision = threading.Thread(target=vision_func, daemon=True)
			thread_vision.start()

		plt.ion()
		plt.figure(1)

		# loop principal
		while car.t < parameters['ts']:
			
			# plota
			if parameters['camera']:
				plt.subplot(211)
				plt.cla()
				plt.gca().imshow(frame, cmap='gray')
				
				plt.subplot(212)
				
			plt.cla()
			t = [traj['t'] for traj in car.traj]
			v = [traj['v'] for traj in car.traj]
			vref = [traj['vref'] for traj in car.traj]
			plt.plot(t, v, 'k')
			plt.plot(t, vref, 'r--')
			plt.ylabel('Vel')
			plt.xlabel('Time')
			
			plt.show()
			plt.pause(1.0)

		# salva os dados coletados
		if parameters['save']:
			car.save()

	finally:
		# termina thread de controle
		stop_event.set()
		if thread_control is not None:
			thread_control.join()
		if parameters['camera']:
			if thread_vision is not None:
				thread_vision.join()
			
		# desliga o carro
		car.close()
		
	print('Terminou...')
