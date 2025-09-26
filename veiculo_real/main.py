# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2025/2
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

MAIN_VEL = 0.5 # m/s
refste = np.deg2rad(0.0)
frame = None

########################################
# thread de controle de velocidade
def control_func():
	
	global car
	global refste
	
	while True:
		# le sensores
		car.step()
		
		# seta direcao
		car.setSteer(refste)
		
		# atua
		car.setVel(MAIN_VEL)
		
		# espera
		time.sleep(0.005)
		
########################################
# thread de visao
def vision_func():
	
	global car
	global refste
	global frame
	
	# pega resolucao da imagem
	W, H = car.cam.getResolution()
	
	# loop principal
	while True:
		# pega image
		frame = car.getImage(gray=True)
		
		# detecta aruco
		frame, point = car.cam.detectAruco(frame, aruco_id=23)
		
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
				'ts'					: 10.0, 	# tempo da execucao
				'save'					: True,		# salva dados da trajetoria
				'logfile'				: 'logs/',	# log file
				'camera'				: False,	# habilitar camera e thread de visao
				'ultrasonic_steering' 	: True,		# mover ultrasom com estercamento
			}
	
	# cria comunicacao com o carrinho
	car = class_car.Car(parameters)
	car.startMission()

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

	# desliga o carro
	car.close()
	
	print('Terminou...')
