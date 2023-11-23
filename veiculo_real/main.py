# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import class_car as cp
import numpy as np
import cv2
import matplotlib.pyplot as plt
import threading
import time

# cria carrinho
car = cp. Car()
car.startMission()

terminar = False

MAIN_VEL = 0.8

refvel = MAIN_VEL
refste = np.deg2rad(0.0)
frame = car.getImage(gray=True)
W = frame.shape[1]
H = frame.shape[0]

plt.ion()
plt.figure(1)

########################################
# thread de controle de velocidade
def control_func():
	
	global car
	global refste
	global refvel
	
	while not terminar:
		# lê sensores
		car.step()
		
		# seta direcao
		car.setSteer(refste)
		
		# atua
		car.setVel(refvel)
		
		#if 5.0 < car.t < 10.0:
		#	car.setU(0.5)
		#else:
		#	car.setU(0.0)
		
		# espera
		time.sleep(0.01)
		
########################################
# thread de visão
def vision_func():
	
	global car
	global refste
	global refvel
	global frame
	
	# Load the dictionary of ArUco markers.
	aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
	# Create a parameters object for ArUco detection.
	parameters = cv2.aruco.DetectorParameters_create()
	
	while not terminar:
		
		# pega image
		frame = car.getImage(gray=True)
		
		# Detect ArUco markers in the frame.
		corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
		
		# velocidade padrão
		refvel = MAIN_VEL
		
		# nao vi nada, continua
		if ids is None:
			continue
		
		# Iterate through detected markers
		for i, marker_id in enumerate(ids):
			
			# se vir o aruco 24
			if np.squeeze(marker_id) == 24:
							
				# Get the corner coordinates of the current marker
				marker_corners = corners[i][0]
				
				# Calculate the centroid of the marker
				centroid_x = int(sum(marker_corners[:, 0]) / 4)
				centroid_y = int(sum(marker_corners[:, 1]) / 4)
				
				# aumenta velocidade de referencia
				refvel = 1.5*MAIN_VEL
				
				# estercamento aponta para o aruco
				cx = centroid_x - W/2
				refste = -np.deg2rad(20.0*cx/(W/2))

########################################
# disparar threads
thread_control = threading.Thread(target=control_func)
thread_vision = threading.Thread(target=vision_func)
thread_control.start()
thread_vision.start()

########################################
# loop principal
while car.t < 30.0:
	
	# plota
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
	
plt.pause(10.0)

terminar = True
thread_control.join()
thread_vision.join()
del car
print('Terminou...')
