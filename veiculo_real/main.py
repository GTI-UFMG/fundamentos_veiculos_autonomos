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

########################################
def control():
	
	while car.t < 50.0:
		# lê senores
		car.step()
		
		# seta direcao
		car.setSteer(np.deg2rad(20.0*np.sin(0.1*car.t)))
		
		# atua
		car.setVel(refvel)
		
########################################
# cria comunicação com o carrinho
car = cp. Car()
car.startMission()

# Load the dictionary of ArUco markers.
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
# Create a parameters object for ArUco detection.
parameters = cv2.aruco.DetectorParameters_create()

refvel = 1.0

thread = threading.Thread(target=control)
thread.start()

plt.ion()
plt.figure(1)
	
########################################
while car.t < 50.0:
	
	# pega image
	frame = car.getImage(gray=True)
	
	# Detect ArUco markers in the frame.
	corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

	if ids is not None:
		# Draw detected markers on the frame.
		cv2.aruco.drawDetectedMarkers(frame, corners, ids)
		refvel = 1.5
	else:
		refvel = 1.0
	
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
	plt.pause(0.01)
	
print('Terminou...')
