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
plt.ion()
plt.figure(1)

########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia()
car.startMission()

while car.t < 50.0:
	
	# lê senores
	car.step()
	
	# atua
	car.setSteer(20.0*np.sin(0.2*car.getTime()))
	car.setVel(np.abs(np.sin(0.5*car.getTime())))
	
	# lê e exibe camera
	frame = car.getImage()
	image_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
	plt.clf()
	plt.gca().imshow(image_bgr, origin='lower')
	plt.show()
	plt.pause(0.01)
	
print('Terminou...')
