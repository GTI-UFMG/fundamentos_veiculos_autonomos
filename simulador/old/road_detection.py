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
def roadDetection(frame_bgr):
	
	# converte pra tons de cinza
	frame_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
	
	# aplica filtro
	frame_blur = cv2.GaussianBlur(frame_gray, (5,5), 0)
	
	# detecta bordas
	edges = cv2.Canny(frame_blur, 50, 155)
	
	# calcula Hough Line Transform
	lines = cv2.HoughLines(edges, 1, np.pi/180, threshold=30, min_theta=np.radians(-50), max_theta=np.radians(50))  # Adjust threshold as needed

	# desenha linhas
	image_with_lines = frame_bgr.copy()
	
	if lines is not None:
		for rho, theta in lines[:, 0]:
			
			if theta > np.pi/2:
				continue
			
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a * rho
			y0 = b * rho
			x1 = int(x0 + 1000 * (-b))
			y1 = int(y0 + 1000 * (a))
			x2 = int(x0 - 1000 * (-b))
			y2 = int(y0 - 1000 * (a))
			cv2.line(image_with_lines, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Draw lines in red
			
	return image_with_lines

########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia()
car.startMission()

while car.t < 50.0:
	
	# lê senores
	car.step()
	
	# atua
	delta = -2.0*(0.0 - car.p[0]) + 5.0*(0.0 - car.w)
	car.setSteer(-delta)
	
	if car.t < 1.0:
		car.setU(1.0)
	else:
		car.setU(0.0)
		
	# detecta estrada	
	image_with_lines = roadDetection(car.getImage())
	
	########################################
	plt.clf()
	plt.gca().imshow(image_with_lines, origin='lower')
	#plt.gca().imshow(edges, origin='lower', cmap='gray')
	#
	plt.axis('off')
	plt.show()
	plt.pause(0.01)
	
print('Terminou...')
