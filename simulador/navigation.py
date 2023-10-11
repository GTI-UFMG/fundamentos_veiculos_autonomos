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
plt.figure(1, figsize=(10, 6))

# HSV values
GREEN_HSV_MIN = [35, 150, 50]
GREEN_HSV_MAX = [85, 255, 255]
BLUE_HSV_MIN  = [90, 150, 50]
BLUE_HSV_MAX  = [130, 255, 255]

WHITE_HSV_MIN  = [0,    0, 150]
WHITE_HSV_MAX  = [255, 30, 255]

# mapa de plot
mapa = cv2.cvtColor(cv2.imread('./coppelia/pista.png'), cv2.COLOR_RGB2BGR)

########################################
# detecta blobs em HSV
def blob(image, min_hsv, max_hsv, color=(0, 255, 0)):
	
	# Convert the image to the HSV color space
	hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

	# Define the lower and upper bounds of the color you want to detect
	lower_color = np.array(min_hsv) 	# hue_min, saturation_min, value_min
	upper_color = np.array(max_hsv) 	# hue_max, saturation_max, value_max

	# Create a mask using the inRange() function to extract the color of interest
	mask = cv2.inRange(hsv, lower_color, upper_color)

	# Find contours in the mask
	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	# Initialize variables to keep track of the largest blob
	largest_area = 0
	largest_contour = None

	# Loop through the contours and calculate areas
	for i, contour in enumerate(contours):
		# Calculate the area of the contour
		area = cv2.contourArea(contour)
		
		# If the current blob has a larger area, update the largest area and contour
		if area > largest_area:
			largest_area = area
			largest_contour = contour
	
	# Draw the largest contour on the original image
	cx = cy = -1
	if largest_contour is not None:
		# Calculate the centroid of the largest contour
		M = cv2.moments(largest_contour)
		cx = int(M["m10"] / M["m00"])
		cy = int(M["m01"] / M["m00"])

		cv2.drawContours(image, [largest_contour], -1, color, 2)  # Green contour around the largest blob
	
	return image, largest_area, cx, cy
	
########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia()
car.startMission()

x = []
y = []
time = []

while car.t < 100.0:
	
	# lê senores
	car.step()
		
	# pega imagem
	image = car.getImage()
	
	# blobs
	image, green_area, _, _ = blob(image, GREEN_HSV_MIN, GREEN_HSV_MAX, color=(0, 255, 0))
	image, blue_area, _, _  = blob(image, BLUE_HSV_MIN,  BLUE_HSV_MAX,  color=(0, 0, 255))
	image, white_area, cx, cy = blob(image, WHITE_HSV_MIN, WHITE_HSV_MAX,  color=(255, 255, 255))
	
	# estercamento
	if cx > 0:
		refx = image.shape[1]/2.0
		delta = -0.02*(refx - cx)
	else:
		delta = 0.0
	
	if green_area > blue_area:
		delta += -5.0
	if green_area < blue_area:
		delta += 5.0
		
	# seta direcao
	car.setSteer(delta)
	
	# velocidade
	car.setVel(2.0)
	
	########################################
	# salva e mostra valores	
	plt.subplot(121)
	plt.cla()
	plt.gca().imshow(image, origin='lower')
	plt.title('t = %.1f' % car.t)
	
	plt.subplot(122)
	plt.cla()
	x.append(car.p[0])
	y.append(car.p[1])
	time.append(car.t)
	#
	# plota mapa
	plt.imshow(mapa, extent=[-7.5, 7.5, -7.5, 7.5], alpha=0.99)
	plt.plot(x, y, 'r')
	plt.axis('equal')
	plt.box(False)
	plt.ylabel('y [m]')
	plt.ylabel('x [m]')
	
	plt.show()
	plt.pause(0.01)

car.stopMission()
print('Terminou...')
