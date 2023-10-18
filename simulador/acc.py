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

DIST_REF = 5.0
VREF = 1.0

# detector de arucos
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

"camera parameters"
fx = 519 
fy = 519
cx = 300 #image.shape[1]/2.0 #600 
cy = 200 #image.shape[0]/2.0 #400
cameraMatrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)
distorsionCoeff = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

objPoints = 0.3*np.array([[-0.5, -0.5, 0.], [0.5, -0.5, 0.], [0.5, 0.5, 0.], [-0.5, 0.5, 0.]])

########################################
def detect_Aruco(image):
	
	# Convert the frame to grayscale (Aruco marker detection works better in grayscale)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	
	# retira ruido
	gray = cv2.GaussianBlur(gray, (5,5), 0)

	# detecta arucos
	markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)
	
	# Draw detected markers
	if markerIds is not None:
		cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds)
	
	distance = None
	for marker_corner in markerCorners:
		valid, rvec, tvec = cv2.solvePnP(objPoints, marker_corner, cameraMatrix, distorsionCoeff)
		cv2.drawFrameAxes(image, cameraMatrix, distorsionCoeff, rvec, tvec, 1)
		
		# Calculate the Euclidean distance
		distance = np.linalg.norm(tvec)
		
	return image, distance
		
########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia()
car.startMission()

dist = []

while car.t < 100.0:
	
	# lê senores
	car.step()
		
	# pega imagem
	image = car.getImage()
	
	# detecta AR marker
	image_ar, distance = detect_Aruco(image)
		
	# seta direcao
	delta = -2.0*(0.0 - car.p[0]) + 5.0*(0.0 - car.w)
	car.setSteer(-delta)
	
	# velocidade
	if distance:
		Kp = -0.05
		Kv =  0.10
		u = Kp*(DIST_REF - distance) + Kv*(VREF - car.v)
		car.setU(u)
		
	dist.append(distance)
	
	########################################
	# salva e mostra valores	
	plt.subplot(121)
	plt.cla()
	plt.gca().imshow(image_ar, origin='lower', cmap='gray')
	plt.title('t = %.1f' % car.t)
	
	plt.subplot(122)
	plt.cla()
	vel = [traj['v'] for traj in car.traj]
	time = [traj['t'] for traj in car.traj]
	plt.plot(time, dist)
	plt.plot([time[0], time[-1]], [DIST_REF, DIST_REF], 'r--')
	
	plt.show()
	plt.pause(0.01)

car.stopMission()
print('Terminou...')
