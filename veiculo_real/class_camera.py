# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import time
import cv2
from picamera2 import Picamera2

RESOLUTION = (1296, 972) #(640, 480)
FORMAT = 'XRGB8888'

########################################
# classe da camera
########################################
class Camera:
	########################################
	# construtor
	def __init__(self):
		
		# inicializa e configura a camera
		self.cam = Picamera2()
		self.cam.configure(self.cam.create_preview_configuration(main={"format": FORMAT, "size": RESOLUTION}))
		self.cam.start()
		
		self.yaw = 0.0
		self.cx = 0
		
	########################################
	def getImage(self, gray=False):
		
		# captura imagem RGB
		img = self.cam.capture_array()
		
		# se pedir gray, converte
		if gray:
			img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		
		return img
		
	########################################
	# função para testar a camera
	def test(self, total_time=10.0):
		ti = float(time.time())
		while (float(time.time()) - ti) <= total_time:
			img = self.getImage()
			if img is None:
				print('Não foi possível carregar a imagem.')
			else:
				# Exiba a imagem em uma janela chamada 'Imagem'
				cv2.imshow('Imagem', img)
		
	########################################
	# detecta o teto
	def getRef(self):
		
		# Define the lower and upper bounds for black in HSV
		lower_black = np.array([0, 0, 0])
		upper_black = np.array([255, 100, 50])
		
		# pega a image
		frame = self.getImage()
		W = frame.shape[1]
		H = frame.shape[0]
		
		# aplica filtro
		frame = cv2.GaussianBlur(frame, (7,7), 0)
		
		# Convert the frame to the HSV color space
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		# Create a binary mask for the black objects
		mask = cv2.inRange(hsv, lower_black, upper_black)
		
		# Define a kernel for erosion
		kernel = np.ones((7, 7), np.uint8)

		# Perform opening
		mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
		
		# Find contours in the mask
		contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
				
		# Find the largest contour (assumed to be the object of interest)
		if len(contours) > 0:
			largest_contour = max(contours, key=cv2.contourArea)
		
		try:
			ALFA = 0.9
			BETA = 0.9
			
			# Fit an ellipse to the largest contour
			ellipse = cv2.fitEllipse(largest_contour)
			
			# Extract the angle of the major axis from the ellipse
			yaw_m = ellipse[2] - 90.0
			if 0.0 < yaw_m < 180.0:
				yaw_m -= 180.0
			self.yaw = ALFA*self.yaw + (1.0-ALFA)*yaw_m
			
			self.cx = int(BETA*self.cx + (1.0-BETA)*ellipse[0][0])
			self.cx = np.clip(self.cx, 0, W)
			cy = int(H/2)

			# Calculate the endpoint of the vector based on the centroid and orientation angle
			vector_length = 100  # Adjust the length of the vector as needed
			end_point = (
				int(self.cx + vector_length * np.cos(np.radians(yaw))),
				int(cy + vector_length * np.sin(np.radians(yaw)))
			)
			
			return self.cx, self.yaw
		
	########################################
	# destrutor
	def __del__(self):
		self.cam.close()
		
########################################
#cam = Camera()
#cam.test()

