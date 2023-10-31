# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import cv2
from picamera2 import Picamera2

RESOLUTION = (640, 480)
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
		
	########################################
	def getImage(self, gray=False):
		
		# captura imagem RGB
		img = self.cam.capture_array()
		
		# se pedir gray, converte
		if gray:
			img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		
		return img
		
	########################################
	# destrutor
	def __del__(self):
		self.cam.close()
