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
			if imagem is None:
				print('Não foi possível carregar a imagem.')
			else:
				# Exiba a imagem em uma janela chamada 'Imagem'
				cv2.imshow('Imagem', self.getImage())
		
	########################################
	# destrutor
	def __del__(self):
		self.cam.close()
		
########################################
#cam = Camera()
#cam.test()

