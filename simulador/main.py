# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2024/1
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import class_car as cp
import numpy as np
import cv2
import matplotlib.pyplot as plt

plt.figure(1)
plt.ion()

########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia()

# começa a simulação
car.startMission()

while car.t < 50.0:
	
	# lê senores
	car.step()
	
	# seta direcao
	car.setSteer(np.deg2rad(5.0*np.sin(car.t)))
	
	# atua
	if car.t < 2.0:
		car.setU(0.2)
	else:
		car.setU(0.0)
	
	# pega imagem
	image = car.getImage()
	
	########################################
	# plota	
	plt.subplot(121)
	plt.cla()
	plt.gca().imshow(image, origin='lower')
	plt.title('t = %.1f' % car.t)
	
	plt.subplot(122)
	plt.cla()
	t = [traj['t'] for traj in car.traj]
	v = [traj['v'] for traj in car.traj]
	plt.plot(t,v)
	plt.ylabel('v[m/s]')
	plt.xlabel('t[s]')
	
	plt.show()
	plt.pause(0.01)

car.stopMission()
print('Terminou...')
