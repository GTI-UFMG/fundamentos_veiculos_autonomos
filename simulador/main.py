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
plt.ion()
plt.figure(1)

########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia()
car.startMission()

while car.t < 50.0:
	
	# lê senores
	car.step()
	
	# seta direcao
	delta = -2.0*(0.0 - car.p[0]) + 5.0*(0.0 - car.w)
	car.setSteer(-delta)
	
	# atua
	if car.t < 2.0:
		car.setU(0.1)
	else:
		car.setU(0.0)
	
	# plota
	plt.clf()
	t = [traj['t'] for traj in car.traj]
	v = [traj['v'] for traj in car.traj]
	plt.plot(t,v)
	plt.show()
	plt.pause(0.01)
	
print('Terminou...')
