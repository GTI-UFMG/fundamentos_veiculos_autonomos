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
car = cp. Car()
car.startMission()

while car.t < 50.0:
	
	# lê senores
	car.step()
	
	# seta direcao
	car.setSteer(np.deg2rad(20.0*np.sin(0.1*car.t)))
	
	# atua
	car.setVel(1.0)
	
	# plota
	plt.clf()
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
