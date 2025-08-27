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
plt.rcParams['figure.figsize'] = (8,6)

# Globais
parameters = {	'car_id'	: 0,
				'ts'		: 60.0, 			# tempo da simulacao
				'save'		: True,
				'logfile'	: 'logs/',
			}
	
########################################
# thread de controle de velocidade
########################################
def control_func(car):
		
	# seta direcao
	car.setSteer(np.deg2rad(5.0*np.sin(car.t)))

	# atua
	if car.t < 2.0:
		car.setU(0.2)
	else:
		car.setU(0.0)
		
########################################
# thread de visão
########################################
def vision_func(car):
		
	# pega imagem
	image = car.getImage()
	
	# ultrasom
	dist = car.getDistance()
	print('Ultrasonic distance: ', np.round(dist,2))
	
	return image
				
########################################
# executa controle
########################################
def run(parameters):
	
	plt.figure(1)
	plt.ion()
	
	# cria comunicação com o carrinho
	car = cp.Car(parameters)
	
	# começa a simulação
	car.startMission()

	# main loop
	while car.t <= parameters['ts']:
		
		# lê senores
		car.step()
		
		# funcao de controle
		control_func(car)
		
		# funcao de visao
		image = vision_func(car)
		
		########################################
		# plota	
		plt.subplot(211)
		plt.cla()
		plt.gca().imshow(image, origin='lower')
		plt.title('t = %.1f' % car.t)
		
		plt.subplot(212)
		plt.cla()
		t = [traj['t'] for traj in car.traj]
		v = [traj['v'] for traj in car.traj]
		plt.plot(t,v)
		plt.ylabel('v[m/s]')
		plt.xlabel('t[s]')
		
		plt.show()
		plt.pause(0.01)

	# termina a missao
	car.stopMission()
	# salva
	if parameters['save']:
		car.save(parameters['logfile'])
	
	plt.ioff()
	print('Terminou...')

########################################
########################################
if __name__=="__main__":
	run(parameters)
