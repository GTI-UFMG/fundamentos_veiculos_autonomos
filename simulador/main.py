# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2026/1
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import class_car as cp
import numpy as np
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = (10,10)

# Globais
parameters = {	
				'ts'		: 10.0, 			# tempo da simulacao
				'save'		: True,
				'logfile'	: 'logs/',
			}
	
########################################
# thread de controle de velocidade
########################################
def control_func(car):
		
	# seta direcao
	car.set_steer(np.deg2rad(5.0*np.sin(car.t)))

	# atua
	if car.t < 5.0:
		car.set_u(0.3)
	else:
		car.set_u(0.0)
		
########################################
# thread de visão
########################################
def vision_func(car):
		
	# pega imagem
	image = car.get_image(gray=False)
	
	# ultrasom
	dist, _ = car.get_distance()
	print(f'Ultrasonic distance: {dist:.1f}')
	
	return image
				
########################################
# main program
########################################
if __name__ == "__main__":
	
	plt.figure(1)
	plt.ion()
	
	# cria comunicação com o carrinho
	car = cp.Car(parameters)
	
	try:
		# começa a simulação
		car.start_mission()

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
			plt.gca().imshow(image, cmap='gray')
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

		# salva
		if parameters['save']:
			car.save()
			
	finally:
		car.close()
