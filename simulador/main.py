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
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = (8,6)

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
	image = car.get_image()
	
	# ultrasom
	dist, _ = car.get_distance()
	#print('Ultrasonic distance: ', np.round(dist,2))
	
	return image
				
########################################
# executa controle
########################################
def run(parameters):
	
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
		car.stop_mission()
		# salva
		if parameters['save']:
			car.save(parameters['logfile'])
			
	finally:
		car.close()
	
	plt.ioff()

########################################
########################################
if __name__=="__main__":
	run(parameters)
