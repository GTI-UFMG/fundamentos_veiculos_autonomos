# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
# -*- coding: utf-8 -*-
from fva_car import Car
import numpy as np
import matplotlib.pyplot as plt
import threading
import time

MAIN_VEL = 0.7
refste = 0.0
frame = None
stop_event = threading.Event()

########################################
# thread de visao
def vision_func():

	global car
	global refste
	global frame

	W, H = car.cam.get_resolution()

	while not stop_event.is_set():

		# pega imagem
		frame = car.get_image(gray=True)

		# detecta aruco
		frame, point = car.cam.detect_aruco(frame, aruco_id=23)

		if point is None:
			continue

		# esterçamento aponta para o aruco
		cx = point[0] - W/2
		refste = -np.deg2rad(20.0*cx/(W/2))


########################################
# main
########################################
if __name__ == "__main__":

	parameters = {
		'ts'                   : 20.0,
		'save'                 : True,
		'logfile'              : 'logs/',
		'camera'               : False,
		'ultrasonic_steering'  : False,
		'us_buzzer'            : False,
		'initial_position'     : [0, 0, np.deg2rad(0)]
	}

	car = Car(parameters)

	thread_vision = None

	try:
		car.start_mission()

		# inicia visao somente se solicitada
		if parameters['camera']:
			thread_vision = threading.Thread(
				target=vision_func,
				daemon=True
			)
			thread_vision.start()

		if parameters['camera']:
			plt.ion()
			plt.figure(1)

		t_plot = time.monotonic()

		# controle fica na thread principal
		while car.t < parameters['ts']:

			# atualiza sensores
			if not car.step():
				break

			# direcao
			car.set_steer(refste)

			# ultrassom
			dist, valid = car.get_distance()

			if (not valid) or (dist < 0.20):
				print(f"Colisao: distance {dist:.2f} [m]")
				car.set_vel(0.0)
			else:
				car.set_vel(MAIN_VEL)

			# telemetria para plots remotos
			print(
				f"DATA,"
				f"{car.t:.3f},"
				f"{car.p[0]:.3f},"
				f"{car.p[1]:.3f},"
				f"{car.v:.3f},"
				f"{car.vref:.3f},"
				f"{car.a:.3f},"
				f"{car.u:.3f},"
				f"{car.w:.3f},"
				f"{car.th:.3f}",
				flush=True
			)

			# atualiza grafico aproximadamente 1 Hz
			if time.monotonic() - t_plot >= 1.0:

				if parameters['camera'] and frame is not None:
					plt.cla()
					plt.imshow(frame, cmap='gray')
					plt.pause(0.001)

				t_plot = time.monotonic()

		# salva dados
		if parameters['save']:
			car.save()

	finally:
		# termina a thread de visao
		stop_event.set()
		if thread_vision is not None:
			thread_vision.join(timeout=1.0)

		car.close()

	print('Terminou...')
