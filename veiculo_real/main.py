# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
from fva_car import Car
import numpy as np
import threading
import time

########################################
# Globais
########################################
MAIN_VEL = 0.7

# Frequencia da telemetria enviada para a GUI
TELEMETRY_PERIOD = 0.1 # 10 Hz

########################################
# thread de visao
########################################
def vision_func(car, vision_data, stop_event):

	W, H = car.cam.get_resolution()

	while not stop_event.is_set():

		# pega imagem
		frame = car.get_image(gray=True)

		if frame is None:
			time.sleep(0.02)
			continue

		# detecta aruco (substituir pelo algoritmo do aluno, se necessario)
		frame, point = car.cam.detect_aruco(frame, aruco_id=23)

		# A classe Camera cuida de toda a transmissao.
		car.cam.publish_frame(frame)

		if point is None:
			continue

		# esterçamento aponta para o aruco
		cx = point[0] - W/2

		vision_data["refste"] = -np.deg2rad(20.0*cx/(W/2))

########################################
# main
########################################
if __name__ == "__main__":

	parameters = {
				'ts'					: 30.0, 	# tempo da execucao
				'save'					: True,		# salva dados da trajetoria
				'logfile'				: 'logs/',	# log file
				'camera'				: False,	# habilitar camera e thread de visao
				'us_buzzer'				: True,		# aviso sonoro para objetos proximos
				'initial_position'		: [0, 0, np.deg2rad(0)]	# (x, y, theta) configuracao inicial
			}

	car = Car(parameters)

	vision_data = {
		"refste": 0.0
	}

	stop_event = threading.Event()
	thread_vision = None

	try:
		# inicializa veiculo
		car.start_mission()

		########################################
		# inicia visao somente se solicitada
		########################################
		if parameters['camera']:
			car.cam.start_stream()
			thread_vision = threading.Thread(target=vision_func, args=(car, vision_data, stop_event), daemon=True)
			thread_vision.start()

		########################################
		# temporizador da telemetria
		########################################
		t_telemetry = time.monotonic()

		########################################
		# controle fica na thread principal
		########################################
		while car.t < parameters['ts']:

			########################################
			# atualiza sensores
			########################################
			if not car.step():
				break

			# direcao
			car.set_steer(vision_data["refste"])

			# ultrassom
			dist, _ = car.get_distance()
			if dist < 0.30:
				print(f"Colisao: distance {dist:.2f} [m]")
				car.set_vel(0.0)
			else:
				car.set_vel(MAIN_VEL)

			########################################
			# telemetria para interface remota
			#
			# O controle continua executando em
			# todas as iteracoes do loop.
			#
			# Apenas a telemetria enviada para
			# a GUI e subamostrada para 10 Hz.
			########################################
			now = time.monotonic()
			if (now - t_telemetry) >= TELEMETRY_PERIOD:
				print(
					f"DATA,"
					f"{car.t:.2f},"
					f"{car.p[0]:.2f},"
					f"{car.p[1]:.2f},"
					f"{car.v:.2f},"
					f"{car.vref:.2f},"
					f"{car.a:.2f},"
					f"{car.u:.2f},"
					f"{car.w:.2f},"
					f"{car.th:.2f}",
					flush=True
				)

				t_telemetry = now

		########################################
		# salva dados
		########################################
		if parameters['save']:
			car.save()

	finally:
		########################################
		# termina a thread de visao
		########################################
		stop_event.set()

		if thread_vision is not None:
			thread_vision.join(timeout=1.0)

		car.close()

	print('Terminou...')
