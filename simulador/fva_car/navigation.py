# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import numpy as np

MIN_SPEED = 1.0
WAYPOINT_RADIUS = 1.0

########################################
# Navegacao
########################################
class Navigation:

	########################################
	# construtor
	########################################
	def __init__(self, car):

		self.car = car

	########################################
	# calcula distancia ate waypoint
	########################################
	def distance_to_waypoint(self, waypoint):
		
		x, y = self.car.p
		xw, yw = waypoint

		dx = xw - x
		dy = yw - y

		return np.sqrt(dx**2 + dy**2)

	########################################
	# calcula direcao ate waypoint
	########################################
	def heading_to_waypoint(self, waypoint):

		x, y = self.car.p
		xw, yw = waypoint

		dx = xw - x
		dy = yw - y

		return np.arctan2(dy, dx)
		
	########################################
	# calcula erro de orientacao ate waypoint
	########################################
	def heading_error(self, waypoint):

		# direcao desejada
		heading_ref = self.heading_to_waypoint(waypoint)

		# orientacao atual
		heading = self.car.th

		# erro angular entre -pi e pi
		error = np.arctan2(
			np.sin(heading_ref - heading),
			np.cos(heading_ref - heading)
		)

		return error
		
	########################################
	# controla orientacao ate waypoint
	########################################
	def steer_to_waypoint2(self, waypoint, Kp=0.2):

		error = self.heading_error(waypoint)

		self.car.set_steer(Kp*error)

		return self.car.st
		
	def steer_to_waypoint(self, waypoint, Kp=0.2):

		error = self.heading_error(waypoint)

		st_ref = Kp*error

		# limite da taxa de esterçamento
		st_rate_max = np.deg2rad(20.0)
		dst_max = st_rate_max*self.car.dt

		dst = st_ref - self.car.st
		dst = np.clip(dst, -dst_max, dst_max)

		st = self.car.st + dst

		self.car.set_steer(st)

		return self.car.st
		
	########################################
	# controla velocidade ate waypoint
	########################################
	def speed_to_waypoint(self, waypoint, Kv=0.1):

		# distancia ate o waypoint
		distance = self.distance_to_waypoint(waypoint)

		# referencia proporcional a distancia
		vref = max(Kv*distance, MIN_SPEED)

		# envia referencia ao carro
		self.car.set_vel(vref)

		return self.car.vref
		
	########################################
	# verifica se waypoint foi atingido
	########################################
	def waypoint_reached(self, waypoint, radius=2.0):

		distance = self.distance_to_waypoint(waypoint)

		return distance <= radius
		
	########################################
	# navega ate waypoint
	########################################
	def go_to_waypoint(self, waypoint, radius=2.0, Kv=0.2, Kp=1.0):

		# verifica se chegou
		if self.waypoint_reached(waypoint, radius):
			return True

		# controle de direcao
		self.steer_to_waypoint(waypoint, Kp)

		# controle de velocidade
		self.speed_to_waypoint(waypoint, Kv)

		return False
		
########################################
# main teste
########################################
if __name__ == "__main__":

	import matplotlib.pyplot as plt

	try:
		from .car import Car
	except ImportError:
		from car import Car

	########################################
	# parametros
	########################################
	parameters = {
		'ts'      : 300.0,
		'save'    : True,
		'logfile' : 'logs/',
		'beep'    : True,
	}

	########################################
	# carrega waypoints
	########################################
	data = np.genfromtxt(
		'../waypoints/final_dubins_path.csv',
		delimiter=',',
		names=True,
		dtype=None,
		encoding='utf-8'
	)

	waypoints = np.column_stack(
		(data['x'], data['y'])
	)

	# ignora o primeiro ponto START
	waypoints = waypoints[1:]

	########################################
	# cria carro e navegacao
	########################################
	car = Car(parameters)
	nav = Navigation(car)

	########################################
	# grafico
	########################################
	traj_x = []
	traj_y = []

	plt.ion()
	plt.figure()

	########################################
	# inicia missao
	########################################
	try:

		car.start_mission()

		# waypoint atual
		i = 0

		while (car.t <= parameters['ts'] and i < len(waypoints)):

			################################
			# atualiza estados do carro
			################################
			car.step()

			# waypoint atual
			waypoint = waypoints[i]

			################################
			# navegacao
			################################
			reached = nav.go_to_waypoint(
				waypoint,
				radius=WAYPOINT_RADIUS,
				Kv=0.2,
				Kp=0.05
			)

			################################
			# salva trajetoria para plot
			################################
			traj_x.append(car.p[0])
			traj_y.append(car.p[1])

			################################
			# grafico
			################################
			plt.clf()

			# caminho dos waypoints
			plt.plot(
				waypoints[:, 0],
				waypoints[:, 1],
				'--',
				label='Waypoints'
			)

			# trajetoria realizada
			plt.plot(
				traj_x,
				traj_y,
				'-',
				label='Trajetoria'
			)

			# posicao atual do carro
			plt.plot(
				car.p[0],
				car.p[1],
				'o',
				markersize=10,
				label='Carro'
			)

			# waypoint atual
			plt.plot(
				waypoint[0],
				waypoint[1],
				'x',
				markersize=12,
				label='Waypoint atual'
			)

			plt.xlabel('x [m]')
			plt.ylabel('y [m]')
			plt.title(
				f'Waypoint {i + 1}/{len(waypoints)}'
			)

			plt.axis('equal')
			plt.grid()
			plt.legend()

			plt.show(block=False)
			plt.pause(0.01)

			################################
			# informacoes no terminal
			################################
			'''print(
				f"WP: {i + 1}/{len(waypoints)} | "
				f"Pos: ({car.p[0]:+.2f}, {car.p[1]:+.2f}) m | "
				f"Dist: {nav.distance_to_waypoint(waypoint):.2f} m | "
				f"Erro: "
				f"{np.rad2deg(nav.heading_error(waypoint)):+.1f} deg | "
				f"Vel: {car.v:+.2f} m/s | "
				f"Ref: {car.vref:+.2f} m/s | "
				f"Steer: {np.rad2deg(car.st):+.1f} deg"
			)'''

			################################
			# chegou ao waypoint
			################################
			if reached:

				print(f"Waypoint {i + 1} atingido!")

				# proximo waypoint
				i += 1

			####################################
			# fim da rota
			####################################
			if i == len(waypoints):
				print("Todos os waypoints foram atingidos!")
				break

		####################################
		# salva log
		####################################
		if parameters['save']:
			car.save()

	except KeyboardInterrupt:
		print("\nMissao interrompida pelo usuario.")
	
	finally:

		car.close()

		####################################
		# mantem figura aberta
		####################################
		plt.ioff()
		plt.show()
