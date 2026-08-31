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
	def steer_to_waypoint(self, waypoint, Kp=1.0):

		error = self.heading_error(waypoint)

		self.car.set_steer(Kp*error)

		return self.car.st
		
	########################################
	# controla velocidade ate waypoint
	########################################
	def speed_to_waypoint(self, waypoint, Kv=0.2):

		# distancia ate o waypoint
		distance = self.distance_to_waypoint(waypoint)

		# referencia proporcional a distancia
		vref = Kv*distance

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
			self.car.set_vel(0.0)
			self.car.set_steer(0.0)
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

	try:
		from .car import Car
	except ImportError:
		from car import Car

	# parametros
	parameters = {
		'ts'                   : 60.0,
		'save'                 : False,
		'logfile'              : 'logs/',
		'camera'               : False,
		'ultrasonic_steering'  : True,
		'us_buzzer'            : True,
	}

	# waypoint local [m]
	waypoint = (10.0, 5.0)

	# cria carro e navegacao
	car = Car(parameters)
	nav = Navigation(car)

	try:
		# inicia missao
		car.start_mission()

		while car.t <= parameters['ts']:

			# atualiza estados
			car.step()

			# navega
			reached = nav.go_to_waypoint(waypoint)

			# mostra estados
			print(
				f"Pos: ({car.p[0]:+.2f}, {car.p[1]:+.2f}) m | "
				f"Dist: {nav.distance_to_waypoint(waypoint):.2f} m | "
				f"Erro: {np.rad2deg(nav.heading_error(waypoint)):+.1f} deg | "
				f"Vel: {car.v:+.2f} m/s | "
				f"Ref: {car.vref:+.2f} m/s | "
				f"Steer: {np.rad2deg(car.st):+.1f} deg"
			)

			# chegou
			if reached:
				print("Waypoint atingido!")
				break

		# salva
		if parameters['save']:
			car.save()

	finally:
		car.close()
