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

		position = self.car.get_gps_pos()

		if position is None:
			return None

		x, y = position
		xw, yw = waypoint

		dx = xw - x
		dy = yw - y

		return np.sqrt(dx**2 + dy**2)

	########################################
	# calcula direcao ate waypoint
	########################################
	def heading_to_waypoint(self, waypoint):

		position = self.car.get_gps_pos()

		if position is None:
			return None

		x, y = position
		xw, yw = waypoint

		dx = xw - x
		dy = yw - y

		return np.arctan2(dy, dx)
