# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import subprocess
import re
import numpy as np

########################################
# Globais
########################################


########################################
# classe para usar GPS de celulares Android
########################################
class GPS:
	########################################
	# construtor
	########################################
	def __init__(self):
		self.position = None
		self.new_position = False
		self.last_et = None
		self.origin = None
		
	########################################
	# define origem do sistema local
	########################################
	def set_origin(self):

		position = self.get_position()

		if position is None:
			return False

		self.origin = {
			"lat": position["lat"],
			"lon": position["lon"]
		}

		return True

	########################################
	# posicao
	########################################
	def get_position(self):
		
		self.new_position = False

		# tenta achar o celular conectado
		try:
			result = subprocess.run(
				["adb", "shell", "dumpsys", "location"],
				capture_output=True,
				text=True,
				timeout=2
			)

		except (subprocess.TimeoutExpired, FileNotFoundError):
			return None
		
		# Nao encontrou
		if result.returncode != 0:
			return None

		for line in result.stdout.splitlines():

			if "last location=Location[gps" in line:

				match = re.search(
					r"Location\[gps\s+(-?\d+\.\d+),(-?\d+\.\d+).*?"
					r"hAcc=([\d.]+).*?"
					r"et=(\S+).*?"
					r"vel=([\d.]+)",
					line
				)

				if match:
					new_position = {
										"lat": float(match.group(1)),
										"lon": float(match.group(2)),
										"accuracy": float(match.group(3)),
										"speed": float(match.group(5))
									}

					et = match.group(4)

					self.new_position = (et != self.last_et)
					self.last_et = et

					self.position = new_position

					return self.position

		return None
		
	########################################
	# converte GPS para coordenadas locais
	########################################
	def get_xy(self, position=None):

		if self.origin is None:
			return None

		if position is None:
			position = self.get_position()

		if position is None:
			return None

		R = 6371000.0  # raio medio da Terra [m]

		lat0 = np.deg2rad(self.origin["lat"])

		dlat = np.deg2rad(
			position["lat"] - self.origin["lat"]
		)

		dlon = np.deg2rad(
			position["lon"] - self.origin["lon"]
		)

		x = R * dlon * np.cos(lat0)
		y = R * dlat

		return x, y

########################################
# main test
########################################
if __name__ == "__main__":

	import time

	gps = GPS()

	print("Aguardando GPS...")

	# define a posicao atual como origem
	if gps.set_origin():

		print("Origem definida:")
		print(gps.origin)
		print()

		try:
			while True:
				position = gps.get_position()
				xy = gps.get_xy(position)

				if xy is not None:
					x, y = xy

					print(
						f"x = {x:7.2f} m | "
						f"y = {y:7.2f} m"
					)

				else:
					print("GPS nao disponivel")

				time.sleep(1)
		except KeyboardInterrupt:
			print("\nFim.")

	else:
		print("Nao foi possivel definir a origem.")
