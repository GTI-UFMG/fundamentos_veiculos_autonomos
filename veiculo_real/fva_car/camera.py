# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2026/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import time
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"  # forca backend X11
import cv2
from pathlib import Path

#---------------
# As vezes o OpenCV/PyTorch tenta usar todos os cores e o pico de consumo explode.
# Limitar threads do PyTorch/OpenCV antes de importar:
os.environ["OMP_NUM_THREADS"] = "2"
os.environ["OPENBLAS_NUM_THREADS"] = "2"
cv2.setNumThreads(2)
#---------------

########################################
# Globais
########################################
RESOLUTION = (640, 480)  # (width, height)
FRAME_RATE = 30          # target FPS (best-effort)
MODEL = Path(__file__).resolve().parent / "modelos" / "best.pt" # brazilian-traffic-signs.v3i.yolov8

########################################
# classe da camera (USB webcam via OpenCV)
########################################
class Camera:
	########################################
	# construtor
	def __init__(	self, 
					cam_index: int = None, 
					resolution=RESOLUTION, 
					fps: int = FRAME_RATE):
		
		# identifica camera disponivel
		if cam_index is None:
			cam_index = self.find_camera()
			if cam_index is None:
				raise RuntimeError("Camera nao encontrada.")
		
		# cria a camera		
		self.cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)
		if not self.cap.isOpened():
			# tenta sem CAP_V4L2, caso contrario
			self.cap = cv2.VideoCapture(cam_index)
			if not self.cap.isOpened():
				raise RuntimeError(f"Nao foi possivel abrir a camera (index={cam_index}).")

		# tenta definir codec MJPG para melhor desempenho (se a camera suportar)
		#fourcc = cv2.VideoWriter_fourcc(*"MJPG")
		fourcc = cv2.VideoWriter_fourcc(*"YUY2")
		self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)

		# configura resolucao
		w, h = resolution
		self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  int(w))
		self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(h))

		# configura taxa de quadros (best-effort)
		self.cap.set(cv2.CAP_PROP_FPS, float(fps))

		# tenta desligar autofocus e fixar foco (nem todas webcams suportam)
		# 0 = off, 1 = on
		if self.cap.get(cv2.CAP_PROP_AUTOFOCUS) != -1:
			self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
		# tenta ajustar exposicao automatica (opcional)
		if self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE) != -1:
			# Em muitas cameras do Linux, 1 significa Auto, 0.25 Manual; varia por driver
			self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
		
		# modelo da YOLO
		self.model = None
		
		# biblioteca de Arucos
		self.detect_fn = None

	########################################
	# procura uma camera disponivel
	def find_camera(self):
		for index in range(10):
			cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
			if cap.isOpened():
				ok, frame = cap.read()
				cap.release()
				if ok and frame is not None:
					return index
		return None
		
	########################################
	# pega resolucao da imagem
	def get_resolution(self):
		w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
		h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
		return int(w), int(h)
	
	########################################
	# pega taxa de quadros configurada
	def get_fps(self):
		return self.cap.get(cv2.CAP_PROP_FPS)

	########################################
	# captura uma imagem da camera
	def get_image(self, gray: bool = False):
		ok, frame = self.cap.read()
		if not ok or frame is None:
			return None

		# OpenCV entrega em BGR; mantendo a API antiga:
		# - se gray=True, converte para escala de cinza
		if gray:
			frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		return frame
			
	########################################
	# modelo detector de placas de transito
	def detect_placa(self, img):
		
		if self.model is None:
			from ultralytics import YOLO
			self.model = YOLO(str(MODEL))
		
		# inferencia
		results = self.model.predict(img, conf=0.25, iou=0.45, imgsz=640, verbose=False)
		
		# desenha resultados
		annotated = results[0].plot()
		
		return annotated

	########################################
	# detecta marcadores de AR
	def detect_aruco(self, img, aruco_id=23):
		
		# inicializa detector ArUco somente no primeiro uso
		if self.detect_fn is None:
			# Load the dictionary of ArUco markers and create a parameters object for ArUco detection.
			if hasattr(cv2.aruco, "ArucoDetector"):
				dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
				parameters = cv2.aruco.DetectorParameters()
				detector   = cv2.aruco.ArucoDetector(dictionary, parameters)
				self.detect_fn  = lambda img: detector.detectMarkers(img)
			else:
				dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
				parameters = cv2.aruco.DetectorParameters_create()
				self.detect_fn  = lambda img: cv2.aruco.detectMarkers(img, dictionary, parameters=parameters)
		
		# Detect ArUco markers in the frame.
		corners, ids, rejected = self.detect_fn(img)
		
		# inicialmente, marcador desejado nao foi encontrado
		center = None
		
		# desenha contornos + ids na imagem colorida
		if ids is not None and len(ids) > 0:
			cv2.aruco.drawDetectedMarkers(img, corners, ids)  # <- aqui desenha
			# opcional: destacar apenas um id
			for i, m_id in enumerate(ids.flatten()):
				if m_id == aruco_id:
					pts = corners[i][0].astype(int)
					cx, cy = pts[:,0].mean(), pts[:,1].mean()
					center = (cx, cy)
					cv2.circle(img, (int(cx), int(cy)), 6, (0,255,0), -1)
		
		return img, center
			
	########################################
	# mostra imagem
	def show(self, img, fps=None):

		if fps is not None:
			cv2.putText(img, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
		
		# mostra imagem
		cv2.imshow("Vehicle front camera", img)

		# retorna False se usuario apertar 'q'
		key = cv2.waitKey(1) & 0xFF
		return key != ord("q")
			
	########################################
	# fecha camera
	def close(self):
		if hasattr(self, 'cap') and self.cap is not None:
			self.cap.release()
		try:
			cv2.destroyAllWindows()
		except cv2.error:
			pass

########################################
# Teste local / remoto
########################################
def _camera_main():
	import argparse
	parser = argparse.ArgumentParser(description="Teste da camera FVA")
	parser.add_argument("--remote", action="store_true", help="Publica imagens para a GUI sem abrir janela")
	parser.add_argument("--prefix", default="/tmp/fva_camera", help="Prefixo dos arquivos temporarios")
	parser.add_argument("--seconds", type=float, default=None)
	parser.add_argument("--fps", type=float, default=3.0, help="FPS de publicacao no modo remoto")
	parser.add_argument("--aruco", action="store_true", help="Detecta marcadores ArUco (ID 23)")
	args = parser.parse_args()
	
	seconds = args.seconds if args.seconds is not None else (60.0 if args.remote else 20.0)
	if seconds <= 0 or args.fps <= 0:
		parser.error("--seconds e --fps devem ser positivos")

	cam = Camera()
	print("Camera ok", flush=True)
	print(f"Resolucao: {cam.get_resolution()}", flush=True)
	print(f"FPS configurado: {cam.get_fps():.1f}", flush=True)
	start = time.monotonic()
	previous = start
	count = 0
	published = 0
	failures = 0
	next_publish = start
	fps_real = 0.0
	
	try:
		while time.monotonic() - start < seconds:
			img = cam.get_image(gray=False)
			if img is None:
				failures += 1
				if failures >= 15:
					raise RuntimeError("Falha repetida na captura de imagens")
				time.sleep(0.1)
				continue
				
			failures = 0
			now = time.monotonic()
			count += 1
			if now - previous >= 1.0:
				fps_real = count / (now - previous)
				previous = now
				count = 0

			if args.remote and now < next_publish:
				time.sleep(min(0.02, next_publish - now))
				continue
				
			# Mantem o comportamento original de deteccao no teste local.
			if args.aruco or not args.remote:
				if not hasattr(cv2, "aruco"):
					raise RuntimeError("OpenCV sem suporte a ArUco")
				img, center = cam.detect_aruco(img)
				if center is not None:
					print(f"ArUco 23 detectado: {center}", flush=True)

			if args.remote:
				# Arquivos publicados atomicamente; leitor SFTP nao ve imagens incompletas.
				frame_path = args.prefix + ".png"
				seq_path = args.prefix + ".seq"
				cv2.putText(img, f"FPS: {fps_real:.1f}", (10, 30),
							cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
				ok, encoded = cv2.imencode(".png", img)
				if not ok:
					raise RuntimeError("Falha ao codificar imagem PNG")
				with open(frame_path + ".tmp", "wb") as f:
					f.write(encoded.tobytes())
				os.replace(frame_path + ".tmp", frame_path)
				published += 1
				with open(seq_path + ".tmp", "w") as f:
					f.write(str(published))
				os.replace(seq_path + ".tmp", seq_path)
				if published % 10 == 0:
					print(f"Quadros publicados: {published} | FPS captura: {fps_real:.1f}", flush=True)
				next_publish = max(next_publish + 1.0 / args.fps, time.monotonic())
			else:
				if not cam.show(img, fps=fps_real):
					break
					
	except KeyboardInterrupt:
		pass
	finally:
		cam.close()
	print(f"Teste concluido. Quadros publicados: {published}" if args.remote else "Teste concluido.", flush=True)

########################################
if __name__ == "__main__":
	_camera_main()
