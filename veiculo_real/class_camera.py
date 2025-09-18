
# -*- coding: utf-8 -*-
########################################
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2025/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import time
import cv2
from ultralytics import YOLO

RESOLUTION = (640, 480)  # (width, height)
FRAME_RATE = 30          # target FPS (best-effort)
CAMERA_INDEX = 0         # default USB webcam index

MODEL = "outros/best.pt" # brazilian-traffic-signs.v3i.yolov8

########################################
# classe da camera (USB webcam via OpenCV)
########################################
class Camera:
	########################################
	# construtor
	def __init__(self, cam_index: int = CAMERA_INDEX, resolution=RESOLUTION, fps: int = FRAME_RATE):
		self.cap = cv2.VideoCapture(cam_index, cv2.CAP_V4L2)

		if not self.cap.isOpened():
			# tenta sem CAP_V4L2, caso contrário
			self.cap = cv2.VideoCapture(cam_index)
			if not self.cap.isOpened():
				raise RuntimeError(f"Não foi possível abrir a webcam (index={cam_index}).")

		# tenta definir codec MJPG para melhor desempenho (se a câmera suportar)
		try:
			#fourcc = cv2.VideoWriter_fourcc(*"MJPG")
			fourcc = cv2.VideoWriter_fourcc(*"YUY2")
			self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
		except Exception:
			pass

		# configura resolução
		w, h = resolution
		self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  int(w))
		self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(h))

		# configura taxa de quadros (best-effort)
		self.cap.set(cv2.CAP_PROP_FPS, float(fps))

		# tenta desligar autofocus e fixar foco (nem todas webcams suportam)
		# 0 = off, 1 = on
		if self.cap.get(cv2.CAP_PROP_AUTOFOCUS) != -1:
			self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
		# tenta ajustar exposição automática (opcional)
		if self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE) != -1:
			# Em muitas câmeras do Linux, 1 significa Auto, 0.25 Manual; varia por driver
			try:
				self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
			except Exception:
				pass

		# warmup
		t0 = time.time()
		while time.time() - t0 < 0.5:
			self.cap.read()
			
		# modelo da YOLO
		self.model = YOLO(MODEL)

	########################################
	# captura uma imagem da camera
	def getImage(self, gray: bool = False):
		ok, frame = self.cap.read()
		if not ok or frame is None:
			return None

		# OpenCV entrega em BGR; mantendo a API antiga:
		# - se gray=True, converte para escala de cinza
		if gray:
			frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		return frame

	########################################
	# show image
	def show(self, img, fps=None):
		
		# coloca informação de fps
		if fps is not None:
			cv2.putText(img, f"FPS: {fps:.1f}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
		
		cv2.imshow('Vehicle front camera', img)
		
		# 1 ms para manter janela responsiva
		if cv2.waitKey(1) & 0xFF == ord('q'):
			# permite sair com 'q' durante o teste
			raise KeyboardInterrupt
			
	########################################
	# modelo detector de placas de transito
	def detectPlaca(self, img):
		
		# inferência
		results = self.model.predict(img, conf=0.25, iou=0.45, imgsz=640, verbose=False)
		
		# desenha resultados
		annotated = results[0].plot()
		
		return annotated

	########################################
	# destrutor
	def __del__(self):
		try:
			if hasattr(self, 'cap') and self.cap is not None:
				self.cap.release()
		except Exception:
			pass
		cv2.destroyAllWindows()

########################################
# main test
########################################
if __name__ == "__main__":
	cam = Camera()
	print('Webcam ok')
	t0 = time.time()

	prev_time = time.time()    # para medir FPS
	frame_count = 0
	fps = FRAME_RATE

	while (time.time() - t0) <= 20.0:
		img = cam.getImage(gray=False)
		if img is None:
			print('Não foi possível capturar a imagem.')
			continue
		
		# detecta placas
		img = cam.detectPlaca(img)

		# cálculo de FPS real
		frame_count += 1
		now = time.time()
		if now - prev_time >= 1.0:     # a cada 1 segundo
			fps = frame_count / (now - prev_time)
			#print(f"FPS real: {fps:.2f}")
			prev_time = now
			frame_count = 0
		  
		# mostra imagem
		cam.show(img, fps=fps)
