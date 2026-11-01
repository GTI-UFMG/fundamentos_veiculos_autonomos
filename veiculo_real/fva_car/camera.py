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
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
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

		# Servidor de video: apenas publica quadros recebidos do algoritmo.
		# Nao abre a camera novamente e nao interfere na captura.
		self._stream_lock = threading.Lock()
		self._stream_frame = None
		self._stream_sequence = 0
		self._stream_server = None
		self._stream_thread = None
		self._stream_fps = 10.0
		self._stream_scale = 0.5
		self._stream_last_publish = 0.0

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
	# Publica um quadro para a interface (BGR ou escala de cinza).
	# Chamar apos o processamento; nao realiza captura adicional.
	def publish_frame(self, frame):
		if frame is None:
			return
		with self._stream_lock:
			now = time.monotonic()
			if now - self._stream_last_publish < 1.0 / self._stream_fps:
				return
			self._stream_last_publish = now
			scale = self._stream_scale
		if scale != 1.0:
			frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
		with self._stream_lock:
			self._stream_frame = frame.copy()
			self._stream_sequence += 1

	########################################
	# Inicia servidor HTTP em segundo plano, sem dependencias extras.
	# /frame.png: compativel com PhotoImage do Tkinter
	# /stream: MJPEG para navegadores, se desejado.
	def start_stream(self, host="0.0.0.0", port=8080, fps=10, scale=0.5):
		if fps <= 0 or not 0 < scale <= 1:
			raise ValueError("fps deve ser positivo e scale deve estar entre 0 e 1")
		with self._stream_lock:
			self._stream_fps = float(fps)
			self._stream_scale = float(scale)
			self._stream_last_publish = 0.0
		if self._stream_server is not None:
			return
		camera = self

		class VideoHandler(BaseHTTPRequestHandler):
			def log_message(self, format, *args):
				pass

			def do_GET(self):
				path = urlsplit(self.path).path
				if path not in ("/", "/frame.png", "/stream"):
					self.send_error(404)
					return
				if path == "/":
					body = b'<html><body><img src="/stream"></body></html>'
					self.send_response(200)
					self.send_header("Content-Type", "text/html; charset=utf-8")
					self.send_header("Content-Length", str(len(body)))
					self.end_headers()
					self.wfile.write(body)
					return
				if path == "/frame.png":
					with camera._stream_lock:
						frame = None if camera._stream_frame is None else camera._stream_frame.copy()
					if frame is None:
						self.send_error(503, "Aguardando primeiro quadro")
						return
					ok, buf = cv2.imencode(".png", frame)
					if not ok:
						self.send_error(500)
						return
					body = buf.tobytes()
					self.send_response(200)
					self.send_header("Content-Type", "image/png")
					self.send_header("Cache-Control", "no-store")
					self.send_header("Content-Length", str(len(body)))
					self.end_headers()
					self.wfile.write(body)
					return
				try:
					self.send_response(200)
					self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
					self.send_header("Cache-Control", "no-store")
					self.end_headers()
					last_seq = -1
					while camera._stream_server is not None:
						with camera._stream_lock:
							seq = camera._stream_sequence
							frame = None if camera._stream_frame is None else camera._stream_frame.copy()
						if frame is not None and seq != last_seq:
							ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
							if ok:
								body = buf.tobytes()
								self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(body)).encode() + b"\r\n\r\n" + body + b"\r\n")
								last_seq = seq
						time.sleep(0.01)
				except (BrokenPipeError, ConnectionResetError, OSError):
					pass

		class Server(ThreadingHTTPServer):
			daemon_threads = True
			allow_reuse_address = True

		server = Server((host, int(port)), VideoHandler)
		self._stream_server = server
		self._stream_thread = threading.Thread(target=server.serve_forever, daemon=True)
		self._stream_thread.start()
		print(f"Camera: transmissao HTTP ativa na porta {port}", flush=True)

	########################################
	def stop_stream(self):
		server = self._stream_server
		if server is None:
			return
		self._stream_server = None
		server.shutdown()
		server.server_close()
		if self._stream_thread is not None:
			self._stream_thread.join(timeout=1.0)
		self._stream_thread = None

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
		self.stop_stream()
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
	parser.add_argument("--stream", action="store_true", help="Transmite imagens por HTTP sem abrir janela")
	parser.add_argument("--port", type=int, default=8081, help="Porta HTTP do teste (padrao 8081)")
	parser.add_argument("--scale", type=float, default=0.5, help="Escala da imagem transmitida")
	parser.add_argument("--prefix", default="/tmp/fva_camera", help="Prefixo dos arquivos temporarios")
	parser.add_argument("--seconds", type=float, default=None)
	parser.add_argument("--fps", type=float, default=10.0, help="FPS da transmissao HTTP (padrao 10)")
	parser.add_argument("--aruco", action="store_true", help="Detecta marcadores ArUco (ID 23)")
	args = parser.parse_args()
	
	seconds = args.seconds if args.seconds is not None else (60.0 if (args.remote or args.stream) else 20.0)
	if seconds <= 0 or args.fps <= 0:
		parser.error("--seconds e --fps devem ser positivos")

	cam = Camera()
	if args.stream:
		cam.start_stream(port=args.port, fps=args.fps, scale=args.scale)
		print(f"HTTP pronto: porta {args.port}, ate {args.fps:g} FPS, escala {args.scale:g}", flush=True)
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

			if args.remote and not args.stream and now < next_publish:
				time.sleep(min(0.02, next_publish - now))
				continue
				
			# Mantem o comportamento original de deteccao no teste local.
			if args.aruco or not (args.remote or args.stream):
				if not hasattr(cv2, "aruco"):
					raise RuntimeError("OpenCV sem suporte a ArUco")
				img, center = cam.detect_aruco(img)
				if center is not None:
					print(f"ArUco 23 detectado: {center}", flush=True)

			if args.stream:
				cam.publish_frame(img)
			elif args.remote:
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
	print(f"Teste concluido. Quadros publicados: {published}" if args.remote and not args.stream else "Teste concluido.", flush=True)

########################################
if __name__ == "__main__":
	_camera_main()
