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
MODEL = Path(__file__).resolve().parent / ".." / "outros" / "best.pt" # brazilian-traffic-signs.v3i.yolov8

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
		# YOLO em segundo plano: apenas o quadro mais recente.
		self._yolo_condition = threading.Condition()
		self._yolo_pending = None
		self._yolo_boxes = []
		self._yolo_thread = None
		self._yolo_stop = False
		self._yolo_error = None
		
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
	# YOLO: inferencia na thread de processamento
	########################################
	def detect_placa(self, img):
		"""Modo sincrono mantido para compatibilidade com chamadas antigas."""
		if self.model is None:
			from ultralytics import YOLO
			self.model = YOLO(str(MODEL))
		results = self.model.predict(img, conf=0.25, iou=0.45,
									 imgsz=320, device="cpu", verbose=False)
		return results[0].plot()

	def start_yolo(self):
		"""Inicia um unico trabalhador YOLO; nao abre janelas OpenCV."""
		with self._yolo_condition:
			if self._yolo_thread is not None and self._yolo_thread.is_alive():
				return
			self._yolo_stop = False
			self._yolo_error = None
			self._yolo_thread = threading.Thread(target=self._yolo_worker, daemon=True)
			self._yolo_thread.start()

	def _yolo_worker(self):
		try:
			from ultralytics import YOLO
			print(f"YOLO: carregando {MODEL}", flush=True)
			model = YOLO(str(MODEL))
			print("YOLO: modelo carregado", flush=True)
			while True:
				with self._yolo_condition:
					while self._yolo_pending is None and not self._yolo_stop:
						self._yolo_condition.wait()
					if self._yolo_stop:
						return
					frame = self._yolo_pending
					self._yolo_pending = None
				t0 = time.perf_counter()
				results = model.predict(frame, conf=0.25, iou=0.45,
										imgsz=320, device="cpu", verbose=False)
				boxes = []
				for box in results[0].boxes:
					x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
					cls = int(box.cls[0].item())
					conf = float(box.conf[0].item())
					name = str(model.names.get(cls, cls)) if isinstance(model.names, dict) else str(model.names[cls])
					boxes.append((x1, y1, x2, y2, name, conf))
				with self._yolo_condition:
					self._yolo_boxes = boxes
				print(f"YOLO: {(time.perf_counter()-t0)*1000:.0f} ms | {len(boxes)} deteccoes", flush=True)
		except Exception as exc:
			with self._yolo_condition:
				self._yolo_error = exc
			print(f"ERRO YOLO: {type(exc).__name__}: {exc}", flush=True)

	def annotate_yolo(self, img):
		"""Envia quadro mais recente ao trabalhador e desenha ultimas caixas."""
		with self._yolo_condition:
			if self._yolo_error is not None:
				raise RuntimeError("Falha na thread YOLO") from self._yolo_error
			self._yolo_pending = img.copy()  # sobrescreve quadros ainda nao processados
			self._yolo_condition.notify()
			boxes = list(self._yolo_boxes)
		for x1, y1, x2, y2, name, conf in boxes:
			cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
			cv2.putText(img, f"{name} {conf:.2f}", (x1, max(20, y1-5)),
						cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
		return img

	def stop_yolo(self):
		with self._yolo_condition:
			self._yolo_stop = True
			self._yolo_pending = None
			self._yolo_condition.notify_all()
		thread = self._yolo_thread
		if thread is not None:
			thread.join(timeout=2.0)
		self._yolo_thread = None

	########################################
	# detecta marcadores ArUco selecionados
	def detect_aruco(self, img, aruco_id=23):
		"""Retorna (imagem, centro) para ID unico, ou (imagem, {id: centro}) para lista.

		Apenas os IDs solicitados sao desenhados. Centro = (cx, cy).
		"""
		multiple = not isinstance(aruco_id, (int,))
		requested = {int(v) for v in aruco_id} if multiple else {int(aruco_id)}
		if any(v < 0 or v >= 100 for v in requested):
			raise ValueError("IDs ArUco devem estar entre 0 e 99 (DICT_5X5_100)")

		if self.detect_fn is None:
			if hasattr(cv2.aruco, "ArucoDetector"):
				dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
				parameters = cv2.aruco.DetectorParameters()
				detector = cv2.aruco.ArucoDetector(dictionary, parameters)
				self.detect_fn = lambda frame: detector.detectMarkers(frame)
			else:
				dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
				parameters = cv2.aruco.DetectorParameters_create()
				self.detect_fn = lambda frame: cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

		corners, ids, _ = self.detect_fn(img)
		centers = {}
		if ids is not None:
			selected_corners = []
			selected_ids = []
			for i, marker_id in enumerate(ids.flatten()):
				marker_id = int(marker_id)
				if marker_id not in requested:
					continue
				pts = corners[i][0]
				cx, cy = float(pts[:, 0].mean()), float(pts[:, 1].mean())
				centers[marker_id] = (cx, cy)
				selected_corners.append(corners[i])
				selected_ids.append([marker_id])
			if selected_ids:
				import numpy as np
				cv2.aruco.drawDetectedMarkers(img, selected_corners, np.array(selected_ids, dtype=np.int32))
				for cx, cy in centers.values():
					cv2.circle(img, (round(cx), round(cy)), 6, (0, 255, 0), -1)

		return img, centers if multiple else centers.get(int(aruco_id))

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
		self.stop_yolo()
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
	parser.add_argument("--aruco", action="store_true", help="Detecta marcadores ArUco")
	parser.add_argument("--normal", action="store_true", help="Exibe imagem sem deteccao")
	parser.add_argument("--placas", action="store_true", help="Detecta placas com YOLO em thread")
	parser.add_argument("--aruco-ids", type=int, nargs="+", default=None, metavar="ID", help="IDs ArUco a identificar, por exemplo: --aruco-ids 4 12 23")
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
	use_aruco = args.aruco or args.aruco_ids is not None
	if sum((bool(use_aruco), args.normal, args.placas)) > 1:
		parser.error("Escolha apenas um modo: --normal, --aruco ou --placas")
	use_yolo = not use_aruco and not args.normal  # mantem YOLO como padrao anterior
	if use_yolo:
		cam.start_yolo()
	if not (args.stream or args.remote):
		cv2.namedWindow("Vehicle front camera", cv2.WINDOW_NORMAL)
		cv2.resizeWindow("Vehicle front camera", 960, 720)
		cv2.waitKey(1)
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
				
			if use_aruco:
				if not hasattr(cv2, "aruco"):
					raise RuntimeError("OpenCV sem suporte a ArUco")
				img, detected = cam.detect_aruco(img, args.aruco_ids if args.aruco_ids is not None else [12, 23])
			elif use_yolo:
				img = cam.annotate_yolo(img)

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
