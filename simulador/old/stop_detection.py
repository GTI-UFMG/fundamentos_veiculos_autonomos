# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import class_car as cp
import numpy as np
import cv2
import matplotlib.pyplot as plt
plt.ion()
plt.figure(1)

# carrega rede neural
net = cv2.dnn.readNet('yolo/yolov3.weights', 'yolo/yolov3.cfg')

# le classes
classes = None
with open('yolo/yolov3.txt', 'r') as f:
	classes = [line.strip() for line in f.readlines()]
# cores de detecção	
COLORS = np.random.uniform(0, 255, size=(len(classes), 3))

STOP_SIGN = 11

########################################
def get_output_layers(net):
	layer_names = net.getLayerNames()
	try:
		output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
	except:
		output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
	return output_layers
   
########################################
def draw_prediction(img, class_id, confidence, x, y, x_plus_w, y_plus_h):
	label = str(classes[class_id])
	color = COLORS[class_id]
	cv2.rectangle(img, (x,y), (x_plus_w,y_plus_h), color, 2)
	cv2.putText(img, label, (x-10,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
	
########################################
# detecta placas de PARE
def stopDetection(imagergb):
	
	# placa ainda não detectada
	placa = False
	
	# inverte image
	image = cv2.flip(imagergb, 0)
	
	Width = image.shape[1]
	Height = image.shape[0]
	scale = 0.00392
	
	# transforma image
	blob = cv2.dnn.blobFromImage(image, scale, (416,416), (0,0,0), True, crop=False)

	# avalia rede neural
	net.setInput(blob)
	outs = net.forward(get_output_layers(net))
	
	# verifica quais classes foram detectadas
	class_ids = []
	confidences = []
	boxes = []
	conf_threshold = 0.8
	nms_threshold = 0.4

	for out in outs:
		for detection in out:
			scores = detection[5:]
			class_id = np.argmax(scores)
			confidence = scores[class_id]
			if confidence > 0.8:
				center_x = int(detection[0] * Width)
				center_y = int(detection[1] * Height)
				w = int(detection[2] * Width)
				h = int(detection[3] * Height)
				x = center_x - w / 2
				y = center_y - h / 2
				class_ids.append(class_id)
				confidences.append(float(confidence))
				boxes.append([x, y, w, h])

	# desenha caixas na image
	indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
	for i in indices:
		try:
			box = boxes[i]
		except:
			i = i[0]
			box = boxes[i]
		
		x = box[0]
		y = box[1]
		w = box[2]
		h = box[3]
		draw_prediction(image, class_ids[i], confidences[i], round(x), round(y), round(x+w), round(y+h))
		
		# placa detectada
		if class_ids[i] == STOP_SIGN:
			placa = True
	
	return image, placa
	
########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia()
car.startMission()

count = True
vel = []
velref = []
time = []

while car.t < 50.0:
	
	# lê senores
	car.step()
	
	# direcao
	delta = -2.0*(0.0 - car.p[0]) + 5.0*(0.0 - car.w)
	car.setSteer(-delta)
		
	# detecta placa	
	if count:
		image, placa = stopDetection(car.getImage())
	
		# velocidade
		if placa:
			car.setVel(0.0)
		else:
			car.setVel(2.0)
	
	########################################
	# salva e mostra valores	
	if count:
		plt.subplot(211)
		plt.cla()
		plt.gca().imshow(image)
	
	plt.subplot(212)
	plt.cla()
	vel.append(car.v)
	velref.append(car.vref)
	time.append(car.t)
	plt.plot(time, vel, 'r')
	plt.plot(time, velref, 'k--')
	plt.ylabel('velocidade')
	
	plt.show()
	plt.pause(0.01)
	
	count = not count
	
print('Terminou...')
