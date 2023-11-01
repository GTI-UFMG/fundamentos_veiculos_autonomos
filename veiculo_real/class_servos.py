# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import numpy as np
import time
from adafruit_servokit import ServoKit

SERVO_STEERING = 0
SERVO_THROTTLE  = 1
SERVO_PAN  = 4
SERVO_TILT = 5

MAX_STERRING_ANGLE  = 20.0
ZERO_STERRING_ANGLE = 100.0
GAIN_STERRING_ANGLE = 50.0/MAX_STERRING_ANGLE

ZERO_THROTTLE_ANGLE = 95.0
GAIN_THROTTLE_ANGLE = 0.2*90.0

########################################
# Adafruit 16-channel servo driver
########################################
class Servos:
    ########################################
    # construtor
    def __init__(self, steering=0.0, throttle=0.0):
        
        # Set channels to the number of servo channels on your kit.
        # 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
        self.kit = ServoKit(channels=16)
        
        # inicializa o esterçamento
        self.setSteer(steering)
        
        # inicializa traçao
        self.setU(throttle)
        
        # marcha re
        self.reverse = False
    
    ########################################
    # seta torque do motor
    def setU(self, u):
        
        # satura aceleracao
        #u = np.clip(u, -1.0, 1.0)
        u = np.clip(u, 0.0, 1.0)
        
        '''# passar marcha ré
        if (u < 0.0) and (not self.reverse):
            self.reverse = True
            self.backward()
        if u >=0:
            self.reverse = False'''
            
        # calibracao
        u = GAIN_THROTTLE_ANGLE*u + ZERO_THROTTLE_ANGLE
        
        # envia comando
        u = np.clip(u, 0, 180)
        self.kit.servo[SERVO_THROTTLE].angle = u
        
        return
    
    ########################################
    # seta steer do veiculo (st in rad)
    def setSteer(self, st):
        
        # converte para graus
        st = np.rad2deg(st)
        
        # satura angulo de estercamento
        st = np.clip(st, -MAX_STERRING_ANGLE, MAX_STERRING_ANGLE)
        
        # aplica calibracao devido a reduções do eixo
        st = GAIN_STERRING_ANGLE*st + ZERO_STERRING_ANGLE
        
        # envia comando
        st = np.clip(st, 0.0, 180.0)
        self.kit.servo[SERVO_STEERING].angle = st
        
        return
    
    ########################################
    # mode de marcha ré
    def backward(self):
        
        self.kit.servo[SERVO_THROTTLE].angle = ZERO_STERRING_ANGLE - 20.0
        time.sleep(0.08)
        self.kit.servo[SERVO_THROTTLE].angle = ZERO_STERRING_ANGLE
        time.sleep(0.08)
        self.kit.servo[SERVO_THROTTLE].angle = ZERO_STERRING_ANGLE - 20.0
        time.sleep(0.08)
        self.kit.servo[SERVO_THROTTLE].angle = ZERO_STERRING_ANGLE
        
    ########################################
    # angulo de pan da camera entre [-90, 90]
    def setPan(self, ang):
            
        # converte para graus
        ang = np.rad2deg(ang)
        # angulo centrado em zero
        ang += 90.0        
        
        # envia comando
        ang = np.clip(ang, 0.0, 180.0)
        self.kit.servo[SERVO_PAN].angle = ang

    ########################################
    # angulo de tilt da camera entre [-90, 90]
    def setTilt(self, ang):
        
        # converte para graus
        ang = np.rad2deg(ang)
        
        # limite fisico por causa do fio da camera
        ang = np.clip(ang, -40.0, 40.0)
        
        # angulo centrado em zero
        ang += 90.0        
        
        # envia comando
        ang = np.clip(ang, 0.0, 180.0)
        self.kit.servo[SERVO_TILT].angle = ang
    ########################################
    # destrutor
    def __del__(self):
        None
