# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import numpy as np
from adafruit_servokit import ServoKit

SERVO_STEERING = 0
SERVO_THROTLE  = 1

MAX_STERRING_ANGLE  = 20.0
ZERO_STERRING_ANGLE = 100.0
GAIN_STERRING_ANGLE = 50.0/MAX_STERRING_ANGLE

########################################
# Adafruit 16-channel servo driver
########################################
class Servos:
    ########################################
    # construtor
    def __init__(self):
        
        # Set channels to the number of servo channels on your kit.
        # 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
        self.kit = ServoKit(channels=16)
    
    ########################################
    # seta torque do motor
    def setU(self, u):
        self.kit.servo[SERVO_THROTLE].throttle = u
    
    ########################################
    # seta steer do veiculo (st in rad)
    def setSteer(self, st):
        
        # converte para graus
        st = np.rad2deg(st)
        
        # aplica calibracao devido a reduções do eixo
        st = GAIN_STERRING_ANGLE*st + ZERO_STERRING_ANGLE
        
        # envia comando
        st = np.clip(st, 0, 180)
        self.kit.servo[SERVO_STEERING].angle = st
        
        return
    
    ########################################
    # destrutor
    def __del__(self):
        self.kit.deinit()
        
########################################
# teste
########################################
import time

atuador = Servos()

#atuador.setSteer(np.deg2rad(0.0))
#time.sleep(1.0)

'''for i in np.linspace(-20.0, 20.0, 20):
    print(i)
    atuador.setSteer(np.deg2rad(i))
    time.sleep(1.0)'''
    
    
for i in np.linspace(1.0, -1.0, 20):
    print(i)
    atuador.setU(i)
    time.sleep(1.0)

atuador.setU(0.0)
