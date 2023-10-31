# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import serial
import numpy as np

DEVICE = '/dev/ttyUSB0'
BAUDRATE = 57600

REDUCAO_EIXO = 7.8
RAIO_RODA = 0.08

########################################
# classe para ler velocidade do robo
########################################
class Encoder:
    ########################################
    # construtor
    def __init__(self):
        
        # abrindo porta serial
        self.ser = serial.Serial(DEVICE, BAUDRATE)
        
    ########################################
    # lê velocidade
    def getVel(self):
        
        # apaga os dados que já chegaram para pegar o mais recente
        self.ser.flushInput()
        
        # recebe o RPM do eixo do motor pela serial
        rpm = float(self.ser.readline().decode('utf-8').strip())
        
        # redução do eixo do motor para a roda
        rpm = rpm/REDUCAO_EIXO
        
        # converte velocidade de rpm para m/s
        vel = RAIO_RODA*(np.pi/30.0)*rpm
        
        return vel
    
    ########################################
    # destrutor
    def __del__(self):
        # Feche a porta serial quando terminar
        self.ser.close()
