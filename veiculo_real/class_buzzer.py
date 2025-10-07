# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2025/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import RPi.GPIO as GPIO
import time

BUZZER_PIN = 17

########################################
# classe para sons de indicacao
########################################
class Buzzer:
    ########################################
    # construtor
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUZZER_PIN, GPIO.OUT)
    
    ########################################
    # beep com timer
    def beep(self, timer=0.2, n=1):        
        for _ in range(n):
            GPIO.output(BUZZER_PIN, GPIO.HIGH)  # Liga buzzer
            time.sleep(timer)
            GPIO.output(BUZZER_PIN, GPIO.LOW)   # Desliga buzzer
            time.sleep(timer)
    
    ########################################
    # musiquinha final
    def victory_tune(self, n=1):
        for _ in range(n):
            pattern = [0.1, 0.1, 0.1, 0.3, 0.1, 0.5]
            for d in pattern:
                self.beep(timer=d)
            time.sleep(0.01)
        
    ########################################
    # fecha
    def close(self):
        GPIO.cleanup()

########################################
# main test
########################################
if __name__=="__main__":
    
    bz = Buzzer()
    t0 = time.time()
    try:
        while (time.time() - t0) <= 5.0:
            bz.beep()
            
    except KeyboardInterrupt:
        bz.close()

    bz.victory_tune(n=2)
    
    bz.close()
