# -*- coding: utf-8 -*-
########################################
# Disciplina: Topicos em Engenharia de Controle e Automacao IV (ENG075): 
# Fundamentos de Veiculos Autonomos - 2025/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automacao
# DELT - Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import time

BUZZER_PIN = 17

########################################
# classe para sons de indicacao
########################################
class Buzzer:
    ########################################
    # construtor
    def __init__(self):
        
        # Detectar a versao da Raspberry
        self.rpi_version = self.detect_rpi_version()
        print(f"[INFO] Detected Raspberry Pi {self.rpi_version}")
        
        # Raspberry Pi 5
        if self.rpi_version == 5:
            import lgpio as GPIO
            self.GPIO = GPIO
            # abre o chip GPIO0 (principal)
            self.chip = self.GPIO.gpiochip_open(0)
            # configura o pino como saida
            self.GPIO.gpio_claim_output(self.chip, BUZZER_PIN)

        # Raspberry Pi 3/4
        else:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            self.GPIO.setmode(GPIO.BCM)
            self.GPIO.setup(BUZZER_PIN, GPIO.OUT)
        
    ##############################################
    # Raspberry Pi version detector
    ##############################################
    def detect_rpi_version(self):
        try:
            with open('/proc/device-tree/model') as f:
                return 5 if 'Raspberry Pi 5' in f.read() else 4
        except:
            return 4  # Default caso nao consiga detectar
    
    ########################################
    # beep com timer
    def beep(self, timer=0.2, n=1):        
        for _ in range(n):
            
            # Raspberry Pi 5
            if self.rpi_version == 5:
                self.GPIO.gpio_write(self.chip, BUZZER_PIN, 1)  # Liga buzzer
                time.sleep(timer)
                self.GPIO.gpio_write(self.chip, BUZZER_PIN, 0)  # Desliga buzzer
                time.sleep(timer)

            # Raspberry Pi 3/4
            else:                
                self.GPIO.output(BUZZER_PIN, GPIO.HIGH)  # Liga buzzer
                time.sleep(timer)
                self.GPIO.output(BUZZER_PIN, GPIO.LOW)   # Desliga buzzer
                time.sleep(timer)
    
    ########################################
    # musiquinha final
    def victory_tune(self, n=1):
        for _ in range(n):
            pattern = [0.1, 0.1, 0.1, 0.3, 0.1, 0.5]
            for d in pattern:
                self.beep(timer=d)
            time.sleep(0.1)
        
    ########################################
    # Limpeza dos pinos
    ########################################
    def cleanup(self):
        if self.rpi_version == 5:
            self.GPIO.gpio_write(self.chip, BUZZER_PIN, 0)
            self.GPIO.gpiochip_close(self.chip)
        else:
            self.GPIO.output(BUZZER_PIN, GPIO.LOW)   # Desliga buzzer
            self.GPIO.cleanup()

    ########################################
    # Destrutor
    ########################################
    def close(self):
        #Ensure cleanup is called when object is deleted
        self.cleanup()

########################################
# main test
########################################
if __name__=="__main__":
    
    bz = Buzzer()
    t0 = time.time()
    try:
        while (time.time() - t0) <= 5.0:
            bz.beep(timer=0.3)
        
        time.sleep(1.0)    
        bz.victory_tune(n=1)
    except KeyboardInterrupt:
        None
        
    bz.close()
