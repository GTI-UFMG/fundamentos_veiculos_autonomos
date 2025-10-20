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
import threading
import numpy as np

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
        #print(f"[INFO] Detected Raspberry Pi {self.rpi_version}")
        
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
        
        # tempo de subida
        self.timer = 0.0
        
        # lock de secao critica
        self.lock = threading.Lock()
        self.stop = threading.Event()
        # thread de leitura
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        
    ##############################################
    # Raspberry Pi version detector
    def detect_rpi_version(self):
        try:
            with open('/proc/device-tree/model') as f:
                return 5 if 'Raspberry Pi 5' in f.read() else 4
        except:
            return 4  # Default caso nao consiga detectar
    
    ########################################
    # thread para setar o beep
    def _run(self):
        while not self.stop.is_set():
            with self.lock:
                timer = self.timer
                
            # atua no buzzer
            if timer > 0.0:
                # Raspberry Pi 5
                if self.rpi_version == 5:
                    self.GPIO.gpio_write(self.chip, BUZZER_PIN, 1)  # Liga buzzer
                    time.sleep(timer)
                    self.GPIO.gpio_write(self.chip, BUZZER_PIN, 0)  # Desliga buzzer
                # Raspberry Pi 3/4
                else:                
                    self.GPIO.output(BUZZER_PIN, self.GPIO.HIGH)  # Liga buzzer
                    time.sleep(timer)
                    self.GPIO.output(BUZZER_PIN, self.GPIO.LOW)   # Desliga buzzer
                
                time.sleep(timer)
                with self.lock:
                    self.timer = 0.0
            else:
                time.sleep(0.01)
        
    ########################################
    # seta tempo de subida
    def beep(self, timer, block=True):
        # timer
        timer = np.max([0.0, timer])
        # espera descer
        if block:
            while True:
                with self.lock:
                    if self.timer == 0:
                        break
                    pass
        # novo timer
        with self.lock:
            self.timer = timer
    
    ########################################
    # musiquinha final
    def victory_tune(self):
        pattern = [0.1, 0.1, 0.1, 0.3, 0.1, 0.5]
        for d in pattern:
            self.beep(timer=d)
            time.sleep(d)
            
    ########################################
    # Limpeza dos pinos
    def cleanup(self):
        if self.rpi_version == 5:
            self.GPIO.gpio_write(self.chip, BUZZER_PIN, 0)
            self.GPIO.gpiochip_close(self.chip)
        else:
            self.GPIO.output(BUZZER_PIN, self.GPIO.LOW)   # Desliga buzzer
            self.GPIO.cleanup()
            
    ########################################
    # Desliga e fecha
    def close(self):
        # termina a thread
        self.stop.set()
        self.thread.join()
        
        #Ensure cleanup is called when object is deleted
        self.cleanup()

########################################
# main test
########################################
if __name__=="__main__":
    
    bz = Buzzer()
    try:
        t0 = time.time()
        while (time.time() - t0) <= 5.0:
            bz.beep(timer=0.3)
        
        bz.victory_tune()
        #time.sleep(5.0)
    
    except KeyboardInterrupt:
        None
        
    bz.close()
