import serial

# Abra a porta serial. O parâmetro "COMX" deve corresponder à porta do seu Arduino.
# O parâmetro "baudrate" deve corresponder à taxa de transmissão configurada no Arduino.
ser = serial.Serial('/dev/ttyUSB1', 57600)  # Substitua 'COM3' pela porta serial correta

try:
    while True:
        # Leia uma linha de dados da porta serial
        data = ser.readline().decode('utf-8').strip()
        
        # Faça algo com os dados recebidos
        print("Dado recebido:", data)

except KeyboardInterrupt:
    print("Recepção interrompida pelo usuário.")

except serial.SerialException as e:
    print(f"Ocorreu um erro na porta serial: {str(e)}")

finally:
    ser.close()  # Feche a porta serial quando terminar
