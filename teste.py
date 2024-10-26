import serial
import struct
import time
import senoide

# Parâmetros da senoide
A = 1        # Amplitude
f = 1        # Frequência (Hz)
Fs = 100     # Frequência de amostragem (Hz)
phi = 0      # Fase (radianos)

# Gera a senoide
y = senoide.senoide(A, f, Fs, phi)

# Configurar a porta serial (ajuste o nome da porta conforme necessário)
ser = serial.Serial('COM3', 115200, timeout=1)  # Use 'COM3' no Windows, '/dev/ttyUSB0' ou '/dev/ttyACM0' no Linux

def send_float(value):
    # Envia um número de ponto flutuante (float) de 32 bits pela UART
    data = struct.pack('<f', value)  # '<f' significa float de 4 bytes (little-endian)
    ser.write(data)

def receive_int32():
    # Recebe um número inteiro de 32 bits pela UART
    data = ser.read(4)  # Lê 4 bytes
    if len(data) == 4:
        return struct.unpack('<I', data)[0]  # Converte de volta para um inteiro
    else:
        return None

try:
    for value in y:
        # Envia cada valor da senoide como um float de 32 bits
        send_float(value)
        time.sleep(0.01)  # Ajuste o tempo conforme necessário para sincronizar a comunicação

    # Após enviar todos os valores, você pode ler um número do microcontrolador
    received = receive_int32()
    if received is not None:
        print(f"Received: {received}")
    else:
        print("No data received")

except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")
