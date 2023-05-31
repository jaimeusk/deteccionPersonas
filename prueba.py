# -*- coding: utf-8 -*-
"""
Created on Wed May 31 18:05:04 2023

@author: amando
"""
import serial
import time

# Configuración del puerto serial
puerto = '/dev/ttyUSB0'  # Reemplaza esto con el puerto serial correcto
baudrate = 115200  # Velocidad en baudios

# Datos de ejemplo para las tablas
tablaRssi = [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15, 16]]
tablaAlarma = [[True, True, True, True], [True, True, True, True], [True, True, True, True], [True, True, True, True]]

# Función para enviar las tablas por el puerto serial
def enviar_tablas():
    with serial.Serial(puerto, baudrate) as ser:
        # Envío de las tablas durante 1 minuto (aproximadamente 20 iteraciones)
        for _ in range(20):
            # Envío de la tablaRssi
            for fila in tablaRssi:
                for valor in fila:
                    # Envío del entero de 16 bits en little-endian
                    ser.write(valor.to_bytes(2, byteorder='little'))
                    time.sleep(0.1)  # Espera de 0.1 segundos entre cada envío de dato
            # Envío de la tablaAlarma
            for fila in tablaAlarma:
                for valor in fila:
                    # Envío del booleano como un byte (True=1, False=0)
                    ser.write(int(valor).to_bytes(1, byteorder='little'))
                    time.sleep(0.1)  # Espera de 0.1 segundos entre cada envío de dato
            time.sleep(3)  # Espera de 3 segundos entre cada envío de tablas

# Llamada a la función para enviar las tablas
enviar_tablas()


