import serial
from colorama import Fore
from os import system
import struct

#   [Constantes]
NUM_MAX_NODOS:int   = 4
MSG_RSSI:int        = 2
MSG_ALRM:int        = 3

#   [Variables]
tablaRssi:list      = [[0.0]*NUM_MAX_NODOS for j in range(NUM_MAX_NODOS-1)]
tablaAlarmas:list   = [[None]*NUM_MAX_NODOS for j in range(NUM_MAX_NODOS-1)] # Realmente no la utilizamos, pero aquí está por si acaso...
tablaColores:list   = [[Fore.RESET]*NUM_MAX_NODOS for j in range(NUM_MAX_NODOS-1)]


s = serial.Serial(port= '/dev/ttyUSB0', baudrate=115200)

#   [Funciones]
def espera_preambulo() -> None:
    """
    Función que leerá el puerto serie hasta que se reciba la secuencia de bytes 
    '\x00\xff\xff\x00', en cuyo caso lerá 4 bytes más antes de salir del bucle.
    """
    secuencia:list[int]=[1,1,1,1]
    contador:int=0

    while True:
        secuencia[contador]=ord(s.read(1))
        if (    secuencia == [0,255,255,0] or 
                secuencia == [0,0,255,255] or 
                secuencia == [255,0,0,255] or 
                secuencia == [255,255,0,0]):
            break # Hemos recibido la secuencia que esperábamos
        contador += 1
        if contador > 3: contador = 0
    s.read(4)




#   [Script Principal]
while True:   
    # Formato del mensaje:
    #   00 FF FF 00 AA BB CC DD TM IM IS XXXX YYYY ZZZZ KKKK -> TOTAL: 19 Bytes
    #   ↳[--Preámbulo--(8B)--]↲  |  |  | [2B] [2B] [2B] [2B]
    #            Tipo de mensaje ↲  |  |   |    |    |    ↳ Medida [3]
    #                    Id Maestro ↲  |   |    |    ↳ Medida [2]
    #                          Id Nodo ↲   |    ↳ Medida [1]
    #                                      ↳ Medida [Base Station]

    espera_preambulo()
    tipo_msg:int = ord(s.read(1))
    if tipo_msg == MSG_RSSI:
        id_nodo:int
        for i in range(10):
            if i == 0:
                s.read(1) # Es el ID del maestro, no lo vamos a usar
            elif i == 1:
                id_nodo=ord(s.read(1))
            elif i==2 or i==4 or i==6 or i == 8:
                tablaRssi[id_nodo-1][int((i-2)/2)]=(struct.unpack("<h",s.read(2))[0]/100)


    elif tipo_msg == MSG_ALRM:
        id_nodo:int
        for i in range(10):
            if i == 0:
                s.read(1) # Es el ID del maestro, no lo vamos a usar
            elif i == 1:
                id_nodo=ord(s.read(1))
            elif i == 3 or i == 5 or i == 7 or i == 9:
                tablaAlarmas[id_nodo-1][int((i-3)/2)] = False
                tablaColores[id_nodo-1][int((i-3)/2)] = Fore.RESET
                if ord(s.read(1)) == 1:
                    tablaAlarmas[id_nodo-1][int((i-3)/2)] = True
                    tablaColores[id_nodo-1][int((i-3)/2)] = Fore.RED
                
                else:
                    tablaColores[id_nodo-1][int((i-3)/2)] = Fore.MAGENTA
            else:
                s.read(1)

        
    ###########################
    ####IMPRIMO TABLA FINAL ###
    ###########################

    system("clear")
    print(f"""
        |  BASE ST |  ID = 1  |  ID = 2  |  ID = 3  |
--------+----------+----------+----------+----------+ 
ID = 1  | {tablaColores[0][0]}{tablaRssi[0][0]:^+8.3f}{Fore.RESET} |     -    | {tablaColores[0][2]}{tablaRssi[0][2]:^+8.3f}{Fore.RESET} | {tablaColores[0][3]}{tablaRssi[0][3]:^+8.3f}{Fore.RESET} |
ID = 2  | {tablaColores[1][0]}{tablaRssi[1][0]:^+8.3f}{Fore.RESET} | {tablaColores[1][1]}{tablaRssi[1][1]:^+8.3f}{Fore.RESET} |     -    | {tablaColores[1][3]}{tablaRssi[1][3]:^+8.3f}{Fore.RESET} |
ID = 3  | {tablaColores[2][0]}{tablaRssi[2][0]:^+8.3f}{Fore.RESET} | {tablaColores[2][1]}{tablaRssi[2][1]:^+8.3f}{Fore.RESET} | {tablaColores[2][2]}{tablaRssi[2][2]:^+8.3f}{Fore.RESET} |     -    |
--------+----------+----------+----------+----------+
Nota: Todas las medidas están expresadas en {Fore.BLUE}dBm{Fore.RESET}.""")

    print(f"""DEBUG:
    Nodo 1:
        tablaRssi = {tablaRssi[0][0]}, {tablaRssi[0][1]}, {tablaRssi[0][2]}, {tablaRssi[0][3]}
        tablaAlarmas = {tablaAlarmas[0][0]}, {tablaAlarmas[0][1]}, {tablaAlarmas[0][2]}, {tablaAlarmas[0][3]}    
    Nodo 2:
        tablaRssi = {tablaRssi[1][0]}, {tablaRssi[1][1]}, {tablaRssi[1][2]}, {tablaRssi[1][3]}
        tablaAlarmas = {tablaAlarmas[1][0]}, {tablaAlarmas[1][1]}, {tablaAlarmas[1][2]}, {tablaAlarmas[1][3]}  
    Nodo 3:
        tablaRssi = {tablaRssi[2][0]}, {tablaRssi[2][1]}, {tablaRssi[2][2]}, {tablaRssi[2][3]}
        tablaAlarmas = {tablaAlarmas[2][0]}, {tablaAlarmas[2][1]}, {tablaAlarmas[2][2]}, {tablaAlarmas[2][3]}""")
