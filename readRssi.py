import serial
from colorama import Fore
from os import system
import struct

#   [Constantes]
NUM_MAX_NODOS:int   = 4
MSG_RSSI:int        = 2
MSG_ALRM:int        = 3
MAX_STRIKES:int     = 3
T_CALIBRACION:int   = 15
TOLERANCIA:float    = 0.05

#   [Variables]
# Nota: al inicializar las tablas de esta forma, una tabla de tamaño x[4][3][2] se inicializaría con [[["x"]*2 for j in range(3)] for i in range(4)]
tablaRssi:list      = [[0.0]*NUM_MAX_NODOS for j in range(NUM_MAX_NODOS-1)]
tablaAlarmas:list   = [[Fore.RESET]*NUM_MAX_NODOS for j in range(NUM_MAX_NODOS-1)]

strikes:list        = [[[False]*T_CALIBRACION for j in range(NUM_MAX_NODOS)]for i in range(NUM_MAX_NODOS-1)] # Array NUM_MAX_NODOS x NUM_MAX_NODOS-1 (HxV)
rssi_historico:list = [[[None]*T_CALIBRACION for j in range(NUM_MAX_NODOS)]for i in range(NUM_MAX_NODOS-1)]
calibrado:list      = [False]*(NUM_MAX_NODOS-1)
posicion_medida:list= [0]*(NUM_MAX_NODOS-1)
rssi_medio:list     = [[0.0]*NUM_MAX_NODOS for j in range(NUM_MAX_NODOS-1)]

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
        
        for i in range(NUM_MAX_NODOS):
            if not calibrado[id_nodo-1]:
                rssi_historico[id_nodo-1][i][posicion_medida[id_nodo-1]]=tablaRssi[id_nodo-1][i]
                strikes[id_nodo-1][i][posicion_medida[id_nodo-1]] = False
                tablaAlarmas[id_nodo-1][i] = Fore.RESET
            else:
                sum_rssi:float = 0.0
                for j in range(T_CALIBRACION):
                    sum_rssi += rssi_historico[id_nodo-1][i][j]
                rssi_medio[id_nodo-1][i] = sum_rssi/T_CALIBRACION

                if(     abs(tablaRssi[id_nodo-1][i])<abs((1-TOLERANCIA)*rssi_medio[id_nodo-1][i]) or 
                        abs(tablaRssi[id_nodo-1][i])>abs((1+TOLERANCIA)*rssi_medio[id_nodo-1][i])):
                    # Añadimos rssi_medio al histórico y un True a Strikes
                    rssi_historico[id_nodo-1][i][posicion_medida[id_nodo-1]] = rssi_medio[id_nodo-1][i]
                    strikes[id_nodo-1][i][posicion_medida[id_nodo-1]] = True
                else:
                    # Añadimos la medida a rssi_histórico y un False a Strikes
                    rssi_historico[id_nodo-1][i][posicion_medida[id_nodo-1]] = tablaRssi[id_nodo-1][i]
                    strikes[id_nodo-1][i][posicion_medida[id_nodo-1]] = False
                
                # Comprobamos si hay más de (ALARMA_STRIKES) alarmas en Strikes
                count_strikes:int = 0
                for j in range(T_CALIBRACION):
                    if strikes[id_nodo-1][i][j]:
                        count_strikes += 1
                if count_strikes >= MAX_STRIKES:
                    tablaAlarmas[id_nodo-1][i] = Fore.RED
                else:
                    tablaAlarmas[id_nodo-1][i] = Fore.RESET
        # Aumentamos el valor de "posicion_medida" y, si es mayor de PERIODO_CALIBRACION, lo reseteamos
        posicion_medida[id_nodo-1]+=1
        if posicion_medida[id_nodo-1] >= T_CALIBRACION:
            posicion_medida[id_nodo-1] = 0
            calibrado[id_nodo-1] = True

    
    #   [Tabla dinámica, se adapta a NUM_MAX_NODOS]
    system("clear")
    
    #   Cabecera tabla dinamica
    print("        |  BASE ST |",end="")
    for i in range(NUM_MAX_NODOS-1):
        print(f"  ID = {i+1}  |", end="")
    print("")

    print("--------+",end="")
    for i in range(NUM_MAX_NODOS):
        print("----------+",end="")
    print("")

    #   Imprimimos fila a fila
    for i in range(1,NUM_MAX_NODOS):
        print(f"ID = {i}  |",end="")

        #   Imprimimos columna a columna
        for j in range(NUM_MAX_NODOS):
            if(i == j):
                print("     -    |",end="")
            else:
                print(f" {tablaAlarmas[i-1][j]}{tablaRssi[i-1][j]:^+8.3f}{Fore.RESET} |",end="")
        print("")

    #   Imprimimos pie de tabla
    print("--------+",end="")
    for i in range(NUM_MAX_NODOS):
        print("----------+",end="")
    print("")

    print(f"Nota: Todas las medidas están expresadas en {Fore.BLUE}dBm{Fore.RESET}.")
    
    
    print(f"Calibrado: ",end="")
    for i in calibrado:
        if i:
            print(f"{Fore.GREEN}True{Fore.RESET} ",end="")
        else:
            print(f"{Fore.RED}False{Fore.RESET} ",end="")
    print(f"\n",end="")
    print(f"Tolerancia: {TOLERANCIA*100}%")
    
    for id in range(NUM_MAX_NODOS-1):
        if calibrado[id]:
            print(f"\t Rango de valores medios nodo {id+1}: ")
            for i in range(NUM_MAX_NODOS):
                if i==0:
                    print(f"\t\t[{id+1}, BaseST]: ", end="")
                elif i-1==id:
                    pass
                else:
                    print(f"\t\t[{id+1}, {i}]:      ", end="")
                
                if i-1 != id:
                    print(f"{(1+TOLERANCIA)*rssi_medio[id][i]:.3f} ~ {(1-TOLERANCIA)*rssi_medio[id][i]:.3f}")

    #for i in rssi_historico: 
    #    print(f"{i}\n")
