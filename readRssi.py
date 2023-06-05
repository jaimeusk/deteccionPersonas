# coding=utf-8

################################################################################
#####                             readRssi.py                              #####
#####                           Amando Antoñano                            #####
#####                Redes de Sensores y Sistemas Autonomos                #####
#####                        Universidad de Sevilla                        #####
################################################################################

### Script para leer del puerto serie mensajes con una tabla con rssi entre nodos y otra tabla alarmas
### - 8 bits para valor boolean TRUE o FALSE
### - 16 bits para medida de RSSI

##La idea es que tenga este formato
##                                      | ID = 1 | ID = 2 | ID = 3 | BASE ST |
##                              --------+--------+--------+--------+---------+ 
##                              ID = 1  |  RSSI  |  RSSI  |  RSSI  |  RSSI   |
##                              ID = 2  |  RSSI  |  RSSI  |  RSSI  |  RSSI   |
##                              ID = 3  |  RSSI  |  RSSI  |  RSSI  |  RSSI   |
##                              BASE ST |  RSSI  |  RSSI  |  RSSI  |  RSSI   |
##                              --------+--------+--------+--------+---------+ 
## SI EN LA TABLA ALARMAS HAY UN TRUE EN tablaAlarma[I][J] EL VALOR CORRESPONDIENTE EN tablaRssi [i][j] de RSSI aparecerá en rojo

import serial


NUM_MAX_FILAS = 4
NUM_MAX_COLUMNAS = 4
index = 0


##Antes definidas de otra forma pero me decian al ejecutarse los for de abajo que los indices se salian
##Asi que busque y encontre esta sugerencia
tablaRssi = [[0] * NUM_MAX_COLUMNAS for _ in range(NUM_MAX_FILAS)]
tablaAlarmas = [[0] * NUM_MAX_COLUMNAS for _ in range(NUM_MAX_FILAS)]



#hay que mirar que USBx es con motelist
s = serial.Serial(port= '/dev/ttyUSB0', baudrate=115200)


while True:
# 	while True:
# 		if not ord(s.read()) == 0x22: continue
# 		if not ord(s.read()) == 0x01: continue
# 		break

##Supuestamente recibire una tabla 4x4 con medidas rssi de 16 bits, es decir, recibire 16x16 = 256 bits = 32 Bytes
##Los primeros 32 Bytes contendran los valores rssi de la tabla rssi
    
    ############# ASI OBTENGO tablaRSSI DEL PUERTO SERIAL ##################

        for i in range (4):
            for j in range(4): 
                ##Leo los bytes de 2 en 2 porque cada 2B tengo un rssi
                r = s.read(2)
                ##Combino ambos bytes para obtener el valor del rssi
                rssi = (r[0]<<8) | r[1]
                tablaRssi [i][j] = rssi
                
    ###OTRA FORMA QUE NO SE SI SERIA CORRECTA
        # for i in range (4):
        #     for j in range(4): 
        #         ##Leo los bytes de 2 en 2 porque cada 2B tengo un rssi
        #         index = 0
        #         r = s.read(32)
        #         ##Combino ambos bytes para obtener el valor del rssi
        #         rssi = ord(r[index])<<8 | ord(r[index + 1])
        #         tablaRssi [i][j] = rssi

        #         ##Asi ire cogiendo las posic 0-1, 2-3, 3-4 de la variable r que contiene los 32B
        #         index += 2
                
                
                
    ###Para la proxima tabla que me llega es distinto porque los valores TRUE y false valen 8 bits
    ###Por lo que tendre 8b x 16 celdas = 128 bits = 16 bytes
                
    ############# ASI OBTENGO tablaAlarmas DEL PUERTO SERIAL ################## 

        for i in range (4):
            for j in range(4): 
                ##Leo los bytes de 2 en 2 porque cada boolean vale 8 bits pero lo recibo en 16 bits 
                r = s.read(2)
                ##Combino el valor de ambos bytes para obtener el boolean
                alarma = (r[0]<<8) | r[1]
                tablaAlarmas [i][j] = alarma

        
        
        
        ###########################
        ####IMPRIMO TABLA FINAL ###
        ###########################
        

        # Imprimir encabezado de columnas, hay 4 y compruebo que la ult tenga BASE ST
        for i in range(NUM_MAX_COLUMNAS):
            if i == (NUM_MAX_COLUMNAS - 1):
                print(" BASE ST |")
            print("| ID = {}".format(i + 1))

            

        print("\n")
        print("--------+--------+--------+--------+---------+")

        # Imprimir filas
        for i in range(NUM_MAX_COLUMNAS):
        # IMPRIMO LA FILA
            if i == (NUM_MAX_COLUMNAS - 1):

                print(" BASE ST |")
            print("| ID = {}".format(i + 1))


        # Imprimir valores de celdas DE ESA FILA
            for j in range(NUM_MAX_FILAS):
                if tablaAlarmas[i][j] == True:
                    ###No es seguro que aplique el color rojo porquqe depende de la terminal y su config
                    print("| \033[31m{}\033[0m".format(tablaRssi[i][j]))  # Impresión en color rojo
                else:
                    print("| {} ".format(tablaRssi[i][j]))

            print("\n")  # PASO A LA PROX FILA

        print("--------+--------+--------+--------+---------+")


s.close()

