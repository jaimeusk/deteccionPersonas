#include <stdio.h>


//La idea es que tenga este formato
//                                      | ID = 1 | ID = 2 | ID = 3 | BASE ST |
//                              --------+--------+--------+--------+---------+ 
//                              ID = 1  |  RSSI  |  RSSI  |  RSSI  |  RSSI   |
//                              ID = 2  |  RSSI  |  RSSI  |  RSSI  |  RSSI   |
//                              ID = 3  |  RSSI  |  RSSI  |  RSSI  |  RSSI   |
//                              BASE ST |  RSSI  |  RSSI  |  RSSI  |  RSSI   |
//                              --------+--------+--------+--------+---------+ 
// SI EN LA TABLA ALARMAS HAY UN TRUE EN tablaAlarma[I][J] EL VALOR CORRESPONDIENTE EN tablaRssi [i][j] de RSSI aparecerá en rojo




// Método para imprimir la tabla
void imprimeTabla(int tablaRssi[NUM_MAX_NODOS_FILAS][NUM_MAX_NODOS_COLUMNAS], bool tablaAlarma[NUM_MAX_NODOS_FILAS][NUM_MAX_NODOS_COLUMNAS]) {
    int i, j;

    // Imprimir encabezado de columnas
    
    for (i = 0; i < NUM_MAX_NODOS_COLUMNAS; i++) {
        printf(" | ID = %d ", i + 1);
        if (i = NUM_MAX_NODOS_COLUMNAS - 1)
        {
            printf ("| BASE ST |");
        }
    }

    printf("\n");
    printf("--------+--------+--------+--------+---------+");

    // Imprimir filas
    for (i = 0; i < NUM_MAX_NODOS_COLUMNAS; i++) {
        //IMPRIMO LA FILA
        printf("ID = %d  |", i + 1);
        if (i = NUM_MAX_NODOS_COLUMNAS - 1)
        {
            printf ("BASE ST |");
        }

        // Imprimir valores de celdas DE ESA FILA
        for (j = 0; j < NUM_MAX_NODOS_FILAS; j++) {
            if (tablaAlarma[i][j] == TRUE) {
                printf("|  \033[31m%d\033[0m ", tablaRssi[i][j]); // Impresión en color rojo
            } else {
                printf("|  %d ", tablaRssi[i][j]);
            }
        }
        printf("|\n"); //PASO A LA PROX FILA
        
    }
    printf("--------+--------+--------+--------+---------+");
}







