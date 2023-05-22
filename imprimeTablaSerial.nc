//La idea es que tenga este formato
//                                      | ID = 1 | ID = 2 | ID = 3 | BASE ST |
//                              --------+--------+--------+--------+---------+ 
//                              ID = 1  |  RSSI  |  RSSI  |  RSSI  |  RSSI   |
//                              ID = 2  |  RSSI  |  RSSI  |  RSSI  |  RSSI   |
//                              ID = 3  |  RSSI  |  RSSI  |  RSSI  |  RSSI   |
//                              BASE ST |  RSSI  |  RSSI  |  RSSI  |  RSSI   |
//                              --------+--------+--------+--------+---------+ 
// SI EN LA TABLA ALARMAS HAY UN TRUE EN tablaAlarma[I][J] EL VALOR CORRESPONDIENTE EN tablaRssi [i][j] de RSSI aparecerá en rojo




#include "printf.h"
#include "Serial.h"

enum {
  NUM_MAX_NODOS_FILAS = 5,
  NUM_MAX_NODOS_COLUMNAS = 5,
  BUFFER_SIZE = 20
};

// Inicializar el módulo de puerto serial
  Serial.init();

// Función para enviar una cadena formateada a través del puerto serial
//El ... indica que la función puede recibir un número variable de argumentos adicionales
void sendSerial(const char *format, ...) {
  va_list args; //Aquí se declara una variable args de tipo va_list
  char buffer[BUFFER_SIZE]; //Este buffer se utilizará para almacenar la cadena formateada resultante.

  va_start(args, format); // Esta función inicializa la lista de argumentos variables. 

  //La función vsnprintf es similar a printf. 
  //En lugar de imprimir la cadena formateada, la guarda en el buffer especificado
  vsnprintf(buffer, BUFFER_SIZE, format, args);

  va_end(args); //Esta función finaliza el uso de la lista de argumentos variables
  
  //Una de las dos deberia funcionar, no tengo claro cual.
  call Serial.send(buffer, strlen(buffer));
  printf(buffer); //Esto enviará la cadena a través del puerto serial.
}

// Función para enviar la tabla a través del puerto serial
void imprimeTabla(int tablaRssi [NUM_MAX_NODOS_FILAS][NUM_MAX_NODOS_COLUMNAS], bool tablaAlarma [NUM_MAX_NODOS_FILAS][NUM_MAX_NODOS_COLUMNAS]) 
{
  for (uint8_t i = 0; i < NUM_MAX_NODOS_COLUMNAS; i++) {
    sendSerial("| ID = %d ", i + 1);
    if (i == NUM_MAX_NODOS_COLUMNAS - 1) {
      sendSerial("| BASE ST |");
    }
  }

  sendSerial("\n");
  sendSerial("--------+--------+--------+--------+---------+");

  for (uint8_t j = 0; j < NUM_MAX_NODOS_FILAS; j++) {
    sendSerial("ID = %d  |", j + 1);
    if (j == NUM_MAX_NODOS_FILAS - 1) {
      sendSerial(" BASE ST |");
    }
    for (uint8_t i = 0; i < NUM_MAX_NODOS_COLUMNAS; i++) {
      if (tablaAlarma[i][j] == TRUE) {
        sendSerial("  \033[31m%d\033[0m  |", tablaRssi[i][j]); // Impresión en color rojo
      } else {
        sendSerial("  %d  |", tablaRssi[i][j]);
      }
    }
    sendSerial("\n");
  }

  sendSerial("--------+--------+--------+--------+---------+");


// Ejemplo de uso AUNQUE CREO QUE NO ME DEJARÁ en nesC
//int main() {
  // Rellenar la tabla con valores de ejemplo
  //for (uint8_t i = 0; i < NUM_MAX_NODOS_COLUMNAS; i++) {
    //for (uint8_t j = 0; j < NUM_MAX_NODOS_FILAS; j++) {
      //tablaRssi[i][j] = (i + 1) * (j + 1);
      //tablaAlarma[i][j] = (i + j) % 2 == 0;
    //}
 // }


  // Enviar la tabla a través del puerto serial
  //imprimeTabla();
  // return 0;
  //}











