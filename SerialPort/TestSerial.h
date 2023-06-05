
#ifndef TEST_SERIAL_H
#define TEST_SERIAL_H

// MACRO PARA DEFINIR MAXIMO NUMERO DE NODOS PERMITIDOS EN EL TDMA
// No tienen que existir todos necesariamente.
#define NUM_MAX_NODOS 4


enum {
  AM_BLINKTORADIO = 17,
  TIMER_PERIODO_COMPLETO = 2000,    // Tiempo total entre el comienzo de dos tramas consecutivas
  TIMER_PERIODO_TRAMA = 300,        // Tiempo desde el comienzo de una trama hasta el final de su TDMA
  TIMER_PERIODO_NODO = TIMER_PERIODO_TRAMA/NUM_MAX_NODOS, // Tiempo que tiene cada nodo para transmitir en el TDMA 
  TIMER_ON_LEDS = TIMER_PERIODO_TRAMA/(3),
  PERIODO_CALIBRACION = 15,
  ALARMA_STRIKES = 3,
  AM_TEST_SERIAL_MSG = 0x89, //Se utiliza para el envío de datos a través del puerto serie
  TAM_ARRAY = (NUM_MAX_NODOS+1)*(NUM_MAX_NODOS*1),
};

// Estructura de datos para envío de datos al puerto serie
typedef nx_struct test_serial_msg {
  // HAY UN LÍMITE DE TAMAÑO DEL ARRAY (Experimentalmente)
  // rssi_prueba[14]  -> FUNCIONA
  // rssi_prueba[15]  -> NO FUNCIONA
  // rssi_prueba[TAM_ARRAY]  --> IDEA ORIGINAL PARA MENSAJE
  nx_uint8_t idNodo;
  nx_int16_t rssi_prueba[NUM_MAX_NODOS];
} test_serial_msg_t;


/*
La estructura del TDMA cuenta con:
- idM:      identificador del maestro
- idS:      array de tamaño maximo del numero de nodos del TDMA
            el orden de aparición del id de los nodos es el slot 
            del que harán uso
- periodo:  tiempo desde que comienza una trama hasta el comienzo
            de la siguiente trama.
- tiemoTrama: tiempo desde que comienza una trama TDMA hasta que finaliza.

tiempoTrama <= periodo SIEMPRE


ESQUEMA EJEMPLO TDMA PARA ACLARAR TIEMPOS:
********************************************************************
**                                                                **
**    |M|S1|S2|...|SN|_______________________|M|S1|S2|...|SN|     **
**    |<----------     periodo    ---------->|                    **
**                                                                **
**    |M|S1|S2|...|SN|                                            **
**    |<-tiempoTram->|                                            **
**                                                                **
********************************************************************

*/
typedef nx_struct TDMAmsg{
  nx_uint8_t idM;
  nx_uint8_t idS[NUM_MAX_NODOS];
  nx_uint16_t periodo;
  nx_uint16_t tiempoTrama;
  nx_uint16_t tiempoNodo; // ¡Esto lo he añadido yo!
}TDMAmsg;


// La estructura de la respuesta es un ejemplo.
typedef nx_struct RespuestaMsg{
  nx_uint8_t idM;
  nx_uint8_t idS;
  nx_uint16_t rssi[NUM_MAX_NODOS+1];
}RespuestaMsg;


#endif
