#ifndef MASTER_H
#define MASTER_H

// Variable para definir el maximo numero de nodos permitidos en el TDMA (incluyendo al BaseStation)
// No tienen que estar todos necesariamente
#define NUM_MAX_NODOS 3


enum {
  AM_BLINKTORADIO = 17,
  TIMER_PERIODO_COMPLETO = 1000,    // Tiempo total entre el comienzo de dos tramas consecutivas
  TIMER_PERIODO_TRAMA = 300,        // Tiempo desde el comienzo de una trama hasta el final de su TDMA
  TIMER_PERIODO_NODO = TIMER_PERIODO_TRAMA/NUM_MAX_NODOS, // Tiempo que tiene cada nodo para transmitir en el TDMA 
  TIMER_ON_LEDS = TIMER_PERIODO_TRAMA/(3),
  PERIODO_CALIBRACION = 15,
  ALARMA_STRIKES = 3
};


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
  nx_uint16_t rssi[NUM_MAX_NODOS];
}RespuestaMsg;

#endif



