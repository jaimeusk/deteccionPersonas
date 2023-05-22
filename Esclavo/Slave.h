// $Id: Slave.h,v 1.4 2006/12/12 18:22:52 vlahan Exp $

#ifndef SLAVE_H
#define SLAVE_H


#define TEMPERATURA 0
#define HUMEDAD 1
#define ILUMINANCIA 2

#define NUM_MAX_NODOS 3


enum {
  AM_SLAVE = 17,
  TIMER_PERIODO_COMPLETO = 1000,
  TIMER_PERIODO_TRAMA = 300,
  TIMER_ON_LEDS = TIMER_PERIODO_TRAMA
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
  nx_uint16_t tiempoNodo;
}TDMAmsg;


typedef nx_struct RespuestaMsg{
  nx_uint8_t idM;
  nx_uint8_t idS;
  nx_uint16_t rssi[NUM_MAX_NODOS];

}RespuestaMsg;



#endif
