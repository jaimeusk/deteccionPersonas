// $Id: Slave.h,v 1.4 2006/12/12 18:22:52 vlahan Exp $

#ifndef SLAVE_H
#define SLAVE_H

#define NUM_MAX_NODOS 4 //Tiene en cuenta el MAESTRO

//Tipos de mensaje
#define msg_TDMA 0
#define msg_RESP 1


enum {
  AM_SLAVE = 17,
  TIMER_ON_LEDS = 750
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
  nx_uint8_t tipoMsg;
  nx_uint8_t idM;
  nx_uint8_t idS[NUM_MAX_NODOS-1];
  nx_uint16_t periodo;
  nx_uint16_t tiempoTrama;
  nx_uint16_t tiempoNodo;
}TDMAmsg;


typedef nx_struct RespuestaMsg{
  nx_uint8_t tipoMsg;
  nx_uint8_t idM;
  nx_uint8_t idS;
  nx_uint16_t rssi[NUM_MAX_NODOS];

}RespuestaMsg;



#endif
