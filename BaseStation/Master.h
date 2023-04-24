// $Id: Master.h,v 1.4 2006/12/12 18:22:52 vlahan Exp $

#ifndef MASTER_H
#define MASTER_H

#define TEMPERATURA 0
#define HUMEDAD 1
#define ILUMINANCIA 2


enum {
  AM_BLINKTORADIO = 17,
  TIMER_PERIOD_MILLI = 3000,
  TIMER_ON_LEDS = TIMER_PERIOD_MILLI/(3*2)
};

typedef nx_struct TDMAmsg{
  nx_uint8_t idM;
  nx_uint8_t idS[3];
  nx_uint16_t tipoPeticion[3];
  nx_uint16_t periodo;
}TDMAmsg;

typedef nx_struct RespuestaMsg{
  nx_uint8_t idM;
  nx_uint8_t idS;
  nx_uint16_t rssi;
  nx_uint16_t tipo;
  nx_uint16_t medida;
}RespuestaMsg;

#endif




