// $Id: MasterC.nc,v 1.6 2010/06/29 22:07:40 scipio Exp $


#include <Timer.h>
#include "Master.h"

module MasterC {
  uses interface Boot;
  uses interface Leds;
  uses interface Timer<TMilli> as Timer0;
  uses interface Timer<TMilli> as Timer1;
  uses interface Packet;
  uses interface AMPacket;
  uses interface AMSend;
  uses interface Receive;
  uses interface SplitControl as AMControl;
}

implementation {

  uint16_t counter;
  message_t pkt;
  bool busy = FALSE;
  int ids[3] = {1,2,3};
  int tipoPet[3] = {TEMPERATURA, HUMEDAD, ILUMINANCIA};
  int orden[3] = {0,0,0};
  int orden_pet[3] = {0,0,0};
  int i, j, min_idx;

  void setLeds(uint16_t val) {
    if (val & 0x01)
      call Leds.led0On();
    else
      call Leds.led0Off();
    if (val & 0x02)
      call Leds.led1On();
    else
      call Leds.led1Off();
    if (val & 0x04)
      call Leds.led2On();
    else
      call Leds.led2Off();
  }

  void swap(int* xp, int* yp){
      int temp = *xp;
      *xp = *yp;
      *yp = temp;
  }

  event void Boot.booted() {
    call AMControl.start();
  }

  event void AMControl.startDone(error_t err) {
    if (err == SUCCESS) {
      call Timer0.startPeriodic(TIMER_PERIOD_MILLI);
    }
    else {
      call AMControl.start();
    }
  }

  event void AMControl.stopDone(error_t err) {
  }

  event void Timer1.fired(){
    setLeds(0);
  }

  event void Timer0.fired() {
    if (!busy) {
      TDMAmsg* tdma = (TDMAmsg*)(call Packet.getPayload(&pkt, sizeof(TDMAmsg)));

      if (tdma == NULL) {
        return;
      }

      // Generamos un array de números aleatorios para ver a quién le toca enviar qué medida
      for (i = 0; i<3; i++){
        orden_pet[i]=rand();
      }
      while(orden_pet[0]==orden_pet[1] || orden_pet[1]==orden_pet[2] || orden_pet[2]==orden_pet[0]){
        if (orden_pet[0]==orden_pet[1]){
          orden_pet[0]=rand();
        }
        if (orden_pet[1]==orden_pet[2]){
          orden_pet[1]=rand();
        }
        if (orden_pet[2]==orden_pet[0]){
          orden_pet[2]=rand();
        }
      }
      // Ordenamos en función de los números calculados
      for (i=0; i<2; i++){
        min_idx=i;
        for (j=i+1; j<3;j++){
          if (orden_pet[j]<orden_pet[min_idx])
            min_idx=j;
        }
        swap(&tipoPet[min_idx], &tipoPet[i]);
      }

      // Generamos otro array de números aleatorios para ver el orden de envío
      for (i = 0; i<3; i++){
        orden[i]=rand();
      }
      while(orden[0]==orden[1] || orden[1]==orden[2] || orden[2]==orden[0]){
        if (orden[0]==orden[1]){
          orden[0]=rand();
        }
        if (orden[1]==orden[2]){
          orden[1]=rand();
        }
        if (orden[2]==orden[0]){
          orden[2]=rand();
        }
      }
      // Ordenamos en función de los números calculados
      for (i=0; i<2; i++){
        min_idx=i;
        for (j=i+1; j<3;j++){
          if (orden[j]<orden[min_idx])
            min_idx=j;
        }
        swap(&ids[min_idx], &ids[i]);
      }

      
      tdma->idM = TOS_NODE_ID;
      for (i=0; i<3; i++){
        tdma->idS[i] = ids[i];
        tdma->tipoPeticion[i] = tipoPet[i];
      }
      tdma->periodo = TIMER_PERIOD_MILLI;
      
      
      
      if (call AMSend.send(AM_BROADCAST_ADDR,
                  &pkt, sizeof(TDMAmsg)) == SUCCESS) {
        busy = TRUE;
      }

    }
  }


  event void AMSend.sendDone(message_t* msg, error_t err) {
    if (&pkt == msg) {
      busy = FALSE;
      setLeds(0);
    }
  }

  event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
    
    if (len == sizeof(RespuestaMsg)) {
      
      RespuestaMsg* rcvPkt = (RespuestaMsg*)payload;
      
      if ((rcvPkt -> idS == 0 || rcvPkt -> idS == 1 || rcvPkt -> idS == 2)  &&
                rcvPkt -> idM == TOS_NODE_ID){
            
        if(rcvPkt -> tipo == TEMPERATURA){
          setLeds(1);
        } else if (rcvPkt -> tipo == HUMEDAD){
          setLeds(2);
        } else if (rcvPkt -> tipo == ILUMINANCIA){
          setLeds(4);
        }
          
        call Timer1.startOneShot(TIMER_ON_LEDS);

       }
    }
    return msg;
  }

}
