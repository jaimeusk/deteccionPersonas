// $Id: MasterC.nc,v 1.6 2010/06/29 22:07:40 scipio Exp $


#include <Timer.h>
#include "Master.h"


module MasterC {
  uses interface Boot;
  uses interface Leds;
  uses interface Timer<TMilli> as TimerTramaTDMA;
  uses interface Timer<TMilli> as TimerLeds;
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

  // Ahora mismo es un ejemplo, los nodos esclavos son 1,2 y 3.
  // En el futuro tendremos que obtenerlos dinamicamente.
  int ids[3] = {1,2,3};
  // int ids[NUM_MAX_NODOS];

  int orden[3] = {0,0,0};
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

  //Comienza a enviar tramas TDMA cada periodo
  event void AMControl.startDone(error_t err) {
    if (err == SUCCESS) {
      call TimerTramaTDMA.startPeriodic(TIMER_PERIODO_COMPLETO);
    }
    else {
      call AMControl.start();
    }
  }

  
  event void AMControl.stopDone(error_t err) {
  }

  // Apaga los leds cuando el temporizador expira.
  event void TimerLeds.fired(){
    setLeds(0);
  }


    /*
  typedef nx_struct TDMAmsg{
    nx_uint8_t idM;
    nx_uint8_t idS[NUM_MAX_NODOS];
    nx_uint16_t periodo;
  }TDMAmsg;
  */
  event void TimerTramaTDMA.fired() {
    if (!busy) {
      TDMAmsg* tdma = (TDMAmsg*)(call Packet.getPayload(&pkt, sizeof(TDMAmsg)));

      if (tdma == NULL) {
        return;
      }


      
      // Generamos un array de números aleatorios para ver a quién le toca enviar qué medida
      /*
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

      */
/*
typedef nx_struct TDMAmsg{
  nx_uint8_t idM;
  nx_uint8_t idS[NUM_MAX_NODOS];
  nx_uint16_t periodo;
  nx_uint16_t tiempoTrama;
}TDMAmsg;
*/
      
      // Rellenamos el mensaje tdma antes de enviarlo
      tdma->idM = TOS_NODE_ID;

      //El orden en el que pedimos las cosas a los nodos no es relevante.
      //tdma->idS = ids;
      
      for (i=0; i<NUM_MAX_NODOS; i++){
        tdma->idS[i] = ids[i];
      }

      tdma->tiempoTrama = TIMER_PERIODO_TRAMA;
      tdma->periodo = TIMER_PERIODO_COMPLETO;
      
      
      
      if (call AMSend.send(AM_BROADCAST_ADDR,
                  &pkt, sizeof(TDMAmsg)) == SUCCESS) {
        busy = TRUE;
      }

    }
  }


  event void AMSend.sendDone(message_t* msg, error_t err) {
    if (&pkt == msg) {
      busy = FALSE;
      setLeds(7);
      call TimerLeds.startOneShot(100); //Breve parpadeo 3 leds-->TDMA ENVIADO
    }
  }



/*

typedef nx_struct RespuestaMsg{
  nx_uint8_t idM;
  nx_uint8_t idS;
  nx_uint16_t rssi;
}RespuestaMsg;

*/

  // Esta funcion es para debuguear que funciona correctamente
  event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
    
    if (len == sizeof(RespuestaMsg)) {
      
      RespuestaMsg* rcvPkt = (RespuestaMsg*)payload;
      
      if ((rcvPkt -> idS == 1 || rcvPkt -> idS == 2 || rcvPkt -> idS == 3)  &&
                rcvPkt -> idM == TOS_NODE_ID){
            
        if(rcvPkt -> idS == 1){
          setLeds(1);
        } else if (rcvPkt -> idS == 2){
          setLeds(2);
        } else if (rcvPkt -> idS == 3){
          setLeds(4);
        }
          
        call TimerLeds.startOneShot(TIMER_ON_LEDS);

       }
    }
    return msg;
  }

}
