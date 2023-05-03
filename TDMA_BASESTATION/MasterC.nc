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


  event void TimerTramaTDMA.fired() {
    if (!busy) {
      TDMAmsg* tdma = (TDMAmsg*)(call Packet.getPayload(&pkt, sizeof(TDMAmsg)));

      if (tdma == NULL) {
        return;
      }

      
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
      
      setLeds(1);
      call TimerLeds.startOneShot(1000);
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
