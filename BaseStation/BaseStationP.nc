// $Id: BaseStationP.nc,v 1.12 2010-06-29 22:07:14 scipio Exp $
  
/* 
 * BaseStationP bridges packets between a serial channel and the radio.
 * Messages moving from serial to radio will be tagged with the group
 * ID compiled into the BaseStation, and messages moving from radio to
 * serial will be filtered by that same group id.
 */

#include "AM.h"
#include "Serial.h"
#include <Timer.h>
#include "BaseStation.h"

module BaseStationP @safe() {
  uses {
    interface Boot;
    interface SplitControl as SerialControl;
    interface SplitControl as RadioControl;

    interface AMSend as UartSend[am_id_t id];
    interface Receive as UartReceive[am_id_t id];
    interface Packet as UartPacket;
    interface AMPacket as UartAMPacket;
    
    interface AMSend as RadioSend[am_id_t id];
    interface Receive as RadioReceive[am_id_t id];
    interface Receive as RadioSnoop[am_id_t id];
    interface Packet as RadioPacket;
    interface AMPacket as RadioAMPacket;

    interface Leds;

    interface Timer<TMilli> as Timer0;
    interface Timer<TMilli> as Timer1;
    interface Packet;
    interface Receive;
  }
}

implementation
{
  // ####################################################
  //                    Variables
  // ####################################################
  uint16_t counter;
  message_t pkt;
  bool busy = FALSE;
  int ids[3] = {1,2,3};
  int tipoPet[3] = {TEMPERATURA, HUMEDAD, ILUMINANCIA};
  int orden[3] = {0,0,0};
  int orden_pet[3] = {0,0,0};
  int i, j, min_idx;

  enum {
    UART_QUEUE_LEN = 12,
    RADIO_QUEUE_LEN = 12,
  };

  uint8_t count = 0;
  

  message_t  uartQueueBufs[UART_QUEUE_LEN];
  message_t  * ONE_NOK uartQueue[UART_QUEUE_LEN];
  uint8_t    uartIn, uartOut;
  bool       uartBusy, uartFull;

  message_t  radioQueueBufs[RADIO_QUEUE_LEN];
  message_t  * ONE_NOK radioQueue[RADIO_QUEUE_LEN];
  uint8_t    radioIn, radioOut;
  bool       radioBusy, radioFull;

  // ####################################################
  //                    Funciones
  // ####################################################

  task void uartSendTask();
  task void radioSendTask();

  void dropBlink() {
    call Leds.led2Toggle();
  }

  void failBlink() {
    call Leds.led2Toggle();
  }

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

  // ####################################################
  //                    Eventos
  // ####################################################
  event void Boot.booted() {
    uint8_t i;

    for (i = 0; i < UART_QUEUE_LEN; i++)
      uartQueue[i] = &uartQueueBufs[i];
    uartIn = uartOut = 0;
    uartBusy = FALSE;
    uartFull = TRUE;

    for (i = 0; i < RADIO_QUEUE_LEN; i++)
      radioQueue[i] = &radioQueueBufs[i];
    radioIn = radioOut = 0;
    radioBusy = FALSE;
    radioFull = TRUE;

    if (call RadioControl.start() == EALREADY)
      radioFull = FALSE;
    if (call SerialControl.start() == EALREADY)
      uartFull = FALSE;
  }

  event void RadioControl.startDone(error_t error) {
    if (error == SUCCESS) {
      radioFull = FALSE;
      call Timer0.startPeriodic(TIMER_PERIOD_MILLI);
    }
    else
      call RadioControl.start();
  }

  event void SerialControl.startDone(error_t error) {
    if (error == SUCCESS) {
      uartFull = FALSE;
    }
  }

  event void SerialControl.stopDone(error_t error) {}
  event void RadioControl.stopDone(error_t error) {}

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

      if (call RadioSend.send(AM_BROADCAST_ADDR, &pkt, sizeof(TDMAmsg)) == SUCCESS) {
        busy = TRUE;
      }

    }
  }

  message_t* ONE receive(message_t* ONE msg, void* payload, uint8_t len);
  
  event message_t *RadioSnoop.receive[am_id_t id](message_t *msg,
						    void *payload,
						    uint8_t len) {
    return receive(msg, payload, len);
  }
  
  event message_t *RadioReceive.receive[am_id_t id](message_t *msg,
						    void *payload,
						    uint8_t len) {
    return receive(msg, payload, len);
  }

  message_t* receive(message_t *msg, void *payload, uint8_t len) {
    message_t *ret = msg;

    atomic {
      // Se Imprime por pantalla
      if (!uartFull)
      {
        ret = uartQueue[uartIn];
        uartQueue[uartIn] = msg;

        uartIn = (uartIn + 1) % UART_QUEUE_LEN;
      
        if (uartIn == uartOut)
          uartFull = TRUE;

        if (!uartBusy)
          {
            post uartSendTask();
            uartBusy = TRUE;
          }
      }
      else
        dropBlink();
      
      // Tratamiento del mensaje
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
    }
    
    return ret;
  }

  uint8_t tmpLen;
  
  task void uartSendTask() {
    uint8_t len;
    am_id_t id;
    am_addr_t addr, src;
    message_t* msg;
    am_group_t grp;
    atomic
      if (uartIn == uartOut && !uartFull)
      {
        uartBusy = FALSE;
        return;
      }

    msg = uartQueue[uartOut];
    tmpLen = len = call RadioPacket.payloadLength(msg);
    id = call RadioAMPacket.type(msg);
    addr = call RadioAMPacket.destination(msg);
    src = call RadioAMPacket.source(msg);
    grp = call RadioAMPacket.group(msg);
    call UartPacket.clear(msg);
    call UartAMPacket.setSource(msg, src);
    call UartAMPacket.setGroup(msg, grp);

    if (call UartSend.send[id](addr, uartQueue[uartOut], len) == SUCCESS)
      call Leds.led1Toggle();
    else
      {
      failBlink();
      post uartSendTask();
      }
  }

  event void UartSend.sendDone[am_id_t id](message_t* msg, error_t error) {
    if (error != SUCCESS)
      failBlink();
    else
      atomic
    if (msg == uartQueue[uartOut])
      {
        if (++uartOut >= UART_QUEUE_LEN)
          uartOut = 0;
        if (uartFull)
          uartFull = FALSE;
      }
    post uartSendTask();
  }

  event message_t *UartReceive.receive[am_id_t id](message_t *msg,
						   void *payload,
						   uint8_t len) {
    message_t *ret = msg;
    bool reflectToken = FALSE;

    atomic
      if (!radioFull)
      {
        reflectToken = TRUE;
        ret = radioQueue[radioIn];
        radioQueue[radioIn] = msg;
        if (++radioIn >= RADIO_QUEUE_LEN)
          radioIn = 0;
        if (radioIn == radioOut)
          radioFull = TRUE;

        if (!radioBusy)
          {
            post radioSendTask();
            radioBusy = TRUE;
          }
      }
      else
	      dropBlink();

      if (reflectToken) {
        //call UartTokenReceive.ReflectToken(Token);
      }
    
    return ret;
  }

  task void radioSendTask() {
    uint8_t len;
    am_id_t id;
    am_addr_t addr,source;
    message_t* msg;
    
    atomic
      if (radioIn == radioOut && !radioFull)
    {
      radioBusy = FALSE;
      return;
    }

    msg = radioQueue[radioOut];
    len = call UartPacket.payloadLength(msg);
    addr = call UartAMPacket.destination(msg);
    source = call UartAMPacket.source(msg);
    id = call UartAMPacket.type(msg);

    call RadioPacket.clear(msg);
    call RadioAMPacket.setSource(msg, source);
    
    if (call RadioSend.send[id](addr, msg, len) == SUCCESS)
      call Leds.led0Toggle();
    else
      {
      failBlink();
      post radioSendTask();
      }
  }

  event void RadioSend.sendDone[am_id_t id](message_t* msg, error_t error) {
    if (error != SUCCESS)
      failBlink();
    else
      atomic
	  if (msg == radioQueue[radioOut])
	  {
	    if (++radioOut >= RADIO_QUEUE_LEN)
	      radioOut = 0;
	    if (radioFull)
	      radioFull = FALSE;
	  }
    
    post radioSendTask();
  }

}  
