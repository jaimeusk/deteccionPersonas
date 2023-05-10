// $Id: BaseStationP.nc,v 1.12 2010-06-29 22:07:14 scipio Exp $
  
/* 
 * BaseStationP bridges packets between a serial channel and the radio.
 * Messages moving from serial to radio will be tagged with the group
 * ID compiled into the BaseStation, and messages moving from radio to
 * serial will be filtered by that same group id.
 */

#include "AM.h"
#include "Serial.h"
#include "Master.h"

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
  }
}

implementation
{
  enum {
    UART_QUEUE_LEN = 12,
    RADIO_QUEUE_LEN = 12,
  };

  int16_t rssi_dbm [NUM_MAX_NODOS][NUM_MAX_NODOS]; // No es lo mismo que rssi_historico porque puede almacenar un valor que haya hecho saltar una alarma
  bool strikes [NUM_MAX_NODOS][NUM_MAX_NODOS][PERIODO_CALIBRACION];
  int16_t rssi_historico [NUM_MAX_NODOS][NUM_MAX_NODOS][PERIODO_CALIBRACION];
  bool alarma [NUM_MAX_NODOS][NUM_MAX_NODOS]; // Enlace entre dos nodos que ha saltado la alarma
  int posicion_medida[NUM_MAX_NODOS];
  bool calibrado = FALSE; // Indicará si se han tomado las primeras medida de calibración
  

  message_t  uartQueueBufs[UART_QUEUE_LEN];
  message_t  * ONE_NOK uartQueue[UART_QUEUE_LEN];
  uint8_t    uartIn, uartOut;
  bool       uartBusy, uartFull;

  message_t  radioQueueBufs[RADIO_QUEUE_LEN];
  message_t  * ONE_NOK radioQueue[RADIO_QUEUE_LEN];
  uint8_t    radioIn, radioOut;
  bool       radioBusy, radioFull;

  task void uartSendTask();
  task void radioSendTask();

  void dropBlink() {
    call Leds.led2Toggle();
  }

  void failBlink() {
    call Leds.led2Toggle();
  }

  event void Boot.booted() {
    uint8_t i;
    for (i = 0; i<NUM_MAX_NODOS; i++)
      posicion_medida[i] = 0;

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
    }
  }

  event void SerialControl.startDone(error_t error) {
    if (error == SUCCESS) {
      uartFull = FALSE;
    }
  }

  event void SerialControl.stopDone(error_t error) {}
  event void RadioControl.stopDone(error_t error) {}

  uint8_t count = 0;

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
    // Tratamos los mensajes de respuesta
    if (len == sizeof(RespuestaMsg)){
      RespuestaMsg* rcvPkt = (RespuestaMsg*)payload;
      int id = rcvPkt->idS;
      int i;
      int j;
      int count_strikes = 0;
      uint16_t rssi_temp;
      int16_t sum_rssi;
      int16_t rssi_medio;


      // Introducimos el valor del los rssi medidos 
      for (i = 0; i<NUM_MAX_NODOS; i++){
        rssi_temp = rcvPkt->rssi[i];
        if (rssi_temp>= 128)
            rssi_dbm[id-1][i] = rssi_temp - 45 - 256;
        else
            rssi_dbm[id-1][i] = rssi_temp - 45;
        

        if (!calibrado){
          //Añadimos la medida a rssi_histórico y un False a Strikes
          rssi_historico[id-1][i][posicion_medida[id-1]] = rssi_dbm[id-1][i];
          strikes[id-1][i][posicion_medida[id-1]] = FALSE;
          alarma[id-1][i] = FALSE;
        }else{
          sum_rssi = 0;
          for(j = 0;j<(PERIODO_CALIBRACION-1);j++){
            sum_rssi += rssi_historico[id-1][i][j];
          }
          rssi_medio = sum_rssi/PERIODO_CALIBRACION;
          if(rssi_dbm[id-1][i]<(0.8*rssi_medio)||rssi_dbm[id-1][i]>(1.2*rssi_medio)){
            // Añadimos rssi_medio a rssi_histórico y un True a Strikes
            rssi_historico[id-1][i][posicion_medida[id-1]] = rssi_medio;
            strikes[id-1][i][posicion_medida[id-1]]=TRUE;
          }else{
            // Añadimos la medida a rssi_histórico y un False a Strikes
            rssi_historico[id-1][i][posicion_medida[id-1]] = rssi_dbm[id-1][i];
            strikes[id-1][i][posicion_medida[id-1]]=FALSE;
          }
          // Comprobamos si hay más de (ALARMA_STRIKES) alarmas en Strikes
          count_strikes = 0;
          for(j = 0; j<PERIODO_CALIBRACION; j++)
            if (strikes[id-1][i][j])
              count_strikes++;
          if (count_strikes >= ALARMA_STRIKES)
            alarma[id-1][i] = TRUE;
          else
            alarma[id-1][i] = FALSE;
        }

        }
        //Aumentamos el valor de "posicion_medida" y, si es mayor de PERIODO_CALIBRACION, lo reseteamos
        posicion_medida[id-1]++;
        if (posicion_medida[id-1] >= PERIODO_CALIBRACION){
          posicion_medida[id-1] = 0;
          calibrado = TRUE;
        }
      
    }

    // Aquí se imprime el contenido del mensaje
    atomic {
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
