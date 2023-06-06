// $Id: Maestro.nc,v 1.12 2010-06-29 22:07:14 scipio Exp $
  
/* 
 * Maestro bridges packets between a serial channel and the radio.
 * Messages moving from serial to radio will be tagged with the group
 * ID compiled into the BaseStation, and messages moving from radio to
 * serial will be filtered by that same group id.
 */

#include "AM.h"
#include "Serial.h"
#include "Master.h"


module MaestroC @safe() {
  uses {
    interface Boot;
    interface SplitControl as SerialControl;
    interface SplitControl as AMControl;

    interface AMSend as UartSend[am_id_t id];
    interface Receive as UartReceive[am_id_t id];
    interface Packet as UartPacket;
    interface AMPacket as UartAMPacket;
    
    //interface AMSend as RadioSend[am_id_t id];
    //interface Receive as RadioReceive[am_id_t id];
    //interface Receive as RadioSnoop[am_id_t id];
    interface Packet;
    interface AMPacket;
    interface AMSend;
    interface Receive;

    interface Timer<TMilli> as TimerTramaTDMA;
    interface Timer<TMilli> as TimerLeds;
    interface Leds;
  }
}

implementation
{
  // ############################
  // #    VARIABLES GLOBALES    #
  // ############################
  enum {
    UART_QUEUE_LEN = (16+16*NUM_MAX_NODOS), //Esta longitud sería válida unicamente
    RADIO_QUEUE_LEN = (16+16*NUM_MAX_NODOS),
  };

  int16_t rssi_dbm [NUM_MAX_NODOS-1][NUM_MAX_NODOS]; // No es lo mismo que rssi_historico porque puede almacenar un valor que haya hecho saltar una alarma
  bool strikes [NUM_MAX_NODOS-1][NUM_MAX_NODOS][PERIODO_CALIBRACION];
  int16_t rssi_historico [NUM_MAX_NODOS-1][NUM_MAX_NODOS][PERIODO_CALIBRACION];
  bool alarma [NUM_MAX_NODOS-1][NUM_MAX_NODOS]; // Enlace entre dos nodos que ha saltado la alarma
  int posicion_medida[NUM_MAX_NODOS-1];
  bool calibrado = FALSE; // Indicará si se han tomado las primeras medida de calibración
  
  uint16_t node_id_master;

  /* VARIABLES USADAS PARA LA ESCRITURA EN EL PUERTO SERIE */
  message_t  uartQueueBufs[UART_QUEUE_LEN];
  message_t  * ONE_NOK uartQueue[UART_QUEUE_LEN];
  uint16_t    uartIn, uartOut;
  bool       uartBusy, uartFull;
  

  message_t  radioQueueBufs[RADIO_QUEUE_LEN];
  message_t  * ONE_NOK radioQueue[RADIO_QUEUE_LEN];
  uint8_t    radioIn, radioOut;
  bool       radioBusy, radioFull;

  uint8_t count = 0;
  uint8_t tmpLen;  
  

  // ########## TDMA ############
  uint16_t counter;
  message_t pkt;
  bool busy = FALSE;

  // Ahora mismo es un ejemplo, los nodos esclavos son 1,2 y 3.
  // En el futuro tendremos que obtenerlos dinamicamente.
  int ids[3] = {1,2,3};
  int i, j, min_idx;

  // ############################
  // #         TAREAS           #
  // ############################

  task void uartSendTask();
  task void RadioSendTask();

  // ############################
  // #        FUNCIONES         #
  // ############################

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

  // ############################
  // #          EVENTOS         #
  // ############################


  event void Boot.booted() {
    node_id_master = TOS_NODE_ID;

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

    if (call AMControl.start() == EALREADY)
      radioFull = FALSE;
    if (call SerialControl.start() == EALREADY)
      uartFull = FALSE;
  }

  event void AMControl.startDone(error_t error) {
    if (error == SUCCESS) {
      radioFull = FALSE;
      //Comienza a enviar tramas TDMA cada periodo
      call TimerTramaTDMA.startPeriodic(TIMER_PERIODO_COMPLETO);
    }else {
      call AMControl.start();
    }
  }

  event void SerialControl.startDone(error_t error) {
    if (error == SUCCESS) {
      uartFull = FALSE;
    }
  }

  event void SerialControl.stopDone(error_t error) {
  }

  event void AMControl.stopDone(error_t error) {
  }

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
      tdma->idM = node_id_master;
      tdma->tipoMsg = msg_TDMA;

      //El orden en el que pedimos las cosas a los nodos no es relevante.
      //tdma->idS = ids;
      
      for (i=0; i<(NUM_MAX_NODOS-1); i++){
        tdma->idS[i] = ids[i];
      }

      tdma->tiempoTrama = TIMER_PERIODO_TRAMA;
      tdma->tiempoNodo = TIMER_PERIODO_NODO;
      tdma->periodo = TIMER_PERIODO_COMPLETO;
      
      
      
      if (call AMSend.send(AM_BROADCAST_ADDR,
                  &pkt, sizeof(TDMAmsg)) == SUCCESS) {
        busy = TRUE;
      }

    }
  }

  event void AMSend.sendDone(message_t* msg, error_t error) {
    if (error != SUCCESS)
      {

      }
    else
      atomic
    if (msg == radioQueue[radioOut])
      {
        if (++radioOut >= RADIO_QUEUE_LEN)
          radioOut = 0;
        if (radioFull)
          radioFull = FALSE;
      }
    if (&pkt == msg) {
      busy = FALSE;
      setLeds(1);
      call TimerLeds.startOneShot(TIMER_ON_LEDS); //Led 1 --> TDMA ENVIADO
    }
    
    post RadioSendTask();
  }

  event message_t* Receive.receive(message_t *msg,
						    void *payload,
						    uint8_t len) {
    
    message_t *ret = msg;
    message_t* msg_send;
    uint8_t i = 0;

    // Tratamos los mensajes de respuesta
    if (len == sizeof(RespuestaMsg)){
      RespuestaMsg* rcvPkt = (RespuestaMsg*)payload;
      if (rcvPkt->tipoMsg == msg_RESP){ // Comprobamos que, efectivamente, sea un mensaje de respuesta
        int id = rcvPkt->idS;
        int count_strikes = 0;
        uint8_t rssi_temp;
        int16_t sum_rssi;
        int16_t rssi_medio;
        setLeds(2);
        call TimerLeds.startOneShot(TIMER_ON_LEDS); //Led 1 --> TDMA ENVIADO

        if ((rcvPkt -> idS == 1 || rcvPkt -> idS == 2 || rcvPkt -> idS == 3)  &&
                  rcvPkt -> idM == node_id_master){
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
      
        // Aquí se envía por el puerto serie. 
        // Enviamos un mensaje con la tabla de RSSI medidos y otro con la tabla alarmas.
        atomic {
          for (i=0;i<2;i++){
            RespuestaMsg* msgSerial = (RespuestaMsg*)(call Packet.getPayload(&pkt, sizeof(RespuestaMsg)));
            msgSerial -> idM = rcvPkt -> idM;
            msgSerial -> idS = rcvPkt -> idS;
            switch(i){
              case 1: // RSSI
                msgSerial -> tipoMsg = msg_RSSI;
                for (j=0; j<(NUM_MAX_NODOS); j++){
                  msgSerial -> rssi[j] = rssi_dbm[rcvPkt -> idS - 1][j];
                }
                break;
              case 0: // Alarmas
                msgSerial -> tipoMsg = msg_ALRM;
                for (j=0; j<(NUM_MAX_NODOS); j++){
                  if (alarma[rcvPkt -> idS - 1][j])
                    msgSerial -> rssi[j] = 1;
                  else
                    msgSerial -> rssi[j] = 1;
                }
                break;
            }

            while(uartFull){}
            if (!uartFull){
              
              ret = uartQueue[uartIn];
              memcpy(msg->data, msgSerial, sizeof(RespuestaMsg));
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
          }
        }

      }
    }
    return ret;
  }

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
    tmpLen = len = call Packet.payloadLength(msg);
    id = call AMPacket.type(msg);
    addr = call AMPacket.destination(msg);
    src = call AMPacket.source(msg);
    grp = call AMPacket.group(msg);
    call UartPacket.clear(msg);
    call UartAMPacket.setSource(msg, src);
    call UartAMPacket.setGroup(msg, grp);

    if (call UartSend.send[id](addr, uartQueue[uartOut], len) == SUCCESS){}
    else
      {
        post uartSendTask();
      }
  }

  event void UartSend.sendDone[am_id_t id](message_t* msg, error_t error) {
    if (error != SUCCESS){}
      //failBlink();
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
            post RadioSendTask();
            radioBusy = TRUE;
          }
      }


    if (reflectToken) {
      //call UartTokenReceive.ReflectToken(Token);
    }
    
    return ret;
  }

  task void RadioSendTask() {
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

    call Packet.clear(msg);
    call AMPacket.setSource(msg, source);
    
    if (call AMSend.send(addr, msg, len) == SUCCESS)
      {//call Leds.led0Toggle();
      }
    else
      {
        //failBlink();
        post RadioSendTask();
      }
  }

  

  



}  
