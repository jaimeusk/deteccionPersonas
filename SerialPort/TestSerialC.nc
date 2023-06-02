// $Id: TestSerialC.nc,v 1.7 2010-06-29 22:07:25 scipio Exp $

/*									tab:4
 * Copyright (c) 2000-2005 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the University of California nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright (c) 2002-2003 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */

/**
 * Application to test that the TinyOS java toolchain can communicate
 * with motes over the serial port. 
 *
 *  @author Gilman Tolle
 *  @author Philip Levis
 *  
 *  @date   Aug 12 2005
 *
 **/

#include "Timer.h"
#include "TestSerial.h"

module TestSerialC {
  uses {
    interface SplitControl as Control;
    interface Leds;
    interface Boot;
    interface Receive;
    interface AMSend;
    interface Timer<TMilli> as TimerIniTDMA;
    interface Timer<TMilli> as TimerApagaLeds;
    interface Timer<TMilli> as TimerFinTDMA;
    interface Packet;
  }
}
implementation {


  /******* INICIO DECLARACIÓN VARIABLES ********/



  uint16_t node_id_master;
  
  /*  Variables que van a utilizarse para el envio de paquetes
      a través del puerto serie */
  message_t packet;
  message_t arrayPaquetes[NUM_MAX_NODOS];
  bool sigueEnviando = TRUE;

  bool locked = FALSE;

  
  /* Variables para distintas utilidades */
  uint8_t i,j,z, min_idx;
  


  /* Variables usadas para implementar alarmas */
  int16_t rssi_dbm [NUM_MAX_NODOS][NUM_MAX_NODOS]; // No es lo mismo que rssi_historico porque puede almacenar un valor que haya hecho saltar una alarma
  bool strikes [NUM_MAX_NODOS][NUM_MAX_NODOS][PERIODO_CALIBRACION];
  int16_t rssi_historico [NUM_MAX_NODOS][NUM_MAX_NODOS][PERIODO_CALIBRACION];
  bool alarma [NUM_MAX_NODOS][NUM_MAX_NODOS]; // Enlace entre dos nodos que ha saltado la alarma
  int posicion_medida[NUM_MAX_NODOS];
  bool calibrado = FALSE; // Indicará si se han tomado las primeras medida de calibración
  

  /* VARIABLES PARA REALIZAR PRUEBAS (BORRAR AL ACABAR) */
  int16_t rssi_prueba[NUM_MAX_NODOS][NUM_MAX_NODOS] = {
        { -60, -70, -80, -90 },
        { -75, -65, -85, -95 },
        { -90, -85, -75, -65 },
        { -10, -20, -30, -40}};


  bool alarma_prueba [NUM_MAX_NODOS][NUM_MAX_NODOS] = {
        {TRUE, FALSE, FALSE, TRUE},
        {FALSE, FALSE, FALSE, FALSE},
        {FALSE, TRUE, TRUE, FALSE},
        {TRUE, TRUE, TRUE, TRUE}};

  


  /* Variables utilizadas para el TDMA */
  uint16_t counter;
  message_t pkt;
  bool busy = FALSE;

  // Ahora mismo es un ejemplo, los nodos esclavos son 1,2 y 3.
  // En el futuro tendremos que obtenerlos dinamicamente.
  int ids[3] = {1,2,3};
  int orden[3] = {0,0,0};
  /* Descomentar cuando ids y orden sean dinamicos (en boot se inicializan)
  int ids[NUM_MAX_NODOS];
  int orden[NUM_MAX_NODOS];
  */
  
  
  /*******    FIN DECLARACION VARIABLES    ************/



  /*******      DECLARACION FUNCIONES      *************/
    
    /*  ENCIENDE LOS LEDS
    Cada valor provoca lo siguiente:
                  (R) (V) (A)
        num       Ld1 Ld2 Ld3
        1 ->       O   -   -
        2 ->       -   O   -
        3 ->       O   O   -
        4 ->       -   -   O
        5 ->       O   -   O
        6 ->       -   O   O
        7 ->       O   O   O
  */
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


   /*******    FIN DECLARACION FUNCIONES    *************/



  /********             EVENTOS             *************/


  event void Boot.booted() {
    node_id_master = TOS_NODE_ID;

    for (i = 0; i<NUM_MAX_NODOS; i++)
      posicion_medida[i] = 0;

    
    call Control.start();

    /*
    for(i=0; i< NUM_MAX_NODOS; i++){
      int ids[i] = i+1;
      int orden[3] = 0;
    }
    */
    
  }

  
  event void AMSend.sendDone(message_t* bufPtr, error_t error) {
    if (&packet == bufPtr) {
      locked = FALSE;
    }
  }

  event void Control.startDone(error_t err) {
    if (err == SUCCESS) {
      call TimerIniTDMA.startPeriodic(TIMER_PERIODO_COMPLETO);
    } else {
      call Control.start();
    }
  }

  event void Control.stopDone(error_t err) {}

  event void TimerApagaLeds.fired(){
    setLeds(0);
  }

  event void TimerFinTDMA.fired(){
    setLeds(0);
  }




  /****  EVENTOS RELATIVOS A TIMERS****/

  event void TimerIniTDMA.fired() {
    
    // Lanzamos un temporizador para que cuando finalice, envie al puerto serie los RSSI
    call TimerFinTDMA.startOneShot(TIMER_PERIODO_TRAMA);

    /* Generamos trama TDMA */
    if (!busy) {
      TDMAmsg* tdma = (TDMAmsg*)(call Packet.getPayload(&pkt, sizeof(TDMAmsg)));

      if (tdma == NULL) {
        return;
      }

      
      // Rellenamos el mensaje tdma antes de enviarlo
      tdma->idM = node_id_master;

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

    /* GENERAMOS PAQUETE rcm PARA PUERTO SERIE */
    if (locked) {
      return;
    }
    else {
      
      
      for(i=0; i<NUM_MAX_NODOS; i++){  
        
        test_serial_msg_t* rcm = (test_serial_msg_t*)call Packet.getPayload(&packet, sizeof(test_serial_msg_t));
        
        if (rcm == NULL) {
          setLeds(1);
          return;
        }
          
        if (call Packet.maxPayloadLength() < sizeof(test_serial_msg_t)) {
          setLeds(2);
          return;
        }

        
          
          rcm->idNodo = i;      
          for(j=0; j<NUM_MAX_NODOS; j++){
            rcm->rssi_prueba[i] = rssi_prueba[i][j];
          }         
        
        
        /* REALIZA UN ENVIO POR CADA ITERACIÓN (No funciona, solo se recibe el último) */
        setLeds(7);
        if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(test_serial_msg_t)) == SUCCESS) {
          locked = TRUE;
        }

      }

      
      
    }
  }



  /******  EVENTO QUE PROCESA LA RECEPCIÓN DE MENSAJES      ******
  *******  YA SEAN A TRAVÉS DEL PUERTO SERIE O DE LA RADIO  *****/
  event message_t* Receive.receive(message_t* msg, 
				   void* payload, uint8_t len) {
    
    message_t *ret = msg; // Creo que se puede borrar??? (Jaime)

    if(len==sizeof(RespuestaMsg)){
      
      RespuestaMsg* rcvPkt = (RespuestaMsg*)payload;
      int id = rcvPkt->idS;
      int count_strikes = 0;
      uint8_t rssi_temp;
      int16_t sum_rssi;
      int16_t rssi_medio;

      // Debugueo con LEDs
      //call TimerApagaLeds.startOneShot(1000);
      if ((rcvPkt -> idS == 1 || rcvPkt -> idS == 2 || rcvPkt -> idS == 3)  &&
                rcvPkt -> idM == node_id_master){
        
        if(rcvPkt -> idS == 1){
          setLeds(1);
        } else if (rcvPkt -> idS == 2){
          setLeds(2);
        } else if (rcvPkt -> idS == 3){
          setLeds(4);
        call TimerApagaLeds.startOneShot(TIMER_ON_LEDS);
        }

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
    return msg;

    } else if(len ==sizeof(test_serial_msg_t)) {
      /*
        AQUI IRÍA EL CODIGO SI QUISIESEMOS PROCESAR
        MENSAJES RECIBIDOS POR EL PUERTO SERIE
      
      */
      return msg;
    } else {
      return msg;
    }
    
    
  }

  
}




