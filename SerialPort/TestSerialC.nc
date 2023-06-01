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
    interface Timer<TMilli> as MilliTimer;
    interface Packet;
  }
}
implementation {

  message_t packet;
  message_t arrayPaquetes[NUM_MAX_NODOS];

  bool locked = FALSE;

  /*Variables usadas para recorrer bucles for */
  uint8_t i,j;
  

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
  
  event void Boot.booted() {
    call Control.start();
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
  
  event void MilliTimer.fired() {

    if (locked) {
      return;
    }
    else {
      //test_serial_msg_t* rcmA[NUM_MAX_NODOS];
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
          rcm->rssi_prueba[j] = rssi_prueba[i][j];
        }
        setLeds(7);
        if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(test_serial_msg_t)) == SUCCESS) {
          locked = TRUE;
        }

      }
      
    }
  }

  event message_t* Receive.receive(message_t* bufPtr, 
				   void* payload, uint8_t len) {
    if (len != sizeof(test_serial_msg_t)) {return bufPtr;}
    else {
      test_serial_msg_t* rcm = (test_serial_msg_t*)payload;
      /*
      if (rcm->counter & 0x1) {
	      call Leds.led0On();
      }
      else {
	      call Leds.led0Off();
      }
      if (rcm->counter & 0x2) {
	      call Leds.led1On();
      }
      else {
	      call Leds.led1Off();
      }
      if (rcm->counter & 0x4) {
	      call Leds.led2On();
      }
      else {
	      call Leds.led2Off();
      }
      */
      return bufPtr;
    }
  }

  event void AMSend.sendDone(message_t* bufPtr, error_t error) {
    if (&packet == bufPtr) {
      locked = FALSE;
    }
  }

  event void Control.startDone(error_t err) {
    if (err == SUCCESS) {
      call MilliTimer.startPeriodic(1000);
    }
  }
  event void Control.stopDone(error_t err) {}
}




