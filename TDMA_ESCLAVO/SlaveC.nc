// $Id: SlaveC.nc,v 1.6 2010/06/29 22:07:40 scipio Exp $


#include <Timer.h>
#include "Slave.h"

module SlaveC {
    uses interface Boot;
    uses interface Leds;
    //uses interface Timer<TMilli> as TimerMaster;
    uses interface Timer<TMilli> as TimerMiSlot;
    uses interface Timer<TMilli> as TimerLeds;
	uses interface Timer<TMilli> as TimerDormir;
    uses interface Packet;
    uses interface AMPacket;
    uses interface AMSend;
    uses interface Receive;
    uses interface SplitControl as AMControl;
}

implementation {

    uint16_t rssi=1;
    uint8_t idMaster = 0;
	uint8_t id_Tx = 0;
    //uint8_t idSlave = 55;
    uint8_t arraySlaves[3];
	uint16_t arrayRSSI[4];
    uint8_t idSlot;
	uint8_t idSlot_1;
    float tiempoEspera = 0;
	float tiempoEsperaDormir = 0;
    bool busy = FALSE;
    message_t pkt;
    RespuestaMsg* respuestaPkt_tx;
	BROADCASTmsg* respuestaNodo_tx;
    uint8_t i = 0;
	bool espera = FALSE;



    void setLeds(uint16_t val) {
        if (val & 0x01)
            {call Leds.led0On();
            call TimerLeds.startOneShot(100);
            call Leds.led2Off();}
        else
            call Leds.led0Off();
        if (val & 0x02)
            {call Leds.led1On();
            call TimerLeds.startOneShot(100);
            call Leds.led2Off();}
        else
            call Leds.led1Off();
        if (val & 0x04)
            {call Leds.led2On();
            call TimerLeds.startOneShot(100);
            call Leds.led2Off();}
        else
            call Leds.led2Off();
    }
	
	void setDelay(float t){
		call TimerDormir.startOneShot(t);
		espera = TRUE;
		while(espera){
		}
	}


    uint16_t getRssi (message_t *msg){
        return (uint16_t) call CC2420Packet.getRssi(msg);
    }

    event void Boot.booted() {
        call AMControl.start();
    }

    event void AMControl.stopDone(error_t err) {}



    /* DEFINICION TEMPORIZADORES */

    //event void TimerMaster.fired() {}

    event void TimerLeds.fired(){
        setLeds(0);
    }

    event void TimerMiSlot.fired() {
        if (call AMSend.send(AM_BROADCAST_ADDR,
            &pkt, sizeof(RespuestaMsg)) == SUCCESS) {
                busy = TRUE;
            }
		if (call AMSend.send(AM_BROADCAST_ADDR,
            &pkt, sizeof(BROADCASTmsg)) == SUCCESS) {
                busy = TRUE;
            }
    }
	
	event void TimerDormir.fired(){
		espera = FALSE;
    }
	
    event void AMControl.startDone(error_t err) {}


    event void AMSend.sendDone(message_t* msg, error_t err) {
        if (&pkt == msg) {
        busy = FALSE;
        }
    }


    event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
           
        if (len == sizeof(TDMAmsg)) {
            TDMAmsg* TDMAmsg_rx = (TDMAmsg*)payload;
            
            //arraySlaves = TDMAmsg_rx->tipoPeticion; 
            // 1º Calculo cuando es mi turno y configuro timers
            for(i = 0; i < 4; i++){
                if(TDMAmsg_rx->idS[i] == TOS_NODE_ID){
                    idSlot = TDMAmsg_rx->idS[i];
                }
            }

            tiempoEspera = (TDMAmsg_rx->periodo)/4 * idSlot;
			idSlot_1 = idSlot + 1;
			//Creo que me estoy liando el periodo que tiene que estar dormido es desde que termina de enviar 
			//entonces es periodo que nos dice la basestation - el tiempo que acaba de transmitir/tiempo que empieza a transmitir o es el periodo de la basestation
			//tiempoEsperaDormir = (TDMAmsg_rx->periodo)-(TDMAmsg_rx->periodo)/4 * idSlot_1;
			
            // 2º Configuro timers
            call TimerMiSlot.startOneShot(tiempoEspera);	        //Configuro cuando debo enviar
			setDelay(TDMAmsg_rx->periodo);							//Configuro cuando debo estar dormido
            //call TimerMaster.startOneShot(TDMAmsg_rx->periodo);   //Configuro cuando debo volver a escuchar al master
            
            
			
	
			
            if (!busy) {
                respuestaPkt_tx =
                    (RespuestaMsg*)(call Packet.getPayload(&pkt, sizeof(RespuestaMsg)));
                if (respuestaPkt_tx == NULL) 
                    return NULL;
                
                respuestaPkt_tx->idS = TOS_NODE_ID;
                respuestaPkt_tx->idM = idMaster;
				//esto creo que hay que cambiarlo por las tomas de medida 
                respuestaPkt_tx->rssi = getRssi(TDMAmsg_rx);
            
            }

            call TimerLeds.startOneShot(500); //Apago los leds

        }
		if(len == sizeof(BROADCASTmsg)){
			BROADCASTmsg* BROADCASTmsg_rx = (BROADCASTmsg*)payload;
			id_Tx = BROADCASTmsg_rx->idTx;
			
			if(id_Tx == 1){
				arrayRSSI[0] = getRssi(BROADCASTmsg_rx);
			}
			if(id_Tx == 2){
				arrayRSSI[1] = getRssi(BROADCASTmsg_rx);
			}
			if(id_Tx == 3){
				arrayRSSI[2] = getRssi(BROADCASTmsg_rx);
			}
			if(id_Tx == 4){
				arrayRSSI[3] = getRssi(BROADCASTmsg_rx);
			}
			if(!busy){
				respuestaNodo_tx =
                    (BROADCASTmsg*)(call Packet.getPayload(&pkt, sizeof(BROADCASTmsg)));
                if (respuestaNodo_tx == NULL) 
                    return NULL;
                
                respuestaNodo_tx->idTx = TOS_NODE_ID;
                respuestaNodo_tx->idRx = id_Tx;
                respuestaNodo_tx->mensaje = 1;
			}
			
		}
        return msg;

    }
    
    
}
