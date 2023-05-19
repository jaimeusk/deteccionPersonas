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
    uses interface Timer<TMilli> as TimerComienzaDormir;
    uses interface Packet;
    uses interface AMPacket;
    uses interface AMSend;
    uses interface Receive;
    uses interface SplitControl as AMControl;
    uses interface CC2420Packet;
}

implementation {

    uint16_t rssi=1;
    uint8_t idMaster = 0;
	uint8_t id_Tx = 0;
    //uint8_t idSlave = 55;
    uint8_t arraySlaves[3];
	uint16_t arrayRSSI[4];
    uint8_t idSlot;
    float tiempoEspera = 0;
	float tiempoDormir = 0;
    uint16_t tiempoTrama = 0;
    uint16_t tiempoGuarda = 10;
    bool busy = FALSE;
    message_t pkt;
    RespuestaMsg* respuestaPkt_tx;
    uint8_t i = 0;
	bool espera = FALSE;
    uint8_t id = 2;
    
    

    nx_uint8_t arrayNodos[NUM_MAX_NODOS];




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
            setLeds(2);
		}
        setLeds(0);
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

    event void TimerComienzaDormir.fired(){
        setDelay(tiempoDormir);
    }

    event void TimerMiSlot.fired() {
        if (call AMSend.send(AM_BROADCAST_ADDR,
            &pkt, sizeof(RespuestaMsg)) == SUCCESS) {
                busy = TRUE;
            }

        if(id == 1){
            setLeds(1);
        } else if (id == 2){
            setLeds(2);
        } else if(id == 3){
            setLeds(4);
        }
        //setLeds(2);
        call TimerLeds.startOneShot(200);

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

            setLeds(7);
            call TimerLeds.startOneShot(1000);
            
            /*
            typedef nx_struct TDMAmsg{
                nx_uint8_t idM;
                nx_uint8_t idS[NUM_MAX_NODOS];
                nx_uint16_t periodo;
                nx_uint16_t tiempoTrama;
                nx_uint16_t tiempoNodo;
            }TDMAmsg;
*/
            idMaster = TDMAmsg_rx->idM;
            
            //arraySlaves = TDMAmsg_rx->tipoPeticion; 
            // 1º Calculo cuando es mi turno y configuro timers
            for(i = 0; i < NUM_MAX_NODOS; i++){
                arrayNodos[i] = TDMAmsg_rx->idS[i];
                
                if(TDMAmsg_rx->idS[i] == id){
                    idSlot = i;
                }
            }
            //2º configuro los timers
            //(TDMAmsg_rx->periodo)/NUM_MAX_NODOS = tiempoNodo??
            tiempoEspera = (TDMAmsg_rx->tiempoNodo) * idSlot;
            //tiempoEspera = (TDMAmsg_rx->periodo)/NUM_MAX_NODOS * idSlot; 
            tiempoDormir = (TDMAmsg_rx->periodo - tiempoGuarda)-(TDMAmsg_rx->tiempoTrama + tiempoGuarda);

            tiempoTrama = (TDMAmsg_rx -> tiempoTrama);
			
            //call TimerMaster.startOneShot(TDMAmsg_rx->periodo);   //Configuro cuando debo volver a escuchar al master
	
			


            /*
            typedef nx_struct RespuestaMsg{
                nx_uint8_t idM;
                nx_uint8_t idS;
                nx_uint16_t rssi[NUM_MAX_NODOS];

            }RespuestaMsg;
            */

            if (!busy) {
                setLeds(5);
                respuestaPkt_tx = (RespuestaMsg*)(call Packet.getPayload(&pkt, sizeof(RespuestaMsg)));
                if (respuestaPkt_tx == NULL) 
                    return NULL;
                
                    respuestaPkt_tx->idS = id;
                    respuestaPkt_tx->idM = idMaster;
				//esto creo que hay que cambiarlo por las tomas de medida
                
                /*    for(i=0; i<NUM_MAX_NODOS; i++){
                        if(arrayNodos[i] != TOS_NODE_ID && idSlot < i){
                            if(arrayNodos[i] == id_Tx){
                                arrayRSSI[i] = getRssi(pktRespuesta_rx);
                            }
                        } else {
                             arrayRRSI[i] = 0;
                        }
                    }   
                */
                respuestaPkt_tx->rssi[i] = 1;
            
            } //Corchete if(!busy)
            //Llamamos a los timers para enviar la información a la basesatiton y mandamos a dormir a los nodos.
            call TimerMiSlot.startOneShot(tiempoEspera);
            call TimerLeds.startOneShot(500); //Apago los leds
            
			call TimerComienzaDormir.startOneShot(tiempoTrama + tiempoGuarda);
            //setDelay(tiempoDormir); 
         }
        return msg;

    }
    
    
}
