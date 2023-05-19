// $Id: SlaveC.nc,v 1.6 2010/06/29 22:07:40 scipio Exp $


#include <Timer.h>
#include "Slave.h"
#include <string.h>

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
	
    uint16_t arrayRSSI_Actual[NUM_MAX_NODOS];
    uint16_t arrayRSSI_Pasado[NUM_MAX_NODOS];

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
    uint8_t id = 1;
    
    

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
        for(i=0; i<NUM_MAX_NODOS; i++){
            arrayRSSI_Actual[i]=0;
            arrayRSSI_Pasado[i]=0;
        }
        call AMControl.start();
    }

    event void AMControl.stopDone(error_t err) {}



    /* DEFINICION TEMPORIZADORES */

    //event void TimerMaster.fired() {}

    event void TimerLeds.fired(){
        setLeds(0);
    }
   
    /*void igualarArray(uint16_t *array1, uint16_t *array2, int tamanoArray) {
        memcpy(array1, array2, tamanoArray * sizeof(uint16_t));
    }
    */

    /*
    void igualarArray(uint16_t arreglo1[], uint16_t arreglo2[], int tamano) {
        for (i = 0; i < tamano; i++) {
            arreglo1[i] = arreglo2[i];
        }
    }
    */

    event void TimerComienzaDormir.fired(){
        for(i=0;i<NUM_MAX_NODOS;i++){
            arrayRSSI_Pasado[i] = arrayRSSI_Actual [i];
        }
        
        //igualarArray(uint16_t *arrayRSSI_Pasado, uint16_t *arrayRSSI_Actual, NUM_MAX_NODOS);
        
            /*  
                ESTA QUITADO YA QUE NOS METE EN UN BUCLE WHILE INFINITO
                NUNCA LLEGA A EXPIRAR EL TEMPORIZADOR [TimerDormir.startOneShot(t)]
                QUE SE LLAMA EN LA FUNCIÓN setDelay(tiempo)
        */
        //setDelay(tiempoDormir);
    }

    event void TimerMiSlot.fired() {
        if (call AMSend.send(AM_BROADCAST_ADDR,
            &pkt, sizeof(RespuestaMsg)) == SUCCESS) {
                busy = TRUE;
            }

    /* SIRVEN PARA COMPROBAR TDMA
        if(id == 1){
            setLeds(1);
        } else if (id == 2){
            setLeds(2);
        } else if(id == 3){
            setLeds(4);
        }
        //setLeds(2);
        call TimerLeds.startOneShot(200);
    */
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

            

            //setLeds(7); //COMPRUEBA RECEPCION TDMA FROM MASTER
            call TimerLeds.startOneShot(1000);
            

            idMaster = TDMAmsg_rx->idM;
            

            // 1º Calculo cuando es mi turno y configuro timers
            for(i = 0; i < NUM_MAX_NODOS; i++){
                arrayNodos[i] = TDMAmsg_rx->idS[i];
                
                if(TDMAmsg_rx->idS[i] == id){
                    idSlot = i;
                }
            }
           
            //2º configuro los timers
           
            tiempoEspera = (TDMAmsg_rx->tiempoNodo) * idSlot;
            tiempoDormir = (TDMAmsg_rx->periodo - tiempoGuarda)-(TDMAmsg_rx->tiempoTrama + tiempoGuarda);
            tiempoTrama = (TDMAmsg_rx -> tiempoTrama);
			

            // CREAMOS MENSAJE DE RESPUESTA
            if (!busy) {
                //setLeds(5);
                respuestaPkt_tx = (RespuestaMsg*)(call Packet.getPayload(&pkt, sizeof(RespuestaMsg)));
                if (respuestaPkt_tx == NULL) 
                    return NULL;
                
                respuestaPkt_tx->idS = id;
                respuestaPkt_tx->idM = idMaster;
				              
                // ENVIO RSSI DEL PASADO CICLO
                for(i=0; i<NUM_MAX_NODOS; i++){
                    respuestaPkt_tx->rssi[i] = arrayRSSI_Pasado[i];
                    setLeds(7);
                }

            
            } //Corchete if(!busy)
            
            //Llamamos a los timers para enviar la información al master y mandamos a dormir a los nodos.
            call TimerMiSlot.startOneShot(tiempoEspera); // Espera tu slot para realizar el envío de datos.
            call TimerLeds.startOneShot(1000); //Apago los leds
            
			
            // Actualizacion de los array de RSSI al final de la trama TDMA completa
            // (CUANDO FUNCIONE) Dormir y ahorrar energía con setDelay(tiempo)
            call TimerComienzaDormir.startOneShot(tiempoTrama + tiempoGuarda);
            
         } else if (len == sizeof(RespuestaMsg)){

            // Recibimos las respuestas de todos los demás nodos y actualizamos nuestro RSSI con ellos
            RespuestaMsg* RespuestaMsg_rx = (RespuestaMsg*)payload;

            rssi = (RespuestaMsg_rx->idS) -1 ;
            arrayRSSI_Actual[rssi] = getRssi((message_t*)RespuestaMsg_rx);

         }
        return msg;

    }
    
    
}
