// $Id: SlaveC.nc,v 1.6 2010/06/29 22:07:40 scipio Exp $


#include <Timer.h>
#include "Slave.h"
#include <string.h>

module SlaveC {
    uses interface Boot;
    uses interface Leds;
    uses interface Timer<TMilli> as TimerMiSlot;
    uses interface Timer<TMilli> as TimerLeds;
	//uses interface Timer<TMilli> as TimerDormir;
    uses interface Packet;
    uses interface AMPacket;
    uses interface AMSend;
    uses interface Receive;
    uses interface SplitControl as AMControl;
    uses interface CC2420Packet;
}

implementation {
    
    uint8_t id = 1; // Variable que guarda el id del esclavo
    uint8_t idMaster = 0; // variable que guarda el id del maestro 
    uint8_t idSlot; //Variable que indica el slot donde te toca transmitir	
    uint8_t i = 0; //Variable que se usa para recorrer los bucles for
    
    uint16_t id_rssi=1; //variable que nos indica en donde se tiene que guardar en la tabla rssi la medida obtenida
    uint16_t arrayRSSI_Actual[NUM_MAX_NODOS]; //Tabla donde se guarda el rssi actual es NUM_MAX_NODO = 3 + 1 por el maestro
    uint16_t arrayRSSI_Pasado[NUM_MAX_NODOS]; //Tabla donde se guarda el rssi pasado que se envia al maestro es NUM_MAX_NODO = 3 + 1 por el maestro
    uint8_t arrayNodos[NUM_MAX_NODOS]; //Tabla que almacena donde va cada nodo en el TDMA

    float tiempoEspera = 0; //Variable que almacena el tiempo de espera para Tx
    uint16_t tiempoTrama = 0; //Variable que almacena el tiempo que dura una trama
    uint16_t tiempoGuarda = 10; //Tiempo de guardia para dar un margen 

    bool busy = FALSE;

    //Variables de paquetes 
    message_t pkt;
    RespuestaMsg* respuestaPkt_tx;
    



    //funcion para el manejo de los leds
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

	//funcion para obtener el rssi
    uint16_t getRssi (message_t *msg){
        return (uint16_t) call CC2420Packet.getRssi(msg);
    }

    //funcion de inicio 
    event void Boot.booted() {
        id = TOS_NODE_ID; // PARA QUE FUNCIONE HAY QUE COMPILARLO CON "make telosb install,id" (sin espacios entre la coma)
        //bucle para inicializar los arrays a 0
        for(i=0; i<NUM_MAX_NODOS; i++){
            arrayRSSI_Actual[i]=0;
            arrayRSSI_Pasado[i]=0;
        }
        call AMControl.start();
    }

    event void AMControl.stopDone(error_t err) {}

    //Fin del timer de los leds
    event void TimerLeds.fired(){
        setLeds(0);
    }
    //Fin del timer para enviar en el slot correspondiente
    event void TimerMiSlot.fired() {
        if (call AMSend.send(AM_BROADCAST_ADDR, &pkt, sizeof(RespuestaMsg)) == SUCCESS) {
                busy = TRUE;
            }
    }
	
    event void AMControl.startDone(error_t err) {}

    event void AMSend.sendDone(message_t* msg, error_t err) {
        if (&pkt == msg) {
            busy = FALSE;
        }   
    }

    //Evento de cuando recibimos un mensaje 
    event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
        
        if (len == sizeof(TDMAmsg)) { //comprobamos si la longitud del mensaje es TDMA_msg
            TDMAmsg* TDMAmsg_rx = (TDMAmsg*)payload;
            if(TDMAmsg_rx->tipoMsg==msg_TDMA){//Comprobamos si, efectivamente, es un mensaje TDMA

                for(i=0;i<=NUM_MAX_NODOS;i++){      //Rellenamos la tabla pasado con los valores de rssi de la anterior interacion 
                    arrayRSSI_Pasado[i] = arrayRSSI_Actual [i];
                }

                arrayRSSI_Actual[0] = getRssi(msg); //obtenemos el valor del rssi del mensaje TDMA y lo guardamos en la posicion 0

                setLeds(2); //LED 2: COMPRUEBA RECEPCION TDMA FROM MASTER
                call TimerLeds.startOneShot(TIMER_ON_LEDS);
                

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
                tiempoTrama = (TDMAmsg_rx -> tiempoTrama);
                

                // CREAMOS MENSAJE DE RESPUESTA
                if (!busy) {
                    respuestaPkt_tx = (RespuestaMsg*)(call Packet.getPayload(&pkt, sizeof(RespuestaMsg)));
                    if (respuestaPkt_tx == NULL) 
                        return NULL;
                    
                    respuestaPkt_tx->idS = id;
                    respuestaPkt_tx->idM = idMaster;
                    respuestaPkt_tx->tipoMsg=msg_RESP;
                                
                    // ENVIO RSSI DEL PASADO CICLO
                    for(i=0; i<=NUM_MAX_NODOS; i++){
                        respuestaPkt_tx->rssi[i] = arrayRSSI_Pasado[i];
                        
                    }
                }
                
                //Llamamos a los timers para enviar la información al master y mandamos a dormir a los nodos.
                call TimerMiSlot.startOneShot(tiempoEspera); // Espera tu slot para realizar el envío de datos.
                setLeds(4); //LED 3: he mandado el RSSI del pasado ciclo
                call TimerLeds.startOneShot(TIMER_ON_LEDS);

            }
         } 
         if (len == sizeof(RespuestaMsg)){

            // Recibimos las respuestas de todos los demás nodos y actualizamos nuestro RSSI con ellos
            RespuestaMsg* RespuestaMsg_rx = (RespuestaMsg*)payload;
            if(RespuestaMsg_rx->tipoMsg==msg_RESP){

                id_rssi = (RespuestaMsg_rx->idS);
                arrayRSSI_Actual[id_rssi] = getRssi(msg);
            }
         }
        return msg;

    }
    
    
}
