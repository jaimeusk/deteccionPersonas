// $Id: SlaveC.nc,v 1.6 2010/06/29 22:07:40 scipio Exp $


#include <Timer.h>
#include "Slave.h"

module SlaveC {
    uses interface Boot;
    uses interface Leds;
    uses interface Timer<TMilli> as TimerMaster;
    uses interface Timer<TMilli> as TimerMiSlot;
    uses interface Timer<TMilli> as TimerLeds;
    uses interface Packet;
    uses interface AMPacket;
    uses interface AMSend;
    uses interface Receive;
    uses interface SplitControl as AMControl;
    uses interface CC2420Packet;
    uses interface Read<uint16_t> as Temperature;
    uses interface Read<uint16_t> as Humidity;
    uses interface Read<uint16_t> as Light;
}

implementation {

    uint16_t rssi=1;
    uint8_t idMaster = 0;
    //uint8_t idSlave = 55;
    uint8_t arraySlaves[3];
    uint8_t idSlot;
    uint16_t tipoPeticion;
    uint16_t valorSensor;
    float temperatura;
    float humedad;
    float luminosidad;
    float tiempoEspera = 0;
    bool busy = FALSE;
    message_t pkt;
    RespuestaMsg* respuestaPkt_tx;
    uint8_t i = 0;



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


    uint16_t getRssi (message_t *msg){
        return (uint16_t) call CC2420Packet.getRssi(msg);
    }

    event void Boot.booted() {
        call AMControl.start();
    }

    event void AMControl.stopDone(error_t err) {}



    /* DEFINICION TEMPORIZADORES */

    event void TimerMaster.fired() {}

    event void TimerLeds.fired(){
        setLeds(0);
    }

    event void TimerMiSlot.fired() {
        if (call AMSend.send(AM_BROADCAST_ADDR,
            &pkt, sizeof(RespuestaMsg)) == SUCCESS) {
                busy = TRUE;
            }
    }

    event void AMControl.startDone(error_t err) {}


    event void AMSend.sendDone(message_t* msg, error_t err) {
        if (&pkt == msg) {
        busy = FALSE;
        }
    }


    /* LECTURA SENSORES */
    event void Temperature.readDone(error_t result, uint16_t val){
        temperatura = (-39.6 + 0.01 * val);
    }

    event void Humidity.readDone(error_t result, uint16_t val){
        humedad = (-2.0468 + 0.0367 * val - 1.5955e-06 * val * val);
    }

    event void Light.readDone(error_t result, uint16_t val){
        luminosidad = (2.5 * val * 6250)/4096;
    }




    event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len){
           
        if (len == sizeof(TDMAmsg)) {
            TDMAmsg* TDMAmsg_rx = (TDMAmsg*)payload;
            
            //arraySlaves = TDMAmsg_rx->tipoPeticion; 
            // 1º Calculo cuando es mi turno y configuro timers
            for(i = 0; i < 3; i++){
                if(TDMAmsg_rx->idS[i] == TOS_NODE_ID){
                    idSlot = TDMAmsg_rx->idS[i];
                    tipoPeticion = TDMAmsg_rx->tipoPeticion[i];
                }
            }

            tiempoEspera = (TDMAmsg_rx->periodo)/3 * idSlot;

            // 2º Configuro timers
            call TimerMiSlot.startOneShot(tiempoEspera);            //Configuro cuando debo enviar
            call TimerMaster.startOneShot(TDMAmsg_rx->periodo);     //Configuro cuando debo volver a escuchar al master
            
            
            // 3º ¿Que medida debo contestar?
            setLeds(7); //Se enciende todos los LED(Rx)
            idMaster = TDMAmsg_rx->idM;
            
            
            if(tipoPeticion == TEMPERATURA){

                call Temperature.read(); // Medimos la temperatura
                valorSensor = (uint16_t) temperatura;
                setLeds(1);

            } else if (tipoPeticion == HUMEDAD){

                call Humidity.read();    // Medimos la humedad
                valorSensor = (uint16_t) humedad;
                setLeds(2);

            } else if (tipoPeticion == ILUMINANCIA){

                call Light.read();       // Medimos la luminosidad
                valorSensor = (uint16_t) luminosidad;
                setLeds(4);

            } else {
                    setLeds(0); // ERROR = LEDS APAGADOS
            }

            
            if (!busy) {
                respuestaPkt_tx =
                    (RespuestaMsg*)(call Packet.getPayload(&pkt, sizeof(RespuestaMsg)));
                if (respuestaPkt_tx == NULL) 
                    return NULL;
                
                respuestaPkt_tx->idS = TOS_NODE_ID;
                respuestaPkt_tx->idM = idMaster;
                respuestaPkt_tx->rssi = getRssi(TDMAmsg_rx);
                respuestaPkt_tx->tipo = tipoPeticion;
                respuestaPkt_tx->medida = valorSensor;
            
            }

            call TimerLeds.startOneShot(500); //Apago los leds

        }
        return msg;

    }
    
    
}
