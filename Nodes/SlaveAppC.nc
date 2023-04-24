// $Id: SlaveAppC.nc,v 1.5 2010/06/29 22:07:40 scipio Exp $

#include <Timer.h>
#include "Slave.h"

configuration SlaveAppC {
}
implementation {
    components MainC;
    components LedsC;
    components SlaveC as App;
    components new TimerMilliC() as TimerMaster;
    components new TimerMilliC() as TimerMiSlot;
    components new TimerMilliC() as TimerLeds;
    
    components ActiveMessageC;
    components new AMSenderC(AM_SLAVE);
    components new AMReceiverC(AM_SLAVE);
    components CC2420ActiveMessageC;

    components new SensirionSht11C() as Sht11;
    components new HamamatsuS10871TsrC() as S10871;
    components new HamamatsuS1087ParC() as S1087;
    App.Temperature -> Sht11.Temperature;
    App.Humidity -> Sht11.Humidity;
    App.Light -> S10871;


    App.Boot -> MainC;
    App.Leds -> LedsC;
    App.TimerMaster -> TimerMaster;
    App.TimerMiSlot -> TimerMiSlot;
    App.TimerLeds -> TimerLeds;
    App.Packet -> AMSenderC;
    App.AMPacket -> AMSenderC;
    App.AMControl -> ActiveMessageC;
    App.AMSend -> AMSenderC;
    App.Receive -> AMReceiverC;
    App -> CC2420ActiveMessageC.CC2420Packet;
}