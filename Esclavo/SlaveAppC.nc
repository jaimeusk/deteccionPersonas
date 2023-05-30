// $Id: SlaveAppC.nc,v 1.5 2010/06/29 22:07:40 scipio Exp $

#include <Timer.h>
#include "Slave.h"
#include <string.h>


configuration SlaveAppC {
}
implementation {
    components MainC;
    components LedsC;
    components SlaveC as App;
    components new TimerMilliC() as TimerMiSlot;
    components new TimerMilliC() as TimerLeds;
	//components new TimerMilliC() as TimerDormir;
    components CC2420ActiveMessageC;
    
    components ActiveMessageC;
    components new AMSenderC(AM_SLAVE);
    components new AMReceiverC(AM_SLAVE);




    App.Boot -> MainC;
    App.Leds -> LedsC;
    App.TimerMiSlot -> TimerMiSlot;
    App.TimerLeds -> TimerLeds;
	//App.TimerDormir -> TimerDormir;
    App.Packet -> AMSenderC;
    App.AMPacket -> AMSenderC;
    App.AMControl -> ActiveMessageC;
    App.AMSend -> AMSenderC;
    App.Receive -> AMReceiverC;
    App -> CC2420ActiveMessageC.CC2420Packet;

}