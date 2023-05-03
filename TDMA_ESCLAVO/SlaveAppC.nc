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
	components new TimerMilliC() as TimerDormir;
    
    components ActiveMessageC;
    components new AMSenderC(AM_SLAVE);
    components new AMReceiverC(AM_SLAVE);
    components CC2420ActiveMessageC;




    App.Boot -> MainC;
    App.Leds -> LedsC;
    //App.TimerMaster -> TimerMaster;
    App.TimerMiSlot -> TimerMiSlot;
    App.TimerLeds -> TimerLeds;
	App.TimerDormir -> TimerDormir;
    App.Packet -> AMSenderC;
    App.AMPacket -> AMSenderC;
    App.AMControl -> ActiveMessageC;
    App.AMSend -> AMSenderC;
    App.Receive -> AMReceiverC;

}