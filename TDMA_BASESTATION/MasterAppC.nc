// $Id: MasterAppC.nc,v 1.5 2010/06/29 22:07:40 scipio Exp $


#include <Timer.h>
#include "Master.h"

configuration MasterAppC {
}
implementation {
  components MainC;
  components LedsC;
  components MasterC as App;
  components new TimerMilliC() as TimerTramaTDMA;
  components new TimerMilliC() as TimerLeds;
  components ActiveMessageC;
  components new AMSenderC(AM_BLINKTORADIO);
  components new AMReceiverC(AM_BLINKTORADIO);

  App.Boot -> MainC;
  App.Leds -> LedsC;
  App.TimerTramaTDMA -> TimerTramaTDMA;
  App.TimerLeds -> TimerLeds;
  App.Packet -> AMSenderC;
  App.AMPacket -> AMSenderC;
  App.AMControl -> ActiveMessageC;
  App.AMSend -> AMSenderC;
  App.Receive -> AMReceiverC;
}









