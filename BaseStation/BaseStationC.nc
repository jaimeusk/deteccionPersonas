// $Id: BaseStationC.nc,v 1.7 2010-06-29 22:07:13 scipio Exp $

#include <Timer.h>
#include "BaseStation.h"

configuration BaseStationC {
}
implementation {
  components MainC, LedsC;
  components BaseStationP as App;
  components ActiveMessageC as Radio;
  components SerialActiveMessageC as Serial;

  components new TimerMilliC() as Timer0;
  components new TimerMilliC() as Timer1;
  components new AMSenderC(AM_BLINKTORADIO);
  components new AMReceiverC(AM_BLINKTORADIO);
  
  // Sacado de BaseStationC.nc
  MainC.Boot <- App; // (?)

  App.RadioControl -> Radio;
  App.SerialControl -> Serial;
  
  App.UartSend -> Serial;
  App.UartReceive -> Serial.Receive;
  App.UartPacket -> Serial;
  App.UartAMPacket -> Serial;
  
  App.RadioSend -> Radio;
  App.RadioReceive -> Radio.Receive;
  App.RadioSnoop -> Radio.Snoop;
  App.RadioPacket -> Radio;
  App.RadioAMPacket -> Radio;
  
  App.Leds -> LedsC;

  // Sacado de MasterAppC.nc
  //App.Boot -> MainC; // (?)
  App.Timer0 -> Timer0;
  App.Timer1 -> Timer1;

}
