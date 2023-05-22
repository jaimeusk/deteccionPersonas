// $Id: BaseStationC.nc,v 1.7 2010-06-29 22:07:13 scipio Exp $
#include "Master.h"


configuration BaseStationC {
}
implementation {
  components MainC, BaseStationP, LedsC; 
  components ActiveMessageC as Radio, SerialActiveMessageC as Serial;
  
  components new TimerMilliC() as TimerTramaTDMA;
  components new TimerMilliC() as TimerLeds;
  components new AMSenderC(AM_BLINKTORADIO);
  components new AMReceiverC(AM_BLINKTORADIO);

  MainC.Boot <- BaseStationP;

  //BaseStationP.RadioControl -> Radio;
  BaseStationP.SerialControl -> Serial;
  
  BaseStationP.UartSend -> Serial;
  BaseStationP.UartReceive -> Serial.Receive;
  BaseStationP.UartPacket -> Serial;
  BaseStationP.UartAMPacket -> Serial;
  
  /* 
    Se pasa de escuchar todos los canales a escuchar un único canal
  BaseStationP.RadioSend -> Radio;
  BaseStationP.RadioReceive -> Radio.Receive;
  BaseStationP.RadioSnoop -> Radio.Snoop;
  BaseStationP.RadioPacket -> Radio;
  BaseStationP.RadioAMPacket -> Radio;
  */
  BaseStationP.Packet -> AMSenderC;
  BaseStationP.AMPacket -> AMSenderC;
  BaseStationP.AMControl -> Radio;
  BaseStationP.AMSend -> AMSenderC;
  BaseStationP.Receive -> AMReceiverC;

  BaseStationP.TimerTramaTDMA -> TimerTramaTDMA;
  BaseStationP.TimerLeds -> TimerLeds;

  BaseStationP.Leds -> LedsC;
}
