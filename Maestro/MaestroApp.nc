// $Id: MaestroApp.nc,v 1.7 2010-06-29 22:07:13 scipio Exp $
#include "Master.h"


configuration MaestroApp {
}
implementation {
  components MainC, MaestroC, LedsC; 
  components ActiveMessageC as Radio, SerialActiveMessageC as Serial;
  
  components new TimerMilliC() as TimerTramaTDMA;
  components new TimerMilliC() as TimerLeds;
  components new AMSenderC(AM_BLINKTORADIO);
  components new AMReceiverC(AM_BLINKTORADIO);

  MainC.Boot <- MaestroC;

  //MaestroC.RadioControl -> Radio;
  MaestroC.SerialControl -> Serial;
  
  MaestroC.UartSend -> Serial;
  MaestroC.UartReceive -> Serial.Receive;
  MaestroC.UartPacket -> Serial;
  MaestroC.UartAMPacket -> Serial;
  
  /* 
    Se pasa de escuchar todos los canales a escuchar un único canal
  MaestroC.RadioSend -> Radio;
  MaestroC.RadioReceive -> Radio.Receive;
  MaestroC.RadioSnoop -> Radio.Snoop;
  MaestroC.RadioPacket -> Radio;
  MaestroC.RadioAMPacket -> Radio;
  */
  MaestroC.Packet -> AMSenderC;
  MaestroC.AMPacket -> AMSenderC;
  MaestroC.AMControl -> Radio;
  MaestroC.AMSend -> AMSenderC;
  MaestroC.Receive -> AMReceiverC;

  MaestroC.TimerTramaTDMA -> TimerTramaTDMA;
  MaestroC.TimerLeds -> TimerLeds;

  MaestroC.Leds -> LedsC;
}
