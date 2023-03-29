#include <Timer.h>
#include "base_station.h"

configuration base_stationC {
}

implementation{
    components MainC;
    components LedsC;
    components base_stationC as App;
    components new TimerMilliC() as Timer0;
    components ActiveMessageC;
    components new AMSenderC(AM_BASESTATION);
    components new AMReceiverC(AM_BASESTATION);

    App.Boot -> MainC;
    App.Leds -> LedsC;
    App.Timer0 -> Timer0;
    App.Packet -> AMSenderC;
    App.AMControl -> ActiveMessageC;
    App.AMSend -> AMSenderC;
    App.Receive -> AMReceiverC;
}