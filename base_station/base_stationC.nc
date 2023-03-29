#include <Timer.h>
#include "base_station.h"

module base_stationC{
    uses interface Boot;
    uses interface Timer<TMilli> as Timer0;
    uses interface Packet;
    uses interface AMPacket;
    uses interface AMSend;
    uses interface Receive;
    uses interface SplitControl as AMControl;
}

implementation{
    
}