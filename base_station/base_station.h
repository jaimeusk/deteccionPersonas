#ifndef BASESTATION_H
#define BASESTATION_H

enum{
    AM_BASESTATION = 9,     // Canal 9
    TIMER_PERIOD_MILI = 250 // 250ms
};

typedef nx_struct routingMsg{
    nx_uint16_t nodeid;
    nx_uint16_t num_saltos;
} routingMsg;
#endif