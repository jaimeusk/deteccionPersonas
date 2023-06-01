
#ifndef TEST_SERIAL_H
#define TEST_SERIAL_H

#define NUM_MAX_NODOS 4

typedef nx_struct test_serial_msg {
  nx_uint16_t counter;
  nx_uint8_t idNodo;
  nx_int16_t rssi_prueba[NUM_MAX_NODOS];
} test_serial_msg_t;
/*  NO PODEMOS USAR UN ARRAY MULTIDIMENSIONAL rssi[DIM_A][DIM_B]
    Ya que después los mensajes no se envian correctamente, debido
    a que nesC no calcula correctamente el sizeOf(ArrayMultiDim)
    habría que realizar alguna adaptación si finalmente hacemos eso.
*/

/*
typedef nx_struct tabla_rssi {
  nx_uint16_t counter;
  int16_t rssi_prueba[NUM_MAX_NODOS][NUM_MAX_NODOS]
} tabla_rssi_msg;
*/

enum {
  AM_TEST_SERIAL_MSG = 0x89,
};

#endif
