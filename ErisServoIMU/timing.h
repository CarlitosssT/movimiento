#ifndef TIMING_H
#define TIMING_H

#include "Eris.h"

// Duerme el hilo hasta el instante tDue (micros()). El tick del RTOS es de
// 1 ms, así que duerme en pasos de 1 ms: el error queda por debajo de un tick
// y no se acumula porque el siguiente instante se calcula desde tDue.
static inline void waitUntilUs(uint32_t tDue){
  while ((int32_t)(tDue - micros()) > 0){
    eris_sleep_ms(1);
  }
}

#endif
