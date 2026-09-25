#ifndef CUSTOMTYPES_H
#define CUSTOMTYPES_H

#include <eris_customtypes.h>  // eriscommon: floatSample_t, AnalogSample<N>

// Muestra del feature "IMU". Todo float (24 bytes, sin relleno) para que el
// PC la lea con struct.unpack('<6f').
typedef struct {
  float timestamp;   // ms desde t0 (S_TIME)
  float hip;         // ángulos articulares (°) respecto a la pose neutra
  float knee;
  float ankle;
  float trajIndex;   // puntos de trayectoria reproducidos desde T_GO (-1 si no hay)
  float trajQueue;   // puntos en cola esperando reproducirse
} JointSample_t;

#endif
