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


// Muestra del feature "EXT": IMU externo (BNO055, BMI160 o MPU6050) y fase estimada. Todo float
// (32 bytes) para que el PC la lea con struct.unpack('<8f').
typedef struct {
  float timestamp;   // ms desde t0 (S_TIME)
  float theta;       // pitch del IMU externo ya con signo/offset y filtrado (°)
  float omega;       // velocidad (°/s)
  float alpha;       // aceleración angular (°/s²)
  float phase;       // fase estimada del ciclo (0..1)
  float hip;         // articulaciones elegidas de la tabla (°)
  float knee;
  float ankle;
} ExtSample_t;

#endif
