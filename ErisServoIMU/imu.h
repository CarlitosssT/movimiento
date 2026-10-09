#ifndef IMU_H
#define IMU_H

#include "Eris.h"

namespace IMU{
  extern ErisBuffer<JointSample_t> buffer;

  // Configura los MPU6050, calibra (pata QUIETA ~3 s) y lanza el hilo de 200 Hz.
  // Llamar después de Servos::start(), que es quien inicia el bus Wire.
  void start();
  void requestZero();              // toma la pose actual como nuevo cero (no bloquea)
  void getAngles(float out[3]);    // últimos ángulos hip, knee, ankle
}

#endif
