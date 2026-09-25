#ifndef SERVOS_H
#define SERVOS_H

#include "Eris.h"

namespace Servos{
  void start();
  bool ready();   // false when no PCA9685 answered at boot
  void move(uint8_t channel, float angle);
  void moveAll(float* angles);
  void smoothMove(uint8_t channel, float angle);
  void smoothMoveAll(float* angles);
  void demoStart();
  void demoStop();
}

#endif
