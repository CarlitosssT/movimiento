#ifndef EXTERNAL_IMU_H
#define EXTERNAL_IMU_H

#include "Eris.h"
#include "phase_match.h"

// IMU externo (BNO055, BMI160 o MPU6050, se detecta solo) y seguimiento de fase de la marcha.
// Un hilo a 100 Hz lee el sensor, calcula pitch / velocidad / aceleración,
// busca la fase en la tabla (phase_table.h) y, si está activado, manda el
// hip / knee / ankle de esa fase a los servos.
namespace ExtIMU{
  extern ErisBuffer<ExtSample_t> buffer;

  // Detecta el sensor (BNO055 en 0x28/0x29, BMI160 o MPU6050 en 0x68/0x69), lo configura y lanza el hilo.
  // Con el BMI160 y el MPU6050 calibra el giroscopio (quieto ~2 s).
  // Llamar después de Servos::start(), que es quien inicia el bus Wire.
  void start();
  bool ready();

  void enable(bool on);     // true: los servos siguen la fase (para la trayectoria y el demo)
  bool enabled();

  // servo = offset + signo * articulación, por cada una de hip / knee / ankle
  void setMapping(const uint8_t channel[3], const int8_t sign[3], const float offsetDeg[3]);
  // El pitch que se ve vale 0° en la pose calibrada (arranque, PH_GYRO o requestZero). La tabla recibe
  // ese pitch + offset (por defecto la media de la tabla): offset es el pitch del pie en la pose calibrada.
  void setImu(int8_t sign, float offsetDeg);
  void requestZero();       // toma la pose actual como 0° del pitch
  void setWeights(float wTheta, float wOmega, float wAlpha);
  void setMode(PhaseMatcher::Mode mode);
  void requestGyroCal();    // BMI160 / MPU6050: recalibra el sesgo del giroscopio (sensor quieto)
  void printStatus();
}

#endif
