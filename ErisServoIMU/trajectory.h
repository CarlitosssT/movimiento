#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "Eris.h"

// Reproducción de trayectorias a periodo fijo.
// El PC envía puntos a una cola circular (P ...) y un hilo los aplica a los
// servos cada dtUs. Si la cola se vacía a mitad de camino los servos se quedan
// en el último punto (underrun) y la reproducción sigue cuando llegan más.
namespace Trajectory{
  void start();

  bool setChannels(const uint8_t* channels, uint8_t n);   // también vacía la cola
  uint8_t numChannels();
  void setPeriodUs(uint32_t dtUs);

  // Valores en décimas de grado de servo (0..1800). false si la cola está llena
  // o el número de valores no coincide con los canales configurados.
  bool push(const int16_t* tenths, uint8_t n);

  void go();     // empezar a reproducir
  void end();    // no llegarán más puntos: parar cuando la cola se vacíe
  void stop();   // parar ya y vaciar la cola

  float index();      // puntos reproducidos desde go() (-1 si está parada)
  uint16_t queued();  // puntos en cola
  void printStatus();
}

#endif
