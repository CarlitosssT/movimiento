
#ifndef STREAMING_H
#define STREAMING_H

#include <Arduino.h>
#include "Eris.h"
/*
All the functions related to streaming of data
*/


namespace Streaming{

bool AddFunction(char * arg);
void ClearFunctions();

//Functions should be declared as void and send the desired data via serial

// Sinusoidal wave
void SineWave();

// Ángulos de los IMUs (JointSample_t)
void IMU();

// IMU externo y fase estimada (ExtSample_t)
void EXT();

void Stream();

}


#endif
