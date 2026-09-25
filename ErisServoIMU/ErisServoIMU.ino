// ErisServoIMU: flavor ErisServoDriver (PCA9685) + 3 MPU6050 en una ESP32
//
//  - Servos: comandos X / Y / DEMO_ON del flavor original, más reproducción de
//    trayectorias a periodo fijo (T_CH, T_DT, P, T_GO, T_END, T_STOP, T_ST).
//  - IMUs: ángulos de hip, knee y ankle con filtro complementario a 200 Hz,
//    enviados como feature "IMU" del streaming de Eris (S_F IMU, S_ON).
//
// Al encender: deja la pata QUIETA en posición neutra ~3 s (esa pose = 0°).
// Envía CAL para tomar la pose actual como nuevo cero.
//
// PCA9685: SDA 21, SCL 22 (Wire), servos hip / knee / ankle en los canales 0, 1, 2
// IMU hip:   SDA 21, SCL 22 (Wire, compartido con la PCA9685)
// IMU knee:  SDA 18, SCL 19 (Wire1)
// IMU ankle: SDA 16, SCL 17 (I2C por software: la ESP32 solo tiene dos controladores)

#include "configuration.h"
#include "Eris.h"
#include "servos.h"
#include "imu.h"
#include "trajectory.h"
#include <modules/heartbeat.h>
#include <modules/sinewave.h>
#include "servo_commands.h"


const char firmwareInfo[]=FIRMWARE_INFO;



void start(){
  /*************** Start Threads ************************/
  Heartbeat::start();
  Error::start(); // Start error notification task (Do not disable)

  // start special tasks from external sources
  SineWave::start();
  Servos::start();      // inicia Wire y la PCA9685
  IMU::start();         // inicia Wire1 y el bus software, calibra (~3 s) y lanza el hilo de 200 Hz
  Trajectory::start();
  // Command interfaces
  SerialCom::start();

}

void setup(){
#ifdef ERIS_SERIAL_RX_BUFFER
  Serial.setRxBufferSize(ERIS_SERIAL_RX_BUFFER);   // antes de Serial.begin()
#endif
  Serial.begin(ERIS_SERIAL_BAUD);
  // Wait for the USB host, but bounded: on a board whose Serial is a plain UART
  // (ESP32 via CP2102/CH340) `!Serial` never clears, so an unbounded wait would
  // hang a board that has no host attached.
  unsigned long t0_wait = millis();
  while (!Serial && (millis() - t0_wait) < 2000) {}
  delay(1000);
  // Setup the initial configuration
  Serial.println("HELLO, This is Eris");
  /*************** Configure HW pins *******************/
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,LOW);
  /******************************************************/
  //Start threads
  ERIS_RUN(start);
}




void loop(){
    eris_sleep_ms(10000);
}
