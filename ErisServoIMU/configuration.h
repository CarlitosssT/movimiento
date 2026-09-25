#ifndef CONFIGURATION_H
#define CONFIGURATION_H


#include <Arduino.h>
#include <eris_board.h>   // board detection + PIN_LED / ERIS_ADC / STRINGIFY

// RTOS selection (nRF52 -> FreeRTOS) is handled automatically by eris_rtos.h.

#define FIRMWARE_VERSION "v3.0-imu"

/* Global configuration such as pins, rates etc */

/***********************************************/

////////////////////////////////////////////////
/// BOARD SELECTION
///
/// Esta variante está pensada para la ESP32 (3 IMUs en tres buses I2C + PCA9685).
///
#if !defined(ERIS_BOARD_ESP32) && !defined(ERIS_BOARD_TEENSY) && !defined(ERIS_BOARD_OTHER)
  #if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    #define ERIS_BOARD_ESP32 1
  #elif defined(TEENSYDUINO)
    #define ERIS_BOARD_TEENSY 1
  #else
    #define ERIS_BOARD_OTHER 1
  #endif
#endif

#if defined(ERIS_BOARD_ESP32)
  //// ---- ESP32 (DevKitC / WROOM-32 and clones) ----
  #ifndef PIN_LED
  #define PIN_LED 2            // on-board LED. The generic esp32 variant does
  #endif                       // not define LED_BUILTIN at all.
  // Bus 0 (Wire): PCA9685 (0x40) e IMU del hip
  #ifndef PIN_I2C_SDA
  #define PIN_I2C_SDA 21
  #endif
  #ifndef PIN_I2C_SCL
  #define PIN_I2C_SCL 22
  #endif
  #define ERIS_I2C_BEGIN()  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL)
  #ifndef ERIS_SERIAL_BAUD
  #define ERIS_SERIAL_BAUD 921600   // UART-bridge ceiling; 12 Mbps is Teensy-only
  #endif
  // Buffer de recepción serial. El de fábrica (256 B) se desborda cuando el
  // PC adelanta puntos de trayectoria.
  #define ERIS_SERIAL_RX_BUFFER 4096

#elif defined(ERIS_BOARD_TEENSY)
  //// ---- Teensy 3.x / 4.x ----
  #ifndef PIN_LED
  #define PIN_LED 13           // classic Arduino LED
  #endif
  #define ERIS_I2C_BEGIN()  Wire.begin()
  #ifndef ERIS_SERIAL_BAUD
  #define ERIS_SERIAL_BAUD 115200   // USB CDC ignores the value
  #endif

#else
  //// ---- Other boards (SAMD21, nRF52, ...) ----
  #ifndef PIN_LED
  #define PIN_LED LED_BUILTIN
  #endif
  #define ERIS_I2C_BEGIN()  Wire.begin()
  #ifndef ERIS_SERIAL_BAUD
  #define ERIS_SERIAL_BAUD 115200
  #endif
#endif

/////////// MEMORY CONFIGURATION
#define TXBUFFERSIZE 16 // Default size for transmision buffer for streaming
#define STRBUFFERSIZE 64

////////////////////////////////////////////////
/// SERVO CONFIGURATION (PCA9685 via I2C)
///
#define NUM_SERVOS 16
#define SERVO_MIN_PULSE 150   // ~0 degrees (Adafruit PWM ticks out of 4096)
#define SERVO_MAX_PULSE 600   // ~180 degrees
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define PCA9685_I2C_ADDR 0x40 // Default PCA9685 address (A0..A5 solder jumpers add to it)

// Smooth movement (rate-limited stepping with ease-out)
#define SERVO_SMOOTH_SPEED     180.0  // Cruise speed (deg/sec)
#define SERVO_SMOOTH_DECEL_DEG 6.0   // Degrees from target where deceleration begins
#define SERVO_SMOOTH_LOOP_MS   5     // Tick interval (200 Hz)

// I2C bus speed for the PCA9685 (fast-mode; needed so a full NUM_SERVOS
// update fits inside one smooth/demo tick).
#define SERVO_I2C_CLOCK_HZ     400000

// Consecutive failed PCA9685 writes before the driver reports the bus as dead.
#define SERVO_I2C_FAIL_LIMIT   32

// DEMO mode: sine-oscillate all servos (0..NUM_SERVOS-1) with a phase offset
// per channel so they chase each other in a traveling wave.
#define DEMO_MIN_ANGLE   70.0f     // demo lower bound (deg)
#define DEMO_MAX_ANGLE   110.0f    // demo upper bound (deg)
#define DEMO_FREQ_HZ     0.5f      // one full cycle / 2 s

////////////////////////////////////////////////
/// IMU CONFIGURATION (3x MPU6050)
///
/// Un IMU por eslabón, cada uno en su propio bus. La ESP32 solo tiene dos
/// controladores I2C, así que el del ankle va por I2C software (soft_i2c.cpp).
///   hip:   Wire  (SDA 21, SCL 22), compartido con la PCA9685
///   knee:  Wire1 (SDA 18, SCL 19)
///   ankle: soft  (SDA 16, SCL 17)
/// La dirección (0x68 con AD0 a GND, 0x69 con AD0 a 3.3V) se detecta sola.
///
#define PIN_I2C1_SDA      18
#define PIN_I2C1_SCL      19
#define PIN_SOFT_I2C_SDA  16
#define PIN_SOFT_I2C_SCL  17
#define IMU_I2C_CLOCK_HZ  400000
#define IMU_PERIOD_US     5000     // 200 Hz de filtrado
#define IMU_STREAM_DIV    2        // 1 de cada N muestras va al stream (100 Hz)
#define IMU_ALPHA         0.98f    // filtro complementario
#define IMU_GYRO_CAL_N    400      // muestras para el sesgo del giroscopio
#define IMU_ZERO_CAL_N    200      // muestras para la pose neutra (1 s)

////////////////////////////////////////////////
/// TRAJECTORY PLAYBACK
///
/// El PC envía puntos (P ...) a una cola; un hilo los reproduce a periodo fijo.
/// Cada punto debe caber en el buffer de 64 caracteres de SerialCommand:
/// "P" + 12 x " 1800" = 61 caracteres, por eso el máximo es 12 canales.
#define TRAJ_MAX_CH        12
#define TRAJ_BUFFER_POINTS 512     // 2.5 s de colchón a 200 Hz
#define TRAJ_DEFAULT_DT_US 5000    // 200 Hz, igual que los CSV


// DEBUGGING FLAGS
#define DEBUG true

#define FIRMWARE_INFO TOSTRING(FIRMWARE_VERSION) "ErisServoIMU"

#endif
