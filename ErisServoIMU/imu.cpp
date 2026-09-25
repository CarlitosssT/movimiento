#include "Eris.h"
#include "imu.h"
#include "trajectory.h"
#include "timing.h"

#include "soft_i2c.h"

#include <Wire.h>

/*
  3x MPU6050, uno por eslabón -> ángulos de hip, knee y ankle
  (misma lógica que imu_angulos_simple.ino)

  IMU 0 hip:   Wire  (SDA 21, SCL 22)  [compartido con la PCA9685]
  IMU 1 knee:  Wire1 (SDA 18, SCL 19)
  IMU 2 ankle: I2C software (SDA 16, SCL 17)
*/

namespace IMU{

  ErisBuffer<JointSample_t> buffer;

  static const char* NOMBRE[3] = {"hip", "knee", "ankle"};
  static TwoWire* BUS[3] = {&Wire, &Wire1, NULL};   // NULL = bus software
  static SoftI2C  softBus;
  static uint8_t  ADDR[3] = {0x68, 0x68, 0x68};     // se detecta en start()
  static const int8_t SIGN[3] = {+1, +1, +1};       // -1 si un IMU quedó con Z al revés

  static float theta[3];     // ángulo de cada segmento respecto a la vertical
  static float bias[3];      // sesgo del giroscopio
  static float cero[3];      // ángulo de cada articulación en la pose neutra
  static float angles[3];    // últimos ángulos articulares ya con el cero aplicado

  // Recalibración del cero pedida por comando: el hilo la hace sin bloquearse
  static volatile bool zeroRequested = false;
  static uint16_t zeroN = 0;
  static float    zeroSum[3];

  static eris_thread_ref_t imuThread = NULL;

  static float wrap(float a) {
    while (a > 180) a -= 360;
    while (a < -180) a += 360;
    return a;
  }

  // ---- Acceso a los tres buses (hardware o software) ----
  static bool probe(int i, uint8_t addr) {
    if (BUS[i] == NULL) return softBus.probe(addr);
    BUS[i]->beginTransmission(addr);
    return BUS[i]->endTransmission() == 0;
  }

  static bool writeReg(int i, uint8_t reg, uint8_t val) {
    if (BUS[i] == NULL) return softBus.writeReg(ADDR[i], reg, val);
    BUS[i]->beginTransmission(ADDR[i]);
    BUS[i]->write(reg);
    BUS[i]->write(val);
    return BUS[i]->endTransmission() == 0;
  }

  static bool readRegs(int i, uint8_t reg, uint8_t* b, uint8_t n) {
    if (BUS[i] == NULL) return softBus.readRegs(ADDR[i], reg, b, n);
    BUS[i]->beginTransmission(ADDR[i]);
    BUS[i]->write(reg);
    if (BUS[i]->endTransmission(false) != 0) return false;
    if (BUS[i]->requestFrom(ADDR[i], n) != n) return false;
    for (int k = 0; k < n; k++) b[k] = BUS[i]->read();
    return true;
  }

  // Lee aceleración X, Y y giro Z (°/s)
  static bool leer(int i, float &ax, float &ay, float &gz) {
    uint8_t b[14];
    if (!readRegs(i, 0x3B, b, 14)) return false;
    ax = (int16_t)(b[0] << 8 | b[1]);
    ay = (int16_t)(b[2] << 8 | b[3]);
    gz = SIGN[i] * (int16_t)(b[12] << 8 | b[13]) / 65.5;
    return true;
  }

  // Ángulo del segmento según la gravedad
  static float anguloAcc(int i, float ax, float ay) {
    return -SIGN[i] * atan2(ay, ax) * RAD_TO_DEG;
  }

  // Filtro complementario: giroscopio + gravedad
  static void actualizar(float dt) {
    for (int i = 0; i < 3; i++) {
      float ax, ay, gz;
      if (!leer(i, ax, ay, gz)) continue;
      theta[i] = wrap(theta[i] + (gz - bias[i]) * dt);
      theta[i] = wrap(theta[i] + (1 - IMU_ALPHA) * wrap(anguloAcc(i, ax, ay) - theta[i]));
    }
  }

  // Articulación = segmento hijo - segmento padre
  static float articulacion(int j) {
    float padre = (j == 0) ? 0 : theta[j - 1];
    return wrap(theta[j] - padre);
  }

  static void calibrarGiro() {
    float suma[3] = {0};
    for (int n = 0; n < IMU_GYRO_CAL_N; n++) {
      for (int i = 0; i < 3; i++) {
        float ax, ay, gz;
        if (leer(i, ax, ay, gz)) {
          suma[i] += gz;
          theta[i] = anguloAcc(i, ax, ay);
        }
      }
      delay(5);
    }
    for (int i = 0; i < 3; i++) bias[i] = suma[i] / IMU_GYRO_CAL_N;
  }

  static void calibrarCero() {
    float suma[3] = {0};
    for (int n = 0; n < IMU_ZERO_CAL_N; n++) {
      actualizar(IMU_PERIOD_US * 1e-6f);
      for (int j = 0; j < 3; j++) suma[j] += articulacion(j);
      delay(5);
    }
    for (int j = 0; j < 3; j++) cero[j] = suma[j] / IMU_ZERO_CAL_N;
  }

  ERIS_THREAD_WA(waIMU_T, ERIS_STACK_LARGE);
  ERIS_THREAD_FUNC(IMU_T) {
    uint32_t tPrev = micros();
    uint32_t tNext = tPrev + IMU_PERIOD_US;
    uint8_t  n = 0;
    while (1) {
      waitUntilUs(tNext);
      uint32_t now = micros();
      tNext += IMU_PERIOD_US;
      if ((int32_t)(now - tNext) > 0) tNext = now + IMU_PERIOD_US;   // si se atrasó, no recuperar a ráfagas
      float dt = (now - tPrev) * 1e-6f;
      tPrev = now;

      actualizar(dt);

      if (zeroRequested) {
        zeroRequested = false;
        zeroN = 0;
        zeroSum[0] = zeroSum[1] = zeroSum[2] = 0;
        Serial.println("Calibrando cero, no muevas la pata...");
      }
      if (zeroN < IMU_ZERO_CAL_N) {
        for (int j = 0; j < 3; j++) zeroSum[j] += articulacion(j);
        if (++zeroN == IMU_ZERO_CAL_N) {
          for (int j = 0; j < 3; j++) cero[j] = zeroSum[j] / IMU_ZERO_CAL_N;
          Serial.println("Cero listo.");
        }
      }

      for (int j = 0; j < 3; j++) angles[j] = wrap(articulacion(j) - cero[j]);

      if (++n >= IMU_STREAM_DIV) {
        n = 0;
        JointSample_t s;
        s.timestamp = ((float)(micros() - t0)) / 1.0e3;
        s.hip       = angles[0];
        s.knee      = angles[1];
        s.ankle     = angles[2];
        s.trajIndex = Trajectory::index();
        s.trajQueue = Trajectory::queued();
        buffer.append(s);
      }
    }
  }

  void requestZero(){ zeroRequested = true; }

  void getAngles(float out[3]){
    for (int j = 0; j < 3; j++) out[j] = angles[j];
  }

  void start(){
    // Wire ya lo inició Servos::start() (PCA9685); faltan Wire1 y el bus software.
    Wire.setClock(IMU_I2C_CLOCK_HZ);
    Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, IMU_I2C_CLOCK_HZ);
    softBus.begin(PIN_SOFT_I2C_SDA, PIN_SOFT_I2C_SCL);

    for (int i = 0; i < 3; i++) {
      // 0x68 (AD0 a GND) o 0x69 (AD0 a 3.3V)
      bool ok = true;
      if (probe(i, 0x68))      ADDR[i] = 0x68;
      else if (probe(i, 0x69)) ADDR[i] = 0x69;
      else ok = false;

      if (ok) {
        ok = writeReg(i, 0x6B, 0x01);   // despertar
        writeReg(i, 0x1A, 0x03);        // filtro ~44 Hz
        writeReg(i, 0x1B, 0x08);        // giroscopio ±500 °/s
        writeReg(i, 0x1C, 0x08);        // acelerómetro ±4 g
      }
      if (ok) {
        sprintf(strbuffer, "IMU %s en 0x%02X", NOMBRE[i], ADDR[i]);
        Serial.println(strbuffer);
      } else {
        sprintf(strbuffer, "IMU %s no responde (ni 0x68 ni 0x69)", NOMBRE[i]);
        Error::RaiseError(Error::SENSOR, strbuffer);
        Serial.println(strbuffer);
      }
    }

    Serial.println("Calibrando, no muevas la pata...");
    calibrarGiro();
    calibrarCero();
    zeroN = IMU_ZERO_CAL_N;   // el cero de arranque ya está hecho
    Serial.println("Listo.");

    buffer.init();
    imuThread = eris_thread_create(waIMU_T, ERIS_STACK_LARGE, ERIS_NORMAL_PRIORITY+4, IMU_T, NULL);
  }

}
