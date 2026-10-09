#include "Eris.h"
#include "external_imu.h"
#include "phase_table.h"
#include "servos.h"
#include "trajectory.h"
#include "timing.h"

#include "soft_i2c.h"

namespace ExtIMU{

  ErisBuffer<ExtSample_t> buffer;

  enum Chip : uint8_t { NONE, BNO055, BMI160, MPU6050 };
  static Chip    chip = NONE;
  static uint8_t addr = 0;
  static const char* NOMBRE[] = {"NONE", "BNO055", "BMI160", "MPU6050"};

  // ---- BNO055 (0x28 / 0x29, CHIP_ID 0xA0): fusión interna, entrega gravedad y giro ya procesados ----
  static const uint8_t BNO_REG_CHIP_ID  = 0x00;
  static const uint8_t BNO_REG_PAGE_ID  = 0x07;
  static const uint8_t BNO_REG_GYR      = 0x14;   // giroscopio XYZ (6 B, int16 little endian)
  static const uint8_t BNO_REG_GRV      = 0x2E;   // vector de gravedad XYZ (6 B)
  static const uint8_t BNO_REG_CALIB    = 0x35;   // sys[7:6] gyr[5:4] acc[3:2] mag[1:0]
  static const uint8_t BNO_REG_UNIT_SEL = 0x3B;
  static const uint8_t BNO_REG_OPR_MODE = 0x3D;
  static const uint8_t BNO_REG_PWR_MODE = 0x3E;
  static const uint8_t BNO_REG_SYS_TRIG = 0x3F;
  static const uint8_t BNO_CHIP_ID      = 0xA0;
  static const uint8_t BNO_MODE_CONFIG  = 0x00;
  static const uint8_t BNO_MODE_IMUPLUS = 0x08;   // acelerómetro + giroscopio, sin magnetómetro
  static const float   BNO_GYR_LSB_DPS  = 16.0f;  // °/s
  static const float   BNO_GRV_LSB      = 100.0f; // m/s²

  // ---- BMI160 (0x68 / 0x69, CHIP_ID 0xD1): datos crudos, el pitch lo calcula este módulo ----
  static const uint8_t BMI_REG_CHIP_ID   = 0x00;
  static const uint8_t BMI_REG_DATA      = 0x0C;  // gyro XYZ (6 B) y luego acc XYZ (6 B), little endian
  static const uint8_t BMI_REG_ACC_CONF  = 0x40, BMI_REG_ACC_RANGE = 0x41;
  static const uint8_t BMI_REG_GYR_CONF  = 0x42, BMI_REG_GYR_RANGE = 0x43;
  static const uint8_t BMI_REG_CMD       = 0x7E;
  static const uint8_t BMI_CHIP_ID       = 0xD1;
  static const float   BMI_ACC_LSB_G     = 4096.0f;   // ±8 g
  static const float   BMI_GYR_LSB_DPS   = 32.8f;     // ±1000 °/s

  // ---- MPU6050 (0x68 / 0x69): misma lectura que el IMU del ankle (imu.cpp), datos crudos big endian ----
  static const uint8_t MPU_REG_WHO      = 0x75;
  static const uint8_t MPU_REG_DATA     = 0x3B;   // acc XYZ (6 B), temperatura (2 B), gyro XYZ (6 B)
  static const float   MPU_ACC_LSB_G    = 8192.0f;   // ±4 g
  static const float   MPU_GYR_LSB_DPS  = 32.8f;     // ±1000 °/s

  // Bus propio por software (PIN_EXT_SDA / PIN_EXT_SCL): la ESP32 solo tiene dos
  // controladores I2C y ya los usan los otros IMUs. Solo lo toca el hilo de este
  // módulo (y start() antes de lanzarlo), así que no necesita mutex. El bit-bang
  // además respeta el clock stretching del BNO055.
  static SoftI2C bus;

  static bool writeReg(uint8_t reg, uint8_t val){
    return bus.writeReg(addr, reg, val);
  }

  static bool readRegs(uint8_t a, uint8_t reg, uint8_t* b, uint8_t n){
    return bus.readRegs(a, reg, b, n);
  }

  // Giro (°/s) y vector de gravedad en los ejes del sensor. Con el BNO055 es la gravedad de
  // la fusión (m/s²); con el BMI160 y el MPU6050 es la aceleración medida (g). Solo importa su dirección.
  static bool leer(float gyr[3], float grv[3]){
    if (chip == BNO055){
      uint8_t g[6], v[6];
      if (!readRegs(addr, BNO_REG_GYR, g, 6) || !readRegs(addr, BNO_REG_GRV, v, 6)) return false;
      for (int k = 0; k < 3; k++){
        gyr[k] = (int16_t)(g[2*k] | g[2*k + 1] << 8) / BNO_GYR_LSB_DPS;
        grv[k] = (int16_t)(v[2*k] | v[2*k + 1] << 8) / BNO_GRV_LSB;
      }
      return true;
    }
    if (chip == MPU6050){
      uint8_t m[14];
      if (!readRegs(addr, MPU_REG_DATA, m, 14)) return false;
      for (int k = 0; k < 3; k++){
        grv[k] = (int16_t)(m[2*k] << 8 | m[2*k + 1]) / MPU_ACC_LSB_G;
        gyr[k] = (int16_t)(m[8 + 2*k] << 8 | m[8 + 2*k + 1]) / MPU_GYR_LSB_DPS;
      }
      return true;
    }
    uint8_t b[12];
    if (!readRegs(addr, BMI_REG_DATA, b, 12)) return false;
    for (int k = 0; k < 3; k++){
      gyr[k] = (int16_t)(b[2*k]     | b[2*k + 1] << 8)     / BMI_GYR_LSB_DPS;
      grv[k] = (int16_t)(b[6 + 2*k] | b[6 + 2*k + 1] << 8) / BMI_ACC_LSB_G;
    }
    return true;
  }

  // ---- Estado del seguimiento ----
  static PhaseMatcher matcher;
  static volatile bool trackEnabled = false;
  static uint8_t channel[3] = {0, 1, 2};
  static float   mapSign[3] = {+1, +1, +1};
  static float   mapOffset[3] = {0, 0, 0};
  static float   imuSign = EXT_PITCH_SIGN;
  static float   imuOffset = 90.0f;   // theta que ve la tabla en la pose calibrada; start() lo pone en la media de la tabla
  static float   zeroPitch = 0;       // pitch (ya con signo) de la pose calibrada, que cuenta como 0°
  static volatile bool zeroRequested = false;

  static ExtSample_t last;           // última muestra publicada
  static volatile uint8_t calStatus = 0;   // BNO055: sys[7:6] gyr[5:4] acc[3:2] mag[1:0], la actualiza el hilo
  static float lastGyr[3], lastGrv[3];   // última lectura cruda del sensor (diagnóstico en PH_ST)
  static volatile uint32_t nReadOk = 0, nReadFail = 0;   // lecturas I2C buenas y fallidas

  // BMI160 y MPU6050: sesgo del giroscopio y pitch del filtro complementario
  static volatile bool gyroCalRequested = false;
  static float bias = 0;
  static float pitchC = 0;

  static eris_thread_ref_t extThread = NULL;

  static float wrap(float a) {
    while (a > 180) a -= 360;
    while (a < -180) a += 360;
    return a;
  }

  static float emaAlpha(float fcHz, float dt){
    return dt / (dt + 1.0f / (2.0f * (float)M_PI * fcHz));
  }

  // BMI160 y MPU6050: promedia el giro en el eje del pitch con el sensor quieto
  static float calibrarGiro(){
    float sum = 0;
    int n = 0;
    for (int k = 0; k < EXT_GYRO_CAL_N; k++){
      float gyr[3], acc[3];
      if (leer(gyr, acc)){ sum += gyr[EXT_AXIS_W]; n++; }
      delay(EXT_PERIOD_US / 1000);
    }
    return n ? sum / n : 0;
  }

  ERIS_THREAD_WA(waExt_T, ERIS_STACK_LARGE);
  ERIS_THREAD_FUNC(Ext_T) {
    uint32_t tPrev = micros();
    uint32_t tNext = tPrev + EXT_PERIOD_US;
    bool primera = true;
    float thetaM = 0, omegaM = 0, alphaM = 0;
    int fails = 0;
    uint8_t nCal = 0;

    while (1){
      waitUntilUs(tNext);
      uint32_t now = micros();
      tNext += EXT_PERIOD_US;
      if ((int32_t)(now - tNext) > 0) tNext = now + EXT_PERIOD_US;   // atrasado: no recuperar a ráfagas
      float dt = (now - tPrev) * 1e-6f;
      tPrev = now;

      if (gyroCalRequested){
        gyroCalRequested = false;
        Serial.println("Calibrando giroscopio externo, no muevas el IMU...");
        bias = calibrarGiro();
        Serial.println("Giroscopio externo listo.");
        tPrev = tNext = micros();
        primera = true;
        continue;
      }

      float gyr[3], grv[3];
      if (!leer(gyr, grv)){
        nReadFail = nReadFail + 1;
        if (++fails == 50){
          Error::RaiseError(Error::SENSOR, (char *)"External IMU stopped answering (check wiring)");
        }
        continue;
      }
      fails = 0;
      nReadOk = nReadOk + 1;
      for (int k = 0; k < 3; k++){ lastGyr[k] = gyr[k]; lastGrv[k] = grv[k]; }

      float pitch, rate;
      float angGrav = -atan2f(grv[EXT_AXIS_V], grv[EXT_AXIS_U]) * RAD_TO_DEG;   // misma convención que imu.cpp
      if (chip == BNO055){
        // La fusión del BNO055 ya da una gravedad limpia y un giroscopio calibrado
        pitch = angGrav;
        rate  = EXT_GYRO_SIGN * gyr[EXT_AXIS_W];
        if (++nCal >= 50){                    // 2 veces por segundo: el bus solo lo usa este hilo
          nCal = 0;
          uint8_t c;
          if (readRegs(addr, BNO_REG_CALIB, &c, 1)) calStatus = c;
        }
      } else {
        // BMI160 y MPU6050: filtro complementario, giroscopio + gravedad (igual que el ankle)
        rate = EXT_GYRO_SIGN * (gyr[EXT_AXIS_W] - bias);
        float mag = sqrtf(grv[0]*grv[0] + grv[1]*grv[1] + grv[2]*grv[2]);
        if (primera){
          pitchC = angGrav;
        } else {
          pitchC = wrap(pitchC + rate * dt);
          if (fabsf(mag - 1.0f) < EXT_ACC_GATE_G){   // sin corrección si el sensor acelera mucho
            pitchC = wrap(pitchC + (1 - EXT_ALPHA) * wrap(angGrav - pitchC));
          }
        }
        pitch = pitchC;
      }

      // Pitch con cero en la pose calibrada (al arrancar, tras PH_GYRO o con PH_ZERO). El salto de
      // atan2 queda en ±180° respecto a esa pose, lejos de donde se mueve el segmento.
      float rel = imuSign * pitch;
      if (primera || zeroRequested){ zeroPitch = rel; zeroRequested = false; }
      float theta = wrap(rel - zeroPitch);

      // Señales del prototipo: theta, omega filtradas y alpha derivada de omega
      float omega = imuSign * rate;
      if (primera){
        thetaM = theta; omegaM = omega; alphaM = 0;
        primera = false;
      } else {
        float omegaPrev = omegaM;
        thetaM += emaAlpha(EXT_EMA_THETA_HZ, dt) * (theta - thetaM);
        omegaM += emaAlpha(EXT_EMA_OMEGA_HZ, dt) * (omega - omegaM);
        alphaM += emaAlpha(EXT_EMA_ALPHA_HZ, dt) * ((omegaM - omegaPrev) / dt - alphaM);
      }

      float x[3] = {thetaM + imuOffset, omegaM, alphaM};   // la tabla espera el pitch absoluto del pie, no el relativo a la pose calibrada
      int row = matcher.step(x);
      const float* j = PHASE_JOINTS[row];

      if (trackEnabled){
        for (int k = 0; k < 3; k++){
          Servos::move(channel[k], mapOffset[k] + mapSign[k] * j[k]);
        }
      }

      ExtSample_t s;
      s.timestamp = ((float)(micros() - t0)) / 1.0e3;
      s.theta = thetaM; s.omega = omegaM; s.alpha = alphaM;
      s.phase = (float)row / PHASE_M;
      s.hip = j[0]; s.knee = j[1]; s.ankle = j[2];
      last = s;
      buffer.append(s);
    }
  }

  bool ready(){ return chip != NONE; }
  bool enabled(){ return trackEnabled; }

  void enable(bool on){
    if (on){
      if (chip == NONE) return;
      Trajectory::stop();
      Servos::demoStop();
    }
    trackEnabled = on;
  }

  void setMapping(const uint8_t ch[3], const int8_t sign[3], const float offsetDeg[3]){
    for (int k = 0; k < 3; k++){
      channel[k] = ch[k] < NUM_SERVOS ? ch[k] : 0;
      mapSign[k] = sign[k] < 0 ? -1.0f : 1.0f;
      mapOffset[k] = offsetDeg[k];
    }
  }

  void setImu(int8_t sign, float offsetDeg){
    float nuevo = sign < 0 ? -1.0f : 1.0f;
    if (nuevo != imuSign) zeroRequested = true;   // con otro signo el cero guardado ya no vale
    imuSign = nuevo;
    imuOffset = offsetDeg;
    matcher.reset();
  }

  void requestZero(){ zeroRequested = true; }

  void setWeights(float wt, float wo, float wa){ matcher.setWeights(wt, wo, wa); }
  void setMode(PhaseMatcher::Mode m){ matcher.setMode(m); }

  void requestGyroCal(){
    if (chip == BMI160 || chip == MPU6050) gyroCalRequested = true;
    else Serial.println("El BNO055 calibra su giroscopio solo (queda quieto unos segundos)");
  }

  void printStatus(){
    uint8_t cal = calStatus;
    Serial.printf("PH_ST present=%d sensor=%s addr=0x%02X cal=%d%d%d%d track=%d theta=%.1f omega=%.1f alpha=%.0f phase=%.3f hip=%.1f knee=%.1f ankle=%.1f grv=%.2f,%.2f,%.2f gyr=%.1f,%.1f,%.1f reads=%lu fails=%lu\n",
                  chip != NONE, NOMBRE[chip], addr, cal >> 6 & 3, cal >> 4 & 3, cal >> 2 & 3, cal & 3,
                  trackEnabled, last.theta, last.omega, last.alpha, last.phase, last.hip, last.knee, last.ankle,
                  lastGrv[0], lastGrv[1], lastGrv[2], lastGyr[0], lastGyr[1], lastGyr[2],
                  (unsigned long)nReadOk, (unsigned long)nReadFail);
  }

  static void iniciarBNO055(){
    writeReg(BNO_REG_OPR_MODE, BNO_MODE_CONFIG);
    delay(25);
    writeReg(BNO_REG_SYS_TRIG, 0x20);            // reset
    delay(700);
    uint8_t id = 0;
    for (int k = 0; k < 20 && !(readRegs(addr, BNO_REG_CHIP_ID, &id, 1) && id == BNO_CHIP_ID); k++) delay(50);
    writeReg(BNO_REG_PAGE_ID, 0);
    writeReg(BNO_REG_PWR_MODE, 0x00);            // normal
    writeReg(BNO_REG_UNIT_SEL, 0x00);            // m/s², °/s, grados
    writeReg(BNO_REG_SYS_TRIG, 0x00);
    delay(10);
    writeReg(BNO_REG_OPR_MODE, BNO_MODE_IMUPLUS);
    delay(20);
  }

  static void iniciarBMI160(){
    writeReg(BMI_REG_CMD, 0xB6);                 // reset
    delay(100);
    writeReg(BMI_REG_CMD, 0x11);                 // acelerómetro en modo normal
    delay(10);
    writeReg(BMI_REG_CMD, 0x15);                 // giroscopio en modo normal
    delay(100);
    writeReg(BMI_REG_ACC_CONF, 0x29);            // 200 Hz, filtro normal
    writeReg(BMI_REG_ACC_RANGE, 0x08);           // ±8 g
    writeReg(BMI_REG_GYR_CONF, 0x29);            // 200 Hz, filtro normal
    writeReg(BMI_REG_GYR_RANGE, 0x01);           // ±1000 °/s
    Serial.println("Calibrando giroscopio externo, no muevas el IMU...");
    bias = calibrarGiro();
    Serial.println("Listo.");
  }

  static void iniciarMPU6050(){
    writeReg(0x6B, 0x01);                        // despertar (reloj del giroscopio X)
    delay(100);
    writeReg(0x1A, 0x03);                        // filtro ~44 Hz
    writeReg(0x1B, 0x10);                        // giroscopio ±1000 °/s
    writeReg(0x1C, 0x08);                        // acelerómetro ±4 g
    delay(50);
    Serial.println("Calibrando giroscopio externo, no muevas el IMU...");
    bias = calibrarGiro();
    Serial.println("Listo.");
  }

  void start(){
    matcher.init(PHASE_FEATS, PHASE_M, PHASE_SCALE, PhaseMatcher::RECOVER);
    float media = 0;                                     // pitch medio del pie en la tabla
    for (int i = 0; i < PHASE_M; i++) media += PHASE_FEATS[i][0];
    imuOffset = media / PHASE_M;
    buffer.init();   // siempre: S_F EXT lee la cola aunque no haya sensor (saldría vacía)
    bus.begin(PIN_EXT_SDA, PIN_EXT_SCL, EXT_I2C_HALF_PERIOD_US);

    // Se busca el chip por su registro de identidad. Tras el encendido el BNO055 tarda ~650 ms en responder.
    for (int intento = 0; intento < 10 && chip == NONE; intento++){
      for (uint8_t a = 0x28; a <= 0x29 && chip == NONE; a++){
        uint8_t id;
        if (readRegs(a, BNO_REG_CHIP_ID, &id, 1) && id == BNO_CHIP_ID){ addr = a; chip = BNO055; }
      }
      for (uint8_t a = 0x68; a <= 0x69 && chip == NONE; a++){
        uint8_t id, who;
        if (!readRegs(a, BMI_REG_CHIP_ID, &id, 1)) continue;          // nadie responde en esa dirección
        if (id == BMI_CHIP_ID){ addr = a; chip = BMI160; }
        else if (readRegs(a, MPU_REG_WHO, &who, 1)){ addr = a; chip = MPU6050; }
      }
      if (chip == NONE) delay(100);
    }
    if (chip == NONE){
      Error::RaiseError(Error::SENSOR, (char *)"External IMU not found (BNO055 0x28/0x29, BMI160 or MPU6050 0x68/0x69)");
      Serial.println("IMU externo DESACTIVADO (no hay BNO055, BMI160 ni MPU6050 en SDA 25 / SCL 26)");
      return;
    }

    if (chip == BNO055) iniciarBNO055();
    else if (chip == BMI160) iniciarBMI160();
    else iniciarMPU6050();

    sprintf(strbuffer, "IMU externo %s en 0x%02X", NOMBRE[chip], addr);
    Serial.println(strbuffer);

    extThread = eris_thread_create(waExt_T, ERIS_STACK_LARGE, ERIS_NORMAL_PRIORITY+4, Ext_T, NULL);
  }

}
