/*
  3x MPU6050 -> ángulos de hombro, codo y carpo (ESP32, sin multiplexor)

  Bus 0: SDA 21, SCL 22 -> IMU 1 húmero (AD0 a GND), IMU 2 antebrazo (AD0 a 3.3V)
  Bus 1: SDA 32, SCL 33 -> IMU 3 caña (AD0 a GND)

  Al encender: deja la pata QUIETA en posición neutra ~3 s (esa pose = 0°).
  Envía 'c' para tomar la pose actual como nuevo cero.
*/

#include <Wire.h>

TwoWire* BUS[3]  = {&Wire, &Wire, &Wire1};
uint8_t  ADDR[3] = {0x68, 0x69, 0x68};
int8_t   SIGN[3] = {+1, +1, +1};   // -1 si un IMU quedó con Z al revés

const float ALPHA = 0.98;          // filtro complementario
const float DT    = 0.005;         // 200 Hz

float theta[3];     // ángulo de cada segmento respecto a la vertical
float bias[3];      // sesgo del giroscopio
float cero[3];      // ángulo de cada articulación en la pose neutra

float wrap(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

void writeReg(int i, uint8_t reg, uint8_t val) {
  BUS[i]->beginTransmission(ADDR[i]);
  BUS[i]->write(reg);
  BUS[i]->write(val);
  BUS[i]->endTransmission();
}

// Lee aceleración X, Y y giro Z (°/s)
bool leer(int i, float &ax, float &ay, float &gz) {
  BUS[i]->beginTransmission(ADDR[i]);
  BUS[i]->write(0x3B);
  if (BUS[i]->endTransmission(false) != 0) return false;
  if (BUS[i]->requestFrom(ADDR[i], (uint8_t)14) != 14) return false;
  uint8_t b[14];
  for (int k = 0; k < 14; k++) b[k] = BUS[i]->read();
  ax = (int16_t)(b[0] << 8 | b[1]);
  ay = (int16_t)(b[2] << 8 | b[3]);
  gz = SIGN[i] * (int16_t)(b[12] << 8 | b[13]) / 65.5;
  return true;
}

// Ángulo del segmento según la gravedad
float anguloAcc(int i, float ax, float ay) {
  return -SIGN[i] * atan2(ay, ax) * RAD_TO_DEG;
}

// Filtro complementario: giroscopio + gravedad
void actualizar() {
  for (int i = 0; i < 3; i++) {
    float ax, ay, gz;
    if (!leer(i, ax, ay, gz)) continue;
    theta[i] = wrap(theta[i] + (gz - bias[i]) * DT);
    theta[i] = wrap(theta[i] + (1 - ALPHA) * wrap(anguloAcc(i, ax, ay) - theta[i]));
  }
}

// Articulación = segmento hijo - segmento padre
float articulacion(int j) {
  float padre = (j == 0) ? 0 : theta[j - 1];
  return wrap(theta[j] - padre);
}

void calibrarGiro() {
  float suma[3] = {0};
  for (int n = 0; n < 400; n++) {
    for (int i = 0; i < 3; i++) {
      float ax, ay, gz;
      if (leer(i, ax, ay, gz)) {
        suma[i] += gz;
        theta[i] = anguloAcc(i, ax, ay);
      }
    }
    delay(5);
  }
  for (int i = 0; i < 3; i++) bias[i] = suma[i] / 400;
}

void calibrarCero() {
  float suma[3] = {0};
  for (int n = 0; n < 200; n++) {
    actualizar();
    for (int j = 0; j < 3; j++) suma[j] += articulacion(j);
    delay(5);
  }
  for (int j = 0; j < 3; j++) cero[j] = suma[j] / 200;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(25, 26, 400000);
  Wire1.begin(32, 33, 400000);

  for (int i = 0; i < 3; i++) {
    writeReg(i, 0x6B, 0x01);  // despertar
    writeReg(i, 0x1A, 0x03);  // filtro ~44 Hz
    writeReg(i, 0x1B, 0x08);  // giroscopio ±500 °/s
    writeReg(i, 0x1C, 0x08);  // acelerómetro ±4 g
  }

  Serial.println("Calibrando, no muevas la pata...");
  calibrarGiro();
  calibrarCero();
  Serial.println("Listo.");
}

void loop() {
  static unsigned long t = micros();
  static int n = 0;
  if (micros() - t < 5000) return;
  t += 5000;

  actualizar();

  if (Serial.available() && Serial.read() == 'c') calibrarCero();

  if (++n >= 4) {   // imprime a 50 Hz
    n = 0;
    Serial.printf("hombro:%.2f,codo:%.2f,carpo:%.2f\n",
                  wrap(articulacion(0) - cero[0]),
                  wrap(articulacion(1) - cero[1]),
                  wrap(articulacion(2) - cero[2]));
  }
}
