#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <Arduino.h>

// I2C maestro por software (bit-bang) para un tercer bus: la ESP32 solo tiene
// dos controladores I2C por hardware (Wire y Wire1). Pines en open-drain con
// pull-up interno; las placas GY-521 ya traen sus propios pull-ups.
// Solo lo usa el hilo de los IMUs, así que no necesita mutex.
class SoftI2C {
  public:
    void begin(uint8_t sdaPin, uint8_t sclPin, uint8_t halfPeriodUs = 2);  // ~200 kHz
    bool probe(uint8_t addr);
    bool writeReg(uint8_t addr, uint8_t reg, uint8_t val);
    bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n);

  private:
    uint8_t sda = 0, scl = 0, hp = 2;
    void sdaHigh() { digitalWrite(sda, HIGH); }
    void sdaLow()  { digitalWrite(sda, LOW); }
    void sclLow()  { digitalWrite(scl, LOW); }
    void sclHigh();
    void wait()    { delayMicroseconds(hp); }
    void startCond();
    void stopCond();
    bool writeByte(uint8_t b);        // true si hubo ACK
    uint8_t readByte(bool ack);
};

#endif
