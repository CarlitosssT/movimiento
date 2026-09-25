#include "soft_i2c.h"

void SoftI2C::begin(uint8_t sdaPin, uint8_t sclPin, uint8_t halfPeriodUs){
  sda = sdaPin;
  scl = sclPin;
  hp  = halfPeriodUs;
  pinMode(sda, OUTPUT_OPEN_DRAIN | PULLUP);
  pinMode(scl, OUTPUT_OPEN_DRAIN | PULLUP);
  sdaHigh();
  digitalWrite(scl, HIGH);
  wait();
  // Libera el bus si un esclavo quedó a mitad de byte (p. ej. tras un reset)
  for (int k = 0; k < 9 && digitalRead(sda) == LOW; k++){
    sclLow();  wait();
    sclHigh(); wait();
  }
  stopCond();
}

// Suelta SCL y espera a que suba (clock stretching), con límite de ~1 ms
void SoftI2C::sclHigh(){
  digitalWrite(scl, HIGH);
  for (int k = 0; k < 1000 && digitalRead(scl) == LOW; k++) delayMicroseconds(1);
}

void SoftI2C::startCond(){
  sdaHigh(); sclHigh(); wait();
  sdaLow();  wait();
  sclLow();  wait();
}

void SoftI2C::stopCond(){
  sdaLow();  wait();
  sclHigh(); wait();
  sdaHigh(); wait();
}

bool SoftI2C::writeByte(uint8_t b){
  for (uint8_t m = 0x80; m; m >>= 1){
    if (b & m) sdaHigh(); else sdaLow();
    wait();
    sclHigh(); wait();
    sclLow();
  }
  sdaHigh(); wait();                 // soltar SDA para leer el ACK
  sclHigh(); wait();
  bool ack = digitalRead(sda) == LOW;
  sclLow(); wait();
  return ack;
}

uint8_t SoftI2C::readByte(bool ack){
  uint8_t b = 0;
  sdaHigh();
  for (uint8_t k = 0; k < 8; k++){
    wait();
    sclHigh(); wait();
    b = (b << 1) | (digitalRead(sda) == HIGH);
    sclLow();
  }
  if (ack) sdaLow(); else sdaHigh();
  wait();
  sclHigh(); wait();
  sclLow();
  sdaHigh(); wait();
  return b;
}

bool SoftI2C::probe(uint8_t addr){
  startCond();
  bool ok = writeByte(addr << 1);
  stopCond();
  return ok;
}

bool SoftI2C::writeReg(uint8_t addr, uint8_t reg, uint8_t val){
  startCond();
  bool ok = writeByte(addr << 1) && writeByte(reg) && writeByte(val);
  stopCond();
  return ok;
}

bool SoftI2C::readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n){
  startCond();
  if (!writeByte(addr << 1) || !writeByte(reg)){
    stopCond();
    return false;
  }
  startCond();                       // start repetido
  if (!writeByte((addr << 1) | 1)){
    stopCond();
    return false;
  }
  for (uint8_t k = 0; k < n; k++) buf[k] = readByte(k < n - 1);
  stopCond();
  return true;
}
