#include "Eris.h"
#include "servos.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

namespace Servos{

  Adafruit_PWMServoDriver pwm(PCA9685_I2C_ADDR);

  // Derived constants from configuration
  static constexpr float maxStep = SERVO_SMOOTH_SPEED * SERVO_SMOOTH_LOOP_MS / 1000.0f;
  static constexpr float gain    = maxStep / SERVO_SMOOTH_DECEL_DEG;
  static constexpr float epsilon = 0.02f;

  // Demo sweep derived from its configured sub-range
  static constexpr float demoCenter = 0.5f * (DEMO_MIN_ANGLE + DEMO_MAX_ANGLE);
  static constexpr float demoAmp    = 0.5f * (DEMO_MAX_ANGLE - DEMO_MIN_ANGLE);

  // Smooth movement state
  static float currentAngles[NUM_SERVOS];
  static float targetAngles[NUM_SERVOS];
  static eris_thread_ref_t smoothThread = NULL;

  // Demo (sine oscillation) state
  static volatile bool demoEnabled = false;
  static uint32_t      demoT0      = 0;

  // I2C health. `pcaPresent` is latched by start() from the address probe;
  // while it is false nothing is written to the bus at all. `failStreak` /
  // `faultLatched` catch a board that answers at boot and is unplugged later.
  static bool     pcaPresent  = false;
  static uint16_t failStreak  = 0;
  static bool     faultLatched = false;

  static float stepToward(float current, float target){
    float diff = target - current;
    if (diff > -epsilon && diff < epsilon) return target;
    float step = diff * gain;
    if (step > maxStep) step = maxStep;
    if (step < -maxStep) step = -maxStep;
    return current + step;
  }

  static void applyAngle(uint8_t channel, float angle){
    if (!pcaPresent) return;   // no driver on the bus; nothing to write to
    // Float math for full 12-bit PCA9685 resolution (~4096 steps)
    float pulse = SERVO_MIN_PULSE + (angle - SERVO_MIN_ANGLE) * (float)(SERVO_MAX_PULSE - SERVO_MIN_PULSE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
    // setPWM() returns 0 on success, 1 when the I2C transaction was not
    // acknowledged. Report a sustained outage once, not once per failed
    // write -- at 200 Hz x NUM_SERVOS that would flood the serial link.
    if (pwm.setPWM(channel, 0, (uint16_t)(pulse + 0.5f)) != 0){
      if (failStreak < SERVO_I2C_FAIL_LIMIT) failStreak++;
      if (failStreak >= SERVO_I2C_FAIL_LIMIT && !faultLatched){
        faultLatched = true;
        Error::RaiseError(Error::SENSOR, (char *)"PCA9685 stopped acknowledging (check wiring/power)");
      }
    }
    else {
      if (faultLatched) eriscommon::println("PCA9685 I2C recovered");
      failStreak   = 0;
      faultLatched = false;
    }
  }

  // Thread that steps servos toward their targets (sole I2C writer for the loop).
  // When demo is enabled it drives every channel with a phase-offset sine; the
  // demo sets current==target so the smooth-stepping pass below skips them.
  ERIS_THREAD_WA(waSmoothServo_T, ERIS_STACK_MEDIUM);
  ERIS_THREAD_FUNC(SmoothServo_T) {
    while(1){
      if (demoEnabled){
        float t = (float)(micros() - demoT0) / 1.0e6f;   // seconds since DEMO_ON
        for (uint8_t i = 0; i < NUM_SERVOS; i++){
          float phase = (2.0f * M_PI * i) / NUM_SERVOS;   // traveling wave
          float angle = demoCenter + demoAmp *
                        sinf(2.0f * M_PI * DEMO_FREQ_HZ * t + phase);
          if (angle < DEMO_MIN_ANGLE) angle = DEMO_MIN_ANGLE;
          if (angle > DEMO_MAX_ANGLE) angle = DEMO_MAX_ANGLE;
          currentAngles[i] = angle;   // keep state in sync so it won't snap on exit
          targetAngles[i]  = angle;
          applyAngle(i, angle);
        }
      }
      for (uint8_t i = 0; i < NUM_SERVOS; i++){
        float diff = targetAngles[i] - currentAngles[i];
        if (diff < -epsilon || diff > epsilon){
          currentAngles[i] = stepToward(currentAngles[i], targetAngles[i]);
          applyAngle(i, currentAngles[i]);
        }
      }
      eris_sleep_ms(SERVO_SMOOTH_LOOP_MS);
    }
  }

  bool ready(){ return pcaPresent; }

  void demoStart(){ demoT0 = micros(); demoEnabled = true; }
  void demoStop(){  demoEnabled = false; }

  void start(){
    ERIS_I2C_BEGIN();   // per-board; ESP32 needs explicit SDA/SCL pins

    // begin() probes the address (Adafruit_I2CDevice::detected() -- a real
    // ACK test) and returns false when nothing answers. Without this check a
    // missing or mis-addressed board looks exactly like a working one: every
    // later setPWM() just NACKs in silence.
    pcaPresent = pwm.begin();
    if (!pcaPresent){
      sprintf(strbuffer, "PCA9685 not found at I2C 0x%02X", (unsigned)PCA9685_I2C_ADDR);
      Error::RaiseError(Error::SENSOR, strbuffer);
      Serial.println("Servos DISABLED (no PCA9685 on the bus)");
      return;   // no smooth thread, no bus traffic
    }

    // After begin(): it calls Wire.begin() itself, which on some cores resets
    // the bus back to 100 kHz. Fast-mode is what lets a full NUM_SERVOS
    // update fit inside one tick.
    Wire.setClock(SERVO_I2C_CLOCK_HZ);
    pwm.setPWMFreq(50); // 50Hz for standard servos
    // Initialize smooth state to center position
    for (uint8_t i = 0; i < NUM_SERVOS; i++){
      currentAngles[i] = 90.0;
      targetAngles[i] = 90.0;
    }
    // Start smooth movement thread
    smoothThread = eris_thread_create(waSmoothServo_T, ERIS_STACK_MEDIUM, ERIS_NORMAL_PRIORITY+1, SmoothServo_T, NULL);
    Serial.println("Servos ready");
  }

  void move(uint8_t channel, float angle){
    if (channel >= NUM_SERVOS) return;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    // Immediate move — also update smooth state so thread doesn't fight
    currentAngles[channel] = angle;
    targetAngles[channel] = angle;
    applyAngle(channel, angle);
  }

  void moveAll(float* angles){
    for (uint8_t i = 0; i < NUM_SERVOS; i++){
      move(i, angles[i]);
    }
  }

  void smoothMove(uint8_t channel, float angle){
    if (channel >= NUM_SERVOS) return;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    targetAngles[channel] = angle;
  }

  void smoothMoveAll(float* angles){
    for (uint8_t i = 0; i < NUM_SERVOS; i++){
      smoothMove(i, angles[i]);
    }
  }

}
