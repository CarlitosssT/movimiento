#include "Eris.h"
#include "trajectory.h"
#include "servos.h"
#include "timing.h"

namespace Trajectory{

  enum State : uint8_t { IDLE, RUNNING, ENDING };

  // Cola circular: un único productor (hilo de comandos serie) y un único
  // consumidor (hilo de reproducción). head/tail solo los escribe su dueño.
  static int16_t  points[TRAJ_BUFFER_POINTS][TRAJ_MAX_CH];
  static volatile uint16_t head = 0;   // siguiente hueco libre (productor)
  static volatile uint16_t tail = 0;   // siguiente punto a reproducir (consumidor)

  static uint8_t  channels[TRAJ_MAX_CH];
  static volatile uint8_t nChannels = 0;
  static volatile uint32_t dtUs = TRAJ_DEFAULT_DT_US;

  static volatile State state = IDLE;
  static volatile uint32_t played = 0;
  static volatile uint32_t underruns = 0;
  static volatile uint32_t rejected = 0;

  static eris_thread_ref_t trajThread = NULL;

  static uint16_t count(){
    return (uint16_t)((head + TRAJ_BUFFER_POINTS - tail) % TRAJ_BUFFER_POINTS);
  }

  static void clear(){
    ERIS_CRITICAL_ENTER();
    tail = head;
    ERIS_CRITICAL_EXIT();
  }

  static void apply(const int16_t* p){
    for (uint8_t k = 0; k < nChannels; k++){
      Servos::move(channels[k], p[k] / 10.0f);
    }
  }

  ERIS_THREAD_WA(waTraj_T, ERIS_STACK_MEDIUM);
  ERIS_THREAD_FUNC(Traj_T) {
    uint32_t tDue = micros();
    State last = IDLE;
    while (1){
      State s = state;
      if (s == IDLE){
        last = IDLE;
        eris_sleep_ms(2);
        continue;
      }
      if (last == IDLE) tDue = micros();   // arranque: primer punto inmediato
      last = s;

      waitUntilUs(tDue);
      if (state == IDLE) continue;   // T_STOP mientras esperaba

      if (count() == 0){
        if (s == ENDING){
          state = IDLE;
          Serial.println("Trayectoria terminada");
          continue;
        }
        // Underrun: mantener la última posición y reintentar en el siguiente periodo
        underruns = underruns + 1;
        tDue = micros() + dtUs;
        continue;
      }

      // Si el hilo se atrasó más de un periodo, saltar puntos para no ir en cámara lenta
      // (sección crítica para que un T_STOP no vacíe la cola a mitad de lectura)
      int16_t p[TRAJ_MAX_CH];
      ERIS_CRITICAL_ENTER();
      do {
        memcpy(p, points[tail], sizeof(p));
        tail = (tail + 1) % TRAJ_BUFFER_POINTS;
        played = played + 1;
        tDue += dtUs;
      } while ((int32_t)(micros() - tDue) >= 0 && count() > 0);
      ERIS_CRITICAL_EXIT();

      apply(p);
    }
  }

  void start(){
    for (uint8_t k = 0; k < TRAJ_MAX_CH; k++) channels[k] = k;
    nChannels = TRAJ_MAX_CH;
    trajThread = eris_thread_create(waTraj_T, ERIS_STACK_MEDIUM, ERIS_NORMAL_PRIORITY+4, Traj_T, NULL);
  }

  bool setChannels(const uint8_t* ch, uint8_t n){
    if (n == 0 || n > TRAJ_MAX_CH) return false;
    for (uint8_t k = 0; k < n; k++){
      if (ch[k] >= NUM_SERVOS) return false;
    }
    stop();
    for (uint8_t k = 0; k < n; k++) channels[k] = ch[k];
    nChannels = n;
    return true;
  }

  uint8_t numChannels(){ return nChannels; }

  void setPeriodUs(uint32_t us){
    if (us >= 1000) dtUs = us;
  }

  bool push(const int16_t* tenths, uint8_t n){
    if (n != nChannels || count() >= TRAJ_BUFFER_POINTS - 1){
      rejected = rejected + 1;
      return false;
    }
    for (uint8_t k = 0; k < n; k++) points[head][k] = tenths[k];
    head = (head + 1) % TRAJ_BUFFER_POINTS;
    return true;
  }

  void go(){
    Servos::demoStop();
    played = 0;
    underruns = 0;
    rejected = 0;
    state = RUNNING;
  }

  void end(){
    if (state == RUNNING) state = ENDING;
  }

  void stop(){
    state = IDLE;
    clear();
  }

  float index(){ return state == IDLE ? -1.0f : (float)played; }

  uint16_t queued(){ return count(); }

  void printStatus(){
    static const char* names[] = {"IDLE", "RUNNING", "ENDING"};
    Serial.printf("T_ST state=%s played=%lu queued=%u underruns=%lu rejected=%lu ch=%u dt_us=%lu\n",
                  names[state], (unsigned long)played, (unsigned)count(),
                  (unsigned long)underruns, (unsigned long)rejected,
                  (unsigned)nChannels, (unsigned long)dtUs);
  }

}
