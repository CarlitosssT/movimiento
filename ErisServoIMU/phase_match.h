#ifndef PHASE_MATCH_H
#define PHASE_MATCH_H

// Port a C++ de PhaseMatcher (phase_match_sim.py). Sin dependencias de Arduino
// para poder probarlo en el PC contra la version de Python.
//
// Elige la fila de la tabla que mejor explica cada muestra [theta, omega, alpha]:
//   WINDOW  = solo una ventana alrededor de la fase anterior
//   GLOBAL  = toda la tabla en cada muestra
//   RECOVER = ventana, pero salta a la mejor fase global si es mucho mejor
//             durante RECOVER_SAMPLES muestras seguidas

#include <stdint.h>

#define PHASE_RECOVER_RATIO   0.5f
#define PHASE_RECOVER_SAMPLES 5

class PhaseMatcher {
  public:
    enum Mode : uint8_t { WINDOW, GLOBAL, RECOVER };

    void init(const float (*feats)[3], int rows, const float* scale, Mode m = RECOVER){
      F = feats; M = rows; mode = m;
      for (int k = 0; k < 3; k++) invScale[k] = 1.0f / scale[k];
      back = (int)(0.015f * M);
      fwd  = (int)(0.06f * M);
      setWeights(1, 1, 0);
      reset();
    }

    void setWeights(float wt, float wo, float wa){ w[0] = wt; w[1] = wo; w[2] = wa; }
    void setMode(Mode m){ mode = m; }
    void reset(){ prev = -1; lost = 0; }

    // Devuelve la fila elegida para la muestra x = {theta, omega, alpha}
    int step(const float x[3]){
      if (prev < 0 || mode == GLOBAL){
        prev = bestGlobal(x, NULL);
        return prev;
      }
      float best = 1e30f;
      int bestRow = prev;
      for (int d = -back; d <= fwd; d++){
        int row = ((prev + d) % M + M) % M;
        float c = cost(row, x);
        if (c < best){ best = c; bestRow = row; }
      }
      prev = bestRow;
      if (mode == RECOVER){
        float costAll;
        int rowAll = bestGlobal(x, &costAll);
        if (costAll < PHASE_RECOVER_RATIO * best){
          if (++lost >= PHASE_RECOVER_SAMPLES){ prev = rowAll; lost = 0; }
        } else {
          lost = 0;
        }
      }
      return prev;
    }

    int rows() const { return M; }

  private:
    const float (*F)[3] = 0;
    int M = 0, back = 0, fwd = 0, prev = -1, lost = 0;
    float invScale[3], w[3];
    Mode mode = RECOVER;

    float cost(int row, const float x[3]) const {
      float c = 0;
      for (int k = 0; k < 3; k++){
        float e = (F[row][k] - x[k]) * invScale[k];
        c += w[k] * e * e;
      }
      return c;
    }

    int bestGlobal(const float x[3], float* costOut) const {
      float best = 1e30f;
      int bestRow = 0;
      for (int row = 0; row < M; row++){
        float c = cost(row, x);
        if (c < best){ best = c; bestRow = row; }
      }
      if (costOut) *costOut = best;
      return bestRow;
    }
};

#endif
