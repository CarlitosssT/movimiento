"""Genera ErisServoIMU/phase_table.h a partir del CSV de un ciclo.

Usa las mismas funciones que phase_match_sim.py, asi que la tabla que busca la
ESP32 es exactamente la del prototipo: M fases uniformes del ciclo, cada una con
los rasgos del pitch del pie (theta, omega, alpha) y los angulos hip/knee/ankle.

Uso (desde la carpeta del proyecto):
    wearable/Scripts/python tools/gen_phase_table.py
"""
import sys
from pathlib import Path

import numpy as np

RAIZ = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RAIZ))

import phase_match_sim as sim  # noqa: E402

SALIDA = RAIZ / "ErisServoIMU" / "phase_table.h"


def fila(v):
    return "  {" + ", ".join(f"{x:.5f}f" for x in v) + "},\n"


def main():
    coef, foot, period = sim.load_coefficients()
    feats, hip, knee, scale = sim.build_table(coef, foot, period)
    ankle = sim.eval_series(coef["ankle"], np.arange(sim.M) / sim.M)[0]

    with open(SALIDA, "w", encoding="utf-8", newline="\n") as f:
        f.write("// Generado por tools/gen_phase_table.py. No editar a mano.\n")
        f.write(f"// Pata {sim.LEG}, {sim.N_HARM} armonicos, periodo {period:.4f} s\n")
        f.write("#ifndef PHASE_TABLE_H\n#define PHASE_TABLE_H\n\n")
        f.write(f"#define PHASE_M {sim.M}\n")
        f.write(f"#define PHASE_PERIOD_S {period:.6f}f\n\n")
        f.write("// Desviacion estandar de cada rasgo (normaliza el costo): theta, omega, alpha\n")
        f.write("static const float PHASE_SCALE[3] = {" + ", ".join(f"{x:.5f}f" for x in scale) + "};\n\n")
        f.write("// Rasgos del pitch del pie en cada fase: theta [deg], omega [deg/s], alpha [deg/s^2]\n")
        f.write("static const float PHASE_FEATS[PHASE_M][3] = {\n")
        f.writelines(fila(r) for r in feats)
        f.write("};\n\n// Angulos articulares en cada fase [deg]: hip, knee, ankle\n")
        f.write("static const float PHASE_JOINTS[PHASE_M][3] = {\n")
        f.writelines(fila(r) for r in np.column_stack([hip, knee, ankle]))
        f.write("};\n\n#endif\n")
    print(f"{SALIDA.relative_to(RAIZ)}: {sim.M} fases, periodo {period:.3f} s, escala {np.round(scale, 2)}")


if __name__ == "__main__":
    main()
