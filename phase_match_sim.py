"""Prototipo: estimar la fase del ciclo de la pata FR a partir de las senales de
un IMU en el ankle (angulo, velocidad, aceleracion) y elegir hip/knee de la
trayectoria predefinida. El IMU se simula a partir de la propia tabla.
"""
import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from plot_one_cycle import CSV_PATH

LEG = "fr"
N_HARM = 4  # armonicos de la tabla (los elegidos en plot_one_cycle.py)
M = 508  # puntos de la tabla de busqueda (4x las muestras del CSV)
FS = 100.0  # Hz, tasa del IMU
DURATION_S = 10.0
SKIP_S = 1.0  # no se cuenta el arranque en las metricas

SCENARIOS = {
    "limpio": dict(s_theta=0.3, s_omega=2.0, speed_var=0.0, gain=1.0, bias=0.0),
    "ruidoso": dict(s_theta=1.0, s_omega=10.0, speed_var=0.0, gain=1.0, bias=0.0),
    "ruidoso+vel +-15%": dict(s_theta=1.0, s_omega=10.0, speed_var=0.15, gain=1.0, bias=0.0),
    "marcha distinta": dict(s_theta=1.0, s_omega=10.0, speed_var=0.15, gain=0.85, bias=3.0),
}
# nombre, pesos (theta, omega, alpha), modo de busqueda:
#   window  = solo ventana alrededor de la fase anterior
#   global  = toda la tabla en cada muestra
#   recover = ventana, pero salta a la mejor fase global si es mucho mejor
RECOVER_RATIO = 0.5
RECOVER_SAMPLES = 5  # muestras seguidas con la global mucho mejor antes de saltar
CONFIGS = [
    ("theta", (1, 0, 0), "window"),
    ("theta+omega", (1, 1, 0), "window"),
    ("theta+omega+0.3alpha", (1, 1, 0.3), "window"),
    ("theta+omega+alpha", (1, 1, 1), "window"),
    ("theta+omega (global)", (1, 1, 0), "global"),
    ("theta+omega (recover)", (1, 1, 0), "recover"),
    ("th+om+0.3al (recover)", (1, 1, 0.3), "recover"),
]


def load_coefficients():
    df = pd.read_csv(CSV_PATH)
    n = len(df)
    period = n * float(np.median(np.diff(df["time_s"])))
    coef = {j: np.fft.rfft(df[f"{LEG}_{j}_joint"].to_numpy())[: N_HARM + 1] / n for j in ("hip", "knee", "ankle")}
    foot = coef["hip"] + coef["knee"] + coef["ankle"]
    foot[0] -= 180.0  # pitch global del pie: hip + knee + ankle - 180
    return coef, foot, period


def eval_series(c, p):
    """x(p), dx/dp, d2x/dp2 de una serie de Fourier de un lado (p en ciclos)."""
    k = np.arange(len(c))
    w = c.copy()
    w[1:] *= 2
    e = np.exp(2j * np.pi * np.outer(p, k))
    return (e @ w).real, (e @ (1j * 2 * np.pi * k * w)).real, (e @ (-((2 * np.pi * k) ** 2) * w)).real


def ema(x, fc, dt):
    a = dt / (dt + 1 / (2 * np.pi * fc))
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


def make_stream(foot, period, scn, rng):
    """Senales del IMU (filtradas causalmente) de una cabra con marcha variable."""
    dt = 1 / FS
    t = np.arange(0, DURATION_S, dt)
    f_var = rng.uniform(0.15, 0.4)
    phi0 = rng.uniform(0, 2 * np.pi)
    rate = 1 + scn["speed_var"] * np.sin(2 * np.pi * f_var * t + phi0)  # ciclos por periodo nominal
    drate = scn["speed_var"] * 2 * np.pi * f_var * np.cos(2 * np.pi * f_var * t + phi0)
    p = rng.uniform() + np.cumsum(rate) * dt / period
    dp = rate / period
    d2p = drate / period
    th, dth, d2th = eval_series(foot, p % 1.0)
    theta = scn["gain"] * th + scn["bias"]
    omega = scn["gain"] * dth * dp
    alpha = scn["gain"] * (d2th * dp**2 + dth * d2p)
    theta_m = ema(theta + rng.normal(0, scn["s_theta"], t.size), 20, dt)
    omega_m = ema(omega + rng.normal(0, scn["s_omega"], t.size), 20, dt)
    alpha_m = ema(np.gradient(omega_m, dt), 10, dt)  # aceleracion derivada del giroscopio
    truth = np.column_stack([theta, omega, alpha])
    return t, p % 1.0, np.column_stack([theta_m, omega_m, alpha_m]), truth


def build_table(coef, foot, period):
    p = np.arange(M) / M
    th, dth, d2th = eval_series(foot, p)
    feats = np.column_stack([th, dth / period, d2th / period**2])
    hip = eval_series(coef["hip"], p)[0]
    knee = eval_series(coef["knee"], p)[0]
    return feats, hip, knee, feats.std(axis=0)


class PhaseMatcher:
    """Elige la fila de la tabla que mejor explica cada muestra del IMU (una a la vez)."""

    def __init__(self, feats, scale, weights, mode):
        self.feats, self.scale, self.mode = feats, scale, mode
        self.w = np.asarray(weights, float)
        self.back, self.fwd = int(0.015 * M), int(0.06 * M)
        self.everything = np.arange(M)
        self.prev = None
        self.lost = 0

    def _cost(self, rows, x):
        return (self.w * ((self.feats[rows] - x) / self.scale) ** 2).sum(axis=1)

    def step(self, x):
        if self.prev is None or self.mode == "global":
            self.prev = int(np.argmin(self._cost(self.everything, x)))
            return self.prev
        cand = (self.prev + np.arange(-self.back, self.fwd + 1)) % M
        cost = self._cost(cand, x)
        best = cost.min()
        self.prev = int(cand[np.argmin(cost)])
        if self.mode == "recover":
            cost_all = self._cost(self.everything, x)
            if cost_all.min() < RECOVER_RATIO * best:
                self.lost += 1
                if self.lost >= RECOVER_SAMPLES:
                    self.prev = int(np.argmin(cost_all))
                    self.lost = 0
            else:
                self.lost = 0
        return self.prev


def match(stream, feats, scale, weights, mode):
    matcher = PhaseMatcher(feats, scale, weights, mode)
    return np.array([matcher.step(x) for x in stream])


def errors(idx, p_true, hip, knee, t, coef):
    keep = t >= SKIP_S
    est = idx / M
    d = (est - p_true + 0.5) % 1.0 - 0.5
    hip_true = eval_series(coef["hip"], p_true)[0]
    knee_true = eval_series(coef["knee"], p_true)[0]
    rms = lambda e: float(np.sqrt(np.mean(e[keep] ** 2)))
    return rms(d) * 100, rms(hip[idx] - hip_true), rms(knee[idx] - knee_true)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=6)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    coef, foot, period = load_coefficients()
    feats, hip_tab, knee_tab, scale = build_table(coef, foot, period)
    print(f"Periodo {period:.3f} s | tabla {M} puntos | IMU {FS:.0f} Hz | {DURATION_S:.0f} s x {args.seeds} semillas")
    print(f"Referencia sin informacion (hip/knee fijos en su media): hip {hip_tab.std():.1f} deg, knee {knee_tab.std():.1f} deg RMS\n")

    header = f"{'escenario':20s} {'configuracion':28s} {'fase [% ciclo]':>15s} {'hip [deg]':>10s} {'knee [deg]':>11s}"
    print(header)
    print("-" * len(header))
    example = None
    for s_name, scn in SCENARIOS.items():
        streams = [make_stream(foot, period, scn, np.random.default_rng(seed)) for seed in range(args.seeds)]
        for c_name, weights, mode in CONFIGS:
            res = []
            for t, p_true, stream, truth in streams:
                idx = match(stream, feats, scale, weights, mode)
                res.append(errors(idx, p_true, hip_tab, knee_tab, t, coef))
                if s_name == "ruidoso+vel +-15%" and c_name == "theta+omega (recover)" and example is None:
                    example = (t, p_true, idx, stream, truth)
            ph, hp, kn = np.mean(res, axis=0)
            print(f"{s_name:20s} {c_name:28s} {ph:15.2f} {hp:10.2f} {kn:11.2f}")
        print()

    t, p_true, idx, stream, truth = example
    est_p = idx / M
    hip_true = eval_series(coef["hip"], p_true)[0]
    knee_true = eval_series(coef["knee"], p_true)[0]
    fig, axs = plt.subplots(6, 1, figsize=(11, 13), sharex=True)
    for ax, k, label in zip(axs[:3], range(3), ["Pitch del pie [deg]", "Velocidad [deg/s]", "Aceleracion [deg/s²]"]):
        ax.plot(t, truth[:, k], color="0.6", linewidth=2, label="real (sin ruido)")
        ax.plot(t, stream[:, k], color="tab:purple", linewidth=1, label="medido por el IMU (entrada)")
        ax.set_ylabel(label)
        ax.legend(loc="upper right", fontsize=8)
    axs[2].set_title("la aceleracion no se usa en esta configuracion (peso 0)", fontsize=8, loc="left")
    axs[3].plot(t, p_true, label="fase real")
    axs[3].plot(t, est_p, "--", label="fase estimada")
    axs[3].set_ylabel("Fase [ciclos]")
    axs[4].plot(t, hip_true, label="hip real")
    axs[4].plot(t, hip_tab[idx], "--", label="hip enviado")
    axs[4].set_ylabel("Hip [deg]")
    axs[5].plot(t, knee_true, label="knee real")
    axs[5].plot(t, knee_tab[idx], "--", label="knee enviado")
    axs[5].set_ylabel("Knee [deg]")
    axs[5].set_xlabel("Time [s]")
    for ax in axs[3:]:
        ax.legend(loc="upper right", fontsize=8)
    for ax in axs:
        ax.grid(True, alpha=0.3)
    fig.suptitle("Escenario 'ruidoso+vel +-15%', theta+omega con reenganche, primera semilla")
    fig.tight_layout()
    if args.no_show:
        fig.savefig("phase_match_sim.png", dpi=110)
    else:
        plt.show()


if __name__ == "__main__":
    main()
