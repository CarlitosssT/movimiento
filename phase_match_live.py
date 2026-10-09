"""Vista en tiempo real del prototipo de seleccion de fase (phase_match_sim.py).

Simula una marcha de cabra irregular (velocidad variable, pausas, amplitud y
offset que derivan) vista por un IMU en el ankle, y la reproduce en vivo: se
ven las senales del IMU, el hip/knee que elige la pata robot a partir de ellas
y una vista de la pata (referencia vs. configuracion elegida).
"""
import argparse
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec

import phase_match_sim as sim

L_HIP, L_KNEE, L_ANKLE = 12.1, 11.1, 15.3  # cm (columna "Front" de la tabla de especificaciones)
BASE_ANGLE_OFFSET_DEG = -90  # igual que plot_leg_actuator_angles.ipynb
WINDOW_S = 5.0
WEIGHTS = (1, 1, 0)  # theta + omega, la mejor configuracion de la simulacion
MODE = "recover"


def leg_points(hip, knee, ankle):
    phi_hip = np.radians(hip + BASE_ANGLE_OFFSET_DEG)
    phi_knee = phi_hip + np.radians(knee) - np.pi / 2
    phi_ankle = phi_knee + np.radians(ankle) - np.pi / 2
    d = lambda th: np.array([np.sin(th), -np.cos(th)])
    p0 = np.zeros(2)
    p1 = p0 + L_HIP * d(phi_hip)
    p2 = p1 + L_KNEE * d(phi_knee)
    p3 = p2 + L_ANKLE * d(phi_ankle)
    return np.array([p0, p1, p2, p3])


def make_irregular_stream(foot, period, duration, seed):
    """Marcha poco ciclica: velocidad variable, pausas, y amplitud/offset que derivan."""
    rng = np.random.default_rng(seed)
    dt = 1 / sim.FS
    t = np.arange(0, duration, dt)
    rate = np.ones_like(t)
    for _ in range(3):
        rate += rng.uniform(0.05, 0.12) * np.sin(2 * np.pi * rng.uniform(0.05, 0.5) * t + rng.uniform(0, 2 * np.pi))
    for c in rng.uniform(3, max(4, duration - 1), max(1, int(duration / 12))):
        rate -= 0.8 * np.exp(-0.5 * ((t - c) / rng.uniform(0.3, 0.7)) ** 2)  # pasos casi detenidos
    rate = np.clip(rate, 0.1, None)
    p = rng.uniform() + np.cumsum(rate) * dt / period
    gain = 1 + 0.06 * np.sin(2 * np.pi * 0.07 * t + rng.uniform(0, 2 * np.pi))
    bias = 2.0 * np.sin(2 * np.pi * 0.05 * t + rng.uniform(0, 2 * np.pi))
    theta = gain * sim.eval_series(foot, p % 1.0)[0] + bias
    omega = np.gradient(theta, dt)
    theta_m = sim.ema(theta + rng.normal(0, 1.0, t.size), 20, dt)
    omega_m = sim.ema(omega + rng.normal(0, 8.0, t.size), 20, dt)
    alpha_m = sim.ema(np.gradient(omega_m, dt), 10, dt)
    return t, p % 1.0, np.column_stack([theta_m, omega_m, alpha_m])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0, help="segundos de senal simulada")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--speed", type=float, default=1.0, help="velocidad de reproduccion (1 = tiempo real)")
    parser.add_argument("--snapshot", type=float, default=None, help="guarda una imagen en el instante dado y sale")
    args = parser.parse_args()

    coef, foot, period = sim.load_coefficients()
    feats, hip_tab, knee_tab, scale = sim.build_table(coef, foot, period)
    ankle_tab = sim.eval_series(coef["ankle"], np.arange(sim.M) / sim.M)[0]
    t, p_true, stream = make_irregular_stream(foot, period, args.duration, args.seed)
    hip_ref = sim.eval_series(coef["hip"], p_true)[0]
    knee_ref = sim.eval_series(coef["knee"], p_true)[0]
    ankle_ref = sim.eval_series(coef["ankle"], p_true)[0]

    matcher = sim.PhaseMatcher(feats, scale, WEIGHTS, MODE)
    idx = np.zeros(len(t), int)
    done = {"n": 0}

    fig = plt.figure(figsize=(14, 8))
    gs = GridSpec(4, 2, width_ratios=[2, 1], figure=fig)
    ax_th = fig.add_subplot(gs[0, 0])
    ax_om = fig.add_subplot(gs[1, 0], sharex=ax_th)
    ax_hip = fig.add_subplot(gs[2, 0], sharex=ax_th)
    ax_knee = fig.add_subplot(gs[3, 0], sharex=ax_th)
    ax_leg = fig.add_subplot(gs[:, 1])

    (ln_th,) = ax_th.plot([], [], color="tab:purple", label="IMU externo (entrada)")
    (ln_om,) = ax_om.plot([], [], color="tab:purple", label="IMU externo (entrada)")
    (ln_hip_ref,) = ax_hip.plot([], [], color="0.6", linewidth=2, label="referencia (fase real)")
    (ln_hip,) = ax_hip.plot([], [], color="tab:orange", label="pata robot (elegido)")
    (ln_knee_ref,) = ax_knee.plot([], [], color="0.6", linewidth=2, label="referencia (fase real)")
    (ln_knee,) = ax_knee.plot([], [], color="tab:orange", label="pata robot (elegido)")
    for ax, label, lim in [
        (ax_th, "Pitch pie [deg]", (30, 125)),
        (ax_om, "Velocidad [deg/s]", (-900, 900)),
        (ax_hip, "Hip [deg]", (35, 65)),
        (ax_knee, "Knee [deg]", (85, 135)),
    ]:
        ax.set_ylabel(label)
        ax.set_ylim(*lim)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
    ax_th.set_xlim(-WINDOW_S, 0)
    ax_knee.set_xlabel("Time relative to now [s]")

    reach = L_HIP + L_KNEE + L_ANKLE
    ax_leg.set_xlim(-reach * 1.1, reach * 1.1)
    ax_leg.set_ylim(-reach * 1.1, reach * 0.4)
    ax_leg.set_aspect("equal")
    ax_leg.set_title("Pata FR")
    ax_leg.set_xlabel("x [cm]")
    ax_leg.set_ylabel("y [cm]")
    (ln_leg_ref,) = ax_leg.plot([], [], "o-", color="0.6", linewidth=6, markersize=8, label="referencia")
    (ln_leg,) = ax_leg.plot([], [], "o-", color="tab:orange", linewidth=3, markersize=6, label="elegida")
    ax_leg.legend(loc="lower left", fontsize=8)
    info = ax_leg.text(0.03, 0.97, "", transform=ax_leg.transAxes, va="top", fontsize=9, family="monospace")

    def advance(n):
        while done["n"] < n:
            idx[done["n"]] = matcher.step(stream[done["n"]])
            done["n"] += 1

    artists = (ln_th, ln_om, ln_hip_ref, ln_hip, ln_knee_ref, ln_knee, ln_leg_ref, ln_leg, info)

    def draw(n):
        n = max(1, min(n, len(t)))
        advance(n)
        lo = max(0, n - int(WINDOW_S * sim.FS))
        sl = slice(lo, n)
        now = t[n - 1]
        rel = t[sl] - now  # eje x fijo (tiempo respecto a "ahora"): permite blit
        ln_th.set_data(rel, stream[sl, 0])
        ln_om.set_data(rel, stream[sl, 1])
        ln_hip_ref.set_data(rel, hip_ref[sl])
        ln_hip.set_data(rel, hip_tab[idx[sl]])
        ln_knee_ref.set_data(rel, knee_ref[sl])
        ln_knee.set_data(rel, knee_tab[idx[sl]])
        i = idx[n - 1]
        ln_leg.set_data(*leg_points(hip_tab[i], knee_tab[i], ankle_tab[i]).T)
        ln_leg_ref.set_data(*leg_points(hip_ref[n - 1], knee_ref[n - 1], ankle_ref[n - 1]).T)
        err_hip = np.sqrt(np.mean((hip_tab[idx[sl]] - hip_ref[sl]) ** 2))
        err_knee = np.sqrt(np.mean((knee_tab[idx[sl]] - knee_ref[sl]) ** 2))
        d_phase = ((i / sim.M - p_true[n - 1] + 0.5) % 1.0 - 0.5) * 100
        info.set_text(
            f"t = {now:5.1f} s\nfase real {p_true[n - 1]:.2f}  est. {i / sim.M:.2f}\n"
            f"err fase {d_phase:+5.1f} %\nRMS ult. {WINDOW_S:.0f} s:\n  hip {err_hip:4.1f} deg\n  knee {err_knee:4.1f} deg"
        )
        return artists

    fig.suptitle("Seleccion de fase en vivo: IMU externo (marcha irregular) -> hip/knee de la pata robot")
    fig.tight_layout()

    if args.snapshot is not None:
        draw(int(args.snapshot * sim.FS))
        fig.savefig("phase_match_live.png", dpi=100)
        return

    start = {"t0": None}

    def frame(_):
        if start["t0"] is None:
            start["t0"] = time.perf_counter()
        n = int((time.perf_counter() - start["t0"]) * args.speed * sim.FS) + 1
        result = draw(n)
        if n >= len(t):
            anim.event_source.stop()
        return result

    # blit: solo se redibujan las lineas y el texto; los ejes, la grilla y las
    # leyendas se dejan en el fondo, lo que baja mucho el costo por cuadro.
    anim = FuncAnimation(fig, frame, interval=20, blit=True, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
