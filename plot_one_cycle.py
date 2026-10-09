from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.widgets import Button, RadioButtons

CSV_PATH = Path(__file__).with_name("leg_actuator_angles_one_cycle.csv")
LEG = "fr"
JOINTS = ["hip", "knee", "ankle"]
HARMONICS = [4, 6, 8, 12]
DEFAULT_HARMONICS = 4
ORIGINAL_ONLY = "solo original"
ROW_LABELS = ["Angle [deg]", "Velocity [deg/s]", "Acceleration [deg/s²]"]


def periodic_derivative(x, dt):
    # El CSV es un ciclo exacto repetido: diferencias centrales circulares.
    return (np.roll(x, -1) - np.roll(x, 1)) / (2 * dt)


def fourier_smooth(x, dt, n_harmonics):
    """Pasa-bajos de Fourier de una senal periodica (un ciclo exacto).

    Conserva el armonico 0 y los primeros n_harmonics; devuelve la senal
    suavizada y sus derivadas primera y segunda calculadas analiticamente.
    """
    n = len(x)
    spectrum = np.fft.rfft(x)
    spectrum[n_harmonics + 1:] = 0
    omega = 2 * np.pi * np.fft.rfftfreq(n, d=dt)
    smooth = np.fft.irfft(spectrum, n)
    velocity = np.fft.irfft(1j * omega * spectrum, n)
    acceleration = np.fft.irfft(-(omega**2) * spectrum, n)
    return smooth, velocity, acceleration


def export_smoothed(df, dt, n_harmonics, out_dir=CSV_PATH.parent):
    """Guarda el ciclo con FR suavizada a n_harmonics (otras columnas sin cambios)."""
    out = df.copy()
    for joint in JOINTS:
        col = f"{LEG}_{joint}_joint"
        out[col] = fourier_smooth(df[col].to_numpy(), dt, n_harmonics)[0]
    path = Path(out_dir) / f"leg_actuator_angles_one_cycle_smooth_n{n_harmonics}.csv"
    out.to_csv(path, index=False)
    return path


def build_figure(df, dt, out_dir=CSV_PATH.parent):
    t = df["time_s"].to_numpy()
    fig, axs = plt.subplots(3, 3, figsize=(15, 9), sharex=True)
    smooth_lines = {n: [] for n in HARMONICS}  # n -> lineas (9 paneles)

    for col, joint in enumerate(JOINTS):
        raw = df[f"{LEG}_{joint}_joint"].to_numpy()
        raw_vel = periodic_derivative(raw, dt)
        original = [raw, raw_vel, periodic_derivative(raw_vel, dt)]
        for row in range(3):
            axs[row, col].plot(t, original[row], color="0.6", linewidth=1.5, label="original")
        for n in HARMONICS:
            smooth = fourier_smooth(raw, dt, n)
            for row in range(3):
                (line,) = axs[row, col].plot(t, smooth[row], color="tab:red", linewidth=2, label=f"{n} armónicos")
                smooth_lines[n].append(line)
        axs[0, col].set_title(joint)
    for row in range(3):
        axs[row, 0].set_ylabel(ROW_LABELS[row])
        for col in range(3):
            axs[row, col].grid(True, alpha=0.3)
    for col in range(3):
        axs[-1, col].set_xlabel("Time [s]")
        axs[-1, col].set_xlim(t[0], t[-1])
    fig.tight_layout(rect=[0, 0, 0.86, 0.96])

    ax_radio = fig.add_axes([0.88, 0.48, 0.11, 0.26])
    ax_radio.set_title("Ver / exportar", fontsize=10)
    labels = [ORIGINAL_ONLY] + [f"{n} armónicos" for n in HARMONICS]
    radio = RadioButtons(ax_radio, labels, active=labels.index(f"{DEFAULT_HARMONICS} armónicos"))
    ax_button = fig.add_axes([0.88, 0.38, 0.11, 0.06])
    button = Button(ax_button, "Exportar CSV")
    status = fig.text(0.88, 0.33, "", fontsize=8, va="top")

    def selected_harmonics():
        label = radio.value_selected
        return None if label == ORIGINAL_ONLY else HARMONICS[labels.index(label) - 1]

    def refresh(_label=None):
        selected = selected_harmonics()
        for n, lines in smooth_lines.items():
            for line in lines:
                line.set_visible(n == selected)
        for ax in axs.flat:
            ax.relim(visible_only=True)
            ax.autoscale_view()
            ax.legend(loc="best", fontsize=8)
        title = "solo original" if selected is None else f"original vs. {selected} armónicos"
        fig.suptitle(f"Pata {LEG.upper()}: {title} ({len(df)} muestras, {t[-1]:.3f} s)")
        status.set_text("")
        fig.canvas.draw_idle()

    def on_export(_event=None):
        selected = selected_harmonics()
        if selected is None:
            status.set_text("Elige un suavizado\npara exportar")
        else:
            path = export_smoothed(df, dt, selected, out_dir)
            status.set_text(f"Guardado:\n{path.name}")
            print(f"Exportado: {path}")
        fig.canvas.draw_idle()

    radio.on_clicked(refresh)
    button.on_clicked(on_export)
    refresh()
    # Se devuelven los widgets para que sigan referenciados (si no, se recolectan).
    return fig, radio, button, on_export


def main():
    df = pd.read_csv(CSV_PATH)
    dt = float(np.median(np.diff(df["time_s"].to_numpy())))
    _widgets = build_figure(df, dt)
    plt.show()


if __name__ == "__main__":
    main()
