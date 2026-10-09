"""Visor en vivo de los IMUs del firmware ErisServoIMU, en una ventana propia.

Elige con los botones de la izquierda cual IMU graficar:
  hip, knee, ankle : angulo de cada MPU6050 (respecto a la pose neutra) y su derivada.
  externo          : pitch, velocidad y fase del IMU externo, y sus valores CRUDOS por eje
                     (gravedad X/Y/Z y giro X/Y/Z), que sirven para ver que eje se esta moviendo.

El pitch del IMU externo usa la gravedad en los ejes U, V y el giro alrededor del eje W
(EXT_AXIS_U/V/W en configuration.h; ahora el giro es alrededor de X). Gira el sensor y mira cual giro crudo reacciona: ese es el
eje alrededor del cual lo estas girando.

Teclas:  c = calibrar (CAL, PH_GYRO, PH_ZERO: sensores quietos ~2 s)   espacio = pausa   q = salir

Uso:
    wearable2\\Scripts\\python visor_imus.py --puerto COM9
    wearable2\\Scripts\\python visor_imus.py --puerto COM9 --fuente externo --ventana 15

Cierra el notebook (o su conexion) antes: el puerto serie solo lo puede abrir un programa.

Rendimiento: solo se redibujan las lineas (blitting); los ejes, la leyenda y las etiquetas se pintan
una vez y se vuelven a pintar solo cuando cambia la escala vertical, la fuente o el tamano de la ventana.
"""
import argparse
import re
import struct
import threading
import time
from collections import deque

import matplotlib

matplotlib.use("QtAgg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import serial  # noqa: E402
from matplotlib.gridspec import GridSpec  # noqa: E402
from matplotlib.widgets import RadioButtons  # noqa: E402

BAUDIOS = 921600
MUESTRA_IMU = struct.Struct("<6f")   # t_ms, hip, knee, ankle, traj_idx, traj_cola
MUESTRA_EXT = struct.Struct("<8f")   # t_ms, theta, omega, alpha, fase, hip, knee, ankle
FUENTES = ["hip", "knee", "ankle", "externo"]
ETIQUETAS = ["hip (MPU6050)", "knee (MPU6050)", "ankle (MPU6050)", "externo"]
EJES = ["X", "Y", "Z"]
COLORES_EJE = ["tab:red", "tab:green", "tab:blue"]
INTERVALO_MS = 40                    # 25 cuadros por segundo
REAJUSTE_MIN_S = 0.5                 # la escala vertical cambia como mucho 2 veces por segundo

_TEXTO = re.compile(rb"(?:[\t\x20-\x7e]+\r?\n)+")
_PH_ST = re.compile(r"grv=([-\d.]+),([-\d.]+),([-\d.]+)\s+gyr=([-\d.]+),([-\d.]+),([-\d.]+)")


def cobs_decode(datos):
    """Decodifica una trama COBS (PacketSerial) sin el 0x00 final."""
    salida = bytearray()
    i, n = 0, len(datos)
    while i < n:
        codigo = datos[i]
        if codigo == 0 or i + codigo > n:
            raise ValueError("trama COBS invalida")
        salida += datos[i + 1:i + codigo]
        i += codigo
        if codigo < 0xFF and i < n:
            salida.append(0)
    return bytes(salida)


class Enlace:
    """Conexion con la ESP32: un hilo lee tramas y texto; otro pide PH_ST para los valores crudos."""

    def __init__(self, puerto, baudios=BAUDIOS, eco=True, ventana=10.0):
        self.ser = serial.Serial(puerto, baudios, timeout=0.05)
        self.eco = eco
        self.lock = threading.Lock()
        self.lock_tx = threading.Lock()
        # Solo se guarda un poco mas que la ventana visible: copiar menos datos en cada cuadro
        n = int(ventana * 150) + 300
        self.imu = deque(maxlen=n)         # (t_s, hip, knee, ankle)
        self.ext = deque(maxlen=n)         # (t_s, theta, omega, alpha, fase)
        self.crudo = deque(maxlen=int(ventana * 40) + 100)   # (t_s, gx, gy, gz, wx, wy, wz) del externo
        self.mensajes = deque(maxlen=300)  # (n, texto)
        self.n_mensajes = 0
        self.sensor = "?"
        self.lecturas = (0, 0)
        self.tramas_malas = 0
        self.n_imu = 0                     # muestras recibidas en total (para el titulo)
        self.n_ext = 0
        self.t_esp = 0.0                   # instante de la ULTIMA muestra recibida de la ESP32 (s)
        self._buf = b""
        self._corriendo = True
        self.hilo = threading.Thread(target=self._leer, daemon=True)
        self.hilo.start()
        self.hilo_st = threading.Thread(target=self._pedir_estado, daemon=True)
        self.hilo_st.start()

    # ---------- envio ----------
    def enviar(self, comando):
        with self.lock_tx:
            self.ser.write((comando + "\n").encode())

    def esperar_texto(self, fragmento, timeout=15.0):
        t_fin = time.time() + timeout
        while time.time() < t_fin:
            with self.lock:
                if any(fragmento in m for _, m in self.mensajes):
                    return True
            time.sleep(0.05)
        return False

    def _pedir_estado(self):
        while self._corriendo:
            try:
                self.enviar("PH_ST")
            except (serial.SerialException, OSError):
                return
            time.sleep(0.05)               # 20 Hz

    # ---------- recepcion ----------
    def _texto(self, crudo):
        for linea in crudo.decode(errors="replace").splitlines():
            linea = linea.strip()
            if not linea:
                continue
            if linea.startswith("PH_ST"):
                self._estado(linea)
                continue
            with self.lock:
                self.mensajes.append((self.n_mensajes, linea))
                self.n_mensajes += 1
            if self.eco:
                print("<-", linea)

    def _estado(self, linea):
        m = _PH_ST.search(linea)
        if not m:
            return
        v = [float(x) for x in m.groups()]
        s = re.search(r"sensor=(\w+)", linea)
        r = re.search(r"reads=(\d+) fails=(\d+)", linea)
        with self.lock:
            self.crudo.append((self.t_esp, *v))
            if s:
                self.sensor = s.group(1)
            if r:
                self.lecturas = (int(r.group(1)), int(r.group(2)))

    def _paquete(self, trama):
        try:
            d = cobs_decode(trama)
        except ValueError:
            return False
        tipo = d[:1]
        if tipo == b"D" and len(d) >= 2:
            secciones, pos = [], 1
            for formato in (MUESTRA_IMU, MUESTRA_EXT):
                if pos >= len(d):
                    return False
                n = d[pos]
                fin = pos + 1 + n * formato.size
                if fin > len(d):
                    return False
                secciones.append([formato.unpack_from(d, pos + 1 + k * formato.size) for k in range(n)])
                pos = fin
            if pos != len(d):
                return False
            filas, filas_ext = secciones
            with self.lock:
                for t_ms, h, c, ca, _idx, _cola in filas:
                    self._nuevo_tiempo(t_ms / 1000)
                    self.imu.append((t_ms / 1000, h, c, ca))
                    self.n_imu += 1
                for t_ms, th, om, al, fase, _h, _c, _ca in filas_ext:
                    self._nuevo_tiempo(t_ms / 1000)
                    self.ext.append((t_ms / 1000, th, om, al, fase))
                    self.n_ext += 1
            return True
        if tipo in (b"T", b"E"):
            self._texto((b"ERROR: " if tipo == b"E" else b"") + d[1:].replace(b"\0", b" ") + b"\n")
            return True
        return False

    def _nuevo_tiempo(self, t):
        """El reloj de la ESP32 vuelve a 0 con S_TIME (y las primeras muestras traen la hora de arranque):
        si retrocede, se descarta lo anterior para que los datos sigan ordenados y 'ahora' sea el real."""
        if t < self.t_esp - 0.5:
            self.imu.clear()
            self.ext.clear()
            self.crudo.clear()
        self.t_esp = t

    def _trama(self, trama):
        if not trama or self._paquete(trama):
            return
        m = _TEXTO.match(trama)
        if m:
            self._texto(m.group())
            resto = trama[m.end():]
            if not resto or self._paquete(resto):
                return
        self.tramas_malas += 1

    def _procesar(self, datos):
        self._buf += datos
        *tramas, self._buf = self._buf.split(b"\0")
        for trama in tramas:
            self._trama(trama)
        if self._buf.endswith(b"\n") and _TEXTO.fullmatch(self._buf):
            self._texto(self._buf)
            self._buf = b""

    def _leer(self):
        while self._corriendo:
            try:
                datos = self.ser.read(max(1, self.ser.in_waiting))
            except (serial.SerialException, OSError):
                break
            if datos:
                self._procesar(datos)

    def cerrar(self):
        self._corriendo = False
        try:
            self.enviar("S_OFF")
        except (serial.SerialException, OSError):
            pass
        self.hilo.join(timeout=1)
        self.ser.close()


class Visor:
    def __init__(self, enlace, fuente="externo", ventana=10.0):
        self.enlace = enlace
        self.ventana = ventana
        self.fuente = fuente if fuente in FUENTES else "externo"
        self.pausa = False
        self.fondo = None                  # imagen de los ejes ya pintados (sin las lineas)
        self._lim = {}                     # ax -> (lo, hi) de la escala vertical actual
        self._t_ajuste = {}                # ax -> instante del ultimo reajuste

        self.fig = plt.figure(figsize=(13, 8))
        gs = GridSpec(4, 2, width_ratios=[1, 5], figure=self.fig, left=0.03, right=0.97, top=0.92, bottom=0.07,
                      wspace=0.15, hspace=0.35)
        self.ax_sel = self.fig.add_subplot(gs[:, 0])
        self.ax_ang = self.fig.add_subplot(gs[0, 1])
        self.ax_vel = self.fig.add_subplot(gs[1, 1], sharex=self.ax_ang)
        self.ax_grv = self.fig.add_subplot(gs[2, 1], sharex=self.ax_ang)
        self.ax_gyr = self.fig.add_subplot(gs[3, 1], sharex=self.ax_ang)

        self.radio = RadioButtons(self.ax_sel, ETIQUETAS, active=FUENTES.index(self.fuente))
        self.radio.on_clicked(self._al_elegir)
        self.ax_sel.set_title("IMU", fontsize=11)

        (self.l_ang,) = self.ax_ang.plot([], [], color="tab:purple", lw=1.6)
        (self.l_fase,) = self.ax_ang.plot([], [], color="tab:orange", lw=1, alpha=0.7)
        (self.l_vel,) = self.ax_vel.plot([], [], color="tab:orange", lw=1.4)
        self.l_grv = [self.ax_grv.plot([], [], color=c, lw=1.2, label=e)[0] for c, e in zip(COLORES_EJE, EJES)]
        self.l_gyr = [self.ax_gyr.plot([], [], color=c, lw=1.2, label=e)[0] for c, e in zip(COLORES_EJE, EJES)]
        self.ax_grv.legend(loc="upper left", ncol=3, fontsize=8)
        self.ax_gyr.legend(loc="upper left", ncol=3, fontsize=8)
        self.ax_gyr.set_xlabel("Tiempo respecto a ahora (s)")
        for ax in (self.ax_ang, self.ax_vel, self.ax_grv, self.ax_gyr):
            ax.grid(True, alpha=0.3)
            ax.set_autoscale_on(False)
        self.ax_ang.set_xlim(-ventana, 0)
        self.ax_ang.set_ylim(-1, 1)
        self.txt_grv = self.ax_grv.text(0.5, 0.5, "", transform=self.ax_grv.transAxes, ha="center", va="center",
                                        color="0.4")
        self.titulo = self.fig.suptitle("")
        self.sub_gyr = self.ax_gyr.title

        # Lo que cambia en cada cuadro se dibuja aparte (animated): no entra en la imagen de fondo
        self.por_eje = [
            (self.ax_ang, [self.l_ang, self.l_fase]),
            (self.ax_vel, [self.l_vel]),
            (self.ax_grv, self.l_grv + [self.txt_grv]),
            (self.ax_gyr, self.l_gyr + [self.sub_gyr]),
        ]
        for _, artistas in self.por_eje:
            for a in artistas:
                a.set_animated(True)
        self.titulo.set_animated(True)

        for nombre in ("keymap.back", "keymap.forward", "keymap.home", "keymap.pan", "keymap.zoom"):
            if "c" in plt.rcParams[nombre]:
                plt.rcParams[nombre].remove("c")
        self.canvas = self.fig.canvas
        self.canvas.mpl_connect("key_press_event", self._al_teclear)
        self.canvas.mpl_connect("draw_event", self._al_dibujar)
        self._etiquetas()
        self.timer = self.canvas.new_timer(interval=INTERVALO_MS)
        self.timer.add_callback(self.actualizar)

    # ---------- eventos ----------
    def _al_dibujar(self, _evento):
        """Cada pintado completo (tamano de la ventana, escala, fuente) guarda el fondo para el blitting."""
        self.fondo = self.canvas.copy_from_bbox(self.fig.bbox)

    def _al_elegir(self, etiqueta):
        self.fuente = FUENTES[ETIQUETAS.index(etiqueta)]
        self._lim.clear()
        self._etiquetas()
        self.canvas.draw_idle()

    def seleccionar(self, fuente):
        self.radio.set_active(FUENTES.index(fuente))

    def _al_teclear(self, evento):
        if evento.key == "c":
            for cmd in ("CAL", "PH_GYRO", "PH_ZERO"):
                self.enlace.enviar(cmd)
            print("Calibrando: deja los sensores quietos ~2 s")
        elif evento.key == " ":
            self.pausa = not self.pausa

    def _etiquetas(self):
        """Textos fijos de cada fuente (se pintan una sola vez, en el fondo)."""
        if self.fuente == "externo":
            self.ax_ang.set_ylabel("Pitch (deg) / fase (%)")
            self.ax_vel.set_ylabel("Velocidad (deg/s)")
            self.ax_grv.set_ylabel("Gravedad cruda")
            self.ax_gyr.set_ylabel("Giro crudo (deg/s)")
            self.txt_grv.set_text("")
        else:
            self.ax_ang.set_ylabel("Angulo (deg)")
            self.ax_vel.set_ylabel("Derivada (deg/s)")
            self.ax_grv.set_ylabel("")
            self.ax_gyr.set_ylabel("")
            self.txt_grv.set_text("Los valores crudos por eje solo estan\ndisponibles para el IMU externo "
                                  "(el firmware envia\nsolo el angulo de este MPU6050)")
            for l in self.l_grv + self.l_gyr:
                l.set_data([], [])
            self.sub_gyr.set_text("")

    # ---------- datos ----------
    def _ventana(self, datos, t_ahora):
        """Filas de los ultimos `ventana` s como array; la primera columna queda relativa a 'ahora'."""
        with self.enlace.lock:
            filas = list(datos)
        if len(filas) < 2:
            return None
        a = np.array(filas, float)
        a = a[np.searchsorted(a[:, 0], t_ahora - self.ventana):]
        if len(a) < 2:
            return None
        a[:, 0] -= t_ahora
        return a

    def _ajustar(self, ax, *series, minimo=2.0):
        """Reajusta la escala vertical solo si los datos se salen o quedan muy holgados. True si cambio."""
        partes = [np.asarray(s, float).ravel() for s in series if len(s)]
        if not partes:
            return False
        v = np.concatenate(partes)
        v = v[np.isfinite(v)]
        if not len(v):
            return False
        lo, hi = float(v.min()), float(v.max())
        actual = self._lim.get(ax)
        ahora = time.time()
        if actual is not None:
            fuera = lo < actual[0] or hi > actual[1]
            holgado = (hi - lo) < 0.35 * (actual[1] - actual[0])
            if not (fuera or holgado) or ahora - self._t_ajuste.get(ax, 0) < REAJUSTE_MIN_S:
                return False
        span = max(hi - lo, minimo)
        centro = (hi + lo) / 2
        nuevo = (centro - span * 0.65, centro + span * 0.65)
        ax.set_ylim(*nuevo)
        self._lim[ax] = nuevo
        self._t_ajuste[ax] = ahora
        return True

    def actualizar(self):
        if self.pausa or self.fondo is None:
            return
        en = self.enlace
        t_ahora = en.t_esp
        cambio = False
        vacio = ([], [])
        if self.fuente == "externo":
            d = self._ventana(en.ext, t_ahora)
            if d is not None:
                self.l_ang.set_data(d[:, 0], d[:, 1])
                self.l_fase.set_data(d[:, 0], d[:, 4] * 100)
                self.l_vel.set_data(d[:, 0], d[:, 2])
                cambio |= self._ajustar(self.ax_ang, d[:, 1], d[:, 4] * 100, minimo=20)
                cambio |= self._ajustar(self.ax_vel, d[:, 2], minimo=20)
            cambio |= self._crudo(t_ahora)
        else:
            col = FUENTES.index(self.fuente) + 1
            d = self._ventana(en.imu, t_ahora)
            self.l_fase.set_data(*vacio)
            if d is not None:
                ang = d[:, col]
                with np.errstate(all="ignore"):          # tiempos repetidos darian division por cero
                    vel = np.gradient(ang, d[:, 0])
                vel = np.nan_to_num(vel, nan=0.0, posinf=0.0, neginf=0.0)
                vel = np.convolve(vel, np.ones(5) / 5, mode="same")
                self.l_ang.set_data(d[:, 0], ang)
                self.l_vel.set_data(d[:, 0], vel)
                cambio |= self._ajustar(self.ax_ang, ang, minimo=10)
                cambio |= self._ajustar(self.ax_vel, vel, minimo=20)
        malas = f" | muestras: {en.n_imu} imu, {en.n_ext} ext | tramas malas: {en.tramas_malas}"
        if self.fuente == "externo":
            ok, fallas = en.lecturas
            malas += f" | sensor: {en.sensor} | lecturas: {ok} buenas, {fallas} fallidas"
        self.titulo.set_text(f"IMU: {self.fuente}{malas}")

        if cambio:
            self.canvas.draw_idle()          # la escala cambio: pintado completo (y nuevo fondo)
        else:
            self._pintar()

    def _crudo(self, t_ahora):
        d = self._ventana(self.enlace.crudo, t_ahora)
        if d is None:
            for l in self.l_grv + self.l_gyr:
                l.set_data([], [])
            self.txt_grv.set_text("sin datos crudos (PH_ST)")
            self.sub_gyr.set_text("")
            return False
        self.txt_grv.set_text("")
        for k in range(3):
            self.l_grv[k].set_data(d[:, 0], d[:, 1 + k])
            self.l_gyr[k].set_data(d[:, 0], d[:, 4 + k])
        cambio = self._ajustar(self.ax_grv, d[:, 1:4], minimo=1.0)
        cambio |= self._ajustar(self.ax_gyr, d[:, 4:7], minimo=20)
        eje_g = int(np.argmax(np.ptp(d[:, 1:4], axis=0)))
        eje_w = int(np.argmax(np.ptp(d[:, 4:7], axis=0)))
        self.sub_gyr.set_text(f"mayor variacion: gravedad {EJES[eje_g]}, giro {EJES[eje_w]}")
        self.sub_gyr.set_fontsize(9)
        return cambio

    def _pintar(self):
        """Blitting: restaura el fondo y dibuja solo las lineas y los textos que cambian."""
        self.canvas.restore_region(self.fondo)
        for ax, artistas in self.por_eje:
            for a in artistas:
                ax.draw_artist(a)
        self.fig.draw_artist(self.titulo)
        self.canvas.blit(self.fig.bbox)

    def correr(self):
        self.canvas.draw()
        self.timer.start()
        plt.show()


def conectar(puerto, baudios, ventana=10.0):
    print(f"Abriendo {puerto} a {baudios} baudios. La ESP32 se reinicia y calibra: deja la pata y el IMU quietos...")
    enlace = Enlace(puerto, baudios, ventana=ventana)
    if not enlace.esperar_texto("Serial Commands are ready", timeout=20):
        print("(no llego el mensaje de arranque; continuo de todos modos)")
    enlace.enviar("S_F IMU EXT")
    enlace.enviar("S_TIME")
    enlace.enviar("S_ON")
    time.sleep(0.5)
    return enlace


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--puerto", default="COM9", help="puerto serie de la ESP32 (por defecto COM9)")
    ap.add_argument("--baud", type=int, default=BAUDIOS)
    ap.add_argument("--fuente", choices=FUENTES, default="externo", help="IMU con el que empieza")
    ap.add_argument("--ventana", type=float, default=10.0, help="segundos visibles")
    args = ap.parse_args()

    enlace = conectar(args.puerto, args.baud, args.ventana)
    visor = Visor(enlace, args.fuente, args.ventana)
    try:
        visor.correr()
    finally:
        visor.timer.stop()
        enlace.cerrar()
        print("Puerto cerrado")


if __name__ == "__main__":
    main()
