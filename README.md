# Cabra wearable: movimiento

Pata delantera de la cabra con 3 servos (hip, knee, ankle) controlados por una PCA9685, y un IMU
MPU6050 en cada eslabón. Una ESP32 reproduce trayectorias articulares desde un CSV y a la vez
envía los ángulos medidos por los IMUs al PC.

## Contenido

| Archivo | Qué es |
|---|---|
| `ErisServoIMU/` | Firmware de la ESP32, basado en el flavor [ErisServoDriver](https://github.com/JonathanCamargo/Eris) de Eris |
| `Cabra_servos_imus.ipynb` | Notebook para enviar trayectorias, ver los IMUs en vivo y comparar lo medido con lo comandado |
| `leg_actuator_angles_one_cycle.csv` | Un ciclo de marcha (127 puntos, 5 ms) |
| `leg_actuator_angles_trajectory_adapted.csv` | Trayectoria de 60 s (12065 puntos, 5 ms) |
| `imu_angulos_simple.ino`, `Cabra_wearable.ipynb` | Versión anterior: solo IMUs, salida en texto |
| `phase_match_sim.py`, `phase_match_live.py`, `plot_one_cycle.py` | Prototipo en Python del seguimiento de fase (la ESP32 corre la misma lógica) |
| `visor_imus.py` | Visor en vivo en una ventana propia: eliges el IMU (hip, knee, ankle o externo) y ves su orientación; para el externo también los valores crudos por eje |
| `tools/gen_phase_table.py` | Genera `ErisServoIMU/phase_table.h` (tabla de fases) a partir del CSV de un ciclo |

## Conexiones

| Dispositivo | SDA | SCL | Bus en el firmware |
|---|---|---|---|
| PCA9685 | 21 | 22 | `Wire` |
| IMU hip | 21 | 22 | `Wire` (el mismo de la PCA) |
| IMU knee | 18 | 19 | `Wire1` |
| IMU ankle | 16 | 17 | I2C por software (la ESP32 solo tiene 2 controladores I2C) |
| IMU externo (MPU6050, BMI160 o BNO055) | 25 | 26 | I2C por software (segundo bus por software) |

**IMU externo** (va pegado al cuerpo; entrega el pitch para el seguimiento de fase). Puede ser un **MPU6050** (el mismo de los demás IMUs), un **BMI160** o un **BNO055**: el firmware detecta cuál hay conectado. Va en su propio bus:

| Pin | MPU6050 | BMI160 | BNO055 | ESP32 |
|---|---|---|---|---|
| Alimentación | VCC | VIN | VIN | 3V3 |
| Tierra | GND | GND | GND | GND |
| Datos | SDA | SDA | SDA | **GPIO 25** |
| Reloj | SCL | SCL | SCL | **GPIO 26** |
| Dirección | AD0: GND = 0x68, 3V3 = 0x69 | SAO: GND = 0x68, 3V3 = 0x69 | ADR: GND = 0x28, 3V3 = 0x29 | (se detecta sola) |
| Modo I2C | | **CS a 3V3** (obligatorio) | PS0 y PS1 a GND | |
| Otros | INT sin conectar | INT1, INT2 sin conectar | RST, INT sin conectar | |

Con el BMI160 conviene conectar CS y SAO a un nivel definido (los pines al aire hacen que no responda o que cambie de dirección).
La ESP32 solo tiene dos controladores I2C por hardware y los usan los demás IMUs, así que el IMU externo va por un bus I2C por software
(bit-bang, ~100 kHz; `EXT_I2C_HALF_PERIOD_US` en `configuration.h`) que además respeta el clock stretching del BNO055 y espera a que SDA suba con pull-ups débiles.
- **MPU6050:** misma lectura que el IMU del ankle (14 bytes desde 0x3B, giroscopio ±1000 °/s, acelerómetro ±4 g); el firmware calcula el pitch con un filtro complementario y calibra el giroscopio al encender (**deja el sensor quieto ~2 s**). `PH_GYRO` lo recalibra.
- **BNO055:** modo IMUPLUS (acelerómetro + giroscopio con fusión interna, sin magnetómetro); calibra su giroscopio solo. Tarda cerca de 1 s en responder tras el encendido.
- **BMI160:** datos crudos; el firmware calcula el pitch con un filtro complementario y calibra el giroscopio al encender (**deja el sensor quieto ~2 s**). `PH_GYRO` lo recalibra.

Los módulos sin resistencias pull-up propias dependen de las internas de la ESP32 (~45 kΩ), que son débiles: si `PH_ST` muestra lecturas fallidas (`fails=`), pon una resistencia de 4.7 kΩ de SDA a 3V3 y otra de SCL a 3V3, o sube `EXT_I2C_HALF_PERIOD_US` a 5. Lo mismo si el cable al cuerpo es largo (más de unos 30 a 50 cm).
`I2C_SCAN` solo escanea los buses por hardware (`Wire` y `Wire1`); para el IMU externo usa `PH_ST` (`present=1 sensor=BNO055 addr=0x28`).

- Servos en los canales 0 (hip), 1 (knee) y 2 (ankle) de la PCA9685.
- La dirección de cada IMU (0x68 o 0x69, según AD0) se detecta al arrancar.
- Los servos (V+ de la PCA9685) necesitan su propia fuente, con la tierra unida a la de la ESP32.

Los pines se cambian en `ErisServoIMU/configuration.h`.

## Firmware

Necesita el core **esp32 3.3.x** de Arduino y estas librerías:

- `eriscommon` (de Eris)
- Adafruit PWM Servo Driver Library y Adafruit BusIO
- SerialCommand (Arduino-SerialCommand)
- PacketSerial

Abre `ErisServoIMU/ErisServoIMU.ino` en el Arduino IDE, elige la placa *ESP32 Dev Module* y súbelo.
También se puede compilar con arduino-cli:

```
arduino-cli compile --fqbn esp32:esp32:esp32 ErisServoIMU
```

**Al encender, deja la pata quieta en la pose neutra unos 3 s.** Esa pose queda como 0° de los IMUs.

### Comandos serie (921600 baudios, cada comando termina en `\n`)

| Comando | Acción |
|---|---|
| `S_F IMU EXT`, `S_ON`, `S_OFF` | Streaming (paquetes binarios de Eris, 100 Hz): `IMU` = hip/knee/ankle, `EXT` = IMU externo y fase estimada |
| `S_TIME` | Pone a cero el reloj de las muestras |
| `CAL` | Toma la pose actual como nuevo cero de los IMUs |
| `IMU` | Imprime los ángulos actuales en texto |
| `X ch ang` / `Y ch ang` | Mueve un servo de golpe / de forma suave |
| `T_CH c0 c1 ...` | Canales que usa la trayectoria (máx. 12) |
| `T_DT ms` | Periodo entre puntos (5 ms por defecto) |
| `P v0 v1 ...` | Agrega un punto a la cola, en décimas de grado (`900` = 90.0°) |
| `T_GO` / `T_END` / `T_STOP` | Arrancar / terminar cuando se vacíe la cola / parar ya |
| `T_ST` | Estado de la cola (reproducidos, en cola, underruns) |
| `I2C_SCAN` | Lista las direcciones que responden en `Wire` y `Wire1` (no escanea los buses por software) |
| `PH_ON` / `PH_OFF` | Los servos siguen la fase estimada con el IMU externo / dejan de seguirla |
| `PH_CFG c0 s0 o0 c1 s1 o1 c2 s2 o2` | Canal, signo y offset (décimas de grado) de hip, knee y ankle: `servo = offset + signo * ángulo` |
| `PH_IMU signo offset` | Sentido del pitch del IMU externo y pitch del pie (en la tabla) en la pose calibrada, que es lo que se suma al pitch que ves (0° en esa pose) |
| `PH_W wt wo wa` | Pesos de la búsqueda (theta, omega, alpha); `1 1 0` por defecto |
| `PH_MODE n` | Búsqueda en ventana (0), global (1) o con reenganche (2, por defecto) |
| `PH_GYRO` | MPU6050 / BMI160: recalibra el sesgo del giroscopio (sensor quieto) |
| `PH_ZERO` | Toma la pose actual del IMU externo como 0° del pitch |
| `PH_ST` | Estado del seguimiento (sensor detectado, calibración del BNO055, pitch, fase, hip/knee/ankle elegidos) |

El PC mantiene la cola de la ESP32 llena y la ESP32 reproduce los puntos con su propio reloj,
así que el ritmo no depende de los tiempos de Windows.

## Seguimiento de fase con el IMU externo

Misma lógica que `phase_match_sim.py`, ejecutada en la ESP32 a 100 Hz (sin pasar por el USB):

1. `tools/gen_phase_table.py` ajusta 4 armónicos a una pata (`fr`) del CSV y guarda en `phase_table.h` 508 fases del ciclo,
   cada una con el pitch del pie (hip + knee + ankle − 180), su velocidad y aceleración, y los ángulos hip/knee/ankle.
2. El firmware toma del sensor la gravedad y el giroscopio (BNO055: fusión interna; MPU6050 y BMI160: filtro complementario propio), calcula el pitch, lo filtra a 20 Hz y deriva la velocidad y la aceleración.
3. Cada muestra se compara con la tabla en una ventana alrededor de la fase anterior; si otra fase explica mucho mejor la señal durante 5 muestras seguidas, salta a ella.
4. Con `PH_ON`, el hip/knee/ankle de la fase elegida se manda a los servos. Encenderlo detiene la trayectoria, y `T_GO` lo apaga.

Si cambias el CSV, regenera la tabla y vuelve a subir el firmware:

```
wearable2\Scripts\python tools\gen_phase_table.py
```

Montaje: el pitch es el giro alrededor del eje `EXT_AXIS_W` de la placa (ahora **X**), que debe ser perpendicular al plano en que se mueve el segmento
(para girar alrededor de otro eje cambia `EXT_AXIS_U/V/W` en `configuration.h`: Z = 0,1,2; X = 1,2,0; Y = 2,0,1; si la velocidad sale con signo contrario, `EXT_GYRO_SIGN`). El montaje en el cuerpo no coincide con el del pie en el CSV:
ajusta `IMU_SIGNO` (por defecto -1, igual que `EXT_PITCH_SIGN` en `configuration.h`) y `IMU_OFFSET` en el notebook (`alinear_offset()` lo hace con unos segundos de marcha).

## Visor de IMUs (`visor_imus.py`)

Para comprobar qué eje toma cada IMU. Cierra antes el notebook (el puerto serie solo lo abre un programa):

```
wearable2\Scripts\python visor_imus.py --puerto COM8
```

- Botones de la izquierda: `hip`, `knee`, `ankle` (ángulo de cada MPU6050 y su derivada) o `externo` (pitch, velocidad y fase).
- Con `externo` también muestra la **gravedad y el giro crudos en X, Y y Z** (se piden con `PH_ST`, 20 veces por segundo) y escribe en el título cuál eje varía más.
  Si giras el sensor y reacciona un eje distinto de `EXT_AXIS_W` (Z por defecto), cambia `EXT_AXIS_U/V/W` en `configuration.h`.
- Teclas: `c` calibra (CAL, PH_GYRO, PH_ZERO), espacio pausa, `q` sale. Opciones: `--fuente externo`, `--ventana 15`.

## Entorno de Python

El venv `wearable2` ya está creado en esta carpeta (el kernel de Jupyter se llama **Python (wearable)**). Para crearlo de nuevo:

```
py -3.12 -m venv wearable2
wearable2\Scripts\python -m pip install -r requirements.txt
wearable2\Scripts\python -m ipykernel install --user --name wearable --display-name "Python (wearable)"
```

En VS Code, abre el notebook y elige el kernel **Python (wearable)**.

## Uso del notebook

1. Cambia `PUERTO` (por ejemplo `COM5`) y ejecuta la primera celda. Espera a que termine la calibración.
2. En la celda de trayectoria elige el archivo, `PIERNA = "fr"` o `"fl"` y `REPETICIONES`.
   Ajusta `signo` y `offset` en `MAPEO` según cómo montaste cada servo: `servo = offset + signo * ángulo`.
3. Ejecuta la celda de envío y luego la de gráfica en vivo. Teclas en la ventana:
   `t` reproducir, `x` detener, `i` ir al inicio, `c` nuevo cero de los IMUs y del IMU externo (pose actual = 0°), `r` todos los servos a 90°,
   `p` / `o` encender / apagar el seguimiento de fase del IMU externo.
4. Las muestras de los IMUs se guardan en `imus_servos_<fecha>.csv`. Los datos del IMU externo solo se grafican, no se guardan. La celda de análisis compara cada IMU
   con su articulación de la trayectoria.
5. Cierra el puerto con la última celda antes de volver a subir el firmware.
