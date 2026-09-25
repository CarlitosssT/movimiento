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

## Conexiones

| Dispositivo | SDA | SCL | Bus en el firmware |
|---|---|---|---|
| PCA9685 | 21 | 22 | `Wire` |
| IMU hip | 21 | 22 | `Wire` (el mismo de la PCA) |
| IMU knee | 18 | 19 | `Wire1` |
| IMU ankle | 16 | 17 | I2C por software (la ESP32 solo tiene 2 controladores I2C) |

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
| `S_F IMU`, `S_ON`, `S_OFF` | Streaming de los IMUs (paquetes binarios de Eris, 100 Hz) |
| `S_TIME` | Pone a cero el reloj de las muestras |
| `CAL` | Toma la pose actual como nuevo cero de los IMUs |
| `IMU` | Imprime los ángulos actuales en texto |
| `X ch ang` / `Y ch ang` | Mueve un servo de golpe / de forma suave |
| `T_CH c0 c1 ...` | Canales que usa la trayectoria (máx. 12) |
| `T_DT ms` | Periodo entre puntos (5 ms por defecto) |
| `P v0 v1 ...` | Agrega un punto a la cola, en décimas de grado (`900` = 90.0°) |
| `T_GO` / `T_END` / `T_STOP` | Arrancar / terminar cuando se vacíe la cola / parar ya |
| `T_ST` | Estado de la cola (reproducidos, en cola, underruns) |

El PC mantiene la cola de la ESP32 llena y la ESP32 reproduce los puntos con su propio reloj,
así que el ritmo no depende de los tiempos de Windows.

## Entorno de Python

El venv `wearable` ya está creado en esta carpeta. Para crearlo de nuevo:

```
py -3.12 -m venv wearable
wearable\Scripts\python -m pip install -r requirements.txt
wearable\Scripts\python -m ipykernel install --user --name wearable --display-name "Python (wearable)"
```

En VS Code, abre el notebook y elige el kernel **Python (wearable)**.

## Uso del notebook

1. Cambia `PUERTO` (por ejemplo `COM5`) y ejecuta la primera celda. Espera a que termine la calibración.
2. En la celda de trayectoria elige el archivo, `PIERNA = "fr"` o `"fl"` y `REPETICIONES`.
   Ajusta `signo` y `offset` en `MAPEO` según cómo montaste cada servo: `servo = offset + signo * ángulo`.
3. Ejecuta la celda de envío y luego la de gráfica en vivo. Teclas en la ventana:
   `t` reproducir, `x` detener, `i` ir al inicio, `c` nuevo cero de los IMUs.
4. Las muestras se guardan en `imus_servos_<fecha>.csv`. La celda de análisis compara cada IMU
   con su articulación de la trayectoria.
5. Cierra el puerto con la última celda antes de volver a subir el firmware.
