#include "Eris.h"
#include "servo_commands.h"
#include "servos.h"
#include "trajectory.h"
#include "imu.h"
#include "external_imu.h"

#include <Wire.h>

namespace SerialCom {

// Every servo command is a no-op when the driver never came up, so say so
// instead of acking a move that cannot happen.
static bool servosAvailable() {
    if (Servos::ready()) return true;
    Serial.println("Error: PCA9685 not available");
    return false;
}

void ServoMove() {
    if (!servosAvailable()) return;
    // X <ch> <angle>  OR  X <a0> <a1> ... <a15>
    char *arg;
    float args[NUM_SERVOS];
    int nargs = 0;

    arg = sCmd.next();
    while (arg != NULL && nargs < NUM_SERVOS) {
        args[nargs++] = atof(arg);
        arg = sCmd.next();
    }

    if (nargs == 2) {
        uint8_t channel = (uint8_t)args[0];
        float angle = args[1];
        Servos::move(channel, angle);
        Serial.print("Servo ");
        Serial.print(channel);
        Serial.print(" -> ");
        Serial.println(angle);
    } else if (nargs == NUM_SERVOS) {
        Servos::moveAll(args);
        Serial.println("All servos moved");
    } else {
        Serial.print("Error: expected 2 or ");
        Serial.print(NUM_SERVOS);
        Serial.println(" arguments");
    }
}

void ServoSmoothMove() {
    if (!servosAvailable()) return;
    // Y <ch> <angle>  OR  Y <a0> <a1> ... <a15>
    char *arg;
    float args[NUM_SERVOS];
    int nargs = 0;

    arg = sCmd.next();
    while (arg != NULL && nargs < NUM_SERVOS) {
        args[nargs++] = atof(arg);
        arg = sCmd.next();
    }

    if (nargs == 2) {
        uint8_t channel = (uint8_t)args[0];
        float angle = args[1];
        Servos::smoothMove(channel, angle);
        Serial.print("Smooth servo ");
        Serial.print(channel);
        Serial.print(" -> ");
        Serial.println(angle);
    } else if (nargs == NUM_SERVOS) {
        Servos::smoothMoveAll(args);
        Serial.println("All servos smooth move");
    } else {
        Serial.print("Error: expected 2 or ");
        Serial.print(NUM_SERVOS);
        Serial.println(" arguments");
    }
}

void DemoOn()  { if (!servosAvailable()) return; Trajectory::stop(); ExtIMU::enable(false); Servos::demoStart(); Serial.println("Demo on");  }
void DemoOff() { Servos::demoStop();  Serial.println("Demo off"); }

/*************** Trayectorias ***************/

// T_CH <c0> <c1> ... : canales PCA9685 a los que va cada columna de P (máx. TRAJ_MAX_CH)
void TrajChannels() {
    uint8_t ch[TRAJ_MAX_CH];
    uint8_t n = 0;
    char *arg = sCmd.next();
    while (arg != NULL && n < TRAJ_MAX_CH) {
        ch[n++] = (uint8_t)atoi(arg);
        arg = sCmd.next();
    }
    if (arg != NULL || !Trajectory::setChannels(ch, n)) {
        Serial.print("Error: T_CH expects 1..");
        Serial.print(TRAJ_MAX_CH);
        Serial.println(" channels < NUM_SERVOS");
        return;
    }
    Serial.print("T_CH ok n=");
    Serial.println(n);
}

// T_DT <ms> : periodo entre puntos (acepta decimales)
void TrajPeriod() {
    char *arg = sCmd.next();
    if (arg == NULL) { Serial.println("Error: T_DT <ms>"); return; }
    Trajectory::setPeriodUs((uint32_t)(atof(arg) * 1000.0f + 0.5f));
    Serial.println("T_DT ok");
}

// P <v0> <v1> ... : un punto, en décimas de grado de servo (900 = 90.0°).
// No responde si todo va bien, para no saturar el enlace.
void TrajPoint() {
    int16_t v[TRAJ_MAX_CH];
    uint8_t n = 0;
    char *arg = sCmd.next();
    while (arg != NULL && n < TRAJ_MAX_CH) {
        v[n++] = (int16_t)atoi(arg);
        arg = sCmd.next();
    }
    if (!Trajectory::push(v, n)) {
        Serial.println("Error: P rejected (queue full or wrong channel count)");
    }
}

void TrajGo()     { if (!servosAvailable()) return; Trajectory::go(); Serial.println("T_GO ok"); }
void TrajEnd()    { Trajectory::end();  Serial.println("T_END ok"); }
void TrajStop()   { Trajectory::stop(); Serial.println("T_STOP ok"); }
void TrajStatus() { Trajectory::printStatus(); }

/*************** IMUs ***************/

void ImuZero() { IMU::requestZero(); }

void ImuPrint() {
    float a[3];
    IMU::getAngles(a);
    Serial.printf("hip:%.2f,knee:%.2f,ankle:%.2f\n", a[0], a[1], a[2]);
}

// I2C_SCAN : direcciones que responden en Wire y Wire1 (los buses por software, ankle y BNO055, no se escanean:
// los usan sus hilos y no se pueden compartir)
// La línea se arma completa y se envía de una vez: con el streaming activo, varios
// Serial.print seguidos se mezclan con los paquetes binarios y la línea llega rota.
static void scanBus(TwoWire& bus, const char* name) {
    char line[96];
    int n = snprintf(line, sizeof(line), "I2C %s:", name);
    for (uint8_t a = 1; a < 127; a++) {
        bus.beginTransmission(a);
        if (bus.endTransmission() == 0 && n < (int)sizeof(line) - 6) {
            n += snprintf(line + n, sizeof(line) - n, " 0x%02X", a);
        }
    }
    Serial.println(line);
}

void I2cScan() {
    scanBus(Wire,  "Wire  (SDA 21, SCL 22: PCA9685 + hip)");
    scanBus(Wire1, "Wire1 (SDA 18, SCL 19: knee)");
}

/*************** IMU externo y fase ***************/

static bool extAvailable() {
    if (ExtIMU::ready()) return true;
    Serial.println("Error: IMU externo no disponible");
    return false;
}

void PhOn() {
    if (!servosAvailable() || !extAvailable()) return;
    ExtIMU::enable(true);
    Serial.println("PH_ON ok");
}

void PhOff() { ExtIMU::enable(false); Serial.println("PH_OFF ok"); }

// PH_CFG <ch0> <sg0> <of0> <ch1> <sg1> <of1> <ch2> <sg2> <of2> : servo = of/10 + sg * articulación
// (en orden hip, knee, ankle; offset en décimas de grado)
void PhConfig() {
    float v[9];
    int n = 0;
    char *arg = sCmd.next();
    while (arg != NULL && n < 9) { v[n++] = atof(arg); arg = sCmd.next(); }
    if (n != 9 || arg != NULL) { Serial.println("Error: PH_CFG expects 9 values"); return; }
    uint8_t ch[3]; int8_t sg[3]; float of[3];
    for (int k = 0; k < 3; k++) {
        ch[k] = (uint8_t)v[3*k];
        sg[k] = v[3*k + 1] < 0 ? -1 : 1;
        of[k] = v[3*k + 2] / 10.0f;
    }
    ExtIMU::setMapping(ch, sg, of);
    Serial.println("PH_CFG ok");
}

// PH_IMU <signo> <offset°> : sentido del pitch y pitch del pie (en la tabla) en la pose calibrada
void PhImu() {
    char *a = sCmd.next(), *b = sCmd.next();
    if (a == NULL || b == NULL) { Serial.println("Error: PH_IMU <sign> <offset>"); return; }
    ExtIMU::setImu(atof(a) < 0 ? -1 : 1, atof(b));
    Serial.println("PH_IMU ok");
}

// PH_W <theta> <omega> <alpha> : pesos de la búsqueda (1 1 0 por defecto)
void PhWeights() {
    char *a = sCmd.next(), *b = sCmd.next(), *c = sCmd.next();
    if (a == NULL || b == NULL || c == NULL) { Serial.println("Error: PH_W <wt> <wo> <wa>"); return; }
    ExtIMU::setWeights(atof(a), atof(b), atof(c));
    Serial.println("PH_W ok");
}

// PH_MODE <0 ventana | 1 global | 2 recover>
void PhMode() {
    char *a = sCmd.next();
    int m = a ? atoi(a) : -1;
    if (m < 0 || m > 2) { Serial.println("Error: PH_MODE 0|1|2"); return; }
    ExtIMU::setMode((PhaseMatcher::Mode)m);
    Serial.println("PH_MODE ok");
}

void PhZero()   { if (!extAvailable()) return; ExtIMU::requestZero(); Serial.println("PH_ZERO ok"); }
void PhGyro()   { if (!extAvailable()) return; ExtIMU::requestGyroCal(); }
void PhStatus() { ExtIMU::printStatus(); }

void registerCommands(SerialCommand& sCmd) {
    sCmd.addCommand("X", ServoMove);
    sCmd.addCommand("Y", ServoSmoothMove);
    sCmd.addCommand("DEMO_ON",  DemoOn);
    sCmd.addCommand("DEMO_OFF", DemoOff);

    sCmd.addCommand("T_CH",   TrajChannels);
    sCmd.addCommand("T_DT",   TrajPeriod);
    sCmd.addCommand("P",      TrajPoint);
    sCmd.addCommand("T_GO",   TrajGo);
    sCmd.addCommand("T_END",  TrajEnd);
    sCmd.addCommand("T_STOP", TrajStop);
    sCmd.addCommand("T_ST",   TrajStatus);

    sCmd.addCommand("CAL", ImuZero);
    sCmd.addCommand("IMU", ImuPrint);
    sCmd.addCommand("I2C_SCAN", I2cScan);

    sCmd.addCommand("PH_ON",   PhOn);
    sCmd.addCommand("PH_OFF",  PhOff);
    sCmd.addCommand("PH_CFG",  PhConfig);
    sCmd.addCommand("PH_IMU",  PhImu);
    sCmd.addCommand("PH_W",    PhWeights);
    sCmd.addCommand("PH_MODE", PhMode);
    sCmd.addCommand("PH_GYRO", PhGyro);
    sCmd.addCommand("PH_ZERO", PhZero);
    sCmd.addCommand("PH_ST",   PhStatus);
}

} // namespace SerialCom
