#include "Eris.h"
#include "servo_commands.h"
#include "servos.h"
#include "trajectory.h"
#include "imu.h"

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

void DemoOn()  { if (!servosAvailable()) return; Trajectory::stop(); Servos::demoStart(); Serial.println("Demo on");  }
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
}

} // namespace SerialCom
