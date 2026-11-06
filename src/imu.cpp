// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Work with the IMU sensor

#include <Wire.h>
#include <FlixPeriph.h>
#include "vector.h"
#include "lpf.h"
#include "util.h"
#include "quaternion.h"

MPU6050 imu(Wire); // uncomment this if you are using MPU6050 instead of MPU9250
//MPU9250 imu(Wire, -1, MPU9250::I2C_ADDR_SEC); // comment out this section if you are using MPU6050 instead of MPU9250
Vector imuRotation(0, 0, PI / 2); // imu orientation as Euler angles

Vector gyro; // gyroscope output, rad/s
Vector gyroBias;

Vector acc; // accelerometer output, m/s/s
Vector accBias;
Vector accScale(1, 1, 1);

//Vector mag; // Magneteometer output, uT


// missing declaration fixes
void print(const char* format, ...);
bool configureIMU();
bool calibrateGyro();
void calibrateAccelOnce();
void pause(float duration);
void printIMUCalibration();
extern float loopRate;
extern bool landed;
extern bool armed;
extern float motors[4];
bool imuHealthy = false;

// ---- Gyro bias / arming interlock ----
// Accelerometer calibration (accBias/accScale) is done once with 'ca' and stored in flash.
// Gyro bias is NOT stored: it is measured at every boot and re-measured in the background
// whenever the drone is disarmed and still, because it drifts with temperature.
extern float dt;
bool gyroReady = false;                       // a valid gyro bias has been measured
static uint32_t disarmedAt = 0;               // millis() when the drone was last disarmed (or booted)
const uint32_t REARM_DELAY_MS = 3000;         // minimum wait after disarm before arming is allowed
bool canArm();



void setupIMU() {
    imuHealthy = false;

    print("Setup IMU\n");

    if (!imu.begin()) {
        print("ERROR: IMU initialization failed\n");
        return;
    }

    if (!configureIMU()) {
        print("ERROR: IMU configuration failed\n");
        return;
    }

    if (imu.status() != 0) {
        print("ERROR: IMU reports status %d\n", imu.status());
        return;
    }

    imuHealthy = true;
    print("IMU initialized and configured successfully\n");

    // Measure gyro bias at every boot (drone must be still). Retries a few times.
    gyroReady = false;
    for (int attempt = 0; attempt < 10 && !gyroReady; attempt++) {
        calibrateGyro();
    }
    if (!gyroReady) {
        print("WARNING: gyro bias not measured yet. Keep the drone still; arming stays blocked until it succeeds.\n");
    }
    disarmedAt = millis();
}

bool configureIMU() {
    bool ok = true;

    ok = imu.setAccelRange(imu.ACCEL_RANGE_4G) && ok;
    ok = imu.setGyroRange(imu.GYRO_RANGE_2000DPS) && ok;
    ok = imu.setDLPF(imu.DLPF_MAX) && ok;
    ok = imu.setRate(imu.RATE_1KHZ_APPROX) && ok;
    ok = imu.setupInterrupt() && ok;

    return ok;
}


// Remember when the drone was last disarmed (any path: stick, CLI, MAVLink, failsafe).
static void trackArmState() {
    static bool wasArmed = false;
    if (wasArmed && !armed) disarmedAt = millis();
    wasArmed = armed;
}

// Arming is allowed only if the IMU works, a gyro bias exists, and 3 s passed since disarm/boot.
bool canArm() {
    return imuHealthy && gyroReady && (millis() - disarmedAt) >= REARM_DELAY_MS;
}

// Background gyro bias update. Collects ~1 s of raw gyro samples while the drone is disarmed
// and still; if the batch is quiet it becomes the new bias. Non-blocking, runs every loop.
static void updateGyroBias(const Vector& rawGyro) {
    static Vector sum, sumSq;
    static int n = 0;
    static float windowTime = 0;

    const float windowLength = 1.0f;   // s
    const float maxStdDev = 0.02f;     // rad/s, same limit as calibrateGyro()
    const float maxBiasJump = 0.2f;    // rad/s, reject absurd changes

    bool quiet = !armed &&
        motors[0] == 0 && motors[1] == 0 && motors[2] == 0 && motors[3] == 0 &&
        (millis() - disarmedAt) >= 300 &&                    // let motors spin down
        fabsf(acc.norm() - ONE_G) < ONE_G * 0.05f;           // not being moved around

    if (!quiet) {
        sum = Vector(0, 0, 0); sumSq = Vector(0, 0, 0); n = 0; windowTime = 0;
        return;
    }

    sum = sum + rawGyro;
    sumSq.x += rawGyro.x * rawGyro.x;
    sumSq.y += rawGyro.y * rawGyro.y;
    sumSq.z += rawGyro.z * rawGyro.z;
    n++;
    windowTime += constrain(dt, 0.0f, 0.1f);

    if (windowTime < windowLength || n < 100) return;

    Vector mean = sum / n;
    Vector stddev(
        sqrtf(max(0.0f, sumSq.x / n - mean.x * mean.x)),
        sqrtf(max(0.0f, sumSq.y / n - mean.y * mean.y)),
        sqrtf(max(0.0f, sumSq.z / n - mean.z * mean.z)));

    bool steady = stddev.x < maxStdDev && stddev.y < maxStdDev && stddev.z < maxStdDev;
    bool plausible = !gyroReady || (mean - gyroBias).norm() < maxBiasJump;

    if (steady && plausible) {
        gyroBias = mean;
        gyroReady = true;
    }

    sum = Vector(0, 0, 0); sumSq = Vector(0, 0, 0); n = 0; windowTime = 0;
}

void readIMU() {
    imu.waitForData();
    imu.getGyro(gyro.x, gyro.y, gyro.z);
    imu.getAccel(acc.x, acc.y, acc.z);
    // imu.getMag(mag.x, mag.y, mag.z); // comment this line if you are using MPU6050 instead of MPU9250

    trackArmState();
    Vector rawGyro = gyro; // gyro before bias correction

    // apply scale and bias
    acc = (acc - accBias) / accScale;
    updateGyroBias(rawGyro);
    gyro = gyro - gyroBias;

    // rotate to body frame
    Quaternion rotation = Quaternion::fromEuler(imuRotation);
    acc = Quaternion::rotateVector(acc, rotation.inversed());
    gyro = Quaternion::rotateVector(gyro, rotation.inversed());
}

// Estimate gyro bias from a stationary batch. The bias is in the IMU sensor
// frame, matching the raw gyro values before the board-orientation rotation.
bool calibrateGyro() {
    if (armed || motors[0] != 0 || motors[1] != 0 ||
        motors[2] != 0 || motors[3] != 0) {
        print("Gyro calibration refused: disarm and ensure all motors are stopped.\n");
        return false;
    }

    const int samples = 1500;
    Vector sum(0, 0, 0);
    Vector sumSquares(0, 0, 0);

    print("Gyro calibration: keep the drone completely still for about 2 seconds...\n");

    for (int i = 0; i < samples; ++i) {
        imu.waitForData();

        Vector sample;
        imu.getGyro(sample.x, sample.y, sample.z);

        sum = sum + sample;

        sumSquares.x += sample.x * sample.x;
        sumSquares.y += sample.y * sample.y;
        sumSquares.z += sample.z * sample.z;
    }

    Vector mean = sum / samples;

    Vector variance(
        sumSquares.x / samples - mean.x * mean.x,
        sumSquares.y / samples - mean.y * mean.y,
        sumSquares.z / samples - mean.z * mean.z
    );

    variance.x = max(0.0f, variance.x);
    variance.y = max(0.0f, variance.y);
    variance.z = max(0.0f, variance.z);

    Vector stddev(
        sqrtf(variance.x),
        sqrtf(variance.y),
        sqrtf(variance.z)
    );

    // Reject a batch that looks like movement/vibration rather than a stationary bias.
    const float maxStdDev = 0.02f; // rad/s, assuming FlixPeriph reports gyro in rad/s

    if (stddev.x > maxStdDev ||
        stddev.y > maxStdDev ||
        stddev.z > maxStdDev) {

        print("Gyro calibration failed: drone moved or vibrated. Std dev (rad/s): %.5f %.5f %.5f\n",
            stddev.x, stddev.y, stddev.z);

        print("Place it on a firm surface and run cg again. Existing bias was retained.\n");
        return false;
    }

    gyroBias = mean;
    gyroReady = true;

    print("Gyro calibration complete. Bias (rad/s): %.6f %.6f %.6f\n",
        gyroBias.x, gyroBias.y, gyroBias.z);

    print("Bias is applied immediately. It is not saved: it is measured again at every boot.\n");

    return true;
}

// Extremes seen on each axis during accel calibration (reset at the start of every run)
static Vector accMax, accMin;

void calibrateAccel() {
    if (armed || motors[0] != 0 || motors[1] != 0 || motors[2] != 0 || motors[3] != 0) {
        print("Accel calibration refused: disarm and stop the motors first.\n");
        return;
    }

    accMax = Vector(-INFINITY, -INFINITY, -INFINITY);
    accMin = Vector(INFINITY, INFINITY, INFINITY);

    print("Calibrating accelerometer\n");
    imu.setAccelRange(imu.ACCEL_RANGE_2G); // the most sensitive mode

    print("1/6 Place level [8 sec]\n");
    pause(8);
    calibrateAccelOnce();

    print("2/6 Place nose up [8 sec]\n");
    pause(8);
    calibrateAccelOnce();

    print("3/6 Place nose down [8 sec]\n");
    pause(8);
    calibrateAccelOnce();

    print("4/6 Place on right side [8 sec]\n");
    pause(8);
    calibrateAccelOnce();

    print("5/6 Place on left side [8 sec]\n");
    pause(8);
    calibrateAccelOnce();

    print("6/6 Place upside down [8 sec]\n");
    pause(8);
    calibrateAccelOnce();

    configureIMU(); // restore the normal measurement range

    // Compute scale and bias only now, after all six positions
    Vector newScale = (accMax - accMin) / 2 / ONE_G;
    Vector newBias = (accMax + accMin) / 2;

    bool sane = true;
    for (int i = 0; i < 3; i++) {
        float sc = i == 0 ? newScale.x : i == 1 ? newScale.y : newScale.z;
        float bi = i == 0 ? newBias.x : i == 1 ? newBias.y : newBias.z;
        if (!(sc >= 0.8f && sc <= 1.2f) || !(fabsf(bi) < 3.0f)) sane = false;
    }

    if (!sane) {
        print("Calibration FAILED (scale %f %f %f, bias %f %f %f). Old values kept.\n",
            newScale.x, newScale.y, newScale.z, newBias.x, newBias.y, newBias.z);
        print("Make sure all six faces were held level and still, then run ca again.\n");
        return;
    }

    accScale = newScale;
    accBias = newBias;

    printIMUCalibration();
    print("✓ Calibration done! Values are saved to flash and reused on every boot.\n");
}

// Average the accelerometer in the current position and update the per-axis extremes
void calibrateAccelOnce() {
    const int samples = 1000;
    Vector mean(0, 0, 0);

    for (int i = 0; i < samples; i++) {
        imu.waitForData();

        Vector sample;
        imu.getAccel(sample.x, sample.y, sample.z);

        mean = mean + sample;
    }

    mean = mean / samples;

    if (mean.x > accMax.x) accMax.x = mean.x;
    if (mean.y > accMax.y) accMax.y = mean.y;
    if (mean.z > accMax.z) accMax.z = mean.z;

    if (mean.x < accMin.x) accMin.x = mean.x;
    if (mean.y < accMin.y) accMin.y = mean.y;
    if (mean.z < accMin.z) accMin.z = mean.z;
}

void printIMUCalibration() {
    print("gyro bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
    print("accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
    print("accel scale: %f %f %f\n", accScale.x, accScale.y, accScale.z);
}

void printIMUInfo() {
    imu.status() ? print("status: ERROR %d\n", imu.status()) : print("status: OK\n");

    print("model: %s\n", imu.getModel());
    print("who am I: 0x%02X\n", imu.whoAmI());
    print("rate: %.0f\n", loopRate);

    print("gyro: %f %f %f\n", gyro.x, gyro.y, gyro.z);
    print("acc: %f %f %f\n", acc.x, acc.y, acc.z);

    imu.waitForData();

    Vector rawGyro, rawAcc, rawMag;

    imu.getGyro(rawGyro.x, rawGyro.y, rawGyro.z);
    imu.getAccel(rawAcc.x, rawAcc.y, rawAcc.z);
    // imu.getMag(rawMag.x, rawMag.y, rawMag.z); // comment this line if you are using MPU6050 instead of MPU9250

    print("raw gyro: %f %f %f\n", rawGyro.x, rawGyro.y, rawGyro.z);
    print("raw acc: %f %f %f\n", rawAcc.x, rawAcc.y, rawAcc.z);
    print("raw mag: %f %f %f\n", rawMag.x, rawMag.y, rawMag.z); // comment this line if you are using MPU6050 instead of MPU9250
}