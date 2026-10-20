// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Work with the IMU sensor

#include <Wire.h>
#include <FlixPeriph.h>
#include "vector.h"
#include "lpf.h"
#include "util.h"
#include "quaternion.h"

//MPU6050 imu(Wire); // uncomment this if you are using MPU6050 instead of MPU9250
MPU9250 imu(Wire, -1, MPU9250::I2C_ADDR_SEC); // comment out this section if you are using MPU6050 instead of MPU9250
Vector imuRotation(0, 0, PI / 2); // imu orientation as Euler angles

Vector gyro; // gyroscope output, rad/s
Vector gyroBias;

Vector acc; // accelerometer output, m/s/s
Vector accBias;
Vector accScale(1, 1, 1);

Vector mag; // Magneteometer output, uT


// missing declaration fixes
void print(const char* format, ...);
void configureIMU();
bool calibrateGyro();
void calibrateAccelOnce();
void pause(float duration);
void printIMUCalibration();
extern float loopRate;
extern bool landed;
extern bool armed;
extern float motors[4];


void setupIMU() {
    print("Setup IMU\n");
    imu.begin();
    configureIMU();
}

void configureIMU() {
    imu.setAccelRange(imu.ACCEL_RANGE_4G);
    imu.setGyroRange(imu.GYRO_RANGE_2000DPS);
    imu.setDLPF(imu.DLPF_MAX);
    imu.setRate(imu.RATE_1KHZ_APPROX);
    imu.setupInterrupt();
}

void readIMU() {
    imu.waitForData();
    imu.getGyro(gyro.x, gyro.y, gyro.z);
    imu.getAccel(acc.x, acc.y, acc.z);
    imu.getMag(mag.x, mag.y, mag.z);

    // apply scale and bias
    acc = (acc - accBias) / accScale;
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

    print("Gyro calibration complete. Bias (rad/s): %.6f %.6f %.6f\n",
        gyroBias.x, gyroBias.y, gyroBias.z);

    print("Bias is applied immediately and stored by the parameter system.\n");

    return true;
}

void calibrateAccel() {
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

    printIMUCalibration();
    print("✓ Calibration done!\n");

    configureIMU();
}

void calibrateAccelOnce() {
    const int samples = 1000;

    static Vector accMax(-INFINITY, -INFINITY, -INFINITY);
    static Vector accMin(INFINITY, INFINITY, INFINITY);

    // Compute the average of the accelerometer readings
    acc = Vector(0, 0, 0);

    for (int i = 0; i < samples; i++) {
        imu.waitForData();

        Vector sample;
        imu.getAccel(sample.x, sample.y, sample.z);

        acc = acc + sample;
    }

    acc = acc / samples;

    // Update the maximum and minimum values
    if (acc.x > accMax.x) accMax.x = acc.x;
    if (acc.y > accMax.y) accMax.y = acc.y;
    if (acc.z > accMax.z) accMax.z = acc.z;

    if (acc.x < accMin.x) accMin.x = acc.x;
    if (acc.y < accMin.y) accMin.y = acc.y;
    if (acc.z < accMin.z) accMin.z = acc.z;

    // Compute scale and bias
    accScale = (accMax - accMin) / 2 / ONE_G;
    accBias = (accMax + accMin) / 2;
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
    imu.getMag(rawMag.x, rawMag.y, rawMag.z); // comment this line if you are using MPU6050 instead of MPU9250

    print("raw gyro: %f %f %f\n", rawGyro.x, rawGyro.y, rawGyro.z);
    print("raw acc: %f %f %f\n", rawAcc.x, rawAcc.y, rawAcc.z);
    print("raw mag: %f %f %f\n", rawMag.x, rawMag.y, rawMag.z); // comment this line if you are using MPU6050 instead of MPU9250
}