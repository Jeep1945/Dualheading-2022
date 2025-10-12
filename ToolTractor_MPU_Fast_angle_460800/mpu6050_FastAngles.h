#ifndef MPU6050_FASTANGLES_H
#define MPU6050_FASTANGLES_H

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#define MPU6050_ADDR 0x68

enum FilterType {
  COMPLEMENTARY,
  KALMAN
};

enum GyroScale {
    MPU_MODE_250,
    MPU_MODE_500,
    MPU_MODE_1000,
    MPU_MODE_2000
};

class mpu6050_FastAngles {
  public:
    mpu6050_FastAngles();
    void begin(GyroScale scale = MPU_MODE_500);
    void calibrateGyro(int calibrations = 1000);
    void saveCalibration();
    bool loadCalibration();
    void resetSensor();
    void zeroAngles();
    void invert(char axis);
    float getAngle(char axis, FilterType filter = COMPLEMENTARY);
    void setComplementaryFactor(float factor);
    void setKalmanQangle(float Q_angle);
    void setKalmanQbias(float Q_bias);
    void setKalmanRmeasure(float R_measure);
    void printSettings();
    void manualCalibration(int x, int y, int z);
    float gyroOffset;
  private:
    // Variáveis de armazenamento dos dados do sensor
    int16_t accX, accY, accZ;
    int16_t gyroX, gyroY, gyroZ;
    float gyroXOffset, gyroYOffset, gyroZOffset;
    float angleX, angleY, angleZ;
    float gyroRateX, gyroRateY, gyroRateZ;
    float accAngleX, accAngleY;
    float elapsedTime;
    unsigned long currentTime, previousTime;
    float accAngleCorrectionFactor;
    float gyroScaleFactor;
    uint8_t invertedX, invertedY, invertedZ;
    int gyroScale;

    // Variáveis do filtro de Kalman
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angleKF_X, biasKF_X;
    float P_KF_X[2][2];
    float angleKF_Y, biasKF_Y;
    float P_KF_Y[2][2];

    // Métodos privados
    void configureGyroScale(GyroScale scale = MPU_MODE_500);
    void readSensorData();
    float complementaryFilter(char axis);
    float kalmanFilter(char axis);
};

#endif
