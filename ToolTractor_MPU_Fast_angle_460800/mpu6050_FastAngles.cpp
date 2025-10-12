#include "mpu6050_FastAngles.h"

// Construtor
mpu6050_FastAngles::mpu6050_FastAngles() {
  gyroXOffset = gyroYOffset = gyroZOffset = 0;
  angleX = angleY = angleZ = 0;
  accAngleCorrectionFactor = 0.98;
  gyroScaleFactor = 131.0; // Default  ±250°/s
  invertedX = invertedY = invertedZ = 0;

  // Start by changing the Kalman filter
  Q_angle = 0.001;
  Q_bias = 0.003;
  R_measure = 0.03;
  angleKF_X = biasKF_X = 0;
  P_KF_X[0][0] = P_KF_X[0][1] = P_KF_X[1][0] = P_KF_X[1][1] = 0;
  angleKF_Y = biasKF_Y = 0;
  P_KF_Y[0][0] = P_KF_Y[0][1] = P_KF_Y[1][0] = P_KF_Y[1][1] = 0;
}

void mpu6050_FastAngles::begin(GyroScale scale) {
    Wire.begin(32,33);

    // Inicializar o MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // Power Management Logger
    Wire.write(0x00); // Close MPU6050
    Wire.endTransmission(true);

    configureGyroScale(scale); // Set the gyroscope scale to the selected scale
    
    // Load calibration from EEPROM or calibrate if necessary
    if (!loadCalibration()) {
        calibrateGyro();
        saveCalibration();
    }

    // Initialize time
    currentTime = micros();
    previousTime = currentTime;
}


void mpu6050_FastAngles::setComplementaryFactor(float factor) {
  if (factor >= 0.0 && factor <= 1.0) {
    accAngleCorrectionFactor = factor;
  } else {
    Serial.println("Factor invalid. Must be between 0.0 and 1.0");
  }
}

void mpu6050_FastAngles::setKalmanQangle(float Q_angle) {
  this->Q_angle = Q_angle;
}

void mpu6050_FastAngles::setKalmanQbias(float Q_bias) {
  this->Q_bias = Q_bias;
}

void mpu6050_FastAngles::setKalmanRmeasure(float R_measure) {
  this->R_measure = R_measure;
}

void mpu6050_FastAngles::configureGyroScale(GyroScale scale) {
    uint8_t fs_sel;
    int gyroScale;
    switch (scale) {
        case MPU_MODE_250:
            fs_sel = 0;
            gyroScaleFactor = 131.0;
            gyroScale = 250;
            break;
        case MPU_MODE_500:
            fs_sel = 1;
            gyroScaleFactor = 65.5;
            gyroScale = 500;
            break;
        case MPU_MODE_1000:
            fs_sel = 2;
            gyroScaleFactor = 32.8;
            gyroScale = 1000;
            break;
        case MPU_MODE_2000:
            fs_sel = 3;
            gyroScaleFactor = 16.4;
            gyroScale = 2000;
            break;
        default:
            fs_sel = 0;
            gyroScaleFactor = 131.0;
            gyroScale = 250;
            break;
    }

    // Configure the gyroscope with the selected scale
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);  // Gyroscope configuration register
    Wire.write(fs_sel << 3);  // Configurator os bits FS_SEL [4:3]
    Wire.endTransmission(true);

    Serial.print("Gyroscope configured with scale: ±");
    Serial.print(gyroScale);
    Serial.println("°/s");
}


void mpu6050_FastAngles::printSettings() {
  Serial.println("Current MPU6050 settings:");
  Serial.print("Gyroscope configured with scale: ±");
  Serial.println("500°/s"); // Considering that we are using ±500°/s

  Serial.println("Complementary Filter Parameters:");
  Serial.print("Correction factor: ");
  Serial.println(accAngleCorrectionFactor);

  Serial.println("Kalman Filter Parameters:");
  Serial.print("Q_angle: "); Serial.println(Q_angle);
  Serial.print("Q_bias: "); Serial.println(Q_bias);
  Serial.print("R_measure: "); Serial.println(R_measure);

  Serial.println("Calibration Offsets:");
  Serial.print("gyroXOffset: "); Serial.println(gyroXOffset);
  Serial.print("gyroYOffset: "); Serial.println(gyroYOffset);
  Serial.print("gyroZOffset: "); Serial.println(gyroZOffset);
}

void mpu6050_FastAngles::calibrateGyro(int calibrations) {
  Serial.println("Calibrating sensor, do not move the MPU6050...");
  delay(2000); // Wait 2 seconds before starting calibration

  long gyroXTotal = 0, gyroYTotal = 0, gyroZTotal = 0;

  for (int i = 0; i < calibrations; i++) {
    readSensorData();

    gyroXTotal += gyroX;
    gyroYTotal += gyroY;
    gyroZTotal += gyroZ;
    delay(3);
  }

  gyroXOffset = gyroXTotal / calibrations;
  gyroYOffset = gyroYTotal / calibrations;
  gyroZOffset = gyroZTotal / calibrations;

  gyroOffset = gyroXOffset + gyroYOffset + gyroZOffset;

  Serial.println("Calibration completed.");
  Serial.print("Calibration Offsets - ");
  Serial.print("gyroXOffset: "); Serial.print(gyroXOffset);
  Serial.print(", gyroYOffset: "); Serial.print(gyroYOffset);
  Serial.print(", gyroZOffset: "); Serial.println(gyroZOffset);
}

void mpu6050_FastAngles::saveCalibration() {
  EEPROM.put(0, gyroXOffset);
  EEPROM.put(sizeof(gyroXOffset), gyroYOffset);
  EEPROM.put(2 * sizeof(gyroXOffset), gyroZOffset);
}

bool mpu6050_FastAngles::loadCalibration() {
  EEPROM.get(0, gyroXOffset);
  EEPROM.get(sizeof(gyroXOffset), gyroYOffset);
  EEPROM.get(2 * sizeof(gyroXOffset), gyroZOffset);

  if (isnan(gyroXOffset) || isnan(gyroYOffset) || isnan(gyroZOffset)) {
    return false;
  }
  return true;
}

void mpu6050_FastAngles::manualCalibration(int x, int y, int z) {
    gyroXOffset = x;
    gyroYOffset = y;
    gyroZOffset = z;

    Serial.println("Manual calibration set.");
    Serial.print("gyroXOffset: "); Serial.println(gyroXOffset);
    Serial.print("gyroYOffset: "); Serial.println(gyroYOffset);
    Serial.print("gyroZOffset: "); Serial.println(gyroZOffset);
}

void mpu6050_FastAngles::resetSensor() {
  // Reinicializar o MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x80); // Reset do dispositivo
  Wire.endTransmission(true);
  delay(100);

  // Reconfigurar após o reset
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  configureGyroScale();
}

void mpu6050_FastAngles::zeroAngles() {
  angleX = angleY = angleZ = 0;
}

void mpu6050_FastAngles::invert(char axis) {
  if (axis == 'X' || axis == 'x') invertedX = !invertedX;
  if (axis == 'Y' || axis == 'y') invertedY = !invertedY;
  if (axis == 'Z' || axis == 'z') invertedZ = !invertedZ;
}

void mpu6050_FastAngles::readSensorData() {
  // Ler dados do acelerômetro
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();

  // Ler dados do giroscópio
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

float mpu6050_FastAngles::complementaryFilter(char axis) {
  // Atualizar o tempo
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  readSensorData();

  // Calcular ângulo do acelerômetro
  accAngleX = atan2(accY, accZ) * 180 / PI;
  accAngleY = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  // Ajustar valores do giroscópio com os offsets
  gyroX -= gyroXOffset;
  gyroY -= gyroYOffset;
  gyroZ -= gyroZOffset;

  // Calcular velocidade angular
  gyroRateX = gyroX / gyroScaleFactor;
  gyroRateY = gyroY / gyroScaleFactor;
  gyroRateZ = gyroZ / gyroScaleFactor;

  // Integrar dados do giroscópio para obter o ângulo
  angleX += gyroRateX * elapsedTime;
  angleY += gyroRateY * elapsedTime;
  angleZ += gyroRateZ * elapsedTime;

  // Aplicar filtro complementar
  angleX = accAngleCorrectionFactor * angleX + (1.0 - accAngleCorrectionFactor) * accAngleX;
  angleY = accAngleCorrectionFactor * angleY + (1.0 - accAngleCorrectionFactor) * accAngleY;

  // Inverter eixos se necessário
  if (axis == 'X' || axis == 'x') return invertedX ? -angleX : angleX;
  if (axis == 'Y' || axis == 'y') return invertedY ? -angleY : angleY;
  if (axis == 'Z' || axis == 'z') return invertedZ ? -angleZ : angleZ;

  return 0;
}

float mpu6050_FastAngles::kalmanFilter(char axis) {
  // Atualizar o tempo
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  readSensorData();

  // Calcular ângulo do acelerômetro
  accAngleX = atan2(accY, accZ) * 180 / PI;
  accAngleY = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;

  // Ajustar valores do giroscópio com os offsets
  gyroX -= gyroXOffset;
  gyroY -= gyroYOffset;
  gyroZ -= gyroZOffset;

  // Calcular velocidade angular
  gyroRateX = gyroX / gyroScaleFactor;
  gyroRateY = gyroY / gyroScaleFactor;
  gyroRateZ = gyroZ / gyroScaleFactor;

  // Aplicar filtro de Kalman no eixo X
  if (axis == 'X' || axis == 'x') {
    // Predição
    angleKF_X += elapsedTime * (gyroRateX - biasKF_X);
    P_KF_X[0][0] += elapsedTime * (elapsedTime * P_KF_X[1][1] - P_KF_X[0][1] - P_KF_X[1][0] + Q_angle);
    P_KF_X[0][1] -= elapsedTime * P_KF_X[1][1];
    P_KF_X[1][0] -= elapsedTime * P_KF_X[1][1];
    P_KF_X[1][1] += Q_bias * elapsedTime;

    // Atualização
    float S = P_KF_X[0][0] + R_measure;
    float K_0 = P_KF_X[0][0] / S;
    float K_1 = P_KF_X[1][0] / S;
    float y = accAngleX - angleKF_X;
    angleKF_X += K_0 * y;
    biasKF_X += K_1 * y;
    float P00_temp = P_KF_X[0][0];
    float P01_temp = P_KF_X[0][1];
    P_KF_X[0][0] -= K_0 * P00_temp;
    P_KF_X[0][1] -= K_0 * P01_temp;
    P_KF_X[1][0] -= K_1 * P00_temp;
    P_KF_X[1][1] -= K_1 * P01_temp;

    return invertedX ? -angleKF_X : angleKF_X;
  }

  // Aplicar filtro de Kalman no eixo Y
  if (axis == 'Y' || axis == 'y') {
    // Predição
    angleKF_Y += elapsedTime * (gyroRateY - biasKF_Y);
    P_KF_Y[0][0] += elapsedTime * (elapsedTime * P_KF_Y[1][1] - P_KF_Y[0][1] - P_KF_Y[1][0] + Q_angle);
    P_KF_Y[0][1] -= elapsedTime * P_KF_Y[1][1];
    P_KF_Y[1][0] -= elapsedTime * P_KF_Y[1][1];
    P_KF_Y[1][1] += Q_bias * elapsedTime;

    // Atualização
    float S = P_KF_Y[0][0] + R_measure;
    float K_0 = P_KF_Y[0][0] / S;
    float K_1 = P_KF_Y[1][0] / S;
    float y = accAngleY - angleKF_Y;
    angleKF_Y += K_0 * y;
    biasKF_Y += K_1 * y;
    float P00_temp = P_KF_Y[0][0];
    float P01_temp = P_KF_Y[0][1];
    P_KF_Y[0][0] -= K_0 * P00_temp;
    P_KF_Y[0][1] -= K_0 * P01_temp;
    P_KF_Y[1][0] -= K_1 * P00_temp;
    P_KF_Y[1][1] -= K_1 * P01_temp;

    return invertedY ? -angleKF_Y : angleKF_Y;
  }

  // No eixo Z, utilizamos apenas o giroscópio
  angleZ += gyroRateZ * elapsedTime;
  return invertedZ ? -angleZ : angleZ;
}

float mpu6050_FastAngles::getAngle(char axis, FilterType filter) {
  if (filter == COMPLEMENTARY) {
    return complementaryFilter(axis);
  } else if (filter == KALMAN) {
    return kalmanFilter(axis);
  }
  return 0;
}
