//-------------------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------------------

void Heading_MPU6050V() {
  // calculate with MPU6050 a heading corridor


  if (millis() - lastReadTime >= readInterval) {
    lastReadTime = millis();

    // Lê os ângulos nos eixos X, Y e Z
    angleX = mpu.getAngle('X', COMPLEMENTARY);
    angleY = mpu.getAngle('Y', COMPLEMENTARY);
    angleZ = mpu.getAngle('Z', COMPLEMENTARY);
    angleZ *= - 1;
    angleZ += 360;
    while (angleZ > 360)
      angleZ -= 360;
    while (angleZ < 0)
      angleZ += 360;
    heading_MPU6050 = angleZ;
    if (debugmode)
    {
      Serial.print("Roll X: "); Serial.print(angleX);
      Serial.print("\Roll Y: "); Serial.print(angleY);
      Serial.print("\Compass Z: "); Serial.println(angleZ);
    }
    for (j = 1; j <= 9; j++) {
      heading_MPU6050_zuvor[j - 1] = heading_MPU6050_zuvor[j];
      if (debugmode)   Serial.print(" h: " + String(j) + ", " + String(heading_MPU6050_zuvor[j - 1]));
    }
    heading_MPU6050_zuvor[9] = heading_MPU6050;
    if  (abs(heading_MPU6050_zuvor[8] - heading_MPU6050_zuvor[9]) > 90) {   // d.h. geht nur bei Sprung 360-0
      if (debugmode) Serial.print(" h8+9: " + String(heading_MPU6050_zuvor[8]) + ", " + String(heading_MPU6050_zuvor[9]));
      if (heading_MPU6050_zuvor[8] > heading_MPU6050_zuvor[9])  heading_MPU6050_zuvor[9] += 360;
      else  heading_MPU6050_zuvor[9] -= 360;
    }
    heading_MPU6050_hzuvormin = 360;
    heading_MPU6050_hzuvormax = 0;
    for (i = 0; i < 9; i++) {
      if (heading_MPU6050_hzuvormin > heading_MPU6050_zuvor[i]) heading_MPU6050_hzuvormin = heading_MPU6050_zuvor[i];
      if (heading_MPU6050_hzuvormax < heading_MPU6050_zuvor[i]) heading_MPU6050_hzuvormax = heading_MPU6050_zuvor[i];
    }
    if (abs(heading_MPU6050_hzuvormax - heading_MPU6050_hzuvormin) > 180)
    {
      heading_MPU6050_hzuvormin += 360.2;
    }
    else
    {
      heading_MPU6050_hzuvormin -= 0.1;
      heading_MPU6050_hzuvormax += 0.1;
    }
    heading_MPU6050_corridor = abs(heading_MPU6050_hzuvormax - heading_MPU6050_hzuvormin);
    if (debugmode)
    {
      Serial.print("min : ");
      Serial.print(heading_MPU6050_hzuvormin);
      Serial.print("  max :  ");
      Serial.print(heading_MPU6050_hzuvormax);
      Serial.print("\tHeading UBX: ");
      Serial.println(heading);
      Serial.print("\tcorridor  : ");
      Serial.println(heading_MPU6050_corridor);
     //Serial.print("\tHeading UBX: ");
      // Serial.println(heading);
    }
  }
}

//-------------------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------------------

void Roll_MPU6050V() {
  //int IMU_MPU6050_direction = 1;       //Drivedirection Y  1:  -Y 2:  X 3:  -X 4:
  if (IMU_MPU6050_direction == 1) roll_MPU6050 = angleY;
  if (IMU_MPU6050_direction == 2) roll_MPU6050 = angleY * -1;
  if (IMU_MPU6050_direction == 3) roll_MPU6050 = angleX * -1;
  if (IMU_MPU6050_direction == 4) roll_MPU6050 = angleX;
}

//-------------------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------------------

void button_linemove() {
  // byte ABline_Direction[2] = {0,0};   // {heading, cm of movement}
  // Button_left_S, Button_middle_S, Button_right_S
  if ((ABline_Direction[0] == 0) || (Button_middle_S == "0")) {
    ABline_Direction[0] = heading;
    ABline_Direction[1] = 0;
  }

  if ((Button_left_S == "0") && (millis() - Button_delay40 > 1500)) {
    ABline_Direction[1] += 2;
    Button_delay40 = millis();
  }

  if ((Button_right_S == "0") && (millis() - Button_delay42 > 1500)) {
    ABline_Direction[1] -= 2;
    Button_delay42 = millis();
  }

  if ((abs(ABline_Direction[0] - heading) > 150) || (abs(ABline_Direction[0] - heading) < 50))
    ABline_Direction[2] = 1;
  else
    ABline_Direction[2] = -1;
}
// ###############################################################################
void RollHeading_MPU() {

  buttonState_left = digitalRead(Button_left);
  buttonState_middle = digitalRead(Button_middle);
  buttonState_right = digitalRead(Button_right);

}
