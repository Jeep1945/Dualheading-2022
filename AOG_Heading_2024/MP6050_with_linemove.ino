//-------------------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------------------

void Heading_MPU6050() {
  heading_MPU6050 = MPU_Heading_S.toFloat();
  heading_MPU6050_drift += 0.000488;
  if (heading_MPU6050_drift > 360) heading_MPU6050_drift -= 360;
  heading_MPU6050 += heading_MPU6050_drift;
  if (heading_MPU6050 < 0) heading_MPU6050 += 360;
  if (heading_MPU6050 > 360) heading_MPU6050 -= 360;

}

//-------------------------------------------------------------------------------------------------//-------------------------------------------------------------------------------------------------

void Roll_MPU6050() {
  //int IMU_MPU6050_direction = 1;       //Drivedirection Y  1:  -Y 2:  X 3:  -X 4:
  if (IMU_MPU6050_direction == 1) roll_MPU6050 = MPU_Roll_S.toFloat();
  if (IMU_MPU6050_direction == 2) roll_MPU6050 = MPU_Roll_S.toFloat() * -1;
  if (IMU_MPU6050_direction == 3) roll_MPU6050 = MPU_Yaw_S.toFloat() * -1;
  if (IMU_MPU6050_direction == 4) roll_MPU6050 = MPU_Yaw_S.toFloat();
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
