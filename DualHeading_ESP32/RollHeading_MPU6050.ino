void RollHeading_MPU() {

  buttonState_left = digitalRead(Button_left);
  buttonState_middle = digitalRead(Button_middle);
  buttonState_right = digitalRead(Button_right);

  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if ((millis() - printtime) > 110) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      /*      Serial.print("ypr\t");
            Serial.print(ypr[0] * 180 / M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180 / M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180 / M_PI);
      */
      printtime = millis();
#endif
      MPU_Heading_S = "";
      MPU_Roll_S = "";
      MPU_Yaw_S = "";
      //  Heading
      MPU_Heading_S.concat(ypr[0] * 180 / M_PI);
      //  Roll
      MPU_Roll_S.concat(ypr[1] * 180 / M_PI);
      //  up/down nose
      MPU_Yaw_S.concat(ypr[2] * 180 / M_PI);

    }
  }
}
