//Core2: this task handles the GPS NMEAs, calculats roll and heading

void Core2code( void * pvParameters ) {

  delay(100);

  for (;;) { //main loop core2


    //  read UBX msg from F9P (heading)    ######################################################################
    if (Dual_Antenna == 1) {
      if (Serial2.available()) {         // If anything comes in Serial2
        incoming_char = Serial2.read();  // ESP32 read RELPOSNED from F9P
        if (i < 4 && incoming_char == ubxmessage.rawBuffer[i]) {
          i++;
        }
        else if (i > 3) {
          ubxmessage.rawBuffer[i] = incoming_char;
          i++;
        }
      }
      if (i > 71) {
        CK_A = 0;
        CK_B = 0;
        for (i = 2; i < 70 ; i++) {
          CK_A = CK_A + ubxmessage.rawBuffer[i];
          CK_B = CK_B + CK_A;
        }

        if (CK_A == ubxmessage.rawBuffer[70] && CK_B == ubxmessage.rawBuffer[71]) {

          if (IMU_MPU6050 > 1) Heading_MPU6050();  // to correct the drift
          heading_relposned();
          rollundheading();   // calculate roll        ########################################################
          PAOGI1_builder();   // built the PAOGI MSG   ########################################################
        }
        else {
          // Serial.println("ACK Checksum Failure: ");
        }
        i = 0;
      }
    }
    else {
      Print_NMEA();
    }

    if ((send_amatron_nmea == 1) && (Dual_Antenna == 1)) Print_NMEA();

    //  LED on == no WiFi or Ethernet, off == WiFi or Ethernet   #########################################
    if ((send_Data_Via > 0) && (Ntriphotspot_an == 0)) {
      if ((WiFi.status() == WL_CONNECTED) || (Ethernet_running)) {
        if ((millis() - WiFi_blink_Time) < 500) {
          digitalWrite(LED_ntrip_ON, HIGH);
        }
        else {
          digitalWrite(LED_ntrip_ON, LOW);
          WiFi_blink_Time = millis();
        }
      }
      else {
        digitalWrite(LED_ntrip_ON, HIGH);
      }
    }

    //  LED on == no Ntrip from ESP32, off == Ntrip from ESP32   #########################################
    if ((Ntriphotspot == 1) || (Ntriphotspot == 2)) {
      if (Ntriphotspot_an == 1) {
        digitalWrite(LED_ntrip_ON, LOW);
      }
      else {
        //digitalWrite(LED_ntrip_ON, HIGH);
      }
    }
    else {
      digitalWrite(LED_ntrip_ON, LOW);
    }
  }
  if (IMU_MPU6050 > 1) RollHeading_MPU();

}//end main loop core 2

//------------------------------------------------------------------------------------------
