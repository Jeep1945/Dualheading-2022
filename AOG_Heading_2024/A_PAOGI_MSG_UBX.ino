void PAOGI1_builder() {

  //  Serial.println(" 2  " + String(rolleast1));
  if (Paogi_true_UBX) {   // calculate roll        ########################################################
    if ((rolleast1 < 1000) && (rolleast1 > 0)) WEcoordinaten = ("00" + String(rolleast1, 7));
    if ((rolleast1 >= 1000) && (rolleast1 < 10000)) WEcoordinaten = ("0" + String(rolleast1, 7));
    if (rolleast1 >= 10000) WEcoordinaten = String(rolleast1, 7);

    if (debugmode) {
      Serial.print("      rollnord  "  + String(rollnord));
      Serial.print("    rolleast  "  + String(rolleast));
      Serial.println(" 1  " + String(rolleast1));
    }
    rollnord1 = abs(rollnord1);
    if (rollnord1 >= 1000)  NScoordinaten = String(rollnord1, 7);
    if (rollnord1 < 1000)  NScoordinaten = ("0" + String(rollnord1, 7));
  }

  //check if headingdirection is possible
  if ((heading_check1 < 4) && (GPSqualin1 > 3)) {
    if (heading - headingzuvor < -200)  {
      heading += 360;
    }
    if (heading - headingzuvor > 200)  {
      headingzuvor += 360;
    }
    if (abs(heading - headingzuvor) < 15) {
      headingzuvor = heading;
      heading_check1 = 0;
    }
    if (heading > 360)  heading -= 360;
    if (heading < 0)  heading += 360;
    if (headingzuvor > 360)  headingzuvor -= 360;
    if (headingzuvor < 0)  headingzuvor += 360;
  }
  else {
    if (heading_check1 > 3)
      heading = headingzuvor;
    heading_check1++;
  }
  if (heading_check1 > 3)   heading_check1 = 0;

  headingzuvor = heading;

  speeed = atof(speedKnots);
  if ((GPSqualin1 < 3) && (speeed > 0.3))  heading = atof(vtgHeading);

  heading_ubx_str = String(headingUBX);
  strglen = heading_ubx_str.length() + 1;
  heading_ubx_str.toCharArray(heading_UBX_ch, strglen);
  heading_str = String(heading);
  strglen = heading_str.length() + 1;
  heading_str.toCharArray(heading_ch, strglen);
  roll_str = String(roll);
  strglen = roll_str.length() + 1;
  roll_str.toCharArray(roll_ch, strglen);
  strglen = NScoordinaten.length() + 1;
  NScoordinaten.toCharArray(NScoordinaten_ch, strglen);
  strglen = WEcoordinaten.length() + 1;
  WEcoordinaten.toCharArray(WEcoordinaten_ch, strglen);

  strcpy(PAOGI_Msg, "");
  strcat(PAOGI_Msg, "$PAOGI,");
  strcat(PAOGI_Msg, fixTime);
  strcat(PAOGI_Msg, ",");
  strcat(PAOGI_Msg, NScoordinaten_ch); //latitude
  strcat(PAOGI_Msg, ",");
  strcat(PAOGI_Msg, latNS);
  strcat(PAOGI_Msg, ",");
  strcat(PAOGI_Msg, WEcoordinaten_ch); //longitude
  strcat(PAOGI_Msg, ",");
  strcat(PAOGI_Msg, lonEW);
  strcat(PAOGI_Msg, ",");
  strcat(PAOGI_Msg, fixQuality);
  strcat(PAOGI_Msg, ",");
  strcat(PAOGI_Msg, numSats);
  strcat(PAOGI_Msg, ",");
  strcat(PAOGI_Msg, HDOP);
  strcat(PAOGI_Msg, ",");
  strcat(PAOGI_Msg, altitude);
  strcat(PAOGI_Msg, ",");
  strcat(PAOGI_Msg, ageDGPS);
  strcat(PAOGI_Msg, ",");
  strcat(PAOGI_Msg, speedKnots);
  strcat(PAOGI_Msg, ",");
  strcat(PAOGI_Msg, heading_ch);
  strcat(PAOGI_Msg, ",");
  strcat(PAOGI_Msg, roll_ch);
  strcat(PAOGI_Msg, ",,,");
  //  strcat(PAOGI_Msg, heading_UBX_ch);
  //  strcat(PAOGI_Msg, ",");
  strcat(PAOGI_Msg, "*");
  OGI_NMEA_CalculateChecksum();
  strcat(PAOGI_Msg, "\r");

  if (send_amatron_nmea == 1) {
    strcpy(GGAnmea_Korr, "");
    if (GNGGAorGPGGA == 1)
      strcat(GGAnmea_Korr, "$GNGGA,");
    else strcat(GGAnmea_Korr, "$GPGGA,");
    strcat(GGAnmea_Korr, fixTime);
    strcat(GGAnmea_Korr, ",");
    strcat(GGAnmea_Korr, NScoordinaten_ch); //latitude
    strcat(GGAnmea_Korr, ",");
    strcat(GGAnmea_Korr, latNS);
    strcat(GGAnmea_Korr, ",");
    strcat(GGAnmea_Korr, WEcoordinaten_ch); //longitude
    strcat(GGAnmea_Korr, ",");
    strcat(GGAnmea_Korr, lonEW);
    strcat(GGAnmea_Korr, ",");
    strcat(GGAnmea_Korr, fixQuality);
    strcat(GGAnmea_Korr, ",");
    strcat(GGAnmea_Korr, numSats);
    strcat(GGAnmea_Korr, ",");
    strcat(GGAnmea_Korr, HDOP);
    strcat(GGAnmea_Korr, ",");
    strcat(GGAnmea_Korr, altitude);
    strcat(GGAnmea_Korr, ",M,");
    strcat(GGAnmea_Korr, seperation);
    strcat(GGAnmea_Korr, ",M,");
    strcat(GGAnmea_Korr, ageDGPS);
    strcat(GGAnmea_Korr, ",,");
    strcat(GGAnmea_Korr, "*");
    Checksum_GGA_Korr();
    strcat(GGAnmea_Korr, "\r");
    
    if (debugmode) {
      Serial.println(PAOGI_Msg);
      Serial.println(GGAnmea_Korr);
      Serial.print  ( "lat :" + String(latitude));
      Serial.print  (" long :" + String(longitude));
      //Serial.print  (" GGAnord :" + GGAnord);
      //Serial.print  (" GGAeast :" + GGAeast);
      Serial.print  (" NSco :" + NScoordinaten);
      Serial.println(" WEco :" + WEcoordinaten);
    }
    sendAMATRON();

  }
  if (((PAOGI_Msg[30] == 'N' || PAOGI_Msg[30] == 'S')) && (PAOGI_Msg[21] == '.')) Paogi_Shit = 0;                 // Paogichek Meyer
  else Paogi_true_UBX = false;
  if (((PAOGI_Msg[46] == 'E' || PAOGI_Msg[46] == 'W')) && (PAOGI_Msg[37] == '.')) Paogi_Shit = 0;                 // Paogichek Meyer
  else Paogi_true_UBX = false;
  if (PAOGI_Msg[57] == ',') Paogi_Shit = 0;
  else Paogi_true_UBX = false;

  //Serial.println(PAOGI_Msg);

  Paogi_Long = strlen(PAOGI_Msg);
  // Serial.println(Paogi_Long);

  //  if (millis() - PVT_send_Time > 80)
  //  {
  if (Paogi_true_UBX)
  {
    if (send_Data_Via == 0) {
      Serial.println(PAOGI_Msg);
      //Serial.println("");
    }
    if (send_Data_Via == 1) {
      int len = strlen(PAOGI_Msg);
      Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
      Eth_udpPAOGI.write(PAOGI_Msg, len);
      Eth_udpPAOGI.endPacket();
      if (((millis() - lastTime) > 10000) && (Ntriphotspot != 0)) { //10000
        //        Serial.println(" I´m still running " + String(millis() / 1000) + "0 Sekunden");
        Serial.println(PAOGI_Msg);
        if (debugmode)  {
          Serial.print("   Ethernet_running : ");
          Serial.print(Ethernet_running);
          Serial.print("   Ethernet.hardwareStatus : ");
          Serial.print(Ethernet.hardwareStatus());
          Serial.print("   Ethernet.linkStatus : ");
          Serial.println(Ethernet.linkStatus());
        }
        lastTime = millis();
      }
    }
    if ((send_Data_Via == 2) || (send_Data_Via == 3)) {
      int len = strlen(PAOGI_Msg);
      udpRoof.writeTo((uint8_t*)PAOGI_Msg, len, ipDestination1, portDestination);
      if ((millis() - lastTime) > 10000) { //10000
        //        Serial.println(" I´m still running " + String(millis() / 1000) + "0 Sekunden");
        Serial.println(PAOGI_Msg);
        lastTime = millis();
      }
    }
  }
  else  {
    //delay(1);
    Paogi_Shit1++;
    //Serial.println(Paogi_Shit1);
    if (Paogi_Shit1 > 10000) Paogi_Shit1 = 0;
    Paogi_true_UBX = true;

  }
  //    PVT_send_Time = millis();
  //  }

}  // end PAOGI_builder
