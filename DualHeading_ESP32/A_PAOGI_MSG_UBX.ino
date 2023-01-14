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
  if ((GPSqualin1 < 3) && (speeed > 0.3))  heading = headingnord;


  RollHeadingrest = "$PAOGI,";
  (RollHeadingrest.concat(GGA_time));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(NScoordinaten));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(GGANordSued));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(WEcoordinaten));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(GGAWestEast));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(GPSqualin1));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(GGA_Sats));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(GGA_hDops));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(GGA_Sealevel));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(GGA_Age));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(speeed));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(String(heading, 4)));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(roll));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(BS));


//    (RollHeadingrest.concat(heading2));
//    (RollHeadingrest.concat(BS));
  /*    (RollHeadingrest.concat(GGDs));
    (RollHeadingrest.concat(BS));
    (RollHeadingrest.concat(heading1));
    (RollHeadingrest.concat(BS));
    (RollHeadingrest.concat(String(heading_MPU6050)));
    (RollHeadingrest.concat(BS));
*/    (RollHeadingrest.concat(headingUBX));
//    (RollHeadingrest.concat(BS));
//    (RollHeadingrest.concat(roll2));
//    (RollHeadingrest.concat(BS));
//    (RollHeadingrest.concat(roll1));
  /*
  //(RollHeadingrest.concat(heading_source));
  (RollHeadingrest.concat(ABline_Direction[1]));
  (RollHeadingrest.concat(ntrip_from_AgopenGPS));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(Ntriphotspot_an));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(heading_dual_MPU));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(gnssFixOk));
  //(RollHeadingrest.concat(Ntriphotspot));
  (RollHeadingrest.concat(BS));
*/
  (RollHeadingrest.concat("*"));
  RollHeadingshit = RollHeadingrest;
  //  Serial.println(RollHeadingrest);

  for (XOR = 0, j = 0; j < RollHeadingrest.length(); j++) { // Berechnung Checksumme
    c = (unsigned char)RollHeadingrest.charAt(j);
    if (c == '*') break;
    if ((c != '$') && (c != '!')) XOR ^= c;
  }
  checksum = String(XOR, HEX);
  checksum.toUpperCase();
  RollHeadingrest.concat(checksum);

  if (((RollHeadingrest.substring(30, 31) == "N" || RollHeadingrest.substring(30, 31) == "S")) && (RollHeadingrest.substring(21, 22) == ".")) Paogi_Shit = 0;                 // Paogichek Meyer
  else Paogi_true_UBX = false;
  if (((RollHeadingrest.substring(46, 47) == "E" || RollHeadingrest.substring(46, 47) == "W")) && (RollHeadingrest.substring(37, 38) == ".")) Paogi_Shit = 0;                 // Paogichek Meyer
  else Paogi_true_UBX = false;
  if (RollHeadingrest.substring(57, 58) == ",") Paogi_Shit = 0;                 // Paogichek Meyer
  else Paogi_true_UBX = false;

  if (!NMEA_OK) {
    Paogi_true_UBX = false;
    //Serial.println("  Hallo NMEA  ");
  }
  //  Serial.println(RollHeadingrest);
  //  Serial.println(GGASatz);

  Paogi_Long = RollHeadingrest.length();
  // Serial.println(Paogi_Long);

  if (Paogi_true_UBX)
  {
    if (send_Data_Via == 0) {
      unsigned int Bytelaenge = Paogi_Long + 1;
      RollHeadingrest.getBytes(UDPPAOGIMsg, Bytelaenge);
      UDPPAOGIMsg[Bytelaenge - 1] = 0x0D;
      UDPPAOGIMsg[Bytelaenge] = 0x0A;
      vTaskDelay(10);
      Serial.write(UDPPAOGIMsg, Bytelaenge + 1);
    }
    if (send_Data_Via == 1) {
      Bytelaenge = Paogi_Long + 1;
      RollHeadingrest.getBytes(ReplyBufferPAOGI, Bytelaenge);
      ReplyBufferPAOGI[Bytelaenge - 1] = 0x0D;
      ReplyBufferPAOGI[Bytelaenge] = 0x0A;
      Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
      Eth_udpPAOGI.write(ReplyBufferPAOGI, Bytelaenge + 1);
      Eth_udpPAOGI.endPacket();
      if ((millis() - lastTime) > 10000) { //10000
        //        Serial.println(" I´m still running " + String(millis() / 1000) + "0 Sekunden");
        Serial.println(RollHeadingrest);
        lastTime = millis();
      }
    }
    if ((send_Data_Via == 2) || (send_Data_Via == 3)) {
      Bytelaenge = Paogi_Long + 1;
      RollHeadingrest.getBytes(ReplyBufferPAOGI, Bytelaenge);
      ReplyBufferPAOGI[Bytelaenge - 1] = 0x0D;
      ReplyBufferPAOGI[Bytelaenge] = 0x0A;
      udpRoof.writeTo(ReplyBufferPAOGI, Bytelaenge + 1, ipDestination1, portDestination);
      if ((millis() - lastTime) > 10000) { //10000
        //        Serial.println(" I´m still running " + String(millis() / 1000) + "0 Sekunden");
        Serial.println(RollHeadingrest);
        lastTime = millis();
      }
    }
  }
  else  {
    //delay(1);
    Paogi_Shit1++;
    Serial.println(Paogi_Shit1);
    if (Paogi_Shit1 > 10000) Paogi_Shit1 = 0;
    Paogi_true_UBX = true;
  }
  if (send_amatron_nmea == 1) {
    GGASatz_Korr.replace(GGAnord, NScoordinaten);
    GGASatz_Korr.replace(GGAeast, WEcoordinaten);
    Checksum_GGA_Korr();
    sendAMATRON();
  }
  //  Serial.println();
  if (debugProtokoll)
  {
    //Serial.println(RollHeadingrest);
  }
  if (debugmode )    Serial.println();
  if (debugmode1)    Serial.println();
  if (debugmode2)    Serial.println();

}  // end PAOGI_builder