void PAOGI1_builder() {

  Paogi_true_UBX = true;

  Coordinaten_check();

  if (rollaktiv == 0) {   // calculate roll        ########################################################
    NScoordinaten = GGAnord;
    WEcoordinaten = GGAeast;
  }

  if ((heading > 5) && (heading < 355) && (GPSqualin1 > 3)) {
    if ((abs(heading - headingzuvor) > 3) && (heading_check1 < 4)) {
      heading = headingzuvor;
      heading_check1++;
    }
    else {
      headingzuvor = heading;
      heading_check1 = 0;
    }
  }
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
  (RollHeadingrest.concat(heading));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(roll));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(BS));
  (RollHeadingrest.concat(BS));
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

  //  Serial.println(RollHeadingrest);
  //  Serial.println(GGASatz);

  Paogi_Long = RollHeadingrest.length();
  // Serial.println(Paogi_Long);

  //  Paogicheck();

  if (Paogi_true_UBX)
  {
    if (send_Data_Via == 0) {
      unsigned int Bytelaenge = Paogi_Long + 1;
      RollHeadingrest.getBytes(UDPPAOGIMsg, Bytelaenge);
      UDPPAOGIMsg[Bytelaenge - 1] = 0x0D;
      UDPPAOGIMsg[Bytelaenge] = 0x0A;
      Serial.write(UDPPAOGIMsg, Bytelaenge + 1);
    }
  }
  else  {
    //    Paogi_Shit1++;
    Paogi_true_UBX = true;
    /*    Serial.println(" ");
        Serial.println(" ");
        Serial.println(RollHeadingshit);
        Serial.println("Shit ist Sch.. " + String(Paogi_Long));
        Serial.println(" ");
        Serial.println(" ");
    */
  }
  if (debugmode )    Serial.println();
  if (debugmode1)    Serial.println();
  if (debugmode2)    Serial.println();

}  // end PAOGI_builder
