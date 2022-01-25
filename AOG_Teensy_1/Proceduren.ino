//Paogicheck---------------------------------------------------------------------------------------------

void Paogicheck() {

  if ((Paogi_Long < 91) || (Paogi_Long > 105) || (jGGA6 != 45)) {
    Paogi_true_UBX = false;
  }
  if ((heading == 0) || (heading == 90) || (heading == 180) || (heading == 270))  Paogi_true_UBX = false;
}   // end void Paogicheck



//Coordinaten_check---------------------------------------------------------------------------------------------

void  Coordinaten_check() {

  //  Serial.print("      rollnord  "  + String(rollnord));
  //  Serial.println("    rolleast  "  + String(rolleast));
  //  Serial.println(" 1  " + String(rolleast1));
  if ((abs(rolleast - rolleast_before) > 15000) && (Coodinate1_check1 < 1)) {
    Paogi_true_UBX = false;
    Coodinate1_check1++;
  }
  else {
    rolleast_before = rolleast;
    Coodinate1_check1 = 0;
  }
  //  Serial.println(" 2  " + String(rolleast1));
  if ((rolleast1 < 1000) && (rolleast1 > 0)) WEcoordinaten = ("00" + String(rolleast1, 7));
  if ((rolleast1 >= 1000) && (rolleast1 < 10000)) WEcoordinaten = ("0" + String(rolleast1, 7));
  if (rolleast1 >= 10000) WEcoordinaten = String(rolleast1, 7);

  if ((abs(rollnord - rollnord_before) > 15000) && (Coodinate1_check2 < 1)) {
    Paogi_true_UBX = false;
    Coodinate1_check2++;
  }
  else {
    rollnord_before = rollnord;
    Coodinate1_check2 = 0;
  }
  rollnord1 = abs(rollnord1);
  if (rollnord1 >= 1000)  NScoordinaten = String(rollnord1, 7);
  if (rollnord1 < 1000)  NScoordinaten = ("0" + String(rollnord1, 7));

}  // end void Coordinaten_check

// *********************************************************************************

void heading_relposnet() {
  //begin parser

  headingUBX  = (long)ubxmessage.rawBuffer[24 + 6] ;            // HeadingUBX read
  headingUBX += (long)ubxmessage.rawBuffer[25 + 6] << 8;
  headingUBX += (long)ubxmessage.rawBuffer[26 + 6] << 16 ;
  headingUBX += (long)ubxmessage.rawBuffer[27 + 6] << 24 ;
  headingUBX  = headingUBX / 100000;
  if (debugProtokoll) {
    Serial.print("  " + String(headingUBX, 2));
  }
  if (headingUBX < 0.1)  headingUBX = headingzuvor;
  else headingUBX += headingcorr;
  if (headingUBX >= 360) {
    headingUBX -= 360;
  }
  heading = headingUBX;
  if (debugProtokoll)    Serial.println(" UBX : " + String(headingUBX));
}



void rollundheading() {
  baseline  = (long)ubxmessage.rawBuffer[20 + 6] ;
  baseline += (long)ubxmessage.rawBuffer[21 + 6] << 8;
  baseline += (long)ubxmessage.rawBuffer[22 + 6] << 16 ;
  baseline += (long)ubxmessage.rawBuffer[23 + 6] << 24 ;
  baseline1 = (long)ubxmessage.rawBuffer[35 + 6];
  baseline1 = baseline1 / 100.0;
  if (debugProtokoll) {
    Serial.print(", B " + String(baseline, 2));
    Serial.print(", B1 " + String(baseline1, 2));
    Serial.print(", BL " + String(baseline + baseline1, 2));
  }
  baseline = baseline + baseline1;

  relPosD  = (long)ubxmessage.rawBuffer[16 + 6] ;              // hight read in cm
  relPosD += (long)ubxmessage.rawBuffer[17 + 6] << 8;
  relPosD += (long)ubxmessage.rawBuffer[18 + 6] << 16 ;
  relPosD += (long)ubxmessage.rawBuffer[19 + 6] << 24 ;
  relPosDH = (long)ubxmessage.rawBuffer[34 + 6];  // hight read in cm
  if (relPosDH >= 128) {
    relPosDH -= 256.0;
  }
  relPosDH = relPosDH / 100.0;
  relPosD = relPosD + relPosDH;          // in mm
  if (debugProtokoll) {
    Serial.print(", DH " + String(relPosDH, 2));
    Serial.print(", D " + String(relPosD, 2));
  }
  relPosD = relPosD / 100.0;

  rollcalc();

  if (debugProtokoll) {
    Serial.print(", roll " + String(roll, 2));
    Serial.print(", rollcorr " + String(rollCorrectionDistance, 2));
    Serial.print(",  ");
  }
}
// ende parser
