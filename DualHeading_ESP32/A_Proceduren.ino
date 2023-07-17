//Coordinaten_check---------------------------------------------------------------------------------------------

void  Coordinaten_check() {


  if (GGAnord.substring(4, 5) == ".") {  // Data from GGA
    if (((abs(nordWinkel - rollnord_before) > 1) || (abs(fixnorddeci - fixnorddeci_before) > 0.0005)) && (Coodinate1_check2 < 4)) {  // Data from GGA only degrees
      Paogi_true_UBX = false;
      Coodinate1_check2++;
      //Serial.println("  Hallo1  ");
    }
    else {
      rollnord_before = nordWinkel;
      fixnorddeci_before = fixnorddeci;
      Coodinate1_check2 = 0;
    }
  }
  else {
    Paogi_true_UBX = false;
    Coodinate1_check2++;
    //Serial.println("  Hallo2  ");
  }

  if (GGAeast.substring(5, 6) == ".") {  // Data from GGA
    if (((abs(eastWinkel - rolleast_before) > 1) || (abs(fixeastdeci - fixeastdeci_before) > 0.0005)) && (Coodinate1_check1 < 4)) {
      Paogi_true_UBX = false;
      Coodinate1_check1++;
    }
    else {
      rolleast_before = eastWinkel;
      fixeastdeci_before = fixeastdeci;
      Coodinate1_check1 = 0;
    }
  }
  else {
    Paogi_true_UBX = false;
    Coodinate1_check1++;
  }
  if (debugmode) {
    //Serial.print("  nordWinkel  ");
    //Serial.print(nordWinkel - rollnord_before);
    Serial.print("  fixnorddeci_before  ");
    Serial.print(fixnorddeci_before);
    Serial.print("  fixnorddeci  ");
    Serial.print(fixnorddeci - fixnorddeci_before, 7);
    Serial.print("  fixeastdeci : ");
    Serial.print(fixeastdeci - fixeastdeci_before, 7);
    Serial.print("  Coodinate1_check2 : ");
    Serial.println(Coodinate1_check2);
  }
}  // end void

// *********************************************************************************

void heading_relposned() {
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
  else headingUBX += 90;
  if (headingUBX >= 360) {
    headingUBX -= 360;
  }
  heading = headingUBX;

  uint32_t flags = ubxmessage.rawBuffer[60 + 6];

  //  Serial.println(flags, BIN);

  gnssFixOk = flags & (1 << 0);
  diffSoln = flags & (1 << 1);
  relPosValid = flags & (1 << 2);
  carrSoln = (flags & (0b11 << 3)) >> 3;
  if (debugmode) {
    Serial.print("  gnssFixOk : ");
    Serial.print (gnssFixOk);
    Serial.print("  diffSoln : ");
    Serial.print (diffSoln);
    Serial.print("  relPosValid : ");
    Serial.print (relPosValid);
    Serial.print(" carrSoln : ");
    Serial.print (carrSoln);
  }
  for (j = 1; j <= 9; j++) {
    hzuvor[j - 1] = hzuvor[j];
    if (debugmode2)   Serial.print(" h: " + String(j) + ", " + String(hzuvor[j - 1]));
  }
  hzuvor[9] = heading;
  if  (abs(hzuvor[8] - hzuvor[9]) > 90) {   // d.h. geht nur bei Sprung 360-0
    if (debugmode2) Serial.print(" h8+9: " + String(hzuvor[8]) + ", " + String(hzuvor[9]));
    if (hzuvor[8] > hzuvor[9])  hzuvor[9] += 360;
    else  hzuvor[9] -= 360;
  }
  for (i = 0; i < 10; i++) {
    if (hzuvormin > hzuvor[i]) hzuvormin = hzuvor[i];
    if (hzuvormax < hzuvor[i]) hzuvormax = hzuvor[i];
  }
  GGDs = 0;
  for (j = 0; j <= 9; j++) {
    GGDs += hzuvor[j] * (j + 1);
  }
  GGDs /= 55.0;
  if (GGDs < 0) {           // Wenn der GGDs unter 0, dann bei allen 10 headings 360 add
    for (j = 0; j <= 9; j++) hzuvor[j] += 360;
  }
  if (GGDs > 360) {  // Wenn der GGDs unter 0, dann bei allen 10 headings 360 sub
    for (j = 0; j <= 9; j++) hzuvor[j] -= 360;
  }
  if (GGDs > 360)   GGDs -= 360;
  if (GGDs < 0)   GGDs += 360;

  if ((GGDs - headingUBX) > 90)    GGDs -= 360;
  if ((GGDs - headingUBX) < -90)   GGDs += 360;

  double Headingfilter1 = Headingfilter;
  double speed_kmh = speeed/1.852;
  if (speed_kmh < 3) {
    Headingfilter1 = Headingfilter + (3 - speed_kmh);
    Headingfilter1 = constrain(Headingfilter1, 1, 10);
    heading2 = headingUBX * (1 - Headingfilter1 / 10) + GGDs * (Headingfilter1 / 10);
  }
  else
    heading2 = Headingfilter;

  if (GGDs > 360)   GGDs -= 360;
  if (GGDs < 0)   GGDs += 360;
  if (heading2 > 360)   heading2 -= 360;
  if (heading2 < 0)   heading2 += 360;
  heading_source = 0;
  if (relPosValid == 1) {
    heading = heading2;
    heading_source = 2;
  }

  if (IMU_MPU6050 > 1) {
    if (abs(hzuvormax - hzuvormin) < 4) { //1000
      //Serial.println(carrSoln);
      if (carrSoln > 0) {  // >
        if (millis() - lastTime_heading > 2000) {
          heading_MPU6050_offset = heading_MPU6050 - GGDs;
        }
      }
      else  lastTime_heading = millis();
    }
    heading1 = heading_MPU6050 - heading_MPU6050_offset;
    if (heading1 < 0) heading1 += 360;
    if (heading1 > 360) heading1 -= 360;
    if (carrSoln > 0) {  // >
      heading = heading1;
      heading_source = 1;
    }
  }
  hzuvormax = GGDs;
  hzuvormin = GGDs;

  if (debugmode) {
    Serial.print("  h : ");
    Serial.print(heading);
    Serial.print("  h1M : ");
    Serial.print(heading1);
    Serial.print(" UBX : ");
    Serial.print(headingUBX);
    Serial.print("  MPU : ");
    Serial.print(heading_MPU6050);
    Serial.print("  Filter : ");
    Serial.print(heading2);
    Serial.print("  GGDs : ");
    Serial.print(GGDs);
    Serial.print("  carrSoln : ");
    Serial.print(carrSoln);
    Serial.print("  Off : ");
    Serial.print(heading_MPU6050_offset);
  }
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
