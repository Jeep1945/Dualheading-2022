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
  heading2 = headingUBX * (1 - Headingfilter / 10) + GGDs * (Headingfilter / 10);
  if (GGDs > 360)   GGDs -= 360;
  if (GGDs < 0)   GGDs += 360;
  if (heading2 > 360)   heading2 -= 360;
  if (heading2 < 0)   heading2 += 360;
  heading_source = 0;
  if (relPosValid == 1) {
    heading = heading2;
    heading_source = 2;
  }
  hzuvormax = GGDs;
  hzuvormin = GGDs;

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
