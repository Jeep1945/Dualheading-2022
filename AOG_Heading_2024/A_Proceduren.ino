// *********************************************************************************

void high_relposned()
{
  int Unit1 = 36;
  seahight_PVT  = (long)ubxmessagePVT.rawBufferPVT[Unit1 + 6] ;            // read UBX high from 2. F9P
  seahight_PVT += (long)ubxmessagePVT.rawBufferPVT[Unit1 + 7] << 8;
  seahight_PVT += (long)ubxmessagePVT.rawBufferPVT[Unit1 + 8] << 16 ;
  seahight_PVT += (long)ubxmessagePVT.rawBufferPVT[Unit1 + 9] << 24 ; // in mm
  seahight_PVT = seahight_PVT / 1000;  //
  //Serial.println(" seahight_PVT  " + String(seahight_PVT, 3));

  int Unit = 56;
  velD_UBX_PVT  = (long)ubxmessagePVT.rawBufferPVT[Unit + 6] ;            // read UBX high from 2. F9P
  velD_UBX_PVT += (long)ubxmessagePVT.rawBufferPVT[Unit + 7] << 8;
  velD_UBX_PVT += (long)ubxmessagePVT.rawBufferPVT[Unit + 8] << 16 ;
  velD_UBX_PVT += (long)ubxmessagePVT.rawBufferPVT[Unit + 9] << 24 ; // in mm
  //velD_UBX_PVT = velD_UBX_PVT ;  //
  //Serial.println(" velD_UBX_PVT  " + String(velD_UBX_PVT, 3));

  int Unit8 = 23;
  numSV_UBX_PVT  = (long)ubxmessagePVT.rawBufferPVT[Unit8 + 6] ;            // read UBX high from 1. F9P
  //numSV_UBX_PVT = velD_UBX_PVT ;  //
  //Serial.println(" numSV_UBX_PVT  " + String(numSV_UBX_PVT, 0));

  int Unit5 = 8;
  double time_UBX_PVT1, PVT_Time;
  time_UBX_PVT1 = (long)ubxmessagePVT.rawBufferPVT[Unit5 + 6];             // read UBX high from 1. F9P
  time_UBX_PVT1 *= 100 ;
  int Unit6 = 9;
  time_UBX_PVT1 += (long)ubxmessagePVT.rawBufferPVT[Unit6 + 6];            // read UBX high from 1. F9P
  time_UBX_PVT1 *= 100 ;
  int Unit7 = 10;
  time_UBX_PVT1 += (long)ubxmessagePVT.rawBufferPVT[Unit7 + 6];            // read UBX high from 1. F9P

  //Serial.println("  PVT_time      " + String(time_UBX_PVT1, 3));

  int Unit4 = 16;
  time_UBX_PVT  = (long)ubxmessagePVT.rawBufferPVT[Unit4 + 6] ;            // read UBX high from 1. F9P
  time_UBX_PVT += (long)ubxmessagePVT.rawBufferPVT[Unit4 + 7] << 8;
  time_UBX_PVT += (long)ubxmessagePVT.rawBufferPVT[Unit4 + 8] << 16 ;
  time_UBX_PVT += (long)ubxmessagePVT.rawBufferPVT[Unit4 + 9] << 24 ;

  PVT_Time = time_UBX_PVT1 + (time_UBX_PVT / 1000000000);

  int Unit2 = 24;
  lon  = (long)ubxmessagePVT.rawBufferPVT[Unit2 + 6];
  lon += (long)ubxmessagePVT.rawBufferPVT[Unit2 + 7] << 8;
  lon += (long)ubxmessagePVT.rawBufferPVT[Unit2 + 8] << 16;
  lon += (long)ubxmessagePVT.rawBufferPVT[Unit2 + 9] << 24;
  //lon  = lon / 100000;
  String lon_String = String(lon, 0);
  String lon_String_d = String(lon, 0);
  if (lon < 1000000000) {
    lon_String_d = lon_String.substring(0, 2);
    lon_String = lon_String.substring(2);
  }
  else {
    lon_String_d = lon_String.substring(0, 3);
    lon_String = lon_String.substring(3);
  }
  lonint = lon_String.toInt();
  lon_degree = ((lonint * 60));
  lon_degree  = lon_degree / 10000000;
  if (debugmode) {
    Serial.println("");
    Serial.println("");
    Serial.println(" lon,dec      " + String(lon, 2 ));
    Serial.println(" lon_String  " + lon_String);
    Serial.println(" lonint      " + String(lonint));
    Serial.print  (" lon_degree   " + lon_String_d + String(lon_degree, 7));
    Serial.println("  PVT_time     " + String(PVT_Time, 3));
    Serial.println("");
  }

  int Unit3 = 28;
  lati  = (long)ubxmessagePVT.rawBufferPVT[Unit3 + 6];
  lati += (long)ubxmessagePVT.rawBufferPVT[Unit3 + 7] << 8;
  lati += (long)ubxmessagePVT.rawBufferPVT[Unit3 + 8] << 16;
  lati += (long)ubxmessagePVT.rawBufferPVT[Unit3 + 9] << 24;
  String lati_String = String(lati, 0);
  String lati_String_d = String(lati, 0);
  lati_String_d = lati_String.substring(0, 2);
  lati_String = lati_String.substring(2);
  latiint = lati_String.toInt();
  lati_degree = ((latiint * 60));
  lati_degree  = lati_degree / 10000000;
  if (debugmode) {
    Serial.println(" lati,dec     " + String(lati, 2 ));
    Serial.println(" GGAnord,dec  " + String((nordWinkel + fixnorddeci) * 10000000, 2 ));
    //Serial.println(" lati_String  " + lati_String);
    //Serial.println(" latiint      " + String(latiint));
    Serial.print  (" lati_degree  " + lati_String_d + String(lati_degree, 7));
    Serial.println("  PVT_time     " + String(PVT_Time, 3));
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
  if (debugmode) {
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
    Serial.println (carrSoln);
  }
  for (j = 1; j <= 9; j++) {
    hzuvor[j - 1] = hzuvor[j];
    if (debugmode)   Serial.print(" h: " + String(j) + ", " + String(hzuvor[j - 1]));
  }
  hzuvor[9] = heading;
  if  (abs(hzuvor[8] - hzuvor[9]) > 90) {   // d.h. geht nur bei Sprung 360-0
    if (debugmode) Serial.print(" h8+9: " + String(hzuvor[8]) + ", " + String(hzuvor[9]));
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
  GGDs /= 55;
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
  speeed = atof(speedKnots);
  double Headingfilter1 = Headingfilter;
  double speed_kmh = speeed / 1.852;
  //speed_kmh = 1;
  if (speed_kmh <= 4) {
    Headingfilter1 = Headingfilter + (4 - speed_kmh);
    Headingfilter1 = constrain(Headingfilter1, 1, 9);
  }

  heading2 = headingUBX * (1 - Headingfilter1 / 10) + GGDs * (Headingfilter1 / 10);

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
    //Serial.print("  h1M : ");
    //Serial.print(heading1);
    Serial.print(" UBX : ");
    Serial.print(headingUBX);
    Serial.print("  heading : ");
    Serial.print(heading);
    //Serial.print("  MPU : ");
    //Serial.print(heading_MPU6050);
    Serial.print("  heading2 : ");
    Serial.print(heading2);
    Serial.print("  GGDs : ");
    Serial.print(GGDs);
    Serial.print("  GGDs1 : ");
    Serial.print(GGDs1);
    Serial.println("    ");
    //Serial.print(heading_MPU6050_offset);
  }
  //  else delay(10);
}



void rollundheading() {
  baseline  = (long)ubxmessage.rawBuffer[20 + 6] ;
  baseline += (long)ubxmessage.rawBuffer[21 + 6] << 8;
  baseline += (long)ubxmessage.rawBuffer[22 + 6] << 16 ;
  baseline += (long)ubxmessage.rawBuffer[23 + 6] << 24 ;
  baseline1 = (long)ubxmessage.rawBuffer[35 + 6];
  baseline1 = baseline1 / 100.0;
  if (debugmode) {
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
  if (debugmode) {
    Serial.print(", DH " + String(relPosDH, 2));
    Serial.print(", D " + String(relPosD, 2));
  }

  rollcalc();

  if (debugmode) {
    Serial.print(", roll " + String(roll, 2));
    Serial.print(", rollcorr " + String(rollCorrectionDistance, 2));
    Serial.print(",  ");
  }
}
// ende parser
