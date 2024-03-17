// If odd characters showed up.
void errorHandler()
{
  //nothing at the moment
}

void GGA_Handler() //Rec'd GGA
{
  // fix time
  parser.getArg(0, fixTime);
  // latitude
  parser.getArg(1, latitude);
  parser.getArg(2, latNS);
  // longitude
  parser.getArg(3, longitude);
  parser.getArg(4, lonEW);
  // fix quality
  parser.getArg(5, fixQuality);
  // satellite #
  parser.getArg(6, numSats);
  // HDOP
  parser.getArg(7, HDOP);
  // altitude
  parser.getArg(8, altitude);
  // Geoid Seperation sealevel
  parser.getArg(10, seperation);
  // time of last DGPS update
  parser.getArg(12, ageDGPS);

  GPSqualin1 = atoi(fixQuality);
  if (GPSqualin1 == 4)  GPSqualintime = millis() + 15000;
  if (GPSqualintime > millis() && (GPSqualin1 != 4)) {
    GPSqualin1 = 4;
    GPSqualistr = String(GPSqualin1);
    strglen = GPSqualistr.length() + 1;
    GPSqualistr.toCharArray(fixQuality, strglen);
  }
}  // end GGA_Handler()

// ##############################################

void VTG_Handler()
{
  // vtg heading
  parser.getArg(0, vtgHeading);
  // vtg Speed knots
  parser.getArg(4, speedKnots);
  // vtg Speed Km/h
  parser.getArg(6, speedKm);
}  // end VTG_Handler()

// ##############################################

void ZDA_Handler()
{
  // zda UTC time
  parser.getArg(0, zdatime);
  // zda UTC day
  parser.getArg(1, zdaday);
  // zda UTC month
  parser.getArg(2, zdamonth);
  // zda UTC year
  parser.getArg(3, zdayear);
  // zda UTC localtime hours
  parser.getArg(4, zdaltzh);
  // zda UTC localtime min
  parser.getArg(5, zdaltzn);

}  // end ZDA_Handler()

// ##############################################

void BuildNmea_GGA(void)
{
  strcpy(GGAnmea, "");
  strcat(GGAnmea, "$GNGGA,");
  strcat(GGAnmea, fixTime);
  strcat(GGAnmea, ",");
  strcat(GGAnmea, latitude);
  strcat(GGAnmea, ",");
  strcat(GGAnmea, latNS);
  strcat(GGAnmea, ",");
  strcat(GGAnmea, longitude);
  strcat(GGAnmea, ",");
  strcat(GGAnmea, lonEW);
  strcat(GGAnmea, ",");
  strcat(GGAnmea, fixQuality);
  strcat(GGAnmea, ",");
  strcat(GGAnmea, numSats);
  strcat(GGAnmea, ",");
  strcat(GGAnmea, HDOP);
  strcat(GGAnmea, ",");
  strcat(GGAnmea, altitude);
  strcat(GGAnmea, ",M,");
  strcat(GGAnmea, seperation);
  strcat(GGAnmea, ",M,");
  strcat(GGAnmea, ageDGPS);
  strcat(GGAnmea, ",,");
  strcat(GGAnmea, "*");
  GGA_NMEA_CalculateChecksum();
  strcat(GGAnmea, "\r");
} // end BuildNmea_GGA

//  ###################################################

void BuildNmea_VTG(void)
{
  strcpy(VTGnmea, "");
  strcat(VTGnmea, "$GNVTG,");
  if (Dual_Antenna == 1)
    strcat(VTGnmea, heading_ch);
  else
    strcat(VTGnmea, vtgHeading);
  strcat(VTGnmea, ",");
  strcat(VTGnmea, "T");
  strcat(VTGnmea, ",");
  strcat(VTGnmea, "");
  strcat(VTGnmea, ",");
  strcat(VTGnmea, "M");
  strcat(VTGnmea, ",");
  strcat(VTGnmea, speedKnots);
  strcat(VTGnmea, ",");
  strcat(VTGnmea, "N");
  strcat(VTGnmea, ",");
  strcat(VTGnmea, speedKm);
  strcat(VTGnmea, ",");
  strcat(VTGnmea, "K");
  strcat(VTGnmea, ",");
  strcat(VTGnmea, "A");
  strcat(VTGnmea, "*");
  VTGSatz_Korr = VTGnmea;
  VTG_NMEA_CalculateChecksum();
  strcat(VTGnmea, "\r");
  VTGSatz = VTGnmea;
}  // end BuildNmea_VTG

//  ###################################################

void BuildNmea_ZDA(void)
{
  strcpy(ZDAnmea, "");
  strcat(ZDAnmea, "$GNZDA,");
  strcat(ZDAnmea, zdatime);
  strcat(ZDAnmea, ",");
  strcat(ZDAnmea, zdaday);
  strcat(ZDAnmea, ",");
  strcat(ZDAnmea, zdamonth);
  strcat(ZDAnmea, ",");
  strcat(ZDAnmea, zdayear);
  strcat(ZDAnmea, ",");
  strcat(ZDAnmea, zdaltzh);
  strcat(ZDAnmea, ",");
  strcat(ZDAnmea, zdaltzn);
  strcat(ZDAnmea, "*");
  ZDA_NMEA_CalculateChecksum();
  ZDASatz = ZDAnmea;
  strcat(ZDAnmea, "\r");
}             // end BuildNmea_ZDA

// ##############################################

void GGA_NMEA_CalculateChecksum(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = GGAnmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp;    // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { asciiHex[chk], 0 };
  strcat(GGAnmea, hex);

  chk = (sum % 16);
  char hex2[2] = { asciiHex[chk], 0 };
  strcat(GGAnmea, hex2);
}  // end GGA_NMEA_CalculateChecksum

// ##############################################

void VTG_NMEA_CalculateChecksum(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = VTGnmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp;    // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { asciiHex[chk], 0 };
  strcat(VTGnmea, hex);

  chk = (sum % 16);
  char hex2[2] = { asciiHex[chk], 0 };
  strcat(VTGnmea, hex2);
}  // end VTG_NMEA_CalculateChecksum

// ##############################################

void ZDA_NMEA_CalculateChecksum(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = ZDAnmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp;    // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { asciiHex[chk], 0 };
  strcat(ZDAnmea, hex);

  chk = (sum % 16);
  char hex2[2] = { asciiHex[chk], 0 };
  strcat(ZDAnmea, hex2);
}// end ZDA_NMEA_CalculateChecksum

// ##############################################


void OGI_NMEA_CalculateChecksum(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = PAOGI_Msg[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp;    // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { asciiHex[chk], 0 };
  strcat(PAOGI_Msg, hex);

  chk = (sum % 16);
  char hex2[2] = { asciiHex[chk], 0 };
  strcat(PAOGI_Msg, hex2);
}// end OGI_NMEA_CalculateChecksum

// ##############################################


void Checksum_GGA_Korr(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = GGAnmea_Korr[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp;    // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = { asciiHex[chk], 0 };
  strcat(GGAnmea_Korr, hex);

  chk = (sum % 16);
  char hex2[2] = { asciiHex[chk], 0 };
  strcat(GGAnmea_Korr, hex2);
}// end Checksum_GGA_Korr

// ##############################################
