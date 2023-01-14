void MSG_build() {

  if ((start == 0) && ((inByte == '$') || (inByte == '!'))) {
    start = 1; nmea = ""; // begin $ or !
  }
  if (start == 1) {
    nmea.concat((char)inByte); // add sign
  }
  if ((inByte == 13) && (start == 1)) { // CR > Datensatzende > NMEA ausgeben
    start = 0;
    if (nmea.substring(3, 6) == "GGA")  {   // Data from GGA,
      for (byte m = 0; m <= nmea.length(); m++) {
        GGASentence[m] = (unsigned char)nmea.charAt(m);
      }
    }  // end from GGA

    if (nmea.substring(3, 6) == "VTG")  {   // Data from VTG,
      for (byte m = 0; m <= nmea.length(); m++) {
        VTGSentence[m] = (unsigned char)nmea.charAt(m);
      }
    }  // end from VTG

    if (nmea.substring(3, 6) == "GSA")  {   // Data from GSA,
      for (byte m = 0; m <= nmea.length(); m++) {
        GSASentence[m] = (unsigned char)nmea.charAt(m);
      }
    }  // end from GSA

    if (nmea.substring(3, 6) == "ZDA")  {   // Data from ZDA,
      for (byte m = 0; m <= nmea.length(); m++) {
        ZDASentence[m] = (unsigned char)nmea.charAt(m);
      }
    }  // end from ZDA
  }
}  // end MSB_build
