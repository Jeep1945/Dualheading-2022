void Print_NMEA()
{
  BuildNmea_GGA();
  //Serial.println(GGAnmea);
  BuildNmea_VTG();
  //Serial.println(VTGnmea);

  if ((millis() - Single_begin_Time) > 100) {
    Single_begin_Time = millis();

    int len_GGA = strlen(GGAnmea);
    int len_VTG = strlen(VTGnmea);

    if ((send_Data_Via == 0) && (!Dual_Antenna)) {
      Serial.println(GGAnmea);
      Serial.println(VTGSatz);
    }
    if ((send_Data_Via > 0) && (!Dual_Antenna)) {
      if (send_Data_Via == 1) {
        Eth_udpPAOGI.beginPacket(Eth_ipDestination, portDestination);
        Eth_udpPAOGI.write(GGAnmea, len_GGA);
        Eth_udpPAOGI.write(VTGnmea, len_VTG);
        Eth_udpPAOGI.endPacket();
      }
      if ((send_Data_Via == 2) || (send_Data_Via == 3)) {
        udpRoof.writeTo((uint8_t*)GGAnmea, len_GGA, ipDestination1, portDestination);
        udpRoof.writeTo((uint8_t*)VTGnmea, len_VTG, ipDestination1, portDestination);
      }
    }
  }// end   timed loop
} //  end Print_NMEA

//##################################

void sendAMATRON() {
  if (Amatron_begin_Time < millis() - 60000)
  {
    BuildNmea_ZDA();
    //VTGSatz_Korr.replace(vtgHeading, heading_ch);

    for (XOR = 0, j = 0; j < VTGSatz_Korr.length(); j++) { // Berechnung Checksumme
      c = (unsigned char)VTGSatz_Korr.charAt(j);
      if (c == '*') break;
      if ((c != '$') && (c != '!')) XOR ^= c;
    }
    checksum = String(XOR, HEX);
    checksum.toUpperCase();
    VTGSatz_Korr += checksum;

    if (GGAnmea_Korr[46] == 'E' || GGAnmea_Korr[46] == 'W') {   //GGA-Satz-Prüfung Meyer
       //Serial.println(GGAnmea_Korr);
      Serial2.println(GGAnmea_Korr);
    }
    Serial2.println(VTGSatz_Korr);
    //Serial.print(VTGSatz_Korr);
    //Serial.print("  Heading  ");
    //Serial.println(String(heading));

    if ((millis() - Single_begin_Time_ZDA) > 900) {
      Single_begin_Time_ZDA = millis();
      Serial2.println(ZDASatz);
      //Serial.println(ZDASatz);
    }  // end   timed loop

    if (debugmode) {
      Serial.println(GGAnmea_Korr);
      Serial.println(VTGSatz_Korr);
      Serial.println(ZDASatz);
    }

  }    // end   timed loop
  SerialNmea.flush();
} //Ende sendAMATRON()

// ************************************************************

/*    $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
  VTG          Track made good and ground speed
  054.7,T      True track made good (degrees)
  034.4,M      Magnetic track made good
  005.5,N      Ground speed, knots
  010.2,K      Ground speed, Kilometers per hour
  48          Checksum

*/
