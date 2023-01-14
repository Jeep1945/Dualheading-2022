//***************************************************************************************************************
// Ntrip Eth
void connectToNtrip_Eth() {
  int Ntrip_connect_Attempt = 0;
  while (Ntrip_connect_Attempt < 3) {
    Serial.println("");
    Serial.print(" Ntrip Host  : "); Serial.println(Ntrip_host);
    Serial.print(" Mountpoint  : "); Serial.println(Ntrip_mntpnt);
    Serial.print(" Ntrip User  : "); Serial.println(Ntrip_user);
    Serial.print(" Ntrip pword : "); Serial.println(Ntrip_passwd);
    Serial.print(" Ntrip_Port  : "); Serial.println(Ntrip_httpPort);
    Serial.println("");
    Serial.println("Requesting SourceTable by Ethernet");
    delay(10);
    Eth_udpNtrip_Router.begin(localPort);
    if (ntrip_e.reqSrcTbl(Ntrip_host, Ntrip_httpPort)) {
      char buffer[1024];
      delay(1000);
      // Serial.println(" Hallo 1 ");
      while (ntrip_e.available()) {
        ntrip_e.readLine(buffer, sizeof(buffer));
        Serial.print(" Buffer by Ethernet : ");
        Serial.println(buffer);
      }
      Serial.print("Requesting SourceTable is OK  by Ethernet\n");
    }
    else {
      Serial.println("SourceTable request error");
    }

    ntrip_e.stop(); //Need to call "stop" function for next request.
    delay(1000);
    Serial.println("Requesting MountPoint's Raw data by Ethernet");
    //
    delay(1000);
    if (!ntrip_e.reqRaw(Ntrip_host, Ntrip_httpPort, Ntrip_mntpnt, Ntrip_user, Ntrip_passwd)) {
      Serial.println("no ntrip connection by Ethernet");
      Serial.println(" ");
      Ntriphotspot_an = 0;  // Ntrip LED is set on
      Ntrip_connect_Attempt++;
      ntrip_attempt++;
      Ntrip_choice();
    }
    else {
      if (GGA_Send_Back_Time != 0)  sendGGA_Eth();
      delay(1000);
      Ntrip_begin_Time = millis();
      Serial.println("Requesting MountPoint is OK, all good by Ethernet");
      Serial.println(" ");
      Ntriphotspot_an = 1;  // Ntrip LED is set off
      Ntrip_connect_Attempt = 4;
      ntrip_attempt = 0;
    }
  }
}  // end connectToNtrip_Eth

//***************************************************************************************************************
// Ntrip WiFi
void connectToNtrip_WiFi() {
  int Ntrip_connect_Attempt = 0;
  while (Ntrip_connect_Attempt < 3) {
    Serial.println("");
    Serial.print(" Ntrip Host  : "); Serial.println(Ntrip_host);
    Serial.print(" Mountpoint  : "); Serial.println(Ntrip_mntpnt);
    Serial.print(" Ntrip User  : "); Serial.println(Ntrip_user);
    Serial.print(" Ntrip pword : "); Serial.println(Ntrip_passwd);
    Serial.print(" Ntrip_Port  : "); Serial.println(Ntrip_httpPort);
    Serial.println("");
    Serial.println("Requesting SourceTable by WiFi");
    delay(10);
    if (ntrip_c.reqSrcTbl(Ntrip_host, Ntrip_httpPort)) {
      char buffer[1024];
      delay(1000);
      while (ntrip_c.available()) {
        ntrip_c.readLine(buffer, sizeof(buffer));
        Serial.print(" Buffer: ");
        Serial.println(buffer);
      }
      Serial.print("Requesting SourceTable is OK\n");
    }
    else {
      Serial.println("SourceTable request error");
    }
    ntrip_c.stop(); //Need to call "stop" function for next request.
    delay(1000);
    Serial.println("Requesting MountPoint's Raw data");
    //
    if (!ntrip_c.reqRaw(Ntrip_host, Ntrip_httpPort, Ntrip_mntpnt, Ntrip_user, Ntrip_passwd)) {
      delay(1000);
      Serial.println("no ntrip connection");
      Serial.println(" ");
      Ntriphotspot_an = 0;  // Ntrip LED is set on
      Ntrip_connect_Attempt++;
      ntrip_attempt++;
      Ntrip_choice();
    }
    else {
      if (GGA_Send_Back_Time != 0)  sendGGA_WiFi();
      delay(1000);
      Ntrip_begin_Time = millis();
      Serial.println("Requesting MountPoint is OK, all good");
      Serial.println(" ");
      Ntriphotspot_an = 1;  // Ntrip LED is set off
      Ntrip_connect_Attempt = 4;
      ntrip_attempt = 0;
    }
  }
}  // end connectToNtrip_WiFi

//sendGGA---------------------------------------------------------------------------------------------

void sendGGA_WiFi() {
  // char GGASatz_send_back[100] = "$GPGGA,051601.650,4812.439,N,01633.778,E,1,12,1.0,0.0,M,0.0,M,,*60";//hc create via www.nmeagen.org
  GGASatz_send_back = "$GPGGA,";
  (GGASatz_send_back.concat(GGA_time));
  (GGASatz_send_back.concat("0,"));
  (GGASatz_send_back.concat(GGAnord.substring(0, 8)));
  (GGASatz_send_back.concat(BS));
  (GGASatz_send_back.concat(GGANordSued));
  (GGASatz_send_back.concat(BS));
  (GGASatz_send_back.concat(GGAeast.substring(0, 9)));
  (GGASatz_send_back.concat(BS));
  (GGASatz_send_back.concat(GGAWestEast));
  (GGASatz_send_back.concat(",1,12,1.0,0.0,M,0.0,M,,*"));
  /*      GGASatz_send_back = ("$GPGGA," + GGAZeit + "0," + GGAnord.substring(0, 8) + "," + GGANordSued + "," + GGAeast.substring(0, 9) +
        "," + GGAWestEast + ",1,12,1.0,0.0,M,0.0,M,,*");
  */
  for (XOR = 0, j = 0; j < GGASatz_send_back.length(); j++) { // Berechnung Checksumme
    c = (unsigned char)GGASatz_send_back.charAt(j);
    if (c == '*') break;
    if ((c != '$') && (c != '!')) XOR ^= c;
  }
  checksum = String(XOR, HEX);
  checksum.toUpperCase();
  GGASatz_send_back.concat(checksum);

  if ((GGA_Send_Back_Time != 0) && ((millis() - startSend_back_Time) > (GGA_Send_Back_Time * 1000))) {  // send back GGA to Ntripserver
    ntrip_c.print(GGASatz_send_back);
    ntrip_c.print("\r\n");
    //  Serial.println(GGASatz_send_back);
    startSend_back_Time = millis();
  }
} // end sendGGA_WiFi

//sendGGA_Ethernet---------------------------------------------------------------------------------------------

void sendGGA_Eth() {
  // char GGASatz_send_back[100] = "$GPGGA,051601.650,4812.439,N,01633.778,E,1,12,1.0,0.0,M,0.0,M,,*60";//hc create via www.nmeagen.org
  GGASatz_send_back = "$GPGGA,";
  (GGASatz_send_back.concat(GGA_time));
  (GGASatz_send_back.concat("0,"));
  (GGASatz_send_back.concat(GGAnord.substring(0, 8)));
  (GGASatz_send_back.concat(BS));
  (GGASatz_send_back.concat(GGANordSued));
  (GGASatz_send_back.concat(BS));
  (GGASatz_send_back.concat(GGAeast.substring(0, 9)));
  (GGASatz_send_back.concat(BS));
  (GGASatz_send_back.concat(GGAWestEast));
  (GGASatz_send_back.concat(",1,12,1.0,0.0,M,0.0,M,,*"));
  /*      GGASatz_send_back = ("$GPGGA," + GGAZeit + "0," + GGAnord.substring(0, 8) + "," + GGANordSued + "," + GGAeast.substring(0, 9) +
        "," + GGAWestEast + ",1,12,1.0,0.0,M,0.0,M,,*");
  */
  for (XOR = 0, j = 0; j < GGASatz_send_back.length(); j++) { // Berechnung Checksumme
    c = (unsigned char)GGASatz_send_back.charAt(j);
    if (c == '*') break;
    if ((c != '$') && (c != '!')) XOR ^= c;
  }
  checksum = String(XOR, HEX);
  checksum.toUpperCase();
  GGASatz_send_back.concat(checksum);

  if ((GGA_Send_Back_Time != 0) && ((millis() - startSend_back_Time) > (GGA_Send_Back_Time * 1000))) {  // send back GGA to Ntripserver
    ntrip_e.print(GGASatz_send_back);
    ntrip_e.print("\r\n");
    //  Serial.println(GGASatz_send_back);
    startSend_back_Time = millis();
  }
} // end sendGGA_Eth

//doEthUDPNtrip---------------------------------------------------------------------------------------------

void doEthUDPNtrip_Router() {
  unsigned int packetLenght = Eth_udpNtrip_Router.parsePacket();
  if (packetLenght > 0) {
    Eth_udpNtrip_Router.read(Eth_NTRIP_packetBuffer, packetLenght);
    for (unsigned int ik = 0; ik < packetLenght; ik++)
    {
      Serial1.write(Eth_NTRIP_packetBuffer[ik]);
    }
    //    Serial1.println(); //really send data from UART buffer
  }  // end of Packet
} // end doEthUDPNtrip_Router


//doEthUDPNtrip---------------------------------------------------------------------------------------------

void doEthUDPNtrip() {
  unsigned int packetLenght = Eth_udpNtrip.parsePacket();
  if (packetLenght > 0) {
    Eth_udpNtrip.read(Eth_NTRIP_packetBuffer, packetLenght);
    for (unsigned int ik = 0; ik < packetLenght; ik++)
    {
      Serial1.write(Eth_NTRIP_packetBuffer[ik]);
    }
    //    Serial1.println(); //really send data from UART buffer
  }  // end of Packet
} // end doEthUDPNtrip


//-------------------------------------------------------------------------------------------------

void doUDPNtrip_WiFi() {
  udpNtrip.onPacket([](AsyncUDPPacket packet)
  {
    if (debugmode) {
      Serial.println("got NTRIP data");
    }
    for (unsigned int i = 0; i < packet.length(); i++)
    {
      Serial1.write(packet.data()[i]);
    }
  });  // end of onPacket call
}

//ntripcheck---------------------------------------------------------------------------------------------

void ntripcheck() {
  if (ntriptime_from_AgopenGPS > (millis() - 4000)) {
    ntrip_from_AgopenGPS = 1;
  }
  else {
    ntrip_from_AgopenGPS = 0;
  }
} // end void ntripcheck
