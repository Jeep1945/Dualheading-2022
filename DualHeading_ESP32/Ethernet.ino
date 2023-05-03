void Eth_Start() {
  Ethernet.init(Eth_CS_PIN);
  //  delay(50);
  Eth_myip[2] = Ethernet_3rd;
  Ethernet.begin(mac, Eth_myip);
  delay(200);
  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :");
  }
  else {
    Serial.println("Ethernet hardware found, checking for connection");
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
      ESP.restart();
      Ethernet_running = false;
    }
    else {
      Serial.println("Ethernet status OK");
      Serial.print("Got IP ");
      Serial.println(Ethernet.localIP());
      if ((Ethernet.localIP()[0] == 0) && (Ethernet.localIP()[1] == 0) && (Ethernet.localIP()[2] == 0) && (Ethernet.localIP()[3] == 0)) {
        //got IP 0.0.0.0 = no DCHP so use static IP
        Ethernet_running = true;
      }
      //use DHCP but change IP ending (x.x.x.80)
      if (!Ethernet_running) {
        for (byte n = 0; n < 3; n++) {
          Eth_myip[n] = Ethernet.localIP()[n];
          Eth_ipDestination[n] = Ethernet.localIP()[n];
        }
        Eth_ipDestination[3] = 255;
        Ethernet.setLocalIP(Eth_myip);
      }
      else {//use static IP
        for (byte n = 0; n < 3; n++) {
          Eth_ipDestination[n] = Eth_myip[n];
        }
        Eth_ipDestination[3] = Eth_ipDest_ending;
        Ethernet.setLocalIP(Eth_myip);
      }

      Ethernet_running = true;
      Serial.print("Ethernet IP of roof module: "); Serial.println(Ethernet.localIP());
      Serial.print("Ethernet sending to IP: "); Serial.println(Eth_ipDestination);
      //init UPD Port sending to AOG
      if (Eth_udpPAOGI.begin(portMy)) // portMy
      {
        Serial.print("Ethernet UDP sending from port: ");
        Serial.println(portMy);
      }
      //init UPD Port getting NTRIP from AOG
      if (Eth_udpNtrip.begin(AOGNtripPort)) // AOGNtripPort
      {
        Serial.print("Ethernet NTRIP UDP listening to port: ");
        Serial.println(AOGNtripPort);
      }
      //init UPD Port sending to AOG
      if (EthUDPToAOG.begin(portMy)) // portMy
      {
        Serial.print("UDP 203 sending to port: ");
        Serial.println(portMy);
      }
      //init UDP listening on port:
      if (EthUDPFromAOG.begin(localPort)) // localPort
      {
        Serial.print("UDP 202 listening on port: ");
        Serial.println(localPort);
      }
    }
    //    Serial.println();
  }
}

//-------------------------------------------------------------------------------------------------

void read_Eth_AGIO() 
{
  src_ip = EthUDPFromAOG.remoteIP();
  EthUDPFromAOG.read(ReplyBufferAGIO, packetLength);
  //Serial.println("Hallo 1");
  uint8_t scanReply[] = { 128, 129, 120, 203, 7, Eth_myip[0], Eth_myip[1], Eth_myip[2], Eth_myip[3], src_ip[0], src_ip[1], src_ip[2], 23 };
  if (ReplyBufferAGIO[0] == 0x80 && ReplyBufferAGIO[1] == 0x81 && ReplyBufferAGIO[2] == 0x7F) //Data
  {
    //Serial.println(ReplyBufferAGIO[3]);
    if (ReplyBufferAGIO[3] == 202)
    {
      // Serial.println("202 0xCA - AgIO scan request");
      //make really sure this is the subnet pgn
      if (ReplyBufferAGIO[4] == 3 && ReplyBufferAGIO[5] == 202 && ReplyBufferAGIO[6] == 202)
      {
        //checksum
        CK_A = 0;
        for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++)
        {
          CK_A = (CK_A + scanReply[i]);
        }
        scanReply[sizeof(scanReply) - 1] = CK_A;
        static uint8_t ipDest[] = { 255, 255, 255, 255  };

        EthUDPToAOG.beginPacket(ipDest, portDestination); //portDestination
        EthUDPToAOG.write(scanReply, sizeof(scanReply));
        EthUDPToAOG.endPacket();
        send_IP_back = true;
        send_IP_back_time = millis();
      }
    }
  }
  if (send_IP_back) {
    static uint8_t ipDest[] = { 255, 255, 255, 255  };
    EthUDPToAOG.beginPacket(ipDest, portDestination); //portDestination
    EthUDPToAOG.write(scanReply, sizeof(scanReply));
    EthUDPToAOG.endPacket();
  }
  if ((millis() - send_IP_back_time) > 60000)
    send_IP_back = false;

}



//-------------------------------------------------------------------------------------------------
