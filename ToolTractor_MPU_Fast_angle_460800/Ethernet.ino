void Eth_Start() {
  delay(5000);
  Ethernet.init(Eth_CS_PIN);
  delay(200);
  Eth_myip[2] = Ethernet_3rd;
  Ethernet.begin(mac, Eth_myip);
  delay(200);
  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :");
    Ethernet_running = false;
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
      Serial.print("Ethernet sending to IP:     "); Serial.println(Eth_ipDestination);
      //init UPD Port sending to AOG
      if (Eth_udpPAOGI.begin(portMy)) // portMy
      {
        Serial.print("Ethernet UDP sending from port: ");
        Serial.println(portMy);
      }

      //init Ntrip sending to Tool
      if (Eth_udpTool.begin(7777)) // localPort
      {
        Serial.print("Ethernet UDP sending to tool: ");
        Serial.println("7777");
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
        Serial.print("UDP 210 listening on port: ");
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
  if (debugmode)
  {
    Serial.print(" 0 ");
    Serial.print(ReplyBufferAGIO[0]);
    Serial.print(" 1 ");
    Serial.print(ReplyBufferAGIO[1]);
    Serial.print(" 2 ");
    Serial.print(ReplyBufferAGIO[2]);
    Serial.print(" 3 ");
    Serial.print(ReplyBufferAGIO[3]);
    Serial.print(" 4 ");
    Serial.println(ReplyBufferAGIO[4]);
  }
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
        // static uint8_t ipDest[] = { 255, 255, 255, 255  };

        //EthUDPToAOG.beginPacket(ipDest, portDestination); //portDestination
        //EthUDPToAOG.write(scanReply, sizeof(scanReply));
        //EthUDPToAOG.endPacket();
        send_IP_back = true;
        send_IP_back_time = millis();
      }
    }
    else if (ReplyBufferAGIO[3] == 210)  //210 0xD2
    {
      Heading_MPU6050 = ((int)ReplyBufferAGIO[5]);
      Roll_MPU6050 = ((int)ReplyBufferAGIO[6]);
      Heading_Dual_MPU = ((int)ReplyBufferAGIO[7]) + 1;
      Roll_Dual_MPU = ((int)ReplyBufferAGIO[8]) + 1;
      Headingfilter = ((int)ReplyBufferAGIO[9]) + 1;
      Rollfilter = ((int)ReplyBufferAGIO[10]) + 1;
      isToolAntenna = ((int)ReplyBufferAGIO[11]);
      isData_Refresh = ((int)ReplyBufferAGIO[12]);

      if (((Heading_MPU6050 == 1) || (Roll_MPU6050 == 1)) && (gyroOffset == -3))
      {
        mpu.begin();
        mpu.calibrateGyro(1000);
        mpu.printSettings();
      }

      EEPROM.put(6, Heading_MPU6050);
      EEPROM.put(7, Roll_MPU6050);
      EEPROM.put(8, Heading_Dual_MPU);
      EEPROM.put(9, Roll_Dual_MPU);
      EEPROM.put(10, Headingfilter);
      EEPROM.put(11, Rollfilter);
      EEPROM.put(12, isToolAntenna);
      EEPROM.put(13, isData_Refresh);
      EEPROM.commit();

      if (debugmode)
      {
        Serial.print("gyroOffset  ");
        Serial.println(gyroOffset);
        Serial.print("Heading_MPU6050  ");
        Serial.println(Heading_MPU6050);
        Serial.print("Roll_MPU6050  ");
        Serial.println(Roll_MPU6050);
        Serial.print("Heading_Dual_MPU  ");
        Serial.println(Heading_Dual_MPU);
        Serial.print("RollMPU  ");
        Serial.println(Roll_Dual_MPU);
        Serial.print("Headingfilter  ");
        Serial.println(Headingfilter);
        Serial.print("Rollfilter  ");
        Serial.println(Rollfilter);
        Serial.print("isToolAntenna  ");
        Serial.println(isToolAntenna);
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

void Data_MPU_SendToAOG()
{
  if (isData_Refresh == 1)
  {    
    PGN_209[5] = 1;

    int16_t CK_A = 0;
    for (uint8_t i = 2; i < PGN_209_Size; i++)
    {
        CK_A = (CK_A + PGN_209[i]);
    }
    PGN_209[PGN_209_Size] = CK_A;

    static uint8_t ipDest[] = { 192, 168, 1, 255  };
    EthUDPToAOG.beginPacket(ipDest, portDestination); //portDestination
    EthUDPToAOG.write(PGN_209, sizeof(PGN_209));
    EthUDPToAOG.endPacket();

  }
}

//-------------------------------------------------------------------------------------------------
