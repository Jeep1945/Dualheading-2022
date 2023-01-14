void Eth_Start() {
  Ethernet.init(Eth_CS_PIN);
  //  delay(50);
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
      if (Eth_udpPAOGI.begin(portMy)) // portMy  portDestination
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
    }
    //    Serial.println();
  }
}
