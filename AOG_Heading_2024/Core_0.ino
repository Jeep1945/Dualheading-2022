//Core1:  this task handles the Wifi and NTRIP Client

void Core1code( void * pvParameters ) {

  Serial.print("Check_connections : ");
  Check_connections();
  Serial.println(Conn);
  if (my_WiFi_Mode != 2) {
    Start_connections();

    if ((my_WiFi_Mode == 0) && (OTA_active) || (Ntriphotspot == 3))  WiFi_Start_AP();
  }
  if ((my_WiFi_Mode != 0) && (OTA_active))  {
    Serial.println("Hallo OTA is activated");
    OTA_update_ESP32();
  }

  for (;;) { // MAIN LOOP FOR THIS CORE 0

    // read Date from AGIO, to send IP addres
    if (send_Data_Via == 1)  {
      packetLength = EthUDPFromAOG.parsePacket();
      if (packetLength > 0)  read_Eth_AGIO();
    }

    // read Date from MPU6050, and/or from 3 buttons to move the line
    if (move_line_buttons == 1) button_linemove();

    // connect ethernet again
    buttonState_Eth = digitalRead(Ethernet_reset);
    if ((buttonState_Eth == 0) && (Ntriphotspot < 2) && (send_Data_Via == 1) && (!Ethernet_running))
      Eth_Start();

    // if ntrip lost, try do connect with second Caster
    if (((Ntriphotspot == 1) || (Ntriphotspot == 2)) && (Ntriphotspot_an == 0) && ((WiFi.status() == WL_CONNECTED) || (Ethernet_running))) {   //  if Ntrip should work
      Ntrip_choice();
    }

    //  End Accesspoint
    if ((my_WiFi_Mode == 2) && (millis() - OTA_begin_Time > timeoutRouter * 1000)) {
      WiFi.mode(WIFI_OFF);
      Serial.println("End Accesspoint");
      (my_WiFi_Mode = 0);
    }

    //  Start a WiFi scan with pressing the button
    buttonState = digitalRead(Button_ReScan);
    if ((buttonState == 0) && (ntrip_from_AgopenGPS == 0) && (Ntriphotspot > 1)) {
      WiFi.mode(WIFI_OFF);
      digitalWrite(LED_ntrip_ON, LOW);
      Network_built_up();
      if (((my_WiFi_Mode == 0) || (my_WiFi_Mode == 2)) && (OTA_active))  WiFi_Start_AP();
      OTA_begin_Time = millis();
      if (OTA_active) OTA_update_ESP32();
    }

    //  Start up WiFi connection if lost
    if ((Ntrip_WiFi) && (WiFi.status() != WL_CONNECTED)) {
      Network_built_up();
    }

    //  receive RTCM3 data by USB from AOG #############################################
    if ((send_Data_Via == 0) && (Ntriphotspot == 0)) {
      if (Serial.available()) {      // If RTCM3 comes in Serial (USB),
        char C = Serial.read();      // read a byte, then
        SerialNmea.write(C);            // send it out SerialNmea from PIN 16 to simpleRTK RX1 1. Antenna = RTCM
        if (C != '  ') {             // if the byte is a newline character
          ntripcheck();
        }
        ntriptime_from_AgopenGPS = millis();
      }
      else {
        ntripcheck();
      }
    }

    // If RTCM3 comes in received by WiFi from Router ####################################
    if ((send_Data_Via == 0) && (Ntriphotspot == 2)) { //  Ntrip_begin_Time
      if (ntrip_c.available()) {         // If RTCM3 comes in received by WIFI
        SerialNmea.write(ntrip_c.read());   // read RTCM3  and send from ESP32 16 to simpleRTK RX 1. Antenna = RTCM
        //Serial.println("send_Data_Via == 0) && (Ntriphotspot == 2");
      }
    }

    //  receive RTCM3 data by Ethernet from AOG  ##########################################
    //   if ((send_Data_Via == 1) && (Ntriphotspot == 0)
    if ((Ethernet_running) && (Ntriphotspot == 0)) {
      doEthUDPNtrip();  // If RTCM3 comes in received by Ethernet from AOG
    }

    // If RTCM3 comes in received by Ethernet from Router ####################################
    if ((send_Data_Via == 1) && (Ntriphotspot == 1) && (ntrip_from_AgopenGPS == 0) && (Ethernet_running)) { //  Ntrip_begin_Time
      if (ntrip_e.available()) {         // If RTCM3 comes in received by Ethernet
        SerialNmea.write(ntrip_e.read());   // read RTCM3  and send from ESP32 16 to simpleRTK RX 1. Antenna = RTCM
        //Serial.println("send_Data_Via == 1) && (Ntriphotspot == 1");
      }
    }

    // If RTCM3 comes in received by Ethernet from Router ####################################
    if ((send_Data_Via == 1) && (Ntriphotspot == 2) && (Ethernet_running)) { //  Ntrip_begin_Time
      if (ntrip_c.available()) {         // If RTCM3 comes in received by WIFI
        SerialNmea.write(ntrip_c.read());   // read RTCM3  and send from ESP32 16 to simpleRTK RX 1. Antenna = RTCM
        //Serial.println("send_Data_Via == 1) && (Ntriphotspot == 2");
      }
    }

    // If RTCM3 comes in received by WiFi from Tablett ####################################
    if ((send_Data_Via == 1) && (Ntriphotspot == 3) && (Ethernet_running)) { //  Ntrip_begin_Time
      doUDPNtrip_WiFi();  // If RTCM3 comes in received by WiFi
    }

    //  receive RTCM3 data by WiFi from Hotspot    ############################################
    if ((send_Data_Via == 2) && (Ntriphotspot == 2) && (ntrip_from_AgopenGPS == 0) && (WiFi.status() == WL_CONNECTED)) { //  Ntrip_begin_Time
      if (ntrip_c.available()) {         // If RTCM3 comes in received by WIFI
        SerialNmea.write(ntrip_c.read());   // read RTCM3  and send from ESP32 16 to simpleRTK RX 1. Antenna = RTCM
        //Serial.println("send_Data_Via == 2) && (Ntriphotspot == 2");
      }
    }

    //  receive RTCM3 data by WiFi from Tablet    ############################################
    if ((send_Data_Via == 3) && (Ntriphotspot == 3) && (ntrip_from_AgopenGPS == 0) && (my_WiFi_Mode == 2)) { //  Ntrip_begin_Time
      doUDPNtrip_WiFi();  // If RTCM3 comes in received by WiFi
    }

    //  receive RTCM3 data by WiFi from Router(AOG)   ########################################
    if ((send_Data_Via == 2) && (Ntriphotspot == 3)) {
      doUDPNtrip_WiFi();  // If RTCM3 comes in received by WiFi
    }

    //  read NMEA msg from F9P (PVT) and pars them in NMEA_read()   ##############################################
    while (SerialNmea.available()) { // If anything comes in Serial1
      parser << SerialNmea.read();
      // read it and send for NMEA_PAOGI
    }

    //  Send GGA MSG back to Base     ###########################################################################
    if ((GGA_Send_Back_Time != 0) && ((send_Data_Via == 2) || (Ntriphotspot == 2)))  sendGGA_WiFi();
    if ((GGA_Send_Back_Time != 0) && (Ntriphotspot == 1))  sendGGA_Eth();

  } // End of (main core 0)
} // End of core1code

//###########################################################################################
