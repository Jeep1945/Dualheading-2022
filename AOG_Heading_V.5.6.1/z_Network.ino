//scan_WiFi_connections---------------------------------------------------------------------------------------------

void scan_WiFi_connections()
{
  delay(WiFi_scan_Delay_Time);
  Serial.println("");
  Serial.println("scan start " + String(WiFi_scan_Attempt));
  net_found = 0;
  // WiFi.scanNetworks will return the number of networks found
  net_found = WiFi.scanNetworks();
  Serial.println("scan done");
  if (net_found == 0) {
    Serial.println("no networks found");
    WiFi_netw_nr = 0;
  }
  else
  {
    network_found = false;
    Serial.print(net_found);
    Serial.println(" networks found");
    for (int i = 0; i < net_found; ++i) {
      Serial.println(String(i + 1) + ". Netzwerk : " + WiFi.SSID(i));
    }
    delay(2000);
    Serial.println("");
    WiFi_netw_nr = 1;
    for (int i = 0; i < net_found; ++i) {
      if (WiFi.SSID(i) == WIFI_Network1) {
        Serial.println("possible to connect with : " + WiFi.SSID(i));
        network_found = true;  // network found in list
        WiFi_netw[1] = 1;
      }
      if (WiFi.SSID(i) == WIFI_Network2) {
        Serial.println("possible to connect with : " + WiFi.SSID(i));
        network_found = true;  // network found in list
        WiFi_netw[2] = 1;
      }
      if (WiFi.SSID(i) == WIFI_Network3) {
        Serial.println("possible to connect with : " + WiFi.SSID(i));
        network_found = true;  // network found in list
        WiFi_netw[3] = 1;
      }
      if (WiFi.SSID(i) == WIFI_Network4) {
        Serial.println("possible to connect with : " + WiFi.SSID(i));
        network_found = true;  // network found in list
        WiFi_netw[4] = 1;
      }
      if (WiFi.SSID(i) == WIFI_Network5) {
        Serial.println("possible to connect with : " + WiFi.SSID(i));
        network_found = true;  // network found in list
        WiFi_netw[5] = 1;
      }
      if (WiFi.SSID(i) == WIFI_Network6) {
        Serial.println("possible to connect with : " + WiFi.SSID(i));
        network_found = true;  // network found in list
        WiFi_netw[6] = 1;
      }
      if (WiFi.SSID(i) == WIFI_Network7) {
        Serial.println("possible to connect with : " + WiFi.SSID(i));
        network_found = true;  // network found in list
        WiFi_netw[7] = 1;
      }
    }
  }
  for (int i = 0; i < 8; ++i) {
    if (WiFi_netw[i] == 1) WiFi_netw_nr = i;
  }
  if (WiFi_scan_Attempt > 4) {
    ESP.restart();
  }
  else delay(10000);
  if (network_found) WiFi_scan_Attempt = 1;
  else WiFi_scan_Attempt++;

}  //end scan_WiFi_connections()

//WiFi---------------------------------------------------------------------------------------------

void connectToWiFi() {
  if (!network_found) {
    delay(10000);
    scan_WiFi_connections();
  }
  else {
    WiFi.mode(WIFI_STA);
    //      Serial.println(WiFi_netw_nr);
    switch (WiFi_netw_nr) {
      case 0:  break;
      case 1: WiFi.begin(WIFI_Network1, WIFI_Password1); break;
      case 2: WiFi.begin(WIFI_Network2, WIFI_Password2); break;
      case 3: WiFi.begin(WIFI_Network3, WIFI_Password3); break;
      case 4: WiFi.begin(WIFI_Network4, WIFI_Password4); break;
      case 5: WiFi.begin(WIFI_Network5, WIFI_Password5); break;
      case 6: WiFi.begin(WIFI_Network6, WIFI_Password6); break;
      case 7: WiFi.begin(WIFI_Network7, WIFI_Password7); break;
    }

    //    Serial.print("try to connect to WiFi: ");

    unsigned long startAttemptTime = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS) {
      delay(300);
      //      Serial.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected");
      Serial.println("WiFi offline ");
      Ntriphotspot = 0;
    }
    if (WiFi.status() == WL_CONNECTED) {
      delay(200);
      Serial.println();
      Serial.println("WiFi Client successfully connected to : ");
      // Serial.print("Username : ");
      switch (WiFi_netw_nr) {
        case 0:  break;
        case 1: Serial.print(" WIFI_Network1  : "); Serial.println(WIFI_Network1); Serial.print(" WIFI_Password1 : "); Serial.println(WIFI_Password1); break;
        case 2: Serial.print(" WIFI_Network2  : "); Serial.println(WIFI_Network2); Serial.print(" WIFI_Password2 : "); Serial.println(WIFI_Password2); break;
        case 3: Serial.print(" WIFI_Network3  : "); Serial.println(WIFI_Network3); Serial.print(" WIFI_Password3 : "); Serial.println(WIFI_Password3); break;
        case 4: Serial.print(" WIFI_Network4  : "); Serial.println(WIFI_Network4); Serial.print(" WIFI_Password4 : "); Serial.println(WIFI_Password4); break;
        case 5: Serial.print(" WIFI_Network5  : "); Serial.println(WIFI_Network5); Serial.print(" WIFI_Password5 : "); Serial.println(WIFI_Password5); break;
        case 6: Serial.print(" WIFI_Network6  : "); Serial.println(WIFI_Network6); Serial.print(" WIFI_Password6 : "); Serial.println(WIFI_Password6); break;
        case 7: Serial.print(" WIFI_Network7  : "); Serial.println(WIFI_Network7); Serial.print(" WIFI_Password7 : "); Serial.println(WIFI_Password7); break;
      }
      Serial.print("IP address: ");
      IPAddress myIP = WiFi.localIP();
      Serial.println(myIP);
      //after connecting get IP from router -> change it to x.x.x.IP Ending (from settings)
      myIP[3] = myIPEnding; //set ESP32 IP to x.x.x.myIP_ending
      Serial.print("changing IP to: ");
      Serial.println(myIP);
      IPAddress gwIP = WiFi.gatewayIP();
      if (!WiFi.config(myIP, gwIP, mask, gwIP)) {
        Serial.println("Network failed to configure");
      }
      delay(200);
      Serial.print("Connected IP - Address : ");
      myIP = WiFi.localIP();
      ipDestination1 = myIP;
      ipDestination1[3] = 255;
      Serial.println(ipDestination1);
      Serial.print("Gateway IP - Address : ");
      Serial.println(gwIP);
      ipDestination[0] = myIP[0];
      ipDestination[1] = myIP[1];
      ipDestination[2] = myIP[2];
      ipDestination[3] = 255;//set IP to x.x.x.255 according to actual network
      digitalWrite(LED_ntrip_ON, LOW);
      my_WiFi_Mode = 1;// WIFI_STA;
      if (udpNtrip.listen(AOGNtripPort))
      {
        Serial.print("NTRIP UDP Listening to port: ");
        Serial.println(AOGNtripPort);
        Serial.println();
      }
      delay(50);
      if (udpRoof.listen(portMy))
      {
        Serial.print("UDP writing to IP: ");
        Serial.println(ipDestination1);
        Serial.print("UDP writing to port: ");
        Serial.println(portDestination);
        Serial.print("UDP writing from port: ");
        Serial.println(portMy);
      }
      delay(200);
    }
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("Connection Failed! Rebooting...");
      delay(5000);
      Ntriphotspot = 0;
      //    ESP.restart();
    }
  }
}  // end connectToWiFi

//---------------------------------------------------------------------
// start WiFi Access Point = only if no existing WiFi

void WiFi_Start_AP() {
  WiFi.mode(WIFI_AP);   // Accesspoint
  WiFi.softAP(ssid_ap, "");
  while (!SYSTEM_EVENT_AP_START) // wait until AP has started
  {
    delay(100);
    Serial.print(".");
  }
  delay(100);//right IP adress only with this delay
  WiFi.softAPConfig(gwip, gwip, mask);  // set fix IP for AP
  delay(300);
  IPAddress myIP = WiFi.softAPIP();
  delay(300);

  //AP_time = millis();
  Serial.print("Accesspoint started - Name : ");
  Serial.println(ssid_ap);
  Serial.print( " IP address: ");
  ipDestination1 = myIP;
  ipDestination1[3] = 255;
  Serial.println(ipDestination1);
  server.begin();
  if (udpNtrip.listen(AOGNtripPort))
  {
    Serial.print("NTRIP UDP Listening to port: ");
    Serial.println(AOGNtripPort);
    Serial.println();
  }
  delay(200);
  my_WiFi_Mode = 2;
}

//-------------------------------------------------------------------------------------------------
void OTA_update_ESP32()  {
  Serial.println("Hallo");
  ArduinoOTA.setHostname("Dualesp32");
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  Serial.println("Ready for OTA ");
}

//-------------------------------------------------------------------------------------------------
