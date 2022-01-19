void  Check_connections() {
  //bool Ntrip_AOG_USB = false, Ntrip_AOG_Eth = false, Ntrip_WiFi = false, Ntrip_Eth_router = false;

  switch (send_Data_Via)  {
    case 0: {
        if ((Ntriphotspot != 0) && (Ntriphotspot != 2))  {
          Ntriphotspot = 0;
          Conn = " choice is not correct or impossible";
        }
        else {
          if (Ntriphotspot == 2) Ntrip_WiFi = true;
        }
        break;
      }
    case 1: {
        Ethernet_need_AOG = true;
        if (Ntriphotspot == 0) Ethernet_need_AOG = true;
        if (Ntriphotspot == 1) Ntrip_Eth_router = true;
        if (Ntriphotspot == 2) Ntrip_WiFi = true;
        if (Ntriphotspot == 3) Ntrip_WiFi = true;
        break;
      }
    case 2:  {
        if ((Ntriphotspot != 2) && (Ntriphotspot != 3))  {
          Ntriphotspot = 2;
          Ntrip_WiFi = true;
          Conn = " choice is not correct or 1 impossible";
        }
        else {
          if (Ntriphotspot == 2) Ntrip_WiFi = true;
          if (Ntriphotspot == 3) Ntrip_WiFi = true;
        }
      }
      break;
    default: break;
    
  }
} // end Check_connection

//  ##########################################################################

void  Start_connections() {

  if ((!client_Eth.connected()) && (Ethernet_need_AOG)) {   //  start Ethernetconnection
    Serial.println("Start Ethernet");
    Eth_Start();
    //    if (!client_Eth.connected())  send_Data_Via = 0;         //  if no Ethernetconnection found, send data via USB
  }
  delay(2000);
  if (Ntrip_Eth_router) {   //  start Ethernetconnection send data and Ntrip from Router
    Serial.println("");
    Serial.println("Start Ntrip with Ethernet on Router");
    connectToNtrip_Eth();
    delay(2000);
  }
  if (Ntrip_WiFi) {   //  start WiFi for Router
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("");
      Serial.println("Start WiFi");
      Network_built_up();
    }  else  Ntriphotspot_an = 1;
  }
} // end Check_connection

//**************************************************************************************

void Network_built_up() {
  Ntriphotspot_an = 0;
  digitalWrite(LED_ntrip_ON, HIGH);
  delay(1000);
  scan_WiFi_connections();
  delay(1000);
  if ((WiFi_netw_nr > 0) && (Ntrip_WiFi)) {
    connectToWiFi();
    delay(1000);
    if ((WiFi.status() == WL_CONNECTED) && (Ntrip_WiFi) && (Ntriphotspot != 3)) {
      Serial.println("");
      Serial.println("Start Ntrip with WiFi on Router");
      connectToNtrip_WiFi();
    }
  }
}// end Network_built_up
//**************************************************************************************
//**************************************************************************************
