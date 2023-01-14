//#include <WiFi.h>
//#include <ESPmDNS.h>

String Version = "2_ESP_Baudwandler_23.12.2022";
//UART1 Data from ESP32 "DualGPS"
#define RXD1 27
#define TXD1 16

//UART2 Data to TTL/Serial Wandler
#define RXD2 25
#define TXD2 17

#define herz1 190
#define herz2 980

int send_Time1Hz, send_Time5Hz,  looptime;
int inByte, start = 0, i , j, m;
String nmea = "";
char GGASentence[100] = "";
char VTGSentence[100] = "";
char GSASentence[100] = "";
char ZDASentence[100] = "";

bool debugmode_amatron = true;  //Protocoll Amatron
//bool debugmode_amatron = false;



void setup() {

  Serial.begin(38400);                                   //UART0 USB;
  delay(50);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);         //UART1 read NMEA from DUAL ESP32
  delay(10);
  Serial2.begin(19200, SERIAL_8N1, RXD2, TXD2);          //UART2 send NMEA to tool terminal (Amatron)
  //delay(10);
  Serial.println("");  Serial.println("");  Serial.println("");
  Serial.println("Version  : ");
  Serial.print(Version);
  Serial.println("");   Serial.println("");
  Serial.println("Serial1 TXD1 connected to Pin: " + String(TXD1));
  Serial.println("Serial1 RXD1 connected to Pin: " + String(RXD1));
  Serial.println("");
  Serial.println("Serial2 TXD2 connected to Pin: " + String(TXD2));
  Serial.println("Serial2 RXD2 connected to Pin: " + String(RXD2));
  Serial.println("");
  Serial.println("");

  send_Time5Hz = millis();
  send_Time1Hz = millis();

}  // end of setup

void loop() {

  if (Serial1.available()) { // If anything comes in Serial1
    inByte = Serial1.read(); // read it and send for PARSER
    MSG_build();
  }

  if ((millis() - send_Time5Hz) > herz1) {
    Serial2.println(GGASentence);
    Serial2.println(VTGSentence);
    send_Time5Hz = millis();
    if (debugmode_amatron) {
      Serial.println(GGASentence);
      Serial.println(VTGSentence);
    }
  }

  if ((millis() - send_Time1Hz) > herz2) {
    Serial2.println(GSASentence);
    Serial2.println(ZDASentence);
    send_Time1Hz = millis();
    if (debugmode_amatron) {
      Serial.println(ZDASentence);
    }
  }

} // end of loop
