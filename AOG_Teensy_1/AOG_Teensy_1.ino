
String VERS = "AOG_Teensy_1.1_25.01.2022";

// Dualhead for AGopenGPS
// Private-use only! (you need to ask for a commercial-use)
// by Franz Husch  31.12.2020 in cooperation with Matthias Hammer
// and Valentin Ernst
// **** OTA is possible over Network  **** //
// Antennas cross to driveDirection on cabin symmetrical
// right Antenna is Rover (A) for position, left Antenna is MB (B) for heading
// Ntrip client for 7 Router or Handy hotspots or from AgopenGPS
// Progamm tries one time to connect,
// you can install a button from GND to PIN 4
// by pressing the button a new WiFi scan starts or Accesspoint for OTA.
// PIN 2 you can add an LED for ntrip is received, have a look to the Photo
// LED is on when no Network could be connected
// Dual_Antenna  0: for single antenna, 1: for Dualantenna
// send_amatron_nmea  to send GGA,VTG,GSA and ZDA to a second ESP32 0: for not 1: for sending,
// rollaktiv (0) for use roll in AgopenGPS, offset and hight of Antenna set in setup AOG
// rollaktiv (1) for use roll by ESP32, offset and hight of Antenna set to 0 in setup AOG
// WiFi_scan_Delay is the mount of sec, you will need start router or hotspot
// in Data Sourcess
// rollaktiv is 0, is done in AOG, so Antenna offset and hight is to do in AOG
// GGA_Send_Back_Time for SAPOS or Apos set to 10,
///by Fix "OGI", Heading GPS "Dual"
//  IMPORTANT  // For serial USB 38400 baude not 115400
//  IMPORTANT  // you have to use the new PVT config

//  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  +++++++++++++++++++++++++++++++  BEGIN Setup +++++++++++++++++++++++++++++++++++++++
//  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int AntDistance = 95;       // distance between the two antennas in cm,+, 0 for automatic distance
double headingcorr = 90;     // right antenna A , left antenna B;
int tractorhight = 280;   // roll is in Position calculated, in AgOpenGPS mit 0 cm
int Dual_Antenna = 1;  // 1: for Dualantenna, 0: for single antenna;
int rollaktiv = 0;     // 0: roll in AOG  1: roll activated in Dualheading
//  IMPORTANT  // For serial USB 115200 baude 
// you have to use the new PVT config
int send_Data_Via = 0;
int Ntriphotspot = 0;

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
//libraries -------------------------------
#include <Wire.h>
#include <math.h>


//loop time variables in microseconds
long lastTime = 0;
float roll = 0;
byte XOR;
char c;
char b;
String t;


unsigned long Single_begin_Time = millis();
unsigned long Single_begin_Time_VTG = millis();
int durchlauf_nord = 0, durchlauf_east = 0, WiFi_scan_Attempt = 1;
double nordWinkel_old, eastWinkel_old;
int buttonState = 0;


// Heading
double heading, headingUBX, headingzuvor = 0, headingzuvorVTG;
double headingUBXmin, headingUBXmax, headingVTGmin, headingVTGmax;
double speeed = 0, headingnord;


// roll
float rollCorrectionDistance = 0.00;
double rollnord = 0.0, rolleast = 0.0;
double rollnord1 = 0.0, rolleast1 = 0.0;
double rollnord_before = 0.0, rolleast_before = 0.0;
double relPosD, relPosDH;
double rollzuvor = 0;
double PI180 = 57.295791;
double baseline, baseline1, baselineHorizontal;
double fixnorddeci, fixeastdeci; // coordinates in decimalen
double fixnorddeci_old, fixeastdeci_old; // coordinates in decimalen

byte CK_A = 0, CK_B = 0;
byte incoming_char;

// NMEA erstellen
int argGGA_Anz[17], argGGA_Anz_ende;
int inByte;
String argGGA9, GGA_Sealevel, GGA_Age;
String argGGA10;
String argGGA11, GGA_time;
int start = 0, GGASats;
String nmea = "", GGAdaten = "", GGAdaten1 = "", VTGdaten = "", VTGspeed = "", VTGheadingnord = "";
String VTGSatz = "", GGASatz_old = "", GGAnord = "", GGAeast = "", GGAZeit = "", GGAWestEast = "", GGANordSued = "";
int j = 0, j2 = 0, jGGA = 0, jGGA2 = 0, jGGA3 = 0, jGGA4 = 0, jGGA5 = 0, jGGA6 = 0, jGGA7 = 0, jGGA78 = 0;
int jVTG1 = 0, jVTG2 = 0, jVTG3 = 0, jVTG4 = 0, jVTG5 = 0;
String GPSquali = "", WEcoordinaten, NScoordinaten, GGASat, GGAHDop;
String GGASatz = "", GGASatz_Korr, VTGSatz_Korr = "", GGASatz_send_back = "";
int GPSqualin1 = 0, GPSqualin2 = 1, GPSqualinzuvor = 1, GPSqualintime = 1, GGA_check = 0;
String GGA_hDops, ZDASatz = "", GGA_hDop, GGA_seahigh;
int  i = 0, ij = 0;
double GGAZeitNummerbevor, GGAZeitNummer;
double GGAage, GGA_seahighs;

// PAOGI erstellen
bool Paogi_true_UBX = true, Paogi_true = true;
String RollHeadingrest = "", RollHeadingshit = "", RollHeadingrest_befor = "", BS = ",";
int Paogi_Long, Coodinate_check1, Coodinate_check2, heading_check1 = 0;
int Paogi_Long1, Coodinate1_check1, Coodinate1_check2, heading1_check1 = 0;
int Paogi_Shit1 = 0;

byte UDPPAOGIMsg[100], UDPVTGMsg[40], UDPGGAMsg[90];
unsigned int Bytelaenge, BytelaengeVTG, BytelaengeGGA;
byte  ReplyBufferVTG[40] = "", ReplyBufferGGA[90] = "";       // a string to send back

//UBX
double speedUBXint, UBX_lon, UBX_lat;
int UBX_lon_int, UBX_lat_int;
double UBX_lon_double, UBX_lat_double;
int UBX_fixtypei, UBX_Sealeveli;
String speedUBXstr, GGA_Sats;
double UBX_Sats, UBX_Sealeveld, UBX_DOP, UBX_fixtype, UBX_Sealevel;


// Chechsum controll
String checksum = "", checksum_GGA = "", checksum_GGA_send = "", checksum_VTG = "", checksum_VTG_send = "";
String check_headingroll = "";
int j_checksum_GGA = 0, j_checksum_VTG = 0;

union UBXMessage {
  struct {
    unsigned char HeaderA;
    unsigned char HeaderB;
    unsigned char cls;
    unsigned char id;
    unsigned short len;
    unsigned char reserved;
    unsigned char reserved1;
    unsigned short refStationId;
    unsigned long iTOW;
    long relPosN;
    long relPosE;
    long relPosD;
    long relPosLength;
    long relPosHeading;
    unsigned long reserved2;
    char relPosHPN;
    char relPosHPE;
    char relPosHPD;
    char relPosHPLength;
    unsigned long accN;
    unsigned long accE;
    unsigned long accD;
    unsigned long accLength;
    unsigned long accHeading;
    unsigned long reserved3;
    unsigned long flags;
  } relposned;
  byte rawBuffer[72];
} ubxmessage;


//bool debugmode = true;  // GGA,VTG,
bool debugmode = false;
//bool debugmode1 = true;  // Heading
bool debugmode1 = false;
//  bool debugmode2 = true;  // Deviation
bool debugmode2 = false;
//  bool debugmode3 = true;  // roll
bool debugmode3 = false;
//bool debugmode_UBX = true;  //  Protocoll UBX einlesen
bool debugmode_UBX = false;
//  bool debugProtokoll = true;  //Protocoll TestStation
bool debugProtokoll = false;
//bool debugmode_amatron = true;  //Protocoll Amatron
bool debugmode_amatron = false;



// Setup procedure ------------------------
void setup() {
  ubxmessage.rawBuffer[0] = 0xB5;
  ubxmessage.rawBuffer[1] = 0x62;
  ubxmessage.rawBuffer[2] = 0x01;
  ubxmessage.rawBuffer[3] = 0x3C;

  delay(10);
  Serial.begin(115200);
  delay(10);
  Serial.println("");
  Serial.println(VERS);
  Serial.println("");
  Serial.println("Start setup");
  Serial1.begin(115200);
  delay(10);
  if (Dual_Antenna == 1) {
    Serial2.begin(115200);
  }
  Serial.println("");
  Serial.println("End setup");
  Serial.println("");

}


void loop() {

  //  receive RTCM3 data by USB from AOG #############################################
  if (Serial.available()) {      // If RTCM3 comes in Serial (USB),
    char C = Serial.read();      // read a byte, then
    Serial1.write(C);            // send it out Serial1 from PIN 16 to simpleRTK RX1 1. Antenna = RTCM
  }

  //  read NMEA msg from F9P (PVT) and pars them in NMEA_read()   ##############################################
  if (Serial1.available()) { // If anything comes in Serial1
    inByte = Serial1.read(); // read it and send for NMEA_PAOGI
    NMEA_read();
  }

  //  read UBX msg from F9P (heading)    ######################################################################
  if (Dual_Antenna == 1) {
    if (Serial2.available()) {         // If anything comes in Serial2
      incoming_char = Serial2.read();  // ESP32 read RELPOSNED from F9P
      if (i < 4 && incoming_char == ubxmessage.rawBuffer[i]) {
        i++;
      }
      else if (i > 3) {
        ubxmessage.rawBuffer[i] = incoming_char;
        i++;
      }
    }
    if (i > 71) {
      CK_A = 0;
      CK_B = 0;
      for (i = 2; i < 70 ; i++) {
        CK_A = CK_A + ubxmessage.rawBuffer[i];
        CK_B = CK_B + CK_A;
      }

      if (CK_A == ubxmessage.rawBuffer[70] && CK_B == ubxmessage.rawBuffer[71]) {

        heading_relposnet();
        rollundheading();   // calculate roll        ########################################################
        PAOGI1_builder();   // built the PAOGI MSG   ########################################################
      }
      else {
        // Serial.println("ACK Checksum Failure: ");
      }
      i = 0;
    }
  }
}//end main loop

//------------------------------------------------------------------------------------------
