String VERS = "Version ToolTractor_MPU_Fast_angle 460800 08.10.2025";
//
// AAA_Readme for instructions

//  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  +++++++++++++++++++++++++++++++  BEGIN Setup +++++++++++++++++++++++++++++++++++++++
//  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

byte Eth_myip[4] = { 192, 168, 1, 124 };//IP address to send UDP data to
//byte Eth_myip[4] = { 192, 168, 1, 124};//IP address to send UDP data to
//byte Eth_myip[4] = { 10, 0, 0, 124 };//IP address to send UDP data to
byte Ethernet_3rd = 1 ;     //{ 192, 168,     [1]   , 124 };//3rd nummer of IP address to send UDP data to

int AntDistance = 115;       // distance between the two antennas in cm,+,
int tractorhight = 290;   // roll is in Position calculated,
int Dual_Antenna = 1;  // 1: for Dualantenna, 0: for single antenna;
int send_amatron_nmea = 0;    // 1: for sending, 0: for not
int isToolAntenna = 0;    // 1: for sending, 0: for not isToolAntenna
int GNGGAorGPGGA = 1 ;  // GNGGA = 1, GPGGA= 2, when send_amatron_nmea = 1;
byte Eth_CS_PIN = 5;       //CS PIN with SPI Ethernet hardware W 5500  SPI config: MOSI 23 / MISO 19 / CLK18 / CS5, GND, 3.3V
int IMU_MPU6050_direction = 2;       //Drivedirection  Y=1:  -Y=2:  X=3:  -X=4:
int IMU_MPU6050 = 0;        // 0: No MPU  1: from MPU6050 installed
int Heading_MPU6050 = 1;    // 0: No MPU  1: MPU6050 for heading used
int Roll_MPU6050 = 1;       // 0: No MPU  1: MPU6050 for roll used
int Heading_Dual_MPU = 1;         // from 1 to 10 1: from Dual   10: from MPU
int Roll_Dual_MPU = 1;      // from 1 to 10 1: from Dual   10: from MPU
int Headingfilter = 1;      // 1: no   10: most filter
int Rollfilter = 1;        // 1: no   10: most filter
int move_line_buttons = 0;       // 0: no   1: buttons to move AB line
int WiFi_scan_Delay = 10;      // wait to scan, for router use 50 sec delay
int Ethernet_Delay = 2;     // wait to connect Ethernet in sec delay
#define WIFI_TIMEOUT_MS 10  // how long try to connect to WiFi in sec
int Baudrate = 2;          // baudrate 1 = 115200,  2 = 460800(at the F9Ps

int send_Data_Via = 1;       // send Data via  0: USB, 1: Ethernet, 2: WiFi with router and Ntrip from AOG 3; WiFi to tablet

int Ntriphotspot = 2;  // 0: Ntrip from AOG(USB or by Ethernet   1: Ntrip by Ethernet via Router
//                        2: Ntrip by WiFi via Hotspot or Router  3: Ntrip by WiFi via Router from AOG
//                        4: no Ntrip at all

//  if router exists, use 1. Network for him
//  2 - 7 Network and Passwords for Hotspots
char WIFI_Network1[24] = "";            // WiFi network Client name
char WIFI_Password1[24] =  "";        // WiFi network password
char WIFI_Network2[24] =  "";            // WiFi network Client name
char WIFI_Password2[24] = "";        // WiFi network password
char WIFI_Network3[24] =  "";            // WiFi network Client name
char WIFI_Password3[24] =  "";        // WiFi network password
char WIFI_Network4[24] =  "";            // WiFi network Client name
char WIFI_Password4[24] =  "";        // WiFi network password
char WIFI_Network5[24] =  "";            // WiFi network Client name
char WIFI_Password5[24] =  "";        // WiFi network password
char WIFI_Network6[24] =  "";            // WiFi network Client name
char WIFI_Password6[24] =  "";        // WiFi network password
char WIFI_Network7[24] =  "";            // WiFi network Client name
char WIFI_Password7[24] =  "";        // WiFi network password

String Ntrip_host1 = "";       // 1. "ntrip caster host";
String Ntrip_mntpnt1 = "";      // 1. "ntrip caster's mountpoint";
String Ntrip_user1 = "";       // 1. "ntrip caster's client user";
String Ntrip_passwd1 = "";      // 1. "ntrip caster's client password";
int Ntrip_httpPort1 = 2101;      // 1. port 2101 is default port of NTRIP caster
int GGA_Send_Back_Time1 = 0;  // after how many seconds a GGA msg is send back to Nripserver

String Ntrip_host2 = "";       // 2. "ntrip caster host";
String Ntrip_mntpnt2 = "";      // 2. "ntrip caster's mountpoint";
String Ntrip_user2 = "";       // 2. "ntrip caster's client user";
String Ntrip_passwd2 = "";      // 2. "ntrip caster's client password";
int Ntrip_httpPort2 = 2101;      // 2. port 2101 is default port of NTRIP caster
int GGA_Send_Back_Time2 = 10;  // after how many seconds a GGA msg is send back to Nripserver

//Accesspoint name
char ssid_ap[24] = "Autonomes Lenken";  // name of Access point,
int timeoutRouter = 600;                 //time (s) to hold AP


//  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  ++++++++++++++++++++++++++++++++  END Setup  +++++++++++++++++++++++++++++++++++++++
//  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
//libraries -------------------------------
#include <Wire.h>
#include <WiFi.h>
#include <math.h>
#include <EEPROM.h>
#include "z_NTRIPClient.h"
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <AsyncUDP.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "zNMEAParser.h"
#include <mpu6050_FastAngles.h>


//connection plan:
// ESP32--- Right F9P GPS pos --- Left F9P Heading-----Sentences
//  RX1----------TX1--------------------------------UBX-Nav-PVT out   (=position+speed)
//  TX1----------RX1--------------------------------RTCM in           (NTRIP comming from AOG to get absolute/correct postion
//  RX2--------------------------------TX1----------UBX-RelPosNED out (=position relative to other Antenna)
//  TX2--------------------------------RX1----------
//               RX2-------------------TX2----------RTCM 1005+1074+1084+1094+1124+1230 (activate in PVT F9P!! = NTRIP for relative positioning)

// IO pins ----------------------------------------------------------------------------------------
#define RX0   3 //USB
#define TX0   1 //USB

#define RX1   27 // simpleRTK TX 1. Antenna = NMEA
#define TX1   16 // simpleRTK RX 1. Antenna = RTCM

#define RX2   25 // simpleRTK TX1 2. Antenna; UBX
#define TX2   17 // simpleRTK RX1 2. Antenna; free


#define Button_left   12
#define Button_middle 14
#define Button_right  26
#define Ethernet_reset  13
#define SerialNmea Serial1

#define EEP_Ident 0x5422
#define EEPROM_SIZE 60

//loop time variables in microseconds
long lastTime = 0;
byte XOR;
char c;
char b;
String t;
bool OTA_update = true;
bool Ethernet_need = false, Ethernet_need_AOG = false;
bool Ntrip_WiFi = false, Ntrip_Eth_router = false;
String Conn = " all right";
int carrSoln;
bool gnssFixOk, diffSoln, relPosValid;


// Ntrip WiFi
byte WiFi_netw_nr = 0;
byte WiFi_netw[8] = {0, 0, 0, 0, 0, 0, 0, 0};
String RTCM_Packet, sString;
unsigned long startSend_back_Time = millis();
unsigned long ntriptime_from_AgopenGPS = millis();
unsigned long WiFi_scan_Delay_Time = millis();
unsigned long WiFi_blink_Time = millis();
unsigned long Amatron_begin_Time = millis();
unsigned long Ntrip_begin_Time = millis();
unsigned long ZCorrectmillisec = millis();
unsigned long Data_MPU_SendToAOGmillisec = millis();
unsigned long Single_begin_Time = millis(), Sendtimedelay;
unsigned long Single_begin_Time_ZDA = millis(), PVT_send_Time;
unsigned long lastTime_heading = millis(), MPU_Time = millis();
unsigned long Button_delay40 = millis(), Button_delay41 = millis(), Button_delay42 = millis();
int WiFi_scan_Attempt = 1;
double nordWinkel_old, eastWinkel_old;
int ntrip_from_AgopenGPS = 0, ntrip_attempt = 0;
bool network_found = false;
int buttonState = 0, buttonState_Eth;
byte IPadress[4] = { 0, 0, 0, 0 };
int net_found = 0, Ntriphotspot_an = 0, WiFi_an = 0;
#define Button_ReScan 4  // pin 4  if button pressed, WiFi scan is starting
#define LED_ntrip_ON  2  // pin 2  if ntrip on without AGopenGPS
char Ntrip_host[40] = "";       //"ntrip caster host";
char Ntrip_mntpnt[40] = "";      //"ntrip caster's mountpoint";
char Ntrip_user[40] = "";       //"ntrip caster's client user";
char Ntrip_passwd[40] = "";      //"ntrip caster's client password";
int Ntrip_httpPort;      //port 2101 is default port of NTRIP caster
int GGA_Send_Back_Time = 0, wait = 30000;
int priority = 1;

//static IP for WiFi to Router
byte myip[4] = { 192, 168, 1, 79 };     // Roofcontrol module
byte gwip[4] = { 192, 168, 1, 1 };      // Gateway IP also used if Accesspoint created
byte mask[4] = { 255, 255, 255, 0 };
byte myDNS[4] = { 8, 8, 8, 8 };         //optional
byte ipDestination[4] = { 192, 168, 1, 255}; //IP address of router to send UDP data to
byte myIPEnding = 79;             //ending of IP adress x.x.x.79 of ESP32

// Ethernet
byte mac[] = { 0x2C, 0xF7, 0xF1, 0x08, 0x00, 0x9A };
byte Eth_ipDest_ending = 100; //ending of IP address to send UDP data to router
unsigned int ToolntripPort = 5122;       //port NTRIP data to Tool
unsigned int portMy = 5124;             //this is port of this module: Autosteer = 5577 IMU = 5566 GPS =
unsigned int AOGNtripPort = 2233;       //port NTRIP data from AOG comes in
unsigned int portDestination = 9999;    //Port of AOG that listens
unsigned int localPort = 8888;       // local port to listen for UDP packets
bool Ethernet_running = false, send_IP_back = false;
char Eth_NTRIP_packetBuffer[512];// buffer for receiving and sending data
byte ReplyBufferPAOGI[120] = "";        // a string to send back
byte ReplyBufferAGIO[120] = "";
int m;
unsigned int packetLength;
unsigned long send_IP_back_time = millis();

IPAddress Eth_ipDestination;
IPAddress ipDestination1;
IPAddress src_ip;
byte my_WiFi_Mode = 0;  // WIFI_STA = 1 = Workstation  WIFI_AP = 2  = Accesspoint


// An EthernetUDP instance to let us send and receive packets over UDP
NTRIPClient_Eth ntrip_e;
NTRIPClient_WiFi ntrip_c;
EthernetUDP EthUDPToAOG;
EthernetUDP EthUDPFromAOG;
EthernetUDP Eth_udpPAOGI;
EthernetUDP Eth_udpTool;
EthernetUDP Eth_udpNtrip;
EthernetUDP Eth_udpNtrip_Router;
WiFiClient client;
EthernetClient client_Eth;
AsyncUDP udpRoof;
AsyncUDP udpNtrip;
WiFiServer server(80);

//MSG 210 for MPU 6050
uint8_t PGN_210[] = { 0x80, 0x81, 0x7f, 0xD2, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC};
int8_t PGN_210_Size = sizeof(PGN_210) - 1;
byte ReplyBufferMPUData[100] = "";
uint8_t PGN_209[] = { 0x80, 0x81, 0x7f, 0xD1, 2, 0, 0, 0xCC};
int8_t PGN_209_Size = sizeof(PGN_209) - 1;
byte ReplyBufferMPUData_Refresh[3] = "";
int isData_Refresh = 0;

// Heading
float heading, heading1, heading2, headingUBX, headingzuvor = 0;
String heading_str, heading_ubx_str;
char heading_ch[20], heading_UBX_ch[20];
double headingUBXmin, headingUBXmax, headingVTGmin, headingVTGmax;
double speeed = 0;

// roll
float roll, rollCorrectionDistance = 0.00, GGDs, GGDs1, GGDs2, GRDs, GRDs1, GRDs2;
String roll_str;
char roll_ch[8];
double rollnord = 0.0, rolleast = 0.0;
double rollnord1 = 0.0, rolleast1 = 0.0;
double rollnord_before = 0.0, rolleast_before = 0.0;
double relPosD, relPosDH;
double rollzuvor = 0;
double PI180 = 57.295791;
double hzuvor[11], hzuvormin, hzuvormax;
double Rzuvor[11];
double headingcorrectur = 0, rollcorrectur = 0;
double baseline, baseline1, baselineHorizontal;
double fixnorddeci, fixeastdeci; // coordinates in decimalen
double fixnorddeci_old, fixeastdeci_old; // coordinates in decimalen
double rollnordabs, rollnordrel, rolleastabs, rolleastrel;

byte CK_A = 0, CK_B = 0;
byte incoming_char;

// NMEA erstellen
byte inByte;
byte inByte_UBX;
String Button_left_S, Button_middle_S, Button_right_S;
String MPU_Heading_S, MPU_Roll_S, MPU_Yaw_S, MPU6050_Data;
int start = 0, strglen;
String nmea = "";
String GGAnord = "", GGAeast = "", GGAWestEast = "", GGANordSued = "";
String GPSqualistr = "", WEcoordinaten, NScoordinaten;
String GGASatz = "", VTGSatz = "", GGASatz_Korr, VTGSatz_Korr = "", GGASatz_send_back = "";
int GPSqualin1 = 0, GPSqualinzuvor = 1, GPSqualintime = 1, GGA_check = 0;
String ZDASatz = "";
int  i = 0, j = 0, ij = 0, iji = 0;
double nordWinkel, eastWinkel;
double fixnorddeci_before = 0.0000, fixeastdeci_before = 0.0000;

/* A parser is declared with 3 handlers at most */
NMEAParser<3> parser;

// Conversion to Hexidecimal
const char* asciiHex = "0123456789ABCDEF";

// the new PAOGI sentence buffer
char GGAnmea[90];
char GGAnmea_Korr[90];
char ZDAnmea[50];
char VTGnmea[60];
char PAOGI_Msg[150];
char ToopNtrip[150];

// GGA
char fixTime[12];
char latitude[15];
char latNS[3];
char longitude[15];
char lonEW[3];
char fixQuality[2];
char numSats[4];
char HDOP[5];
char altitude[12];
char seperation[8];
char ageDGPS[10];

char NScoordinaten_ch[15];
char WEcoordinaten_ch[15];

// VTG
char vtgHeading[12] = { };
char speedKnots[10] = { };
char speedKm[10] = { };

// ZDA
char zdatime[12] = { };
char zdaday[4] = { };
char zdamonth[4] = { };
char zdayear[6] = { };
char zdaltzh[4] = { };
char zdaltzn[4] = { };

// PAOGI erstellen
bool Paogi_true_UBX = true;
String BS = ",";
int Paogi_Long, Coodinate_check1, Coodinate_check2, heading_check1 = 0;
int Paogi_Long1, Coodinate1_check1, Coodinate1_check2, heading1_check1 = 0;
int Paogi_Shit = 0, Paogi_Shit1 = 0;

byte UDPPAOGIMsg[100], UDPVTGMsg[40], UDPGGAMsg[90];
byte  ReplyBufferVTG[40] = "", ReplyBufferGGA[90] = "";    // a string to send back

// Chechsum controll
String checksum = "";

// MPU6050
String RollHeading = "";
int move_ABline = 0;
float roll_MPU6050 = 0, heading_MPU6050 = 0, roll1, roll2;
float roll_MPU6050_diff = 0, heading_MPU6050_diff = 0, heading_MPU6050_offset = 0, heading_MPU6050_drift = 0;
double heading_MPU6050_zuvor[10], heading_MPU6050_hzuvormin, heading_MPU6050_hzuvormax;
double heading_MPU6050_corridor;
//int Button_left = 0, Button_middle = 0, Button_right = 0;
int ABline_Direction[3] = {0, 0, 0};  // {heading, cm of movement, same direction as AB line}
bool MPU6050_received = false;
mpu6050_FastAngles mpu;
double ZCorrect = 0, headingbefore;
unsigned long lastReadTime = 0;
const unsigned long readInterval = 90; // Intervalo de leitura em milissegundos
float angleX = 0;
float angleY = 0;
float angleZ = 0;
float gyroOffset = 0;
int buttonState_left, buttonState_middle, buttonState_right;

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

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  ubxmessage.rawBuffer[0] = 0xB5;  //181
  ubxmessage.rawBuffer[1] = 0x62;  //98
  ubxmessage.rawBuffer[2] = 0x01;  //1
  ubxmessage.rawBuffer[3] = 0x3C;  //60

  delay(10);
  Serial.begin(38400);
  delay(10);
  Serial.println("");
  Serial.println(VERS);
  Serial.println("");
  Serial.println("Start setup");
  Serial.println("");

  if (Baudrate == 1) {
    SerialNmea.begin(115200, SERIAL_8N1, RX1, TX1);
    delay(10);
    if ((Dual_Antenna == 1) || (send_amatron_nmea == 1)) {
      Serial2.begin(115200, SERIAL_8N1, RX2, TX2);
    }
  }
  else
  {
    SerialNmea.begin(460800, SERIAL_8N1, RX1, TX1);
    delay(10);
    if ((Dual_Antenna == 1) || (send_amatron_nmea == 1)) {
      Serial2.begin(460800, SERIAL_8N1, RX2, TX2);
    }
  }
  if (Dual_Antenna == 0) send_amatron_nmea = 0;

  EEPROM.begin(EEPROM_SIZE);

  if (EEPROM.read(0) != EEP_Ident)  // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(6, Heading_MPU6050);
    EEPROM.put(7, Roll_MPU6050);
    EEPROM.put(8, Heading_Dual_MPU);
    EEPROM.put(9, Roll_Dual_MPU);
    EEPROM.put(10, Headingfilter);
    EEPROM.put(11, Rollfilter);
    EEPROM.put(12, isToolAntenna);
    //EEPROM.put(13, isData_Refresh);
    EEPROM.commit();
    Serial.println(" EEPROM first time " );
  }
  else
  {
    EEPROM.get(6, Heading_MPU6050);
    EEPROM.get(7, Roll_MPU6050);
    EEPROM.get(8, Heading_Dual_MPU);
    EEPROM.get(9, Roll_Dual_MPU);
    EEPROM.get(10, Headingfilter);
    EEPROM.get(11, Rollfilter);
    EEPROM.get(12, isToolAntenna);
    //EEPROM.get(13, isData_Refresh);
    Serial.println(" EEPROM read OK " );
  }


  pinMode(Button_ReScan, INPUT_PULLUP);
  pinMode(LED_ntrip_ON, OUTPUT);
  digitalWrite(LED_ntrip_ON, HIGH);
  pinMode(Button_left, INPUT_PULLUP);
  pinMode(Button_middle, INPUT_PULLUP);
  pinMode(Button_right, INPUT_PULLUP);
  pinMode(Ethernet_reset, OUTPUT);

  startSend_back_Time = millis() - (10000);
  ntriptime_from_AgopenGPS = millis();
  WiFi_scan_Delay_Time = millis();
  Amatron_begin_Time = millis();
  ZCorrectmillisec = millis();
  WiFi_blink_Time = millis();
  PVT_send_Time = millis();
  Sendtimedelay = millis();

  //Prevent W5500 to get into Reset Loop Bug
  digitalWrite(Ethernet_reset, LOW); //Reset Ethernet Modul
  delay(300);
  digitalWrite(Ethernet_reset, HIGH); //Start Ethernet Modul

  delay(100);
  Serial.print(" AntDistance : "); Serial.println(AntDistance);       // distance between the two antennas in cm,+, 0 for automatic distance
  Serial.print(" tractorhight : "); Serial.println(tractorhight);   // roll is in Position calculated, in AgOpenGPS mit 0 cm
  Serial.print(" WiFi_scan_Delay : "); Serial.println(WiFi_scan_Delay);      // for router use 50 sec delay
  Serial.print(" Dual_Antenna : "); Serial.println(Dual_Antenna);  // 1: for Dualantenna, 0: for single antenna;
  Serial.print(" send_amatron_nmea : "); Serial.println(send_amatron_nmea);    // 1: for sending, 0: for not
  Serial.print(" GGA_Send_Back_Time : "); Serial.println(GGA_Send_Back_Time);       //CS PIN with SPI Ethernet hardware W 5500  SPI config: MOSI 23 / MISO 19 / CLK18 / CS5, GND, 3.3V
  Serial.print(" IMU_MPU6050 : "); Serial.println(IMU_MPU6050);       // 0: no   1: to dualheading to stabilize heading under trees
  Serial.print(" IMU_MPU6050_direction : "); Serial.println(IMU_MPU6050_direction);       //Drivedirection  Y=1:  -Y=2:  X=3:  -X=4:
  Serial.print(" Roll_Dual_MPU : "); Serial.println(Roll_Dual_MPU);   // from 1 to 10 1: from Dual   10: from MPU
  Serial.print(" move_line_buttons : "); Serial.println(move_line_buttons);       // 0: no   1: buttons to move AB line
  Serial.print(" Headingfilter : "); Serial.println(Headingfilter);       // 1: no   10: most filter
  Serial.print(" Rollfilter : "); Serial.println(Rollfilter);       // 1: no   10: most filter
  Serial.print(" send_Data_Via : "); Serial.println(send_Data_Via);
  Serial.print(" Ntriphotspot : "); Serial.println(Ntriphotspot);


  if  ((WIFI_Network1 != " ") || (Ntriphotspot > 1) || (send_Data_Via == 2)) {
    Serial.println("");
  }
  else {
    Serial.println("Wait for Wifi_Scan; delay_Time");
  }
  delay(50);
  Serial.print("Check_connections : ");
  Check_connections();
  Serial.println(Conn);
  delay(1000);  // 1000
  if (my_WiFi_Mode != 2) {
    Start_connections();
    delay(1000);
  }

  // the dash means wildcard
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);
  if (send_amatron_nmea == 1) {
    parser.addHandler("G-ZDA", ZDA_Handler);
  }

  Headingfilter = constrain(Headingfilter, 1, 10);
  Heading_Dual_MPU = constrain(Heading_Dual_MPU, 1, 10);
  Rollfilter = constrain(Rollfilter, 1, 10);
  Roll_Dual_MPU = constrain(Roll_Dual_MPU, 1, 10);
  if (IMU_MPU6050 != 0)
  {
    Serial.println("");
    Serial.print("IMU_MPU6050  : ");
    Serial.println(IMU_MPU6050);

    mpu.begin();
    mpu.calibrateGyro(1000);
    // Exibe as configurações atuais
    mpu.printSettings();
  }

  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println(VERS);
  Serial.println("");
  Serial.println("End setup");
  Serial.println("");

}


void to_hex_string(uint8_t buf[], size_t len, char* dest) {
  const char* hex = "0123456789ABCDEF";
  for (int i = 0; i < len; i++) {
    *dest++ = hex[(buf[i] >> 4) & 0xF];
    *dest++ = hex[(buf[i]   ) & 0xF];
  }
  *dest = '\0';
}

void read_rtcm_from_wifi_forward_to_udp_and_f9p(int rtcm_available) {
#define RTCM_BUFFER_SIZE 2048
  char RTCMdata[RTCM_BUFFER_SIZE];

  if ( rtcm_available > RTCM_BUFFER_SIZE ) {
    rtcm_available = RTCM_BUFFER_SIZE;
  }

  const int bytesRead = ntrip_c.readBytes(RTCMdata, rtcm_available);
  if ( bytesRead <= 0 ) {
    //Serial.printf("W: ntrip_c.readLine() has read %d bytes\n", bytesRead);
  }
  else {
    const size_t writtenToSerial = SerialNmea.write(RTCMdata, bytesRead);
    if ( bytesRead != writtenToSerial) {
      //Serial.printf("E: SerialNmea.write() bytesRead %d, but only %zu written\n", bytesRead, writtenToSerial);
    }

    size_t ethUdpWritten; //ToolntripPort
    if ( Eth_udpTool.beginPacket(Eth_ipDestination, ToolntripPort) == 0 ) {
      //Serial.println("E: EthUdp/GPS3 beginPacket()");
    }
    else if ( (ethUdpWritten = Eth_udpTool.write(RTCMdata, bytesRead)) < bytesRead ) {
      //Serial.printf("E: EthUdp/GPS3 write(): bytesRead %d, but only %zu written\n", bytesRead, ethUdpWritten);
    }
    else if ( Eth_udpTool.endPacket() == 0 ) {
      //Serial.println("E: EthUdp/GPS3 endPacket()");
      //Serial.println(Eth_ipDestination);

    }
    else if ( bytesRead == writtenToSerial && bytesRead == ethUdpWritten) {
      //char hex[128];
      //to_hex_string((uint8_t*)RTCMdata, 32, hex);
      /*
        Serial.print("I: RTCM bytes forwarded to serialNMEA and ");
        Serial.print(Eth_ipDestination);
        Serial.printf(":%u - %4u bytes\n", ToolntripPort, ethUdpWritten);
      */
      // with hey payload: Serial.printf(":%u - %4u bytes, %s\n", ToolntripPort, ethUdpWritten, hex);
    }
  }
}


unsigned long millis_last = 0;
unsigned long loop_cnt = 0;

void loop_counter() {

  loop_cnt += 1;

  unsigned long millis_curr = millis();
  unsigned long millis_diff = millis_curr - millis_last;
  if ( millis_diff > 1000) {
    Serial.printf("loop_cnt: %6lu, ms: %4lu\n", loop_cnt, millis_diff);
    millis_last = millis_curr;
    loop_cnt = 0;
  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  //loop_counter();

  if (isData_Refresh == 1)
  { Data_MPU_SendToAOG();
    isData_Refresh = 0;
    //Serial.println(" Hallo, Data_MPU_SendToAOG");
  }

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

  //  Start a WiFi scan with pressing the button
  buttonState = digitalRead(Button_ReScan);
  if ((buttonState == 0) && (ntrip_from_AgopenGPS == 0) && (Ntriphotspot > 1)) {
    WiFi.mode(WIFI_OFF);
    digitalWrite(LED_ntrip_ON, LOW);
    Network_built_up();
  }

  //  Start up WiFi connection if lost
  if ((Ntrip_WiFi) && (WiFi.status() != WL_CONNECTED)) {
    Network_built_up();
  }

  //#######################################################################

  //  receive RTCM3 data by USB from AOG #############################################
  if ((send_Data_Via == 0) && (Ntriphotspot == 0)) {
    if (Serial.available()) {      // If RTCM3 comes in Serial (USB),
      char C = Serial.read();      // read a byte, then
      SerialNmea.write(C);            // send it out Serial1 from PIN 16 to simpleRTK RX1 1. Antenna = RTCM
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
      //SerialNmea.write(ntrip_c.read());   // read RTCM3  and send from ESP32 16 to simpleRTK RX 1. Antenna = RTCM
    }
  }

  //  receive RTCM3 data by Ethernet from AOG  ##########################################
  //   if ((send_Data_Via == 1) && (Ntriphotspot == 0)
  if ((Ethernet_running) && (Ntriphotspot == 0)) {
    doEthUDPNtrip();  // If RTCM3 comes in received by Ethernet from AOG
  }

  // If RTCM3 comes in received by Ethernet from Router ####################################
  if ((send_Data_Via == 1) && (Ntriphotspot == 1) && (ntrip_from_AgopenGPS == 0) && (Ethernet_running)) { //  Ntrip_begin_Time
    if (isToolAntenna == 1)
    {
      int available = ntrip_e.available();
      if (( available > 0 )) { //  && (isToolAntenna == 1)){
        read_rtcm_from_wifi_forward_to_udp_and_f9p(available);
      }
    }
    else
    {
      if (ntrip_e.available()) {         // If RTCM3 comes in received by Ethernet
        SerialNmea.write(ntrip_e.read());   // read RTCM3  and send from ESP32 16 to simpleRTK RX 1. Antenna = RTCM
      }
    }
  }

  // If RTCM3 comes in received by WiFi from Router of Hotspot ####################################
  if ((send_Data_Via == 1) && (Ntriphotspot == 2) && (Ethernet_running)) { //  Ntrip_begin_Time
    if (isToolAntenna == 1)
    {
      int available = ntrip_c.available();
      if (( available > 0 )) { //  && (isToolAntenna == 1)){
        read_rtcm_from_wifi_forward_to_udp_and_f9p(available);
      }
    }
    else
    {
      if (ntrip_c.available()) {         // If RTCM3 comes in received by Ethernet
        SerialNmea.write(ntrip_c.read());   // read RTCM3  and send from ESP32 16 to simpleRTK RX 1. Antenna = RTCM
      }
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
    }
  }

  //  receive RTCM3 data by WiFi from Tablet    ############################################
  if ((send_Data_Via == 3) && (Ntriphotspot == 3) && (ntrip_from_AgopenGPS == 0) && (my_WiFi_Mode == 2)) { //  Ntrip_begin_Time
    doUDPNtrip_WiFi();  // If RTCM3 comes in received by WiFi
  }

  //  no Ntrip  ########################################
  //if ((send_Data_Via == 3) && (Ntriphotspot == 4))

  //  receive RTCM3 data by WiFi from Router(AOG)   ########################################
  if ((send_Data_Via == 2) && (Ntriphotspot == 3)) {
    doUDPNtrip_WiFi();  // If RTCM3 comes in received by WiFi
  }

  //  read NMEA msg from F9P (PVT) and pars them in NMEA_read()   ##############################################
  while (SerialNmea.available()) { // If anything comes in SerialNmea
    parser << SerialNmea.read();
  }

  //  Send GGA MSG back to Base     ###########################################################################
  if ((GGA_Send_Back_Time != 0) && ((send_Data_Via == 2) || (Ntriphotspot == 2)))  sendGGA_WiFi();
  if ((GGA_Send_Back_Time != 0) && (Ntriphotspot == 1))  sendGGA_Eth();

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
    
    if (i > 71) {
      CK_A = 0;
      CK_B = 0;
      for (i = 2; i < 70 ; i++) {
        CK_A = CK_A + ubxmessage.rawBuffer[i];
        CK_B = CK_B + CK_A;
      }

      if (CK_A == ubxmessage.rawBuffer[70] && CK_B == ubxmessage.rawBuffer[71]) {

        if (Heading_MPU6050 == 1) Heading_MPU6050V();  // to correct the drift
        heading_relposned();
        rollundheading();   // calculate roll        ########################################################
        PAOGI1_builder();   // built the PAOGI MSG   ########################################################
      }
      else {
        // Serial.println("ACK Checksum Failure: ");
      }
      i = 0;
    
  }
  else {
    Print_NMEA();
  }
	}
  }
  if ((send_amatron_nmea == 1) && (Dual_Antenna == 1)) Print_NMEA();

  //  LED on == no WiFi or Ethernet, off == WiFi or Ethernet   #########################################
  if (((send_Data_Via > 0) && (Ntriphotspot_an == 0)) && (Ntriphotspot > 0)) {
    if ((WiFi.status() == WL_CONNECTED) || (Ethernet_running)) {
      if ((millis() - WiFi_blink_Time) < 500) {
        digitalWrite(LED_ntrip_ON, HIGH);
      }
      else {
        digitalWrite(LED_ntrip_ON, LOW);
        WiFi_blink_Time = millis();
      }
    }
    else {
      digitalWrite(LED_ntrip_ON, HIGH);
    }
  }

  //  LED on == no Ntrip from ESP32, off == Ntrip from ESP32   #########################################
  if ((Ntriphotspot == 1) || (Ntriphotspot == 2)) {
    if (Ntriphotspot_an == 1) {
      digitalWrite(LED_ntrip_ON, LOW);
    }
    else {
      digitalWrite(LED_ntrip_ON, HIGH);
    }
  }
  else {
    digitalWrite(LED_ntrip_ON, LOW);
  }

  if (Roll_MPU6050 == 1) RollHeading_MPU();

}//end main loop

//------------------------------------------------------------------------------------------
