
String VERS = "Version DualHeading_ESP32_03.05.2023";

// AAA_Readme for instructions

//  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  +++++++++++++++++++++++++++++++  BEGIN Setup +++++++++++++++++++++++++++++++++++++++
//  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int AntDistance = 115;       // distance between the two antennas in cm,+, 0 for automatic distance
int tractorhight = 280;   // roll is in Position calculated, in AgOpenGPS mit 0 cm
int Dual_Antenna = 1;  // 1: for Dualantenna, 0: for single antenna;
int send_amatron_nmea = 0;    // 1: for sending, 0: for not
byte Eth_CS_PIN = 5;       //CS PIN with SPI Ethernet hardware W 5500  SPI config: MOSI 23 / MISO 19 / CLK18 / CS5, GND, 3.3V
int OTA_active = 0;     // 0, no OTA possible, 1 OTA for 60 sec after Start
int rollaktiv = 1;     // 0: no roll  1: roll activated in Dualheading
int IMU_MPU6050 = 1;       // 1: to 10   1: from Dual   10: from MPU
int IMU_MPU6050_direction = 2;       //Drivedirection  Y=1:  -Y=2:  X=3:  -X=4:
int Roll_Dual_MPU = 1;       // from 1 to 10 1: from Dual   10: from MPU
int move_line_buttons = 0;       // 0: no   1: buttons to move AB line
int Headingfilter = 1;       // 1: no   10: most filter
int WiFi_scan_Delay = 1;      // wait to scan, for router use 50 sec delay
#define WIFI_TIMEOUT_MS 10  // how long try to connet to WiFi in sec
byte Ethernet_3rd = 5 ;     //{ 192, 168,     [1]   , 124 };//3rd nummer of IP address to send UDP data to

int send_Data_Via = 1;       // send Data via  0: USB, 1: Ethernet, 2: WiFi with router and Ntrip from AOG 3; WiFi to tablet

int Ntriphotspot = 2;  // 0: Ntrip from AOG(USB or by Ethernet   1: Ntrip by Ethernet via Router
//                        2: Ntrip by WiFi via Hotspot or Router  3: Ntrip by WiFi via Router from AOG

//  if router exists, use 1. Network for him
//  2 - 7 Network and Passwords for Hotspots
char WIFI_Network1[24] = "";            // WiFi network Client name
char WIFI_Password1[24] =  "";        // WiFi network password
char WIFI_Network2[24] =  "";            // WiFi network Client name
char WIFI_Password2[24] =  "";        // WiFi network password
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

String Ntrip_host1 = "";       // 1. "ntrip IP adress";
String Ntrip_mntpnt1 = "";      // 1. "ntrip caster's mountpoint";
String Ntrip_user1 = "";       // 1. "ntrip caster's client user";
String Ntrip_passwd1 = "";      // 1. "ntrip caster's client password";
int Ntrip_httpPort1 = 2101;      // 1. port 2101 is default port of NTRIP caster
int GGA_Send_Back_Time1 = 10;  // after how many seconds a GGA msg is send back to Nripserver

String Ntrip_host2 = "";       // 2. "ntrip caster host";
String Ntrip_mntpnt2 = "";      // 2. "ntrip caster's mountpoint";
String Ntrip_user2 = "";       // 2. "ntrip caster's client user";
String Ntrip_passwd2 = "";      // 2. "ntrip caster's client password";
int Ntrip_httpPort2 = 2101;      // 2. port 2101 is default port of NTRIP caster
int GGA_Send_Back_Time2 = 10;  // after how many seconds a GGA msg is send back to Nripserver

//Accesspoint name and password for OTA
char ssid_ap[24] = "Autonomes Lenken";  // name of Access point, if no WiFi for OTA found
int timeoutRouter = 600;                 //time (s) to hold AP for OTA


//  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  ++++++++++++++++++++++++++++++++  END Setup  +++++++++++++++++++++++++++++++++++++++
//  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
//libraries -------------------------------
#include <Wire.h>
#include <WiFi.h>
#include <math.h>
#include "z_NTRIPClient.h"
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <AsyncUDP.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"



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
String RTCM_Packet;
unsigned long startSend_back_Time = millis();
unsigned long ntriptime_from_AgopenGPS = millis();
unsigned long WiFi_scan_Delay_Time = millis();
unsigned long WiFi_blink_Time = millis();
unsigned long Amatron_begin_Time = millis();
unsigned long Ntrip_begin_Time = millis();
unsigned long OTA_begin_Time = millis();
unsigned long Single_begin_Time = millis();
unsigned long Single_begin_Time_VTG = millis();
unsigned long lastTime_heading = millis(), MPU_Time = millis();
unsigned long Button_delay40 = millis(), Button_delay41 = millis(), Button_delay42 = millis();
int durchlauf_nord = 0, durchlauf_east = 0, WiFi_scan_Attempt = 1;
double nordWinkel_old, eastWinkel_old;
int ntrip_from_AgopenGPS = 0, ntrip_attempt = 0;
bool network_found = false;
int buttonState = 0;
byte IPadress[4] = { 0, 0, 0, 0 };
int net_found = 0, Ntriphotspot_an = 0, WiFi_an = 0;
#define Button_ReScan 4  // pin 4  if button pressed, WiFi scan is starting
#define LED_ntrip_ON  2  // pin 2  if ntrip on without AGopenGPS
char Ntrip_host[40] = "";       //"ntrip caster host";
char Ntrip_mntpnt[40] = "";      //"ntrip caster's mountpoint";
char Ntrip_user[40] = "";       //"ntrip caster's client user";
char Ntrip_passwd[40] = "";      //"ntrip caster's client password";
int Ntrip_httpPort;      //port 2101 is default port of NTRIP caster
int GGA_Send_Back_Time = 0;

//static IP for WiFi to Router
//byte myip[4] = { 192, 168, 1, 79 };     // Roofcontrol module
byte gwip[4] = { 192, 168, 1, 1 };      // Gateway IP also used if Accesspoint created
byte mask[4] = { 255, 255, 255, 0 };
byte myDNS[4] = { 8, 8, 8, 8 };         //optional

// Ethernet
byte Eth_myip[4] = { 192, 168, 1, 124 };//IP address to send UDP data to
//byte Eth_myip[4] = { 10, 0, 0, 124 };//IP address to send UDP data to
//byte mac[] = {0x90, 0xA2, 0xDA, 0x10, 0xB3, 0x1B}; // original
byte mac[] = { 0x2C, 0xF7, 0xF1, 0x08, 0x00, 0x9A };
byte Eth_ipDest_ending = 100; //ending of IP address to send UDP data to router
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
EthernetUDP Eth_udpNtrip;
EthernetUDP Eth_udpNtrip_Router;
WiFiClient client;
EthernetClient client_Eth;
AsyncUDP udpRoof;
AsyncUDP udpNtrip;
WiFiServer server(80);
MPU6050 mpu;

// Heading
float heading, heading1, heading2, headingUBX, heading_diff, headingzuvor = 0, headingzuvorVTG;
double headingUBXmin, headingUBXmax, headingVTGmin, headingVTGmax;
double speeed = 0, headingnord;

// roll
float roll, rollCorrectionDistance = 0.00, GGDs;
double rollnord = 0.0, rolleast = 0.0;
double rollnord1 = 0.0, rolleast1 = 0.0;
double rollnord_before = 0.0, rolleast_before = 0.0;
double relPosD, relPosDH;
double rollzuvor = 0;
double PI180 = 57.295791;
double hzuvor[10], hzuvormin, hzuvormax;
double headingcorrectur = 0, rollcorrectur = 0;
double baseline, baseline1, baselineHorizontal;
double fixnorddeci, fixeastdeci; // coordinates in decimalen
double fixnorddeci_old, fixeastdeci_old; // coordinates in decimalen
double rollnordabs, rollnordrel, rolleastabs, rolleastrel;

byte CK_A = 0, CK_B = 0;
byte incoming_char;

// NMEA erstellen
bool NMEA_OK = true;
int argGGA_Anz[17], argGGA_Anz_ende, arg_MPU_buttons[7];
int inByte;
String argGGA9, GGA_Sealevel, GGA_Age;
String argGGA10;
String argGGA11, GGA_time;
String Button_left_S, Button_middle_S, Button_right_S;
String MPU_Heading_S, MPU_Roll_S, MPU_Yaw_S, MPU6050_Data;
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
double nordWinkel, eastWinkel;
double fixnorddeci_before = 0.0000, fixeastdeci_before = 0.0000;

// PAOGI erstellen
bool Paogi_true_UBX = true, Paogi_true = true;
String RollHeadingrest = "", RollHeadingshit = "", RollHeadingrest_befor = "", BS = ",";
int Paogi_Long, Coodinate_check1, Coodinate_check2, heading_check1 = 0;
int Paogi_Long1, Coodinate1_check1, Coodinate1_check2, heading1_check1 = 0;
int Paogi_Shit = 0, Paogi_Shit1 = 0;

byte UDPPAOGIMsg[100], UDPVTGMsg[40], UDPGGAMsg[90];
unsigned int Bytelaenge, BytelaengeVTG, BytelaengeGGA;
byte  ReplyBufferVTG[40] = "", ReplyBufferGGA[90] = "";       // a string to send back

//UBX
double speedUBXint;
String GGA_Sats;

// Chechsum controll
String checksum = "", checksum_GGA = "", checksum_GGA_send = "", checksum_VTG = "", checksum_VTG_send = "";
String check_headingroll = "";
int j_checksum_GGA = 0, j_checksum_VTG = 0;


// MPU6050
String RollHeading = "";
int move_ABline = 0, heading_source = 0;
float roll_MPU6050 = 0, heading_MPU6050 = 0, roll1, roll2;
float roll_MPU6050_diff = 0, heading_MPU6050_diff = 0, heading_MPU6050_offset = 0, heading_MPU6050_drift = 0;
//int Button_left = 0, Button_middle = 0, Button_right = 0;
int ABline_Direction[3] = {0, 0, 0};  // {heading, cm of movement, same direction as AB line}
bool MPU6050_received = false;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 1  // use pin 2 on Arduino mega & most boards
#define LED_PIN 15 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

bool blinkState = false;
float printtime = millis();

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Timers
unsigned long timer = 0;
float timeStep = 0.009;
int buttonState_left, buttonState_middle, buttonState_right;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;

}

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



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  ubxmessage.rawBuffer[0] = 0xB5;
  ubxmessage.rawBuffer[1] = 0x62;
  ubxmessage.rawBuffer[2] = 0x01;
  ubxmessage.rawBuffer[3] = 0x3C;

  delay(10);
  Serial.begin(38400);
  delay(10);
  Serial.println("");
  Serial.println(VERS);
  Serial.println("");
  Serial.println("");
  Serial.println("Start setup");

  Serial1.begin(115200, SERIAL_8N1, RX1, TX1);
  delay(10);
  if ((Dual_Antenna == 1) || (send_amatron_nmea == 1)) {
    Serial2.begin(115200, SERIAL_8N1, RX2, TX2);
  }
  pinMode(Button_ReScan, INPUT_PULLUP);
  pinMode(LED_ntrip_ON, OUTPUT);
  digitalWrite(LED_ntrip_ON, HIGH);
  pinMode(Button_left, INPUT_PULLUP);
  pinMode(Button_middle, INPUT_PULLUP);
  pinMode(Button_right, INPUT_PULLUP);

  startSend_back_Time = millis() - (GGA_Send_Back_Time * 1000);
  ntriptime_from_AgopenGPS = millis();
  WiFi_scan_Delay_Time = millis();
  Amatron_begin_Time = millis();
  OTA_begin_Time = millis();
  WiFi_blink_Time = millis();

  delay(100);
  Serial.print(" AntDistance : "); Serial.println(AntDistance);       // distance between the two antennas in cm,+, 0 for automatic distance
  Serial.print(" tractorhight : "); Serial.println(tractorhight);   // roll is in Position calculated, in AgOpenGPS mit 0 cm
  Serial.print(" WiFi_scan_Delay : "); Serial.println(WiFi_scan_Delay);      // for router use 50 sec delay
  Serial.print(" Dual_Antenna : "); Serial.println(Dual_Antenna);  // 1: for Dualantenna, 0: for single antenna;
  Serial.print(" send_amatron_nmea : "); Serial.println(send_amatron_nmea);    // 1: for sending, 0: for not
  Serial.print(" Eth_CS_PIN : "); Serial.println(Eth_CS_PIN);       //CS PIN with SPI Ethernet hardware W 5500  SPI config: MOSI 23 / MISO 19 / CLK18 / CS5, GND, 3.3V
  Serial.print(" OTA_active : "); Serial.println(OTA_active);     // 0, no OTA possible, 1 OTA for 60 sec after Start
  Serial.print(" IMU_MPU6050 : "); Serial.println(IMU_MPU6050);       // 0: no   1: to dualheading to stabilize heading under trees
  Serial.print(" IMU_MPU6050_direction : "); Serial.println(IMU_MPU6050_direction);       //Drivedirection  Y=1:  -Y=2:  X=3:  -X=4:
  Serial.print(" Roll_Dual_MPU : "); Serial.println(Roll_Dual_MPU);   // from 1 to 10 1: from Dual   10: from MPU
  Serial.print(" move_line_buttons : "); Serial.println(move_line_buttons);       // 0: no   1: buttons to move AB line
  Serial.print(" Headingfilter : "); Serial.println(Headingfilter);       // 1: no   10: most filter
  Serial.print(" send_Data_Via : "); Serial.println(send_Data_Via);
  Serial.print(" Ntriphotspot : "); Serial.println(Ntriphotspot);
  if  ((WIFI_Network1 != " ") || (Ntriphotspot > 1) || (send_Data_Via == 2)) {
    Serial.println("");
  }
  else {
    Serial.println("Delay_Time ");
  }
  delay(5000);
  Serial.print("Check_connections : ");
  Check_connections();
  Serial.println(Conn);
  delay(1000);
  if (my_WiFi_Mode != 2) {
    Start_connections();
    delay(1000);
    if ((my_WiFi_Mode == 0) && (OTA_active) || (Ntriphotspot == 3))  WiFi_Start_AP();
  }
  if ((my_WiFi_Mode != 0) && (OTA_active))  {
    Serial.println("Hallo OTA is activated");
    OTA_update_ESP32();
  }
  // initialize device
  if (IMU_MPU6050 > 1) {
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(32, 33); // original (32, 33)
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    Serial.println("");
    Serial.println("");
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    delay(1000);
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    if (!mpu.testConnection()) {
      Serial.println(mpu.testConnection());
      //IMU_MPU6050 = 1;
      return;
    }
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    //  while (!Serial.available());                 // wait for data
    delay(5000);
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220); // 220
    mpu.setYGyroOffset(76);  //  76 roll
    mpu.setZGyroOffset(-85); // -85
    mpu.setZAccelOffset(1688); // 1788,1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }
  }

  Headingfilter = constrain(Headingfilter, 1, 10);
  Roll_Dual_MPU = constrain(Roll_Dual_MPU, 1, 10);
  Serial.println("");
  Serial.print("IMU_MPU6050  : ");
  Serial.println(IMU_MPU6050);
  Serial.println("");
  Serial.println("");
  Serial.println("End setup");
  Serial.println("");

}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


void loop() {

   packetLength = EthUDPFromAOG.parsePacket();
   if (packetLength > 0)  read_Eth_AGIO();

 // read Date from MPU6050, and/or from 3 buttons to move the line
  if (move_line_buttons == 1) button_linemove();

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


  //Ethernet Start a new Setup with pressing the button+++++++++++++++++++++++++Markus
  //buttonState = digitalRead(Button_ReScan);
  //if ((buttonState == 0) && (Ntriphotspot = 1)) {
  //  ESP.restart();
  //}

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
  //   if ((send_Data_Via == 0) && (Ntriphotspot == 0)
  if (Serial.available()) {      // If RTCM3 comes in Serial (USB),
    char C = Serial.read();      // read a byte, then
    Serial1.write(C);            // send it out Serial1 from PIN 16 to simpleRTK RX1 1. Antenna = RTCM
    if (C != '  ') {             // if the byte is a newline character
      ntripcheck();
    }
    ntriptime_from_AgopenGPS = millis();
  }
  else {
    ntripcheck();
  }

  //  receive RTCM3 data by Ethernet from AOG  ##########################################
  //   if ((send_Data_Via == 1) && (Ntriphotspot == 0)
  if ((Ethernet_running) && (Ntriphotspot == 0)) {
    doEthUDPNtrip();  // If RTCM3 comes in received by Ethernet from AOG

  }

  // If RTCM3 comes in received by WiFi from Router ####################################
  if ((send_Data_Via == 0) && (Ntriphotspot == 2)) { //  Ntrip_begin_Time
    if (ntrip_c.available()) {         // If RTCM3 comes in received by WIFI
      Serial1.write(ntrip_c.read());   // read RTCM3  and send from ESP32 16 to simpleRTK RX 1. Antenna = RTCM
    }
  }

  // If RTCM3 comes in received by Ethernet from Router ####################################
  if ((send_Data_Via == 1) && (Ntriphotspot == 1) && (ntrip_from_AgopenGPS == 0) && (Ethernet_running)) { //  Ntrip_begin_Time
    if (ntrip_e.available()) {         // If RTCM3 comes in received by Ethernet
      Serial1.write(ntrip_e.read());   // read RTCM3  and send from ESP32 16 to simpleRTK RX 1. Antenna = RTCM
    }
  }

  // If RTCM3 comes in received by Ethernet from Router ####################################
  if ((send_Data_Via == 1) && (Ntriphotspot == 2) && (Ethernet_running)) { //  Ntrip_begin_Time
    if (ntrip_c.available()) {         // If RTCM3 comes in received by WIFI
      Serial1.write(ntrip_c.read());   // read RTCM3  and send from ESP32 16 to simpleRTK RX 1. Antenna = RTCM
    }
  }

  // If RTCM3 comes in received by WiFi from Tablett ####################################
  if ((send_Data_Via == 1) && (Ntriphotspot == 3) && (Ethernet_running)) { //  Ntrip_begin_Time
    doUDPNtrip_WiFi ();  // If RTCM3 comes in received by WiFi
  }

  //  receive RTCM3 data by WiFi from Hotspot    ############################################
  if ((send_Data_Via == 2) && (Ntriphotspot == 2) && (ntrip_from_AgopenGPS == 0) && (WiFi.status() == WL_CONNECTED)) { //  Ntrip_begin_Time
    if (ntrip_c.available()) {         // If RTCM3 comes in received by WIFI
      Serial1.write(ntrip_c.read());   // read RTCM3  and send from ESP32 16 to simpleRTK RX 1. Antenna = RTCM
    }
  }

  //  receive RTCM3 data by WiFi from Tablet    ############################################
  if ((send_Data_Via == 3) && (Ntriphotspot == 3) && (ntrip_from_AgopenGPS == 0) && (my_WiFi_Mode == 2)) { //  Ntrip_begin_Time
    doUDPNtrip_WiFi ();  // If RTCM3 comes in received by WiFi
  }

  //  receive RTCM3 data by WiFi from Router(AOG)   ########################################
  if ((send_Data_Via == 2) && (Ntriphotspot == 3)) {
    doUDPNtrip_WiFi ();  // If RTCM3 comes in received by WiFi
  }

  //  read NMEA msg from F9P (PVT) and pars them in NMEA_read()   ##############################################
  if (Serial1.available()) { // If anything comes in Serial1
    inByte = Serial1.read(); // read it and send for NMEA_PAOGI
    NMEA_read();
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
    }
    if (i > 71) {
      CK_A = 0;
      CK_B = 0;
      for (i = 2; i < 70 ; i++) {
        CK_A = CK_A + ubxmessage.rawBuffer[i];
        CK_B = CK_B + CK_A;
      }

      if (CK_A == ubxmessage.rawBuffer[70] && CK_B == ubxmessage.rawBuffer[71]) {

        if (IMU_MPU6050 > 1) Heading_MPU6050();  // to correct the drift
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

  //  LED on == no WiFi or Ethernet, off == WiFi or Ethernet   #########################################
/*  if ((send_Data_Via > 0) && (Ntriphotspot_an == 0)) {
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
*/
  //  LED on == no Ntrip from ESP32, off == Ntrip from ESP32   #########################################
  if ((Ntriphotspot == 1) || (Ntriphotspot == 2)) {
    if (Ntriphotspot_an == 1) {
      digitalWrite(LED_ntrip_ON, LOW);
    }
    else {
      //digitalWrite(LED_ntrip_ON, HIGH);
    }
  }
  else {
    digitalWrite(LED_ntrip_ON, LOW);
  }

  if (OTA_begin_Time > (millis() - 90000)) {
    OTA_update = true;
    ArduinoOTA.handle();   //  update the ESP32 via WiFi.
  }
  else  OTA_update = false;
  if (IMU_MPU6050 > 1) RollHeading_MPU();
}//end main loop

//------------------------------------------------------------------------------------------
