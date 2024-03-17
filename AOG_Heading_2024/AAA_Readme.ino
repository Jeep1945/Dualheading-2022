// Dualhead for AGopenGPS
// Private-use only! (you need to ask for a commercial-use)
// by Franz Husch  18.02.2024 in cooperation with Matthias Hammer
// and Valentin Ernst and many more
// **** OTA is possible over Network  **** //
// Antennas cross to driveDirection on cabin symmetrical
// right Antenna is Rover (A) for position, left Antenna is MB (B) for heading
// Ntrip client for 7 Router or Handy hotspots or from AgopenGPS
// Progamm tries 4 time to connect,
// you can install a button from GND to PIN 4 
// by pressing the button a new WiFi scan starts or Accesspoint for OTA.
// PIN 2 you can add an LED for ntrip is received, have a look to the Photo
// LED is on when no Network could be connected
// Dual_Antenna  0: for single antenna, 1: for Dualantenna
// send_amatron_nmea  to send GGA, VTG, ZDA to a second ESP32 to use it for extern SC  0: for not 1: for sending,
// not anymore rollaktiv (0) for use roll in AgopenGPS, offset and hight of Antenna set in setup AOG
// rollaktiv (1) for use roll by ESP32, offset and hight of Antenna set to 0 in setup AOG
// WiFi_scan_Delay is the mount of sec, you will need start router or hotspot
// in Data Sourcess
// OTA_active Update over the air, connect to WiFi and send update per WiFi
// GGA_Send_Back_Time for SAPOS or Apos set to 10,
//  IMPORTANT  // serial USB baude changed to 38400
//  IMPORTANT  // you have to use the new AMA PVT config for the right antenna
// you get install 2 Ntrips and if you loose first, it changes automaticly to the second
// IMU_MPU6050 = 1;       // 1: to 10   1: from Dual   10: from MPU 
// move_line_buttons = 1;       // 0: no   1: by ethernet buttons to move AB line
// connect ethernet again with button pin 13, 
