# Dualheading
Heading for autonomous driving, specially for AGopenGPS. 
With an ESP32 and 2 Ardusimple F9P, heading and Roll for a vehicle is calculated. 
A little instruction is at the begin of every sketch. 
In Version Dualsingle you connect by USB or Ethernet and you can use Ntrip directly with a handy hotspot or router or by AGopenGPS over wifi or Ethernet.
This version is also for BENJAMINS PCB with two esp32 for AMATRON. 
When you want to use the second esp32, take the dual_1.13_F9P_AMA config for zhe first F9P instead of dual_1.13_F9P_Position..
For the 2.ESP32 use the Program  2._ESP_Baudwandler.
You can input 5 Networks, for example, 5 different User with different handy hotspots.
The program scans WiFi and connect with the first found, if it fails, it tries again.
You can install a button, for scanning WiFi again pin 4 connect to GND.
Also a message is send back to Ntrip server. !!!!!
Also you can update Over The Air, OTA
The Photo is a symbol wireing. the Gpio Pin for LED is 2 and Gpio Pin for Ntrip is 4
Dualsingle is a version where both cores are in use. One for WiFi and second for position.
The benefit is that you have no break in AgopenGPS when Wifi make a break.
Ntrip with APOS is possible.

