# Dualheading
Heading for autonomous driving, specially for AGopenGPS. 
With an ESP32 and 2 Ardusimple F9P, heading and Roll (possible) for a vehicle is calculated. 
A little instruction is at the begin of every sketch. 
In Version you connect by USB, WiF or Ethernet and you can use 2 Ntrip directly with a handy hotspot or router or by AGopenGPS over USB, wifi or Ethernet.
This version is also for BENJAMINS  and my PCB with two esp32 for AMATRON. 
For the 2.ESP32 use the Program  2._ESP_Baudwandler.
You can input 7 Networks, for example, 7 different User with different handy hotspots.
The program scans WiFi and connect with the first found, if it fails, it tries again.
You can install a button, for scanning WiFi again pin 4 connect to GND.
Also a message is send back to Ntrip server. !!!!!
Also you can update Over The Air, OTA
The Photo is a symbol wireing. the Gpio Pin for LED is 2 and Gpio Pin for Ntrip is 4
Ntrip with APOS is possible.

# Absolute Heading Determination With Two GNSS Receivers

## What does it do?

Computes an absolute heading by measuring the relative positions of two GNSS antennas.

Applications:

* [AgOpenGPS](https://github.com/farmerbriantee/AgOpenGPS) replacing inertial measurement unit and magnetic compass (e.g., CMPS14)
* General navigation (e.g., autonomous driving)

## Why should I use it?

* Real-time kinematic positioning (RTK) with differential GNSS data from remote NTRIP host (base station or station network):
    * position, velocity, and time (PVT)
    * heading
* Receives NTRIP client data forwarded from AgOpenGPS or
* NTRIP client in ESP32:
    * login credentials to be provided in the source code (e.g., for SAPOS in Germany)
    * reports own position back to NTRIP broadcaster
    * main advantage: notebook with AgOpenGPS is not connected to UMTS/LTE/5G network, thus no worries about data volume limits
* Data IO via USB, Wi-Fi, or Ethernet (requires additional hardware)
* Supports several Wi-Fi access points (router or smartphone hotspot), but of course not at the same time
* Optional roll calculation
* Optional data forwarding to another ESP32
* Over-the-air (OTA) re-programming 
* two ntrip, when you loose one, change automaticly to the second one.

## What do I need?

### Minimum

* Two [Ardusimple simpleRTK2B](https://www.ardusimple.com/product/simplertk2b-basic-starter-kit-ip65/) GNSS receivers based on the u-blox ZED-F9P chip with antennas. Note: with one GNSS receiver PVT only, but no heading.
* One ESP32 Dev Kit
* External power supply
* Wiring according to figure provided

### Optional

* [Benjamin's PCB](https://oshwlab.com/biohofbolten/dualgps-agopengps-v1-4)
* W5500 Ethernet shield for wired IP networks
* Button between pin 4 and GND initiating a (re-)scan of available Wi-Fi networks
* 2nd ESP32 Dev Kit for AmaTron applications

## How do I configure it?

### Ardusimple simpleRTK2B

Upload the "AMA_PVT" and "HEADING" configurations to the "right" (PVT) and "left" (relative position) GNSS receivers by means of the [u-center](https://www.u-blox.com/en/product/u-center) software. Please note that these settings require firmware v1.13.

### ESP32

Configure the variables in the main .ino file according to your setup and needs. This includes SSIDs and password for your Wi-Fi access points.

The GPIO pin for the LED (???) is 2 and the GPIO pin for NTRIP is 4.

## How do I use it?

Upon startup the program scans Wi-Fi networks within range and connects to the first found. If it fails (or if the optional button is pressed), it will try again.

--------------------

# Absolute Richtungsbestimmung mit zwei GNSS Empfängern

## Was macht das?

Die Software errechnet eine absolute Richtung aus den relativen Positionen zweier GNSS-Antennen zueinander.

Anwendungen:

* [AgOpenGPS](https://github.com/farmerbriantee/AgOpenGPS). Ersetzt die inertiale Messeinheit (IMU) und den Magnetkompass (z.B. CMPS14).
* Allgemeine Navigationsanwendungen wie z.B. autonomes Fahren

## Was kann das?

* Echtzeitkinematik (RTK) unter Verwendung von differentiellen GNSS-Daten von einem NTRIP Host (Referenzstation oder Satellitenreferenzdienst):
    * Position, Geschwindigkeit und Zeit (PVT)
    * Richtung (Heading)
* Kann von AgOpenGPS weitergeleitete NTRIP-Daten empfangen oder
* Verwendung als NTRIP Client im ESP32:
    * Zugangsdaten im Code eintragen (z.B. für SAPOS in Deutschland)
    * Sendet die eigene Position zurück zum Satellitenreferenzdienst
    * Hauptvorteil: Das Notebook/Tablet mit AgOpenGPS benötigt keine Internetverbindung über UMTS/LTE/5G, so dass man sich keine Sorgen um das Datenvolumen machen muss.
* Datenschnittstellen: USB, Wi-Fi, oder Ethernet (benötigt zusätzliche Hardware)
* Kann sich mit mehreren Wi-Fi Access Points verbinden (Router oder Smartphone Hotspot), aber natürlich nicht gleichzeitig
* Optionale Rollwinkelberechnung
* Optionale Datenweiterleitung an einen weiteren ESP32
* Over-the-air (OTA) Reprogrammierung
* 2 Ntrips, wenn einer ausfällt, wird zum 2. verbunden

## Was benötige ich?

### Minimalausstattung

* Zwei [Ardusimple simpleRTK2B](https://www.ardusimple.com/product/simplertk2b-basic-starter-kit-ip65/) GNSS Empfänger mit dem u-blox ZED-F9P Chip sowie Antennen. Hinweis: mit nur einem GNSS Empfänger kann nur PVT bestimmt werden, aber nicht die Richtung.
* Ein ESP32 Dev Kit
* Externe Stromversorgung
* Verdrahtung gemäß beiliegender Zeichnung

### Optional

* [Benjamins Platine](https://oshwlab.com/biohofbolten/dualgps-agopengps-v1-4)
* W5500 Ethernet Shield für drahtgebundene IP-Netzwerke
* Taster zwischen Pin 4 und GND, der ein (erneutes) Scannen verfügbarer Wi-Fi Netzwerke auslöst
* Zweites ESP32 Dev Kit für AmaTron-Anwendungen

## Wie konfiguriere ich das?

### Ardusimple simpleRTK2B

Mittels der [u-center](https://www.u-blox.com/en/product/u-center) Software die "AMA_PVT" und "HEADING" Konfigurationen auf dem "rechten" (PVT) und dem "linken" (relative Position) GNSS Empfänger aufspielen. Diese Einstellungen erfordern Firmware v1.13.

### ESP32

Die Variablen in der Haupt-Ino-Datei entsprechend gewünschtem Aufbau einstellen. Dazu gehören auch die SSIDs und Kennwörter für die Wi-Fi Access Points.

Der GPIO Pin für die LED (???) ist 2 und der GPIO Pin für NTRIP ist 4.

## Wie verwende ich das?

Nach dem Start sucht das Programm nach erreichbaren Wi-Fi Netzwerken und verbindet sich mit dem, das zuerst gefunden wurde und wofür Zugangsdaten hinterlegt sind. Wenn kein Netzwerk gefunden wird (oder falls der optionale Taster gedrückt wird), beginnt die Suche erneut.
