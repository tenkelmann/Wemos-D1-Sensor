// ************************************************************
// Wemos-Sketch: Basis
// inkl. Ultraschall, BME280, Wifi, MQTT, Webserver, Time
// 5/2019 bahnuhr, Dieter Mueller
// ************************************************************
    #include <Arduino.h>
    #include <ESP8266WiFi.h>
    #include <WiFiClient.h>
    #include <ESP8266WebServer.h>
    #include <ESP8266mDNS.h>
    #include <MQTT.h>
    #include <Wire.h>
    #include <NTPtimeESP.h>
    #include "cactus_io_BME280_I2C.h"
// ************************************************************
// Eingaben machen:
// ************************************************************
// BME 280
    int alti = 314; // aktuelle Hoehe über Meeresspiegel = Altitude -> zur Korrektur von pressure
// Network Wifi
    String ssid = "Mueller-xxxx";
    const char* password = "abcdefg";
    IPAddress ip = ipaddr_addr("192.168.xxx.xx"); // feste IP-Adresse fuer den WeMos
// MQTT
    const char* mqtt_user = "abcde"; // Benutzername der MQTT-Instanz auf dem IoBroker
    const char* mqtt_passwd = "ccc"; // Passwort der MQTT-Instanz auf dem IoBroker
    const char* mqtt_ip = "192.168.xxx.xx"; // IP Adresse des IoBroker
    int mqtt_port = 1884; // Port des IoBroker
    const char* mqtt_name = "Wemos-Test"; // Name des Wemos der in IoBroker im Adapter MQTT unter info angezeigt wird.
// Wemos-System
    String Wemos_Device = "Test";   // z.B. Heizoel
   
// ************************************************************
// ab hier keine Veränderung mehr !
// ************************************************************
// HC-SR04 Ultraschall        // VCC = 5V ; GND an GND ; echo an D7 ; trig an D6
    #define echoPin D7   // Echo Pin
    #define trigPin D6   // Trigger Pin
    int maximumRange = 200;  int minimumRange = 0; // Maximum und Minimum range needed
    float duration, distance, distance_alt; // Duration used to calculate distance
// BME280                     // VCC = 3,3V ; GND an GND ; SCL an D1 ; SDA an D2
    BME280_I2C bme(0x76);  // I2C using address 0x76
    float hPa_alt, Grad_alt, feuchte_alt;  // alte Werte
    int hPa_korrektur; 
// Network Wifi
    IPAddress gateway = ipaddr_addr("192.168.xxx.1"); // IP-Adresse des WLAN-Gateways
    IPAddress subnet = ipaddr_addr("255.255.255.000"); // Subnetzmaske
    IPAddress dns1 = ipaddr_addr("8.8.8.8");//dns 1
    IPAddress dns2 = ipaddr_addr("8.8.4.4");//dns 2
    long rssi_alt; // Signalstärke alt
// MQTT
    String mqtt_send = "nein";
    WiFiClient net;
    MQTTClient client;
// Wemos-System
   int serial = 9600; // Übertragungsgeschwindigkeit Serial
   int loop_delay = 3000;  // in ms - Delay in Loop
   int loop_mqtt = 5000;  // in ms - Delay MQTT
   unsigned long letzteMillis_mqtt = 0;   // speichern last time
   int loop_verbindung = 300000;  // in ms - Delay Verbindung -> alle 5 Min. Wifi und mqtt prüfen
   unsigned long letzteMillis_verbindung = 0;   // speichern last time
// Time
   NTPtime NTPch("2.de.pool.ntp.org");   // Choose server pool as required
   strDateTime dateTime;
   int time_offset = 1;  // offset 1 Std.
   int time_summertime = 1;  // 1=Sommerzeit
   String dayStr = ""; String monatStr = ""; String jahrStr = ""; String stdStr = ""; String minStr = ""; String sekStr = ""; 
//Server
   ESP8266WebServer server(80);


// *************************************************
// Unterprogramm: void setup
// *************************************************
void setup()  {
  Serial.begin(serial);
  // Ultraschall
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  // BME 280
    if (!bme.begin()) {
      Serial.println("Es konnte kein BME280 Sensor gefunden werden!");
      Serial.println("Bitte überprüfen Sie die Verkabelung!");
      while (1);
    }
      bme.setTempCal(-1);
  // WiFi, MQTT, Webserver
  setup_wifi();
  setup_mqtt();
  setup_webserver();
}


// *************************************************
// Unterprogramm: void loop
// *************************************************
void loop()  {
  unsigned long aktuellMillis = millis();
    if (aktuellMillis - letzteMillis_mqtt >= loop_mqtt) { mqtt_send = "ja"; } else { mqtt_send = "nein"; }
  // WiFi und MQTT prüfen
    if (aktuellMillis - letzteMillis_verbindung >= loop_verbindung) {
      if (WiFi.status() != WL_CONNECTED) {
        setup_wifi();
      }
      Serial.println("WiFi geprüft und ok");
      client.publish("Wemos_" + Wemos_Device + "/Meldung", "WiFi geprüft und ok.");
    // MQTT prüfen
       if (!client.connect(mqtt_name, mqtt_user, mqtt_passwd)) { // Verbindung zum MQTT wird hergestellt
         setup_mqtt();
       }
       Serial.println("MQTT geprüft und ok");
       client.publish("Wemos_" + Wemos_Device + "/Meldung", "MQTT geprüft und ok.");
    letzteMillis_verbindung = aktuellMillis;
    }
  // Webserver
    server.handleClient(); // auf HTTP-Anfragen warten
  // Time
    Zeit_holen();
  // Ultraschall HC-SR04 Daten abrufen
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
    // Calculate the distance (in cm) based on the speed of sound.
      distance = duration/58.2;
      if (distance >= maximumRange || distance <= minimumRange){
        Serial.println("---");
      }
      Serial.print(distance); Serial.print(" cm\t");
  // BME 280
    hPa_korrektur = (alti / 8) - 1; 
    bme.readSensor(); 
    Serial.print(bme.getPressure_MB() + hPa_korrektur); Serial.print(" hPa\t");    // Pressure in millibars
    Serial.print(bme.getHumidity()); Serial.print(" %\t\t");
    Serial.print(bme.getTemperature_C()); Serial.print(" °C\t");
  // Wifi-RSSI uebertragen
      int rssi = WiFi.RSSI();
      Serial.print(rssi); Serial.println(" WiFi-RSSI");
  // Daten an MQTT übertragen
    if (mqtt_send == "ja") {
      // Ultraschall
	if (distance - distance_alt > 1 || distance - distance_alt < -1) {
          client.publish("Wemos_" + Wemos_Device + "/Distance", String(distance)); 
          client.publish("Wemos_" + Wemos_Device + "/Meldung", "Distance geändert (akt.;alt): " + String(distance) + " ; " + String(distance_alt)); 
          distance_alt = distance;
        }
      // BME 280
        if ((bme.getPressure_MB() + hPa_korrektur - hPa_alt) > 1 ||(bme.getPressure_MB() + hPa_korrektur - hPa_alt) < -1) {
          client.publish("Wemos_" + Wemos_Device + "/BME280-Luftdruck", String(bme.getPressure_MB() + hPa_korrektur)); 
          client.publish("Wemos_" + Wemos_Device + "/Meldung", "Luftdruck geändert (akt.;alt): " + String(bme.getPressure_MB()+ hPa_korrektur) + " ; " + String(hPa_alt)); 
          hPa_alt = (bme.getPressure_MB() + hPa_korrektur);
        }
        if (bme.getHumidity()- feuchte_alt > 1 || bme.getHumidity()- feuchte_alt < -1) {
          client.publish("Wemos_" + Wemos_Device + "/BME280-Luftfeuchtigkeit", String(bme.getHumidity())); 
          client.publish("Wemos_" + Wemos_Device + "/Meldung", "Luftfeuchtigkeit geändert (akt.;alt): " + String(bme.getHumidity()) + " ; " + String(feuchte_alt)); 
          feuchte_alt = bme.getHumidity();
        }
        if (bme.getTemperature_C()- Grad_alt > 1 || bme.getTemperature_C()- Grad_alt < -1) {
          client.publish("Wemos_" + Wemos_Device + "/BME280-Temperatur", String(bme.getTemperature_C())); 
          client.publish("Wemos_" + Wemos_Device + "/Meldung", "Temperatur geändert (akt.;alt): " + String(bme.getTemperature_C()) + " ; " + String(Grad_alt)); 
          Grad_alt = bme.getTemperature_C();
        }
      // Wifi-RSSI
        if (rssi-rssi_alt > 2 || rssi-rssi_alt < -2) {
          client.publish("Wemos_" + Wemos_Device + "/WiFi-RSSI", String(rssi)); 
          client.publish("Wemos_" + Wemos_Device + "/Meldung", "WiFi-RSSI geändert (akt.;alt): " + String(rssi) + " ; " + String(rssi_alt)); 
          rssi_alt = rssi;
        }
    letzteMillis_mqtt = aktuellMillis;
    }
//Delay x before next reading
  delay(loop_delay);
}


// -------------------------------------------------
// Unterprogramm: setup_wifi
// -------------------------------------------------
void setup_wifi() {
  delay(10);
// Connect WiFi
  Serial.println(); Serial.println(); Serial.print("Connecting to "); Serial.println(ssid);
  WiFi.hostname("Wemos_" + Wemos_Device);
  WiFi.config(ip, gateway, subnet, dns1, dns2); WiFi.mode(WIFI_STA); WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println(""); Serial.println(""); Serial.println("WiFi verbunden!");
// Print the IP address
  Serial.print("IP-Addresse: "); Serial.print(WiFi.localIP()); Serial.println("");
}
// -------------------------------------------------
// Unterprogramm: setup_mqtt
// -------------------------------------------------
void setup_mqtt() {
  client.begin(mqtt_ip, mqtt_port, net); // Verbindungsaufbau zu MQTT
  Serial.println(); Serial.print("MQTT verbinden!");
  while (!client.connect(mqtt_name, mqtt_user, mqtt_passwd)) { // Verbindung zum MQTT wird hergestellt
    Serial.print(".");
  }
  Serial.println(); Serial.println("MQTT verbunden!"); Serial.println("");
  client.publish("Wemos_" + Wemos_Device + "/IP-Adresse", myIp() );
  client.publish("Wemos_" + Wemos_Device + "/Meldung", "WiFi und MQTT geprüft und ok.");
}
// -------------------------------------------------
// Unterprogramm: setup_webserver
// -------------------------------------------------
void setup_webserver() {
  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }
  server.begin();
  Serial.println("Webserver gestartet!");
  Serial.print("Benutze diese URL: "); Serial.print("http://"); Serial.print(WiFi.localIP()); Serial.println("/"); Serial.println("");
    // HTTP-Anfragen bearbeiten
      server.on("/" , handleRoot);
      server.onNotFound(handleNoRoot);
}
// -------------------------------------------------
// Unterprogramm: IP Adresse ermitteln
// -------------------------------------------------
String myIp() {
    char myIpString[24];
    IPAddress myIp = WiFi.localIP();
    sprintf(myIpString, "%d.%d.%d.%d", myIp[0], myIp[1], myIp[2], myIp[3]);
    return myIpString;
}
// -------------------------------------------------
// Unterprogramm: Time
// -------------------------------------------------
void Zeit_holen() {
    dateTime = NTPch.getNTPtime(time_offset, time_summertime);
    if(dateTime.valid){
      byte actualHour = dateTime.hour; byte actualMinute = dateTime.minute; byte actualsecond = dateTime.second;
      int actualyear = dateTime.year; byte actualMonth = dateTime.month; byte actualday =dateTime.day;
      byte actualdayofWeek = dateTime.dayofWeek;
      jahrStr = dateTime.year;
      monatStr = dateTime.month < 10 ? "0" + String(dateTime.month) : String(dateTime.month);
      dayStr = dateTime.day < 10 ? "0" + String(dateTime.day) : String(dateTime.day);
      stdStr = dateTime.hour < 10 ? "0" + String(dateTime.hour) : String(dateTime.hour);
      minStr = dateTime.minute < 10 ? "0" + String(dateTime.minute) : String(dateTime.minute);
      sekStr = dateTime.second < 10 ? "0" + String(dateTime.second) : String(dateTime.second);        
      Serial.print("Stand: "); Serial.print(dayStr); Serial.print("."); Serial.print(monatStr); Serial.print("."); Serial.print(dateTime.year);
      Serial.print(" um "); Serial.print(stdStr); Serial.print(":"); Serial.print(minStr); Serial.print(":"); Serial.print(sekStr); Serial.println(" Uhr");
    }  
}
// -------------------------------------------------
// Unterprogramm: Webserver
// -------------------------------------------------
void handleRoot() { // bei Aufruf des Root-Verzeichnisses
    String antwort = "";
    int rssi = WiFi.RSSI();
    antwort = "WeMos: \t\t" + Wemos_Device + "\n";
    antwort = antwort + "==============================================\n\n";
    antwort = antwort + "Stand: \t\t" + dayStr + "." + monatStr + "." + jahrStr + " um " + stdStr + ":" + minStr + ":" + sekStr + " Uhr\n";
  // Ultraschall
    antwort = antwort + "\n";
    antwort = antwort + "Ultraschall:";
    antwort = antwort + "\tDistance: \t\t" + distance + " cm\n";
  // BME 280  
    bme.readSensor(); 
    antwort = antwort + "\n";
    antwort = antwort + "BME 280:";
    antwort = antwort + "\tAltitude: \t\t" + alti + " Meter\n";
    antwort = antwort + "\t\tLuftdruck: \t\t" + (bme.getPressure_MB() + hPa_korrektur) + " hPa\n";
    antwort = antwort + "\t\tLuftfeuchtigkeit: \t" + bme.getHumidity() + " %\n";
    antwort = antwort + "\t\tTemperatur: \t\t" + bme.getTemperature_C() + " Grad Celsius\n";
  // System
    antwort = antwort + "\n";
    antwort = antwort + "System:";
    antwort = antwort + "\t\tVerbunden mit: \t\t" + ssid + "\n";
    antwort = antwort + "\t\tSignalstaerke: \t\t" + String(rssi) + " dBm\n";
    antwort = antwort + "\t\tIP-Adresse: \t\t" + myIp() + "\n";
    antwort = antwort + "\n";
  // MQTT
    antwort = antwort + "ioB-MQTT:";
    antwort = antwort + "\tName: \t\t\t" + mqtt_name + "\n";
    antwort = antwort + "\t\tIP-Adresse: \t\t" + mqtt_ip + "\n";
    antwort = antwort + "\t\tMQTT-Port: \t\t" + String(mqtt_port) + "\n";
    antwort = antwort + "\t\tSende-Intervall: \t" + String(loop_mqtt/1000) + " Sekunden\n";
    antwort = antwort + "\n\n";
 
    antwort = antwort + "Folgende Werte einstellen mit: \t" + myIp() + "/wert?...\n";
    antwort = antwort + "---------------------------------------------------------------------------------------\n";
    antwort = antwort + "Sende-Intervall MQTT an iobroker\t" + "delay=x\t\t" + "(x = Sekunden)\n";
    antwort = antwort + "Altitude des aktuellen Standorts\t" + "alti=x\t\t" + "(x = Meter ueber Meeresspiegel)\n";
    antwort = antwort + "SSID fuer Wlan\t\t\t\t" + "ssid=x\t\t" + "(ssid zum einloggen)\n\n\n";

    antwort = antwort + "@ 05/2019 bahnuhr, Dieter Mueller";
    server.send(200, "text/plain", antwort);
    delay(150);
}
void handleNoRoot() {
    String antwort = "";
    if (server.args() > 0) {
      for (uint8_t i = 0; i < server.args(); i++) {
        if (server.argName(i) == "delay") {
          loop_mqtt = ((server.arg(i)).toInt())*1000;
          Serial.print("Delay eingestellt auf: "); Serial.print(loop_mqtt/1000); Serial.println(" Sekunden"); 
          handleRoot();
        } else if (server.argName(i) == "alti") {
          alti = (server.arg(i)).toInt();
          Serial.print("Altitude eingestellt auf: "); Serial.print(alti); Serial.println(" Meter ueber Meeresspiegel"); 
          handleRoot();
        } else if (server.argName(i) == "ssid") {
          ssid = server.arg(i);
          Serial.print("SSID eingestellt auf: "); Serial.println(ssid);  
          setup_wifi();
          setup_mqtt();
          setup_webserver();
          handleRoot();
        } else {
          server.send(404, "text/plain", "Eingabe ist falsch und konnte nicht bearbeitet werden!");
        }
      }
    } else {
      server.send(404, "text/plain", "Eingabe ist falsch und konnte nicht bearbeitet werden!");
    }
}
