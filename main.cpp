//FreZZe mit EA-Display DOGM204 I2C
//Läuft ab LP V0.2
//Software erstellt 02. September 2017
//Version 1.1 - 06. Januar 2020
//Version 1.2 - 06. April 2020 - Unterstützung für NTP (Normalzeit) eingebaut
//Version 1.3 - 8. Mai 2022 - Wechsel auf Platform IO und Visual Studio Code
//              2. Juli 23 - Letzte Tests V1.3 und Freigabe
//              Achtung! Für den PufferC C15 bzw. C15 wird neu 1000uF benötigt, sonst funktioniert das automatische 
//                       Abspeichern bei Spannungsverlust nicht mehr zuverlässig!

#define VERSION "V1.3"

//Includes Librarys:
#include <Arduino.h>
#include "FS.h"                   //this needs to be first, or it all crashes and burns...
#include <LittleFS.h>

#include "TickerScheduler.h"      //Ticker

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <StreamUtils.h>

//needed for NTP 
#include <TimeLib.h>
#include <WiFiUdp.h>
#include <Timezone.h>             //by Jack Christensen, not included in the Arduino IDE !!!

// Communication for the Display
#include <Wire.h>

#include <String>

//Eigene Funktionen
#include "FreZZe.h"

//Defines for Pins
#include "FreZZe_PINS.h"

//Defines für Zeiten
#include "FreZZe_TIMES.h"

//Defines für das Display
#define I2C_ADRESS 0x3D
#define DISPLAYKONTRAST   10      //nach Test ein guter Wert

//Variablen für die LED Anzeige
int LEDStatus = 0;                //LED Status wird durch das Programm gemäss den oben angegebenen Defines gesetzt und durch die Task LED() wird die LED angesteuert
int LEDStatusIntern = 0;          //LED Status intern für die LED Routine
int LEDStatusCounter = 0;         //LED Status Counter zum zählen bis zum nächsten Statuswechsel
int LEDPeriodCounter = 0;         //Dient als Zähler für das LEDaus vorbereiten (Periodenlänge LED an)....

//define your default values here, if there are different values in config.json, they are overwritten.
char mrc_multicast[40] = "239.50.50.20";
char mrc_port[6] = "2000";

//char array for the protokollhandler 
char c_clock_hour[15] = "03";
char c_clock_minute[15] = "02";
char c_clock_sek[15] = "04";
char c_time[15] = "0";
char c_speed[15] = "0";

//Strings 
String esp_chipid;

//Displaybuffer
char SBuffer[4][80] = {{ "11111111111111111111" },{ "22222222222222222222" },{ "33333333333333333333" },{ "44444444444444444444" }};

//TEMP Variable for Display
char ctempd[200];
String stempd;

//VARIABLEN für die IP-Addressenübernahme
String sIPAddressinformertime;    //Letzte bekannte IP-Adresse, welche akzeptiert wurde
String sIPAddressjustrecieved;    //aktuelle empfangene IP-Adresse
String sIPAddresstemprecieved;    //diese IP-Addresse wird nach einer Zeit akzeptiert und zu sIPAddressinformertime übernommen
int iCounterIPAddress = 0;        //Counter zur Übernahme einer neuen IP-Adresse

//Daten aus dem Protokoll
int  clock_h = 0;   // Sollzeit Stunde
int  clock_m = 0;   // Sollzeit Minute
int  clock_s = 0;   // Sollzeit Sekunde
int  clock_speed = 1; //Zeitverkürzung
bool clock_aktiv = false; //Läuft die Uhr oder steht sie? 

int  tochter_h = 0; // Istzeit auf der 12h Tochteruhr (Stunden)
int  tochter_m = 0; // Istzeit auf der 12h Tochteruhr (Minuten)

int  itelecnt = 0; //Zähler wird als Watchdog für den Telegrammempfang, wird beim Telegrammenpfang 
                   //auf den TELECNTSTART Wert gesetzt und in jeder Schleife um ein reduziert.
                   //ist er 0, wird die Ampel auf rot geschaltet.
#define TELECNTSTART 6000 //Zeitbasis = der Delay() Wert in der Loop Schleife! 1 entspricht 10ms bei Delay(10)

//NTP Variablen
char ntpserver[40] = "ntp.metas.ch";         //NTPServer
unsigned int localPort = 8888;               // local port to listen for UDP packets
const int timeZone = 0;                      // UTC
double dCounterToNTP;                        //if no MRCLOCK telegramm is recieved, the conter gets decressed, on ZERO the NTP time is displayed
const int NTP_PACKET_SIZE = 48;              // NTP time is in the first 48 bytes of message
byte packetBufferNTP[NTP_PACKET_SIZE];       //buffer to hold incoming & outgoing packets

//Timezone
//Central European Time (Frankfurt, Paris)
TimeChangeRule CEST = { "CEST", Last, Sun, Mar, 2, 120 };     //Central European Summer Time
TimeChangeRule CET = { "CET ", Last, Sun, Oct, 3, 60 };       //Central European Standard Time
Timezone CE(CEST, CET);
TimeChangeRule *tcr;        //pointer to the time change rule, use to get the TZ abbrev
time_t utc, local;
time_t prevDisplay = 0;     // when the digital clock was displayed

//Statemaschine für TochterUhrStellen Routine
int  SM = 0;                  // Statemaschine Status
int  SMC = 0;                 // Statemaschine Counter
bool FlagTUStellen;           // Flag ist gesetzt, falls die Tochteruhr einen Tick machen soll
bool FlagTUStellglied;        // Flag wiederspiegelt den Ausgang des TU Stellglieds

//Rücksetztaste
bool FlagButtonPresed = false;
int  CounterButtonPressed;

//Tickerscheduler Objekt

TickerScheduler ts(10);

//flag for saving data
bool shouldSaveConfig = false;

//flag for sleep mode 
bool sleep = false;
bool flag_message_recieved = true;
bool flag_wrongIP = false;

//flag for clock ticks
bool bClockBad = true;  //true on startup, until one falid Clockmessage war recieved

//WifiUDP Class
WiFiUDP wifiUDPMRC;     //für MRClock Telegramme
WiFiUDP wifiUDPNTP;     //für NTP Telegramme

const int MRC_PACKET_SIZE = 2024; // NTP time stamp is in the first 48 bytes of the message

char packetBuffer[ MRC_PACKET_SIZE ]; //buffer to hold incoming and outgoing packets

int nLength = 0;

int time_hour = 0;

int time_minute = 0;

unsigned long millisAfterISR = 0;

//WiFiManager
  //Das Object wird auch noch im LOOP benötigt um die Config zu löschen.... 
  WiFiManager wifiManager;
  //Die beiden Parameter werden durch ein Callback verändert, deswegen müssen sie Global definiert sein
  WiFiManagerParameter custom_time_hour();
  WiFiManagerParameter custom_time_minute();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting up!");

  LED(LEDOff);               // Am Anfang ist die LED aus....

  //ChipID?
  esp_chipid = String(ESP.getChipId());

  //Identify
  Serial.print("Tochteruhrempfänger TUE ");
  Serial.print(VERSION);
  Serial.print(" ESP - ChipID = ");
  Serial.println(esp_chipid); 

  Serial.println("Configuring Pins and Interrupts");

  //Variablen initialisieren
  FlagTUStellen = false;  //FlagTUStellen initialisieren
  SM = 1;
  SMC = 0;                //Statemaschine für die Taktausgabe initialisieren
  
  delay(500);

  //Display initialisieren
  //I2C Pins konfigurieren

  pinMode(DISP_RESET, OUTPUT);
  digitalWrite(DISP_RESET, LOW);
  delay(500);
  digitalWrite(DISP_RESET, HIGH);
  delay(500);
  
  //I2C initialisieren, Display init
  Wire.begin();

  Wire.setClock(200000L);
  
  LCD_Init();

  delay(10);

  // Buffer          "1   5    10   15  19"
  strcpy(SBuffer[0], "FREMO MRCLOCK       ");
  strcpy(SBuffer[1], "Reciever        ");
  strcat(SBuffer[1], VERSION); 
  strcpy(SBuffer[2], "                    ");
  strcpy(SBuffer[3], "INIT.               ");

  LCD_Refresh();

  delay(1000);

  //WiFi.setAutoReconnect(false);
  WiFi.persistent(true); // Wird gebraucht damit das Flash nicht zu oft beschrieben wird.
  WiFi.setAutoReconnect(true);
  
  //Pins konfigurieren
  //GPIO #12 (TaktA)
  //GPIO #13  (TaktB)
  //GPIO #16 (Input für Config Pushbutton) Bei MRC_REC = 5!
  //GPIO #2 = blue LED on ESP
  //GPIO #14 = Interrupt für Spannungseinbruch -> Interrupt wird an der fallenden Flanke ausgelöst
  //GPIO #15 Reset des Displays
  //GPIO #0 Ampel grün/rot

  pinMode(TAKTA, OUTPUT);
  pinMode(TAKTB, OUTPUT);
  pinMode(CONFIGPB, INPUT);
  pinMode(BLED, OUTPUT);
  pinMode(AMPEL, OUTPUT);

  digitalWrite(AMPEL, HIGH);  //Ampel auf Rot stellen

  //Konfiguration des InterruptPINs 
  //Sobald eine + Spannung gemessen wird, wird der Interrupt aktiviert, wird LOW gemessen, dann ist der Interrupt deaktiviert
  pinMode(INT, INPUT);
  delay(10);

  if(digitalRead(INT) == HIGH){     //Interrupt wird aktiviert
    Serial.println("Interrupteingang verdrahtet, wird benutzt!");
    attachInterrupt(digitalPinToInterrupt(INT), ISRSaveData, FALLING);
  }
  else{
    Serial.println("Interrupteingang ist nicht verdrahtet, wird nicht benutzt!");
    pinMode(INT, INPUT_PULLUP);
  }

  digitalWrite(BLED, HIGH);

  ts.add(5, 10, [&](void*) { LEDTS(); }, nullptr, false);

  if(ts.enable(5)){
    Serial.println("LEDTS() = 5 enabled");
  }

  LED(LEDBlinkOnce);         // LED einen Flash blinken -> 1. Blinkimpuls

  //clean FS, for testing
  //SPIFFS.format();
  //SPIFFS.remove("/config.json");
  //SPIFFS.remove("/config2.json");
  
  //read configuration from FS json
  Serial.println("mounting FS...");

  // Falls das Filesystem noch nicht formatiert ist, wird es hier automatisch gemacht.
  if(LittleFS.begin()){
    Serial.println("FS ok!");
  } else {
    LittleFS.format();
    Serial.println("FS formated");
  }

  LED(LEDBlinkOnce);     // LED einen Flash blinken  -> 2. Blinkimpuls

  //Filesystem mounten und Datei suchen, wenn Datei nicht vorhanden dann Neuerstellung
  if (LittleFS.begin()) {
    Serial.println("mounted file system");
    if (LittleFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = LittleFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        //size_t size = configFile.size();

        DynamicJsonDocument jsonDocument(2048);

        // ArduinoJson 6
        ReadLoggingStream loggingStream(configFile, Serial);
        DeserializationError error = deserializeJson(jsonDocument, configFile);
        Serial.print("\n-deserializeJson() -> output:");
        Serial.print(error.c_str());

        /*
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument jsonDocument(size);

        Serial.println("ConfigFile:");
        Serial.println(configFile.read());

        // ArduinoJson 6
        DeserializationError error = deserializeJson(jsonDocument, configFile);
        */
        
        if (!error) 
        {
          Serial.println("\nparsed json =>  ");

          /*
          strcpy(ntpserver, jsonDocument["ntp_server"]);
          strcpy(mrc_multicast, jsonDocument["mrc_multicast"]);
          strcpy(mrc_port, jsonDocument["mrc_port"]);
          strcpy(c_clock_hour, jsonDocument["clock_hour"]);
          strcpy(c_clock_minute, jsonDocument["clock_minute"]);
          if(jsonDocument["FlagTUStellglied"] == "true")
          {
            FlagTUStellglied = true;
            digitalWrite(TAKTA, LOW);
            digitalWrite(TAKTB, LOW);
            Serial.print("FlagTUStellglied = true");
          } else {
            FlagTUStellglied = false;
            digitalWrite(TAKTA, HIGH);
            digitalWrite(TAKTB, HIGH); 
            Serial.print("FlagTUStellglied = false");
          }

          */

          JsonVariant jvntpserver = jsonDocument["ntp_server"];
          if (!jvntpserver.isNull()) {
            Serial.print(" ntpserver=");
            Serial.print(jvntpserver.as<const char*>());
            strcpy(ntpserver, jvntpserver.as<const char*>());
          } else {
            strcpy(ntpserver, "ntp.metas.ch");
            Serial.print(" no value => ntp.metas.ch");
          }

          JsonVariant jvmrc_multicast = jsonDocument["mrc_multicast"];
          Serial.print(", mrc_multicastadress=");
          if (!jvmrc_multicast.isNull()) {
            Serial.print(jvmrc_multicast.as<const char*>());
            strcpy(mrc_multicast, jvmrc_multicast.as<const char*>());
          } else {
            strcpy(mrc_multicast, "239.50.50.20");
            Serial.print(" no value => 239.50.50.20");
          }

          JsonVariant jvmrc_port = jsonDocument["mrc_port"];
          Serial.print(", mrc_port=");
          if (!jvmrc_port.isNull()) {
            Serial.print(jvmrc_port.as<const char*>());
            strcpy(mrc_port, jvmrc_port.as<const char*>());
          } else {
            strcpy(mrc_port, "2000");
            Serial.print("no value => 2000");
          }

          JsonVariant jvc_clock_hour = jsonDocument["clock_hour"];
          Serial.print(", clock_hour=");
          if (!jvc_clock_hour.isNull()) {
            Serial.print(jvc_clock_hour.as<int>());
            itoa(jvc_clock_hour, c_clock_hour, 10);
            //strcpy(c_clock_hour, jvc_clock_hour.as<char*>());
          } else {
            strcpy(c_clock_hour, "0");
            Serial.print("no value => 0");
          }

          JsonVariant jvc_clock_minute = jsonDocument["clock_minute"];
          Serial.print(", clock_minute=");
          if (!jvc_clock_minute.isNull()) {
            Serial.print(jvc_clock_minute.as<int>());
            itoa(jvc_clock_minute, c_clock_minute, 10);
            //strcpy(c_clock_minute, jvc_clock_minute.as<char*>());
          } else {
            strcpy(c_clock_minute, "0");
            Serial.print("no value => 0");
          }

          JsonVariant jvc_flagTUStellglied = jsonDocument["FlagTUStellglied"];
          Serial.print(", FlagTUStellglied=");
          if(!jvc_flagTUStellglied.isNull()){
            if(jsonDocument["FlagTUStellglied"] == true){
              FlagTUStellglied = true;
              digitalWrite(TAKTA, LOW);
              digitalWrite(TAKTB, LOW);
              Serial.print("true");
            }else{
              FlagTUStellglied = false;
              digitalWrite(TAKTA, HIGH);
              digitalWrite(TAKTB, HIGH); 
              Serial.print("false");
            }
          }

          wifiManager.setConnectTimeout(20);   //Timeout etwas vergrössern, sodass der ESP Zeit hat sich zu verbinden!
        } else {
          Serial.println("\n->Error: unable to parse json!\nLoading Values!");
          strcpy(ntpserver, "ntp.metas.ch");
          strcpy(mrc_multicast, "239.50.50.20");
          strcpy(mrc_port, "2000");
          strcpy(c_clock_hour, "0");
          strcpy(c_clock_minute, "0");
          FlagTUStellglied = true;
          digitalWrite(TAKTA, LOW);
          digitalWrite(TAKTB, LOW);
          Serial.print("FlagTUStellglied = true");

          //Wenn die Information nicht stimmt, Uhrenlinie mit der letzten Tickspannung beaufschlagen
          forcerevercetick();

          //Konfiguration WLAN Credentials löschen
          WiFi.persistent(true);
        
          delay(1000);
          WiFi.setAutoReconnect(false);
          delay(100);
          WiFi.disconnect();
          delay(100);
          wifiManager.resetSettings();
          ESP.eraseConfig();
          delay(100);
          wifiManager.setConnectTimeout(3);   //Timeout kleiner, muss eh neu konfiguriert werden
        }
      }
    }
  } else {
    //Config File exsistiert nicht!
    Serial.println("failed to mount FS");
    FlagTUStellglied = true;
    digitalWrite(TAKTA, LOW);
    digitalWrite(TAKTB, LOW);
    Serial.println("failed to load json config");
  }
  //end read

  //Interne Zeit mit der Tochteruhrzeit gleich setzen, sonst läuft uns die Uhr nach einem Neustart los...
  tochter_h = atoi(c_clock_hour);
  tochter_m = atoi(c_clock_minute);

  // Setze Istzeit = Sollzeit, sonst läuft die Uhr schon nach dem Setzen los auf irgendeine Zeit...
  clock_h = tochter_h;
  clock_m = tochter_m;
  
  Serial.print("Tochteruhr zeigt ");
  Serial.print(tochter_h);
  Serial.print(":");
  Serial.print(tochter_m);
  Serial.println(" an. Sie sollte synchronisiert sein!");

   // Buffer          "1   5    10   15  19"
  strcpy(SBuffer[3], "INIT..              ");
  LCD_Refresh();

  delay(100);

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_text_info1("<br><b>FREMO-Zeitzeichenempf&auml;nger FreZZe</b><br>Einstellmen&uuml;<br>");
  WiFiManagerParameter custom_text_info2("Empf&auml;ngt Zeitzeichentelegramme nach dem MRC Protokoll.<br><br>");
  WiFiManagerParameter custom_text_info3("Standarteinstellung MRC: Multicast 239.50.50.20 auf Port 2000.<br>Bitte das WIFI Netz w&auml;len und die Uhrzeit der angeschlossenen Nebenuhr eingeben.<br>");
  WiFiManagerParameter custom_text_h("Stand der Nebenuhr, hier Stunden (ganze Zahl von 0 bis 11):");
  WiFiManagerParameter custom_time_hour("clock_hour", "zul&auml;ssige Werte (00-11)", c_clock_hour, 15);
  WiFiManagerParameter custom_text_m("Stand der Nebenuhr, hier Minuten (ganze Zahl von 0 bis 59):");
  WiFiManagerParameter custom_time_minute("clock_minute", "zul&auml;ssige Werte (00-59)", c_clock_minute, 15);
  WiFiManagerParameter custom_text_expert("<br>Ab hier Experteneinstellungen, nur &auml;ndern wenn wann weiss was man macht!<br>");
  WiFiManagerParameter custom_text_multicast_adress("Empfangsadresse der MRClock Telegramme:");
  WiFiManagerParameter custom_mrc_multicast("mrc_multicast", "mrc_multicast (239.50.50.20)", mrc_multicast, 40);
  WiFiManagerParameter custom_text_multicast_port("Empfangsport der MRClock Telegramme:");
  WiFiManagerParameter custom_mrc_port("mrc_port", "mrc_multicast_port (2000)", mrc_port, 5);
  WiFiManagerParameter custom_text_ntpserver("<br>Zeitserver zum Synchronisieren auf die MEZ:");
  WiFiManagerParameter custom_ntp_server("NTP_Server", "ntp.server.de", ntpserver, 40);
 
  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set APCallback
  wifiManager.setAPCallback(APCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  //add all your parameters here
  wifiManager.addParameter(&custom_text_info1);
  wifiManager.addParameter(&custom_text_info2);
  wifiManager.addParameter(&custom_text_info3);
  wifiManager.addParameter(&custom_text_h);
  wifiManager.addParameter(&custom_time_hour);
  wifiManager.addParameter(&custom_text_m);
  wifiManager.addParameter(&custom_time_minute);  
  wifiManager.addParameter(&custom_text_expert);
  wifiManager.addParameter(&custom_text_ntpserver);
  wifiManager.addParameter(&custom_ntp_server);  
  wifiManager.addParameter(&custom_text_multicast_adress);
  wifiManager.addParameter(&custom_mrc_multicast);
  wifiManager.addParameter(&custom_text_multicast_port);
  wifiManager.addParameter(&custom_mrc_port);

  //reset settings - durch das löschen der Daten, haben wir die Möglichkeit die Uhrzeit bei jedem Start neu einstellen zu können
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  LED(LEDBlinkOnce);         // LED einen Flash blinken    -> 3. Blinkimpuls

  LED(LEDOn);                // Ab jetzt ist die LED an, falls das CAPTIVE PORTAL gestartet ist
                             // Ansonsten 4. Blinkimpuls (lang)

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration //"AutoConnectAP", "password1234"

  if (!wifiManager.autoConnect(String("ZZe" + esp_chipid).c_str())) 
  {
    Serial.println("failed to connect and hit timeout");
    delay(5000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  // Buffer          "1   5    10   15  19"
  strcpy(SBuffer[2], "                    ");
  strcpy(SBuffer[3], " WIFI connected!    ");
  LCD_Refresh();

  delay(1000);

  //Tasks starten   

  //Task Tochteruhr stellen anlegen
  ts.add(2, 10, [&](void*) { TochterUhrStellen(); }, 0);
  ts.add(0, 10, [&](void*) { tick(); }, nullptr, false);
  ts.add(3, 1000, [&](void*) { TochterUhr(); }, nullptr, false);
  ts.add(4, 500, [&](void*) { Displayupdate(); }, nullptr, false);

  if(ts.enable(2)){
    Serial.println("TochterUhrStellen() = 2 enabled");
  }

  if(ts.enable(0))
  {
    Serial.println("tick() = 0 enabled");
  }

  if(ts.enable(3))
  {
    Serial.println("Task TochterUhr() = 3 enabled");
  }

  if(ts.enable(4))
  {
    Serial.println("Task Displayupdate() = 4 enabled");
  }

  //read updated parameters
  strcpy(ntpserver, custom_ntp_server.getValue());
  strcpy(mrc_multicast, custom_mrc_multicast.getValue());
  strcpy(mrc_port, custom_mrc_port.getValue());
  strcpy(c_clock_hour, custom_time_hour.getValue());
  strcpy(c_clock_minute, custom_time_minute.getValue());
  
  //save the custom parameters to FS
  if (shouldSaveConfig) 
  { 
    tochter_h = atoi(c_clock_hour);
    tochter_m = atoi(c_clock_minute);

    //Parameter überprüfen
    if(tochter_h < 0 || tochter_h > 23){
      tochter_h = 0;
    }
    if(tochter_h >= 12){
      tochter_h = tochter_h - 12;
    }
    if(tochter_m < 0 || tochter_h > 59){
      tochter_m = 0;
    }
    
    // Setze Sollzeit = Istzeit, sonst läuft die Uhr schon nach dem Setzen los auf irgendeine Zeit...
    clock_h = tochter_h;
    clock_m = tochter_m;

    Serial.println("Saving configuration to file!");

    if(DataSaving() != 0) {
      Serial.println("Fail to save configuration to file!");
    }
    
    Serial.print("Uhrzeit der Tochteruhr=");
    Serial.print(tochter_h);
    Serial.print(":");
    Serial.println(tochter_m);    
  }

  LED(LEDOff);           // LED aus machen

  Serial.print("local ip:");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.SSID());

  // Buffer          "1   5    10   15  19"
  strcpy(SBuffer[3], " WIFI connected!    ");
  sprintf(SBuffer[1], "slave clock:%02i:%02i   ", tochter_h, tochter_m);
  LCD_Refresh();

  //delay(1000);

  //saving credentials only on change
  //WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  
  // Starting WIFI in Station Mode
  Serial.println("Starting WIFI in Station Mode, Listening to UDP Multicast");
  //wifiUDPMRC;
  //wifiUDPNTP;

  WiFi.mode(WIFI_STA);

  WiFi.printDiag(Serial);

  wifiUDPMRC.beginMulticast(WiFi.localIP(), IPAddress(239,50,50,20), 2000);
  wifiUDPNTP.begin(localPort);

  Serial.print("Local port: ");
  Serial.println(wifiUDPNTP.localPort());
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  delay(5000);

  dCounterToNTP = WaittimeNTP1Sync;        // Wie lange nach dem Aufstarten warten, bis die NTP Zeit übernommen wird?

  LED(LEDBlinkOnce);                      // LED einen Flash blinken    -> 4. Blinkimpuls

  LED(LEDAlive);                          // Ab jetzt zeigt die LED das Leben des Empfängers an..

  // Buffer           "1   5    10   15  19"
  sprintf(SBuffer[0], "SSID: %14s", String(WiFi.SSID()).c_str());
  sprintf(SBuffer[1], "slave clock:%02i:%02i   ", tochter_h, tochter_m);
  sprintf(SBuffer[2], "model clock:%02i:%02i:%02i ", clock_h, clock_m, clock_s );
  if(clock_aktiv)
  {
     sprintf(SBuffer[3], "clock speed: 1:%s   ", c_speed);
  } else {
    // Buffer           "1   5    10   15  19"
    sprintf(SBuffer[3], "clock stopped!      ");
  }
  LCD_Refresh();
}

void loop() 
{
  // loop checks for:
  // - MRCLOCK Telegramm, or
  // - NTP 
  // MRCLOCK has priority to NTP telegramms, if there were no MRCLOCK Telegramm for WaittimeNTPSync [seconds] then, the 
  // Slaveclock is syncronised to NTP Clock until the first MRCLOCK telegramm comes in...
  
  // If a MRCLOCK Telegramm is recieved:
  if (int packetsize = wifiUDPMRC.parsePacket()) 
  {
    dCounterToNTP = WaittimeNTPSync;           // Counter updaten
    Serial.println("@.");
    flag_wrongIP = false;
    // We've received a packet, read the data from it
    wifiUDPMRC.read(packetBuffer, packetsize); // read the packet into the buffer

    //Search for Message like "clock=10:32:35"
    for(unsigned long i=0; i < sizeof(packetBuffer); i++)
    {
      if( packetBuffer[i] == 'c' &&
          packetBuffer[i+1] == 'l' &&
          packetBuffer[i+2] == 'o' &&
          packetBuffer[i+3] == 'c' &&
          packetBuffer[i+4] == 'k' &&
          packetBuffer[i+5] == '=')
      {
        c_time[0] = packetBuffer[i+6];
        c_time[1] = packetBuffer[i+7];
        c_time[2] = packetBuffer[i+8];
        c_time[3] = packetBuffer[i+9];
        c_time[4] = packetBuffer[i+10];
        c_time[5] = packetBuffer[i+11];
        c_time[6] = packetBuffer[i+12];
        c_time[7] = packetBuffer[i+13];
        c_time[8] = packetBuffer[i+14];
            
        // Time Char Array zerlegen
        bool clear_h = true;
        bool clear_m = false;
        bool clear_s = false;
            
        for(int ii=0; ii < 15; ii++)
        {
          if( !clear_h && !clear_m && clear_s)
          {                
            if( c_time[ii] == 0)
            {
              clear_m = false;
              clear_s = false;
              c_clock_hour[ii] = ' ';
              c_clock_minute[ii] = ' ';
              c_clock_sek[ii] = ' ';
            } else {
              c_clock_hour[ii] = ' ';
              c_clock_minute[ii] = ' ';
              c_clock_sek[ii] = c_time[ii];
            }
          }
          if( !clear_h && clear_m && !clear_s) 
          {                
            if( c_time[ii] == ':')
            {
              clear_m = false;
              clear_s = true;
              c_clock_hour[ii] = ' ';
              c_clock_minute[ii] = ' ';
              c_clock_sek[ii] = ' ';
            } else {
              c_clock_hour[ii] = ' ';
              c_clock_minute[ii] = c_time[ii];                  
              c_clock_sek[ii] = ' ';
            }
          }
          if( clear_h && !clear_m && !clear_s)
          {                
            if( c_time[ii] == ':')
            {
              clear_h = false;
              clear_m = true;
              c_clock_hour[ii] = ' ';
              c_clock_minute[ii] = ' ';
              c_clock_sek[ii] = ' ';
            } else {
              c_clock_hour[ii] = c_time[ii];
              c_clock_minute[ii] = ' ';
              c_clock_sek[ii] = ' ';
            }
          }
        }
  
        clock_h = atoi(c_clock_hour);
        clock_m = atoi(c_clock_minute);
        clock_s = atoi(c_clock_sek);

        Serial.print("Empfangenes Zeitzeichen:");
        Serial.print(clock_h);
        Serial.print(":");
        Serial.print(clock_m);
        Serial.print(":");
        Serial.println(clock_s);
      }
    }
     
    //Search for Message like "speed=5"
    for(unsigned long i=0; i < sizeof(packetBuffer); i++)
    {
      if( packetBuffer[i] == 's' &&
          packetBuffer[i+1] == 'p' &&
          packetBuffer[i+2] == 'e' &&
          packetBuffer[i+3] == 'e' &&
          packetBuffer[i+4] == 'd' &&
          packetBuffer[i+5] == '=')
      {
        if(packetBuffer[i+6] == '\n' || packetBuffer[i+6] == '\r')
        {
          c_speed[0] = '\0';
        } else c_speed[0] = packetBuffer[i+6];
        if(packetBuffer[i+7] == '\n' || packetBuffer[i+7] == '\r')
        {
          c_speed[1] = '\0';
        } else c_speed[1] = packetBuffer[i+7];
        if(packetBuffer[i+8] == '\n' || packetBuffer[i+8] == '\r')
        {
          c_speed[2] = '\0';
        } else c_speed[2] = packetBuffer[i+8];
        if(packetBuffer[i+9] == '\n' || packetBuffer[i+9] == '\r')
        {
          c_speed[3] = '\0';
        } else c_speed[3] = packetBuffer[i+9];
        
        c_speed[4] = '\0';

        Serial.print("Speed=");
        Serial.println(c_speed);

        Serial.print("Speed_int=");
        clock_speed = atoi(c_speed);
        Serial.println(clock_speed);
      }
    }

    //Search for Message like "active=yes|no"
    for(unsigned long i=0; i < sizeof(packetBuffer); i++)
    {
      if( packetBuffer[i] == 'a' &&
          packetBuffer[i+1] == 'c' &&
          packetBuffer[i+2] == 't' &&
          packetBuffer[i+3] == 'i' &&
          packetBuffer[i+4] == 'v' &&
          packetBuffer[i+5] == 'e' &&
          packetBuffer[i+6] == '=')
      {
        if( packetBuffer[i+7] == 'y' &&
            packetBuffer[i+8] == 'e' &&
            packetBuffer[i+9] == 's')
        {
          clock_aktiv = true;
          digitalWrite(AMPEL, LOW);  //Ampel auf Grün stellen
          itelecnt = TELECNTSTART;
        } else {
          digitalWrite(AMPEL, HIGH);  //Ampel auf Rot stellen
          clock_aktiv = false;
        }
          
        if(clock_aktiv)
        {
          Serial.println("Uhr läuft!");
        } else Serial.println("Uhr steht!");
      }
    }

    // recieving one message!
    Serial.println("; Ende der Message");
    flag_message_recieved = true;
  }

  //Stimmt die Anzeige auf der Ampel?
  if(clock_aktiv)
  {
    if(!itelecnt)
    {
      digitalWrite(AMPEL, HIGH);  //Ampel auf Rot stellen
      clock_aktiv = false;
    } else itelecnt--;
  }
  
  // Ist der Taster gedrückt?
  ConfigButton();

  //Soll auf NTP-Zeitzeichenempfang umgeschaltet werden?
  if(dCounterToNTP == 0)
  {
    //jetzt wird die NTP Zeit an der Tochteruhr angezeigt...
    if (timeStatus() != timeNotSet) 
    {
      if (now() != prevDisplay) 
      { 
        //update the display only if time has changed
        prevDisplay = now();

        local = CE.toLocal(now(), &tcr);
        
        Serial.print("Hour=");
        Serial.print(hour(local), DEC);
        Serial.print("Minute=");
        Serial.print(minute(local), DEC);
        Serial.print("Seconds=");
        Serial.println(second(local), DEC);
  
        clock_h = hour(local);
        clock_m = minute(local);
        clock_s = second(local);
      }
    }
  } else dCounterToNTP--;
  
  //Tasks bedienen
  yield();
  ts.update();
  Serial.print(".");
  delay(10);
}

// Unterfunktionen

//WifiManager
//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

// CALLBACK wird mit dem Starten der Configurationsseite aufgerufen und stellt die Uhr um zwei Ticks weiter, damit sie mit der internen Uhr synchron läuft
void APCallback(WiFiManager *myWiFiManager){

  String temp;
  String temp1="               ";
  
  // Buffer          "1   5    10   15  19"
  strcpy(SBuffer[2], "CaptivePortal runing");
  strcpy(SBuffer[3], "SSID:");

  temp = myWiFiManager->getConfigPortalSSID();
  strncat(SBuffer[3], temp1.c_str(), 15-strlen(temp.c_str())); 
  strncat(SBuffer[3], temp.c_str(), strlen(temp.c_str()));
  LCD_Refresh();
  
  // Some other things to do?
  WiFiManagerParameter custom_time_hour("clock_hour", "zulaessige Werte (00-11)", c_clock_hour, 15);
  WiFiManagerParameter custom_time_minute("clock_minute", "zulaessige Werte (00-59)", c_clock_minute, 15);
}

//Task zum weiterstellen der Tochteruhr
//Wird zyklisch aufgerufen und prüft, ob die empfangene Uhrzeit mit der angezeigten Uhrzeit übereinstimmt. 
//Stellt sie einen Unterschied fest, setzt sie das Flag "FlagTUStellen", dieses Flag wird durch die Task "TochterUhrStellen" 
//ausgewertet, bearbeitet und zurückgestellt.
//Task verwaltet die Uhrzeit der Tochteruhr.
void tick()
{  
  if( !FlagTUStellen )
  {
    // Zeitzeichen mit Tochteruhr vergleichen, die Tochteruhr ist eine 12h Uhr. Das Zeitzeichen kann auch im 24h gesendet werden.
    if( ( clock_h == tochter_h && clock_m == tochter_m ) || ( clock_h-12 == tochter_h && clock_m == tochter_m ) )
    {
      // Tochteruhr stimmt mit Zeitzeichen überein, nix machen
    } else {
      // Tochteruhr muss gestellt werden.
      tochter_m++;
      if (tochter_m == 60)
      { 
        // Stundensprung
        tochter_h++;
        tochter_m = 0;
        if (tochter_h == 12)
        { 
          // Tagesprung
          tochter_h = 0;
        }
      }
      FlagTUStellen = true;
      Serial.print("Tochteruhr stellt auf Zeit=");
      Serial.print(tochter_h);
      Serial.print(":");
      Serial.print(tochter_m);
      Serial.print(" Zeitzeichen=");
      Serial.print(clock_h);
      Serial.print(":");
      Serial.print(clock_m);
      Serial.print(":");
      Serial.println(clock_s);
    }
  }
}

// Unterfunktionen
// Schreibt Zeit der Tochteruhr auf die Serielle Konsole
void TochterUhr()
{
  Serial.print("Tochteruhr zeigt=");
  Serial.print(tochter_h);
  Serial.print(" : ");
  Serial.print(tochter_m);
  Serial.println(" ");

  return;
}

// Stellt das Stellglied der Tochteruhr auf die letzte Taktausgabe zur Synchronisation
void forcerevercetick(void)
{
  Serial.println("Letzte Taktausgabe beim Ausschalten wiederholen");
  if(!FlagTUStellglied){
    digitalWrite(TAKTA, LOW);
    digitalWrite(TAKTB, HIGH);
  }else{
    digitalWrite(TAKTA, HIGH);
    digitalWrite(TAKTB, LOW);
  }
  delay(500);
  if(FlagTUStellglied){
    digitalWrite(TAKTA, LOW);
    digitalWrite(TAKTB, LOW);
  }else{
    digitalWrite(TAKTA, HIGH);
    digitalWrite(TAKTB, HIGH);
  }
}

//Task gibt den Stelltakt an die Tochteruhr raus. Sobald das Flag "FlagTUStellen" gesetzt ist, wird ein Stellbefehl ausgegeben. 
//Das Flag wird nach der Ausgabe des Stellbefehls und dem Abschalten des Ausgangs wieder zurückgesetzt.
//Abarbeitung nach einer Statemaschine. Somit kann die Zeitdauer des Ausgangsimpuls kontrolliert werden.
//Sobald ein Takt ausgegeben werden soll, wird zuerst die Ladungspumpe eingeschaltet, nachher der Takt ausgeben, die 
//Ausgänge abgeschaltet und die Ladungspumpe wieder abgeschaltet, eher ein neuer Stelltakt ausgegeben werden kann.
//Beim FreZZe ist keine Ladungspumpe eingebaut, die Statemachine ist aber identisch!
void TochterUhrStellen()
{
  switch ( SM ) {
    case 1:
      // Warten auf FlagTUstellen
      if( FlagTUStellen )
      {
        SM = 2;
        Serial.print("FlagTUStellen=");
        Serial.print("true");
      }  
      break;
    case 2:
      // State Chargepump init
      SM = 3;
      SMC = CHARGEPUMPTIME;
      break;
    case 3:
      // Warten auf Spannung
      if( SMC == 0 )
      {
        SM = 4;
        SMC = TAKTTIME; 
        // Tick ausgeben, Ausgänge anschalten
        if(FlagTUStellglied)
        {
          digitalWrite(TAKTA, LOW);
          digitalWrite(TAKTB, HIGH);
          Serial.println("FlagTUStellglied = +");
          FlagTUStellglied = false;
        } else {
          digitalWrite(TAKTA, HIGH);
          digitalWrite(TAKTB, LOW);
          Serial.println("FlagTUStellglied = -");
          FlagTUStellglied = true;
        }
      } else {
        SMC--;
      }
      break;
    case 4:
      // Tick ausgeben
      if( SMC == 0 )
      {
        SM = 5;      
        //Tick zurückstellen
        if(!FlagTUStellglied)
        {
          digitalWrite(TAKTA, HIGH);
          digitalWrite(TAKTB, HIGH);
          Serial.println("FlagTUStellglied = +");
        } else {
          digitalWrite(TAKTA, LOW);
          digitalWrite(TAKTB, LOW);
          Serial.println("FlagTUStellglied = -");
        }
      } else {
        SMC--;
      }
      break;
    case 5:
      // ChargePump abschalten
      SM = 6 ;  
      SMC = TAKTWAITIME;
      break;
    case 6:
      // Warte State nach Stelltaktausgabe
      if( SMC == 0 ){
        SM = 1;
        SMC = 0;
        FlagTUStellen = false;
      }else{
        SMC--;
      }
      break;
    default:
      SM = 0;
  }

  /* ohne switch / case:
  if( SM < 7 && SM > 0)
  {
    if(SM == 1)
    {
      // Warten auf FlagTUstellen
      if( FlagTUStellen )
      {
        SM = 2;
        Serial.print("FlagTUStellen=");
        Serial.print("true");
      }   
    }
    if(SM == 2)
    {
      // State Chargepump init
      SM = 3;
      SMC = CHARGEPUMPTIME;
    }
    if(SM == 3)
    {
      // Warten auf Spannung
      if( SMC == 0 )
      {
        SM = 4;
        SMC = TAKTTIME; 
        // Tick ausgeben, Ausgänge anschalten
        if(FlagTUStellglied){
          digitalWrite(TAKTA, LOW);
          digitalWrite(TAKTB, HIGH);
          Serial.println("FlagTUStellglied = +");
          FlagTUStellglied = false;
        } else {
          digitalWrite(TAKTA, HIGH);
          digitalWrite(TAKTB, LOW);
          Serial.println("FlagTUStellglied = -");
          FlagTUStellglied = true;
        }
      } else {
        SMC--;
      }
    }
    if(SM == 4){
      // Tick ausgeben
      if( SMC == 0 ){
        SM = 5;      
        //Tick zurückstellen
        if(!FlagTUStellglied){
          digitalWrite(TAKTA, HIGH);
          digitalWrite(TAKTB, HIGH);
          Serial.println("FlagTUStellglied = +");
        }else{
          digitalWrite(TAKTA, LOW);
          digitalWrite(TAKTB, LOW);
          Serial.println("FlagTUStellglied = -");
        }
      }else{
        SMC--;
      }
    }
    if(SM == 5){
      // ChargePump abschalten
      SM = 6 ;  
      SMC = TAKTWAITIME;
    }
    if(SM == 6){
      // Warte State nach Stelltaktausgabe
      if( SMC == 0 ){
        SM = 1;
        SMC = 0;
        FlagTUStellen = false;
      }else{
        SMC--;
      }
    }
  } else { // Init Statemaschine
    SM = 1;
    SMC = 0;
  } */

}

//Routine Schreibt die Daten ins EEProm vom ESP
int DataSaving(void)
{
  //Serial.println("Start DataSaving t=");
  //Serial.println(system_get_rtc_time()-millisAfterISR);

  //Serial.println("Saving Data to File...");
  DynamicJsonDocument json(2048);

  //Serial.print("Time after json created: t="); 
  //Serial.println(system_get_rtc_time()-millisAfterISR);

  //read configuration from FS json
  //Serial.println("mounting FS...");

  /*
  // Falls das Filesystem noch nicht formatiert ist, wird es hier automatisch gemacht.
  if(LittleFS.begin()){
    Serial.println("FS ok!");
  } else {
    Serial.println("FS not formated, maybe your ESP8266 is damaged!");
    return -1;
  } */

  json["ntp_server"] = ntpserver;
  json["mrc_multicast"] = mrc_multicast;
  json["mrc_port"] = mrc_port;
  json["clock_hour"] = tochter_h;
  json["clock_minute"] = tochter_m;
  json["FlagTUStellglied"] = FlagTUStellglied;

  //Serial.print("Time after json filled: t="); 
  //Serial.println(system_get_rtc_time()-millisAfterISR);

  File configFile = LittleFS.open("/config.json", "w");
  if (!configFile) {
     Serial.println("failed to open config file for writing");
     return -2;
  }

  //Serial.print("Time after file opend: t="); 
  //Serial.println(system_get_rtc_time()-millisAfterISR);

  serializeJson(json, configFile);
  //Serial.print("Time after json written to file: t="); 
  //Serial.println(system_get_rtc_time()-millisAfterISR);

  configFile.close();
  //LittleFS.end();

  Serial.print("Time after file is closed: t="); 
  Serial.println(system_get_rtc_time()-millisAfterISR);
  
  serializeJson(json, Serial);
  Serial.println("");
  Serial.println("Data stored, File closed!");

  return 0;
}

/*
//Routine Schreibt die Daten ins EEProm vom ESP
int DataSaving2(void)
{
  //Serial.println("Saving Data to File...");
  //DynamicJsonDocument json(2048);

  //read configuration from FS json
  //Serial.println("mounting FS...");

  String dataStringToSave[2048] = {'"' + "ntp_server" + '"' + ':' + '"' + String(ntpserver) + '"' + "," + 
                             '"' + "mrc_multicast" + '"' + ":" +  String(mrc_multicast) + '"' + "," +
                             '"' + "mrc_port" + '"' + ":" + String(mrc_port) + '"' + "," +
                             '"' + "clock_hour" + '"' + ":" + String(tochter_h) + '"' + "," +
                             '"' + "clock_minute" + '"' + ":" + String(tochter_m) + '"' + "," + 
                             '"' + "FlagTUStellglied" + '"' + ":" + '"' + FlagTUStellglied ? "true" : "false" + '"'};

  File configFile = LittleFS.open("/config.json", "w");
  if (!configFile) {
     Serial.println("failed to open config file for writing");
     return -2;
  }

  configFile.write(dataStringToSave->toCharArray);
  confilFile.flush();
  configFile.close();
  //LittleFS.end();

  Serial.println(dataStringToSave);
  Serial.println("");
  Serial.println("Data stored, File closed!");

  return 0;
}

*/

//Routine Prüft den Config Button
void ConfigButton(void)
{
  if(digitalRead(CONFIGPB) == LOW)
  {
    if(FlagButtonPresed)
    {
      if(CounterButtonPressed == 0)
      {
        ts.remove(0);
        while(digitalRead(CONFIGPB) == LOW)
        {
          yield();
          ts.update();
          Serial.print(".");
          delay(100);
          if(digitalRead(BLED) == HIGH)
          {
            digitalWrite(BLED, LOW); 
          } else {
            digitalWrite(BLED, HIGH);
          }
        }

        Serial.println("erasing");
        Serial.printf("LittleFS.remove = %d", LittleFS.remove("/config.json"));
        Serial.print(".");

        WiFi.persistent(true);
        
        delay(1000);
        WiFi.setAutoReconnect(false);
        delay(100);
        WiFi.disconnect();
        delay(100);
        wifiManager.resetSettings();
        ESP.eraseConfig();
        delay(100);

        Serial.print("..ESP Reset..");
        delay(1000);
        ESP.reset();

        delay(5000);
      } else CounterButtonPressed--;
    } else {
      FlagButtonPresed = true;
      CounterButtonPressed = 300;
      Serial.println("BP");

      LEDStatus = LEDOn;       //Blaue LED anschalten
    }
  } else {
    // Taste wird vor dem Ablaufen des CounterButtonPressed losgelassen, dann nur Daten ins Flash sichern
    if(FlagButtonPresed){
      FlagButtonPresed = false;
      CounterButtonPressed = 0;

      Serial.println("Saving Data to File...");

      if(DataSaving() != 0) {
        Serial.println("Fail to save configuration to file!");
      }
      
      ts.add(0, 10, [&](void *) { tick(); }, nullptr, false);

      if(ts.enable(0))
      {
        Serial.println("tick() = 0 enabled");
      }

      LEDStatus = LEDAlive;
    }
  }
}

// Löscht den Bildschirmspeicher
void LCD_ClearBuffer(void)
{
  // Buffer          "1   5    10   15  19"
  strcpy(SBuffer[0], "                    ");
  strcpy(SBuffer[1], "                    ");
  strcpy(SBuffer[2], "                    ");
  strcpy(SBuffer[3], "                    ");
}

// Initalitation des Bildschirms 
void LCD_Init(void)
{
  // Added some delays for funktion!
  delay(100);
  checkbusyflag();
  LCD_Command(0x3A);    //8 bit data length, extension BIT RE=1; REV=0
  delay(100);
  checkbusyflag();
  LCD_Command(0x09);    //4 line display
  delay(100);
  checkbusyflag();
  LCD_Command(0x06);    //Bottom View
  delay(100);
  checkbusyflag();
  LCD_Command(0x1E);    //BS=1
  delay(100);
  checkbusyflag();
  LCD_Command(0x39);    //8 bit data length extension Bit RE=0, IS=1
  delay(100);
  checkbusyflag();
  LCD_Command(0x1B);    //BS0=1 -> Bias=1/6
  delay(100);
  checkbusyflag();
  LCD_Command(0x6E);    //Devider on and set value
  delay(100);
  LCD_Command(0x57);    //Booster on and set contrast (DB1=C5, DB0=C4)
  delay(100);
  LCD_Command(0x72);    //Set contrast (DB3-DB0 = C3-C0)
  delay(100);  
  LCD_Command(0x38);    //8 bit data length extension Bit RE=0; IS=0
  delay(100);
  checkbusyflag();
  LCD_Command(0x0C);    //Display on, cursor on, blink on
  checkbusyflag();
  delay(100);
  LCD_Command(0x3A);    //8-Bit data length extension Bit RE=1;
  checkbusyflag();
  delay(100);
  LCD_2Byte_Command(0x72, 0x0C);  //Select Rom C Caracter Table
  checkbusyflag();
  delay(100);
  LCD_Command(0x38);    //8-Bit data length extension Bit RE=0;
  checkbusyflag();
  //LCD_Clear();
  checkbusyflag();
  delay(2000);
  LCD_Refresh();
  Serial.println("LCD_Refresh()");
  checkbusyflag();
  delay(2000);
}

// Überträgt ein Byte auf dem I2C Bus zum Display
int LCD_Data(int data)
{
  Wire.beginTransmission(I2C_ADRESS); // transmit to device #44 (0x2c)
                                      // device address is specified in datasheet
  Wire.write(0x40);                   // sends value byte
  Wire.write(data);
  return Wire.endTransmission();      // stop transmitting  
}

// Routine schreibt den Standartinhalt des Displays in den Bildschirmspeicher
void Displayupdate(void)
{
  //Buffer            "1   5    10   15  19"
  sprintf(SBuffer[0], "SSID: %14s", String(WiFi.SSID()).c_str());
  sprintf(SBuffer[1], "slave clock:%02i:%02i   ", tochter_h, tochter_m);

  //in der 3. Zeile entweder Modellzeit oder Realzeit angeben
  if(dCounterToNTP == 0)
  {
    //Display beschreiben
    //Buffer            "1   5    10   15  19"
    sprintf(SBuffer[2], "real  clock:%02i:%02i:%02i ", clock_h, clock_m, clock_s );
    // Buffer           "1   5    10   15  19"
    sprintf(SBuffer[3], "NTP Time on clock   ");
  } else 
  {
    //Display beschreiben
    //Buffer            "1   5    10   15  19"
    sprintf(SBuffer[2], "model clock:%02i:%02i:%02i ", clock_h, clock_m, clock_s );
    if(clock_aktiv)
    {
      // Buffer           "1   5    10   15  19"
      sprintf(SBuffer[3], "clock speed: 1:%s       ", c_speed);
    } else {
      // Buffer           "1   5    10   15  19"
      sprintf(SBuffer[3], "clock stopped!      ");
    }
  }
  
  LCD_Refresh();
}

// Routine schreibt den Bildschirmspeicher ins Display
void LCD_Refresh(void)
{

  // erste Zeile
  Wire.beginTransmission(I2C_ADRESS); // transmit to device #44 (0x2c)
                                // device address is specified in datasheet
  Wire.write(0x80);             // sends value byte
  Wire.write(0x80);             // RAM-adress 00h
  Wire.write(0x40);             // sends value byte
  Wire.write(SBuffer[0],20);
  Wire.endTransmission();       // stop transmitting 

  // zweite Zeile
  Wire.beginTransmission(I2C_ADRESS); // transmit to device #44 (0x2c)
                                // device address is specified in datasheet
  Wire.write(0x80);             // sends value byte
  Wire.write(0xA0);
  Wire.write(0x40);             // sends value byte
  Wire.write(SBuffer[1],20);
  Wire.endTransmission();       // stop transmitting 

  // dritte Zeile
  Wire.beginTransmission(I2C_ADRESS); // transmit to device #44 (0x2c)
                                // device address is specified in datasheet
  Wire.write(0x80);             // sends value byte
  Wire.write(0xC0);
  Wire.write(0x40);             // sends value byte
  Wire.write(SBuffer[2],20);
  Wire.endTransmission();       // stop transmitting 
  
  // vierte Zeile
  Wire.beginTransmission(I2C_ADRESS); // transmit to device #44 (0x2c)
                                // device address is specified in datasheet
  Wire.write(0x80);             // sends value byte
  Wire.write(0xE0);             // 4. Zeile adressieren
  Wire.write(0x40);             // jetzt folgen Daten
  Wire.write(SBuffer[3],20); 
  Wire.endTransmission();       // stop transmitting 

}

// Übertragung eines Kommandos ans Display (1 Byte)
int LCD_Command(int command)
{
  Wire.beginTransmission(I2C_ADRESS); // transmit to device #44 (0x2c)
                                      // device address is specified in datasheet
  Wire.write(0x00);                   // sends value byte
  Wire.write(command);
  return Wire.endTransmission();      // stop transmitting  
}

// Übertragung eines Kommandos ans Display (2 Byte)
int LCD_2Byte_Command(int command1, int command2)
{
  Wire.beginTransmission(I2C_ADRESS); // transmit to device #44 (0x2c)
                                      // device address is specified in datasheet
  Wire.write(0x80);                   // sends value byte
  Wire.write(command1 | 0x00);        // continuation bit setting!
  Wire.write(command2); 
  return Wire.endTransmission();      // stop transmitting  
}

// Routine liest das Busyflag vom Display zurück
bool checkbusyflag(void)
{
  byte readdata[10];
  int i = 0;

  Wire.requestFrom(I2C_ADRESS+1, 2);    // request 2 bytes from slave device #2

  while(Wire.available() && i < 10)
  {
    readdata[i++] = Wire.read();
  }

  //Serial.print("Busyflag =");
  if(readdata[0] &= 0x80) 
  {
    Serial.println("true");
    return true;
  } else {
    Serial.println("false");
    return false;
  } 
}

//Routine bedient das LED auf dem ESP6288 Modul
void LEDTS(void){
  if(LEDStatusIntern != LEDStatus){           //LED Status hat sich verändert, alle Counter reseten
    LEDStatusCounter = 0;
    LEDStatusIntern = LEDStatus;
    if(LEDStatusIntern == LEDAusVorbereitet) LEDPeriodCounter = LEDAusVorbperiod;
  }

  //Je nach State LEDStatusIntern andere Anzeigen auswählen
  switch (LEDStatusIntern) 
  {
    case LEDOff:
      digitalWrite(BLED, HIGH); 
      break;
    case LEDOn:
      digitalWrite(BLED, LOW); 
      break;
    case LEDBlinking:
      if(!LEDStatusCounter)
      {
        LEDStatusCounter = LEDblinkingcycle;
        if(digitalRead(BLED)) digitalWrite(BLED, LOW);
        else digitalWrite(BLED, HIGH);
      }
      else LEDStatusCounter--;
      break;
    case LEDAlive:
      if(!digitalRead(BLED))
      {     
        //LED ist ein
        if(!LEDStatusCounter)
        {     
          LEDStatusCounter = LEDalivecycle50;
          digitalWrite(BLED, HIGH); //LED ausschalten
        }
        else LEDStatusCounter--; 
      } else {                      //LED ist aus
        if(!LEDStatusCounter){
          LEDStatusCounter = LEDalivetime;
          digitalWrite(BLED, LOW); //LED einschalten
        }
        else LEDStatusCounter--;
      }
      break;
      case LEDBlinkOnce:
        digitalWrite(BLED, LOW);
        LEDStatus = LEDOff;  
        break;
      case LEDAusVorbereitet:
        if(!LEDStatusCounter)
        {
          LEDStatusCounter = LEDAusVorbcycle;
          if(digitalRead(BLED)) digitalWrite(BLED, LOW);
          else digitalWrite(BLED, HIGH);
        }
        else LEDStatusCounter--;
        if(!LEDPeriodCounter)
        {
          LEDStatus = LEDOff;
        }
        else LEDPeriodCounter--;
        break;
    }
  return;
}

// Interruptroutine schreibt die aktuellen Werte ins EEPROM 
// Wird durch den ISR aufgerufen, sobald die Spannung ausfällt bzw. beim ausschalten
ICACHE_RAM_ATTR void ISRSaveData(void)
{
  millisAfterISR = system_get_rtc_time();
  //Serial.println("ISRSaveData();");
  //detachInterrupt(digitalPinToInterrupt(INT));
  ts.disableAll();
  //WiFi.mode(WIFI_OFF);
  //Serial.println("Time after ts.disableAll(); =");
  //Serial.println(system_get_rtc_time()-millisAfterISR);

  //Serial.println("Interrupt to save data!");
  if(DataSaving() != 0) {
    Serial.println("Fail to save configuration to file!");
  }
  digitalWrite(BLED, HIGH);
  //Serial.println("Data Saved....");
  //Serial.println("Time after datasaving =");
  //Serial.println(system_get_rtc_time()-millisAfterISR);
   
  while(true){
    Serial.print("I");
  }
}

// Spezialanzeigeroutine für die LED wenn der Ticker noch nicht läuft, also im Setup
void LED(int LEDstatus){
  LEDStatus = LEDstatus;       // LED Status setzen dauert 1 sec.
  
  yield();
  ts.update();
  delay(10);
  yield();
  ts.update();
  delay(10);
}

// Schreibt die aktuelle NTP Zeit auf die Serielle Konsole.
void printTime(time_t t)
{
  sPrintI00(hour(t));
  sPrintDigits(minute(t));
  sPrintDigits(second(t));
  Serial.print(' ');
  Serial.print(dayShortStr(weekday(t)));
  Serial.print(' ');
  sPrintI00(day(t));
  Serial.print(' ');
  Serial.print(monthShortStr(month(t)));
  Serial.print(' ');
  Serial.print(year(t));
  Serial.println(' ');
}

//Print an integer in "00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void sPrintI00(int val)
{
  if (val < 10) Serial.print('0');
  Serial.print(val, DEC);
  return;
}

//Print an integer in ":00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void sPrintDigits(int val)
{
  Serial.print(':');
  if (val < 10) Serial.print('0');
  Serial.print(val, DEC);
}


/*-------- NTP code ----------*/

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBufferNTP, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBufferNTP[0] = 0b11100011;   // LI, Version, Mode
  packetBufferNTP[1] = 0;     // Stratum, or type of clock
  packetBufferNTP[2] = 6;     // Polling Interval
  packetBufferNTP[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBufferNTP[12] = 49;
  packetBufferNTP[13] = 0x4E;
  packetBufferNTP[14] = 49;
  packetBufferNTP[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  wifiUDPNTP.beginPacket(address, 123); //NTP requests are to port 123
  wifiUDPNTP.write(packetBufferNTP, NTP_PACKET_SIZE);
  wifiUDPNTP.endPacket();
}

//Routine holt die NTP Zeit vom NTP Zeitserver
time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (wifiUDPNTP.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpserver, ntpServerIP);
  Serial.print(ntpserver);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = wifiUDPNTP.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      wifiUDPNTP.read(packetBufferNTP, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBufferNTP[40] << 24;
      secsSince1900 |= (unsigned long)packetBufferNTP[41] << 16;
      secsSince1900 |= (unsigned long)packetBufferNTP[42] << 8;
      secsSince1900 |= (unsigned long)packetBufferNTP[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}