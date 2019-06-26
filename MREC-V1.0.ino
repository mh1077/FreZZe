//MREC V1.0 mit EA-Display DOGM204 I2C
//Läuft ab LP V0.3
//Software erstellt 02. September 2017

#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <TickerScheduler.h>      //Ticker

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

// Communication for the Display
#include <Wire.h>

#include <String>

//Defines for Pins

#define TAKTA            12       //SPI MISO = GPIO #12 (TaktA)
#define TAKTB            13       //GPIO #13  (TaktB)
#define CONFIGPB         16       //GPIO #16 (Input für Config Pushbutton) Bei MRC_REC = 5!
#define BLED              2       //GPIO #2 = blue LED on ESP
#define INT              14       //GPIO #14 = Interrupt für Spannungseinbruch -> Interrupt wird an der fallenden Flanke ausgelöst
#define DISP_RESET       15       //GPIO #15 Reset des Displays
#define AMPEL            0        //GPIO #0 Ampel grün/rot

#define TAKTTIME          100     //Wie lange wird der Stelltakt ausgegeben? 1 Einheit entspricht 10 ms Taktausgabezeit
#define TAKTWAITIME       100     //So lange darf kein Takt ausgegeben werden     
#define CHARGEPUMPTIME    2       //So lange wird auf die Chargepump gewartet

#define DISPLAYKONTRAST   10      //nach Test ein guter Wert

#define I2C_ADRESS 0x3D

//define your default values here, if there are different values in config.json, they are overwritten.
char ntp_server[40] = "ntp.metas.ch";
char ntp_port[6] = "123";
char mrc_multicast[40] = "239.50.50.20";
char mrc_port[6] = "2000";

//char array for the protokollhandler 
char c_clock_hour[15] = "03";
char c_clock_minute[15] = "02";
char c_clock_sek[15] = "04";
char c_time[15] = "0";
char c_speed[15] = "0      ";

//Displaybuffer
char SBuffer[4][80] = {{ "11111111111111111111" },{ "22222222222222222222" },{ "33333333333333333333" },{ "44444444444444444444" }};

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

//Statemaschine für TochterUhrStellen Routine
int  SM = 0;        // Statemaschine Status
int  SMC = 0;       // Statemaschine Counter
bool FlagTUStellen;          // Flag ist gesetzt, falls die Tochteruhr einen Tick machen soll
bool FlagTUStellglied;       // Flag wiederspiegelt den Ausgang des TU Stellglieds

//Rücksetztaste
bool FlagButtonPresed = false;
int  CounterButtonPressed;

//Tickerscheduler Objekt

TickerScheduler ts(5);

//flag for saving data
bool shouldSaveConfig = false;

//flag for sleep mode 
bool sleep = false;
bool flag_message_recieved = true;

//flag for clock ticks
bool bClockBad = true; //true on startup, until one falid Clockmessage war recieved

//WifiUDP Class
WiFiUDP wifiUDP;

const int MRC_PACKET_SIZE = 2024; // NTP time stamp is in the first 48 bytes of the message

char packetBuffer[ MRC_PACKET_SIZE ]; //buffer to hold incoming and outgoing packets

int nLength = 0;

int time_hour = 0;

int time_minute = 0;

//TEMP Variable for Display
char ctempd[200];
String stempd;

//Strings 
String esp_chipid;

//WiFiManager
//Das Object wird auch noch im LOOP benötigt um die Config zu löschen.... 
WiFiManager wifiManager;

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
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting up!");

  //Variablen initialisieren
  FlagTUStellen = false;  //FlagTUStellen initialisieren
  SM = 1;
  SMC = 0;                //Statemaschine für die Taktausgabe initialisieren
  
  delay(500);

  pinMode(DISP_RESET, OUTPUT);
  digitalWrite(DISP_RESET, LOW);
  delay(500);
  digitalWrite(DISP_RESET, HIGH);
  delay(500);
  
  //ChipID?
  esp_chipid = String(ESP.getChipId());
  
  //I2C initialisieren, Display init
  Wire.begin();

  Wire.setClock(200000L);
  
  LCD_Init();

  delay(10);

  //LCD_ClearBuffer();

  // Buffer          "1   5    10   15  19"
  strcpy(SBuffer[0], "FREMO MRCLOCK       ");
  strcpy(SBuffer[1], "Reciever        V1.0"); 
  strcpy(SBuffer[2], "                    ");
  strcpy(SBuffer[3], "INIT.               ");

  LCD_Refresh();

  delay(1000);

  //WiFi.setAutoReconnect(false);
  WiFi.persistent(true); // Wird gebraucht damit das Flash nicht zu oft beschrieben wird.
  WiFi.setAutoReconnect(true);
  
  //Pins konfigurieren
  //#define TAKTA            12       //SPI MISO = GPIO #12 (TaktA)
  //#define TAKTB            14       //GPIO #4  (TaktB)
  //#define CONFIGPB         16       //GPIO #16 (Input für Config Pushbutton) Bei MRC_REC = 5!
  //#define BLED              2       //GPIO #2 = blue LED on ESP
  //#define AMPEL            10       //GPIO #10 Ampel grün/rot

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

  digitalWrite(BLED,  HIGH);

  //clean FS, for testing
  //SPIFFS.format();
  //SPIFFS.remove("/config.json");
  //SPIFFS.remove("/config2.json");
  
  //read configuration from FS json
  Serial.println("mounting FS...");

  // Falls das Filesystem noch nicht formatiert ist, wird es hier automatisch gemacht.
  if(SPIFFS.begin()){
    Serial.println("FS ok!");
  } else {
    SPIFFS.format();
    Serial.println("FS formated");
  }

  //Taktleitungen als Ausgang initialisieren
  pinMode(TAKTA, OUTPUT);
  pinMode(TAKTB, OUTPUT);

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          strcpy(ntp_server, json["ntp_server"]);
          strcpy(ntp_port, json["ntp_port"]);
          strcpy(mrc_multicast, json["mrc_multicast"]);
          strcpy(mrc_port, json["mrc_port"]);
          strcpy(c_clock_hour, json["clock_hour"]);
          strcpy(c_clock_minute, json["clock_minute"]);
          
          if(json["FlagTUStellglied"] == "true"){
            FlagTUStellglied = true;
            digitalWrite(TAKTA, LOW);
            digitalWrite(TAKTB, LOW);
            Serial.print("FlagTUStellglied = true aus Config");
          }else{
            FlagTUStellglied = false;
            digitalWrite(TAKTA, HIGH);
            digitalWrite(TAKTB, HIGH); 
            Serial.print("FlagTUStellglied = false aus Config");
          }
          forcerevercetick(); //funktioniert nicht, muss ausprobiert werden, warum!
        } 
      }
    } else {
      //Config File exsistiert nicht!
      Serial.println("No Config!");
      FlagTUStellglied = true;
      digitalWrite(TAKTA, LOW);
      digitalWrite(TAKTB, LOW);
      Serial.print("FlagTUStellglied = true, Annahme!");
      Serial.println("failed to load json config");
      forcerevercetick(); //funktioniert nicht, muss ausprobiert werden, warum!
    }
  } else {
    Serial.println("failed to mount FS");
    FlagTUStellglied = true;
    digitalWrite(TAKTA, LOW);
    digitalWrite(TAKTB, LOW);
    Serial.print("FlagTUStellglied = true, Annahme!");
    Serial.println("failed to load json config");
    forcerevercetick(); //funktioniert nicht, muss ausprobiert werden, warum!
  }
  //end read

  //Interne Zeit mit der Tochteruhrzeit gleich setzen, sonst läuft uns die Uhr nach einem Neustart los...
  tochter_h = atoi(c_clock_hour);
  tochter_m = atoi(c_clock_minute);
  
  clock_h = tochter_h;
  clock_m = tochter_m;

  // Buffer          "1   5    10   15  19"
  strcpy(SBuffer[3], "INIT..              ");
  LCD_Refresh();

  delay(100);

  sprintf(c_clock_hour, "%i", tochter_h);
  sprintf(c_clock_minute, "%i", tochter_m);
  
  Serial.print("Tochteruhr zeigt ");
  Serial.print(tochter_h);
  Serial.print(":");
  Serial.print(tochter_m);
  Serial.println(" an. Sie sollte synchronisiert sein");

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  // WiFiManagerParameter custom_ntp_server("ntp_server ntp1.metas.ch", "ntp server", ntp_server, 40);
  // WiFiManagerParameter custom_ntp_port("ntp_port (123)", "ntp_port", ntp_port, 5);
  // WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 32);
  WiFiManagerParameter custom_text_info1("<br><b>FREMO-Zeitzeichenempf&auml;nger FreZZe</b><br>Einstellmen&uuml;<br>");
  WiFiManagerParameter custom_text_info2("Empf&auml;ngt Zeitzeichentelegramme nach dem MRC Protokoll.<br><br>");
  WiFiManagerParameter custom_text_info3("Standarteinstellung MRC: Multicast 239.50.50.20 auf Port 2000.<br>Bitte das WIFI Netz w&auml;len und die Uhrzeit der angeschlossenen Nebenuhr eingeben.<br>");
  WiFiManagerParameter custom_text_h("Stand der Nebenuhr, hier Stunden (ganze Zahl von 0 bis 11):");
  WiFiManagerParameter custom_text_m("Stand der Nebenuhr, hier Minuten (ganze Zahl von 0 bis 59):");
  WiFiManagerParameter custom_mrc_multicast("mrc_multicast", "mrc_multicast (239.50.50.20)", mrc_multicast, 40);
  WiFiManagerParameter custom_mrc_port("mrc_port", "mrc_multicast_port (2000)", mrc_port, 5);
  WiFiManagerParameter custom_time_hour("clock_hour", "zul&auml;ssige Werte (00-11)", c_clock_hour, 15);
  WiFiManagerParameter custom_time_minute("clock_minute", "zul&auml;ssige Werte (00-59)", c_clock_minute, 15);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set APCallback
  wifiManager.setAPCallback(APCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  //add all your parameters here
  //  wifiManager.addParameter(&custom_ntp_server);
  //  wifiManager.addParameter(&custom_ntp_port);
  wifiManager.addParameter(&custom_text_info1);
  wifiManager.addParameter(&custom_text_info2);
  wifiManager.addParameter(&custom_text_info3);
  wifiManager.addParameter(&custom_mrc_multicast);
  wifiManager.addParameter(&custom_mrc_port);
  wifiManager.addParameter(&custom_text_h);
  wifiManager.addParameter(&custom_time_hour);
  wifiManager.addParameter(&custom_text_m);
  wifiManager.addParameter(&custom_time_minute);

  //reset settings - durch das löschen der Daten, haben wir die Möglichkeit die Uhrzeit bei jedem Start neu einstellen zu können
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration //"AutoConnectAP", "password1234"
  if (!wifiManager.autoConnect(String("ZZe" + esp_chipid).c_str())) {
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

  //read updated parameters
  // strcpy(ntp_server, custom_ntp_server.getValue());
  // strcpy(ntp_port, custom_ntp_port.getValue());
  strcpy(mrc_multicast, custom_mrc_multicast.getValue());
  strcpy(mrc_port, custom_mrc_port.getValue());
  strcpy(c_clock_hour, custom_time_hour.getValue());
  strcpy(c_clock_minute, custom_time_minute.getValue());
  
  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");

    DataSaving();
    
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
    
    Serial.print("Uhrzeit der Tochteruhr=");
    Serial.print(tochter_h);
    Serial.print(":");
    Serial.println(tochter_m);    
  }

  Serial.print("local ip:");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.SSID());

  // Buffer          "1   5    10   15  19"
  strcpy(SBuffer[3], " WIFI connected!    ");
  sprintf(SBuffer[1], "slave clock:%02i:%02i   ", tochter_h, tochter_m);
  LCD_Refresh();

  delay(1000);

  //saving credentials only on change
  //WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  
  // Starting WIFI in Station Mode
  Serial.println("Starting WIFI in Station Mode, Listening to UDP Multicast");
  wifiUDP;
  
  WiFi.mode(WIFI_STA);

  WiFi.printDiag(Serial);

  wifiUDP.beginMulticast(WiFi.localIP(), IPAddress(239,50,50,20), 2000);

  //stempd = WiFi.localIP().toString();
  
  // Buffer            "1   5    10   15  19"
  //sprintf(SBuffer[23], "IP: %15s", WiFi.localIP().toString().c_str());
  //LCD_Refresh();

  //delay(5000);
  
  ts.add(0, 10, tick);
  ts.add(1, 60000, UBat);
  ts.add(2, 10, TochterUhrStellen);
  ts.add(3, 1000, TochterUhr);
  ts.add(4, 500, Displayupdate);

  if(ts.enable(0)){
    Serial.println("tick() = 0 enabled");
  }

  if(ts.enable(2)){
    Serial.println("TochterUhrStellen() = 2 enabled");
  }

  if(ts.enable(3)){
    Serial.println("Task TochterUhr() = 3 enabled");
  }

  if(ts.enable(4)){
    Serial.println("Task Displayupdate() = 4 enabled");
  }

  // Buffer           "1   5    10   15  19"
  sprintf(SBuffer[0], "SSID: %14s", String(WiFi.SSID()).c_str());
  sprintf(SBuffer[1], "slave clock:%02i:%02i   ", tochter_h, tochter_m);
  sprintf(SBuffer[2], "model clock:%02i:%02i:%02i ", clock_h, clock_m, clock_s );
  if(clock_aktiv){
     sprintf(SBuffer[3], "clock speed: 1:%s   ", c_speed);
  }
  else{
    // Buffer           "1   5    10   15  19"
    sprintf(SBuffer[3], "clock stopped!      ");
  }
  LCD_Refresh();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (int packetsize = wifiUDP.parsePacket()) {
    Serial.println("@.");
    // We've received a packet, read the data from it
    wifiUDP.read(packetBuffer, packetsize); // read the packet into the buffer

    //Search for Message like "clock=10:32:35"
    for(int i=0; i < sizeof(packetBuffer); i++){
       if( packetBuffer[i] == 'c' &&
           packetBuffer[i+1] == 'l' &&
           packetBuffer[i+2] == 'o' &&
           packetBuffer[i+3] == 'c' &&
           packetBuffer[i+4] == 'k' &&
           packetBuffer[i+5] == '='){
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
            
            for(int ii=0; ii < 15; ii++){
              if( !clear_h && !clear_m && clear_s){                
                if( c_time[ii] == 0){
                  clear_m = false;
                  clear_s = false;
                  c_clock_hour[ii] = ' ';
                  c_clock_minute[ii] = ' ';
                  c_clock_sek[ii] = ' ';
                }else{
                  c_clock_hour[ii] = ' ';
                  c_clock_minute[ii] = ' ';
                  c_clock_sek[ii] = c_time[ii];
                }
              }
              if( !clear_h && clear_m && !clear_s){                
                if( c_time[ii] == ':'){
                  clear_m = false;
                  clear_s = true;
                  c_clock_hour[ii] = ' ';
                  c_clock_minute[ii] = ' ';
                  c_clock_sek[ii] = ' ';
                }else{
                  c_clock_hour[ii] = ' ';
                  c_clock_minute[ii] = c_time[ii];                  
                  c_clock_sek[ii] = ' ';
                }
              }
              if( clear_h && !clear_m && !clear_s){                
                if( c_time[ii] == ':'){
                  clear_h = false;
                  clear_m = true;
                  c_clock_hour[ii] = ' ';
                  c_clock_minute[ii] = ' ';
                  c_clock_sek[ii] = ' ';
                }else{
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
     for(int i=0; i < sizeof(packetBuffer); i++){
       if( packetBuffer[i] == 's' &&
           packetBuffer[i+1] == 'p' &&
           packetBuffer[i+2] == 'e' &&
           packetBuffer[i+3] == 'e' &&
           packetBuffer[i+4] == 'd' &&
           packetBuffer[i+5] == '='){
            if(packetBuffer[i+6] == '\n' || packetBuffer[i+6] == '\r'){
              c_speed[0] = '\0';
            }else c_speed[0] = packetBuffer[i+6];
            if(packetBuffer[i+7] == '\n' || packetBuffer[i+7] == '\r'){
              c_speed[1] = '\0';
            }else c_speed[1] = packetBuffer[i+7];
            if(packetBuffer[i+8] == '\n' || packetBuffer[i+8] == '\r'){
              c_speed[2] = '\0';
            }else c_speed[2] = packetBuffer[i+8];
            if(packetBuffer[i+9] == '\n' || packetBuffer[i+9] == '\r'){
              c_speed[3] = '\0';
            }else c_speed[3] = packetBuffer[i+9];
            c_speed[4] == '\0';

            Serial.println("Speed=");
            Serial.println(c_speed);
  
            clock_speed = atoi(c_speed);
            
            Serial.println(clock_speed);
         }
      }

      //Search for Message like "active=yes|no"
     for(int i=0; i < sizeof(packetBuffer); i++){
       if( packetBuffer[i] == 'a' &&
           packetBuffer[i+1] == 'c' &&
           packetBuffer[i+2] == 't' &&
           packetBuffer[i+3] == 'i' &&
           packetBuffer[i+4] == 'v' &&
           packetBuffer[i+5] == 'e' &&
           packetBuffer[i+6] == '=')
           {
          if(packetBuffer[i+7] == 'y' &&
             packetBuffer[i+8] == 'e' &&
             packetBuffer[i+9] == 's'){
              clock_aktiv = true;
              digitalWrite(AMPEL, LOW);  //Ampel auf Grün stellen
              itelecnt = TELECNTSTART;
             }
          else{
            digitalWrite(AMPEL, HIGH);  //Ampel auf Rot stellen
            clock_aktiv = false;
          }
          
          if(clock_aktiv) Serial.println("Uhr läuft!");
          else Serial.println("Uhr steht!");
          }
      }

      // recieving one message!
      Serial.println("; Ende der Message");
      flag_message_recieved = true;
  }

  //Stimmt die Anzeige auf der Ampel?
  if(clock_aktiv){
    if(!itelecnt){
      digitalWrite(AMPEL, HIGH);  //Ampel auf Rot stellen
      clock_aktiv = false;
    }else itelecnt--;
  }
  
  // Ist der Taster gedrückt?
  ConfigButton();
  
  //Tasks bedienen
  yield();
  ts.update();
  Serial.print(".");
  delay(10);
}

void * tick(){  
  if( !FlagTUStellen ){
    // Zeitzeichen mit Tochteruhr vergleichen, die Tochteruhr ist eine 12h Uhr. Das Zeitzeichen kann auch im 24h gesendet werden.
    if( ( clock_h == tochter_h && clock_m == tochter_m ) || ( clock_h-12 == tochter_h && clock_m == tochter_m ) ){
      // Tochteruhr stimmt mit Zeitzeichen überein, nix machen
    } else {
      // Tochteruhr muss gestellt werden.
      tochter_m++;
      if (tochter_m == 60){ // Stundensprung
        tochter_h++;
        tochter_m = 0;
        if (tochter_h == 12){ // Tagesprung
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

void * UBat(){
  Serial.println("UBat");

  int level = analogRead(A0);

  Serial.print("Level = ");
  Serial.println(level);
}

void * TochterUhr(){
  Serial.print("Tochteruhr zeigt=");
  Serial.print(tochter_h);
  Serial.print(" : ");
  Serial.print(tochter_m);
  Serial.println(" ");
}

void forcerevercetick(void){
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

void * TochterUhrStellen(){
  if( SM < 7 && SM > 0){
    if(SM == 1){
      // Warten auf FlagTUstellen
      if( FlagTUStellen ){
        SM = 2;
        Serial.print("FlagTUStellen=");
        Serial.print("true");
      }   
    }
    if(SM == 2){
      // State Chargepump init
      SM = 3;
      SMC = CHARGEPUMPTIME;
    }
    if(SM == 3){
      // Warten auf Spannung
      if( SMC == 0 ){
        SM = 4;
        SMC = TAKTTIME; 
        // Tick ausgeben, Ausgänge anschalten
        if(FlagTUStellglied){
          digitalWrite(TAKTA, LOW);
          digitalWrite(TAKTB, HIGH);
          Serial.println("FlagTUStellglied = +");
          FlagTUStellglied = false;
        }else{
          digitalWrite(TAKTA, HIGH);
          digitalWrite(TAKTB, LOW);
          Serial.println("FlagTUStellglied = -");
          FlagTUStellglied = true;
        }
      }else{
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
  } 
}

void DataSaving(void){
  Serial.println("Saving Data to File...");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["ntp_server"] = ntp_server;
  json["ntp_port"] = ntp_port;
  json["mrc_multicast"] = mrc_multicast;
  json["mrc_port"] = mrc_port;
  json["clock_hour"] = tochter_h;
  json["clock_minute"] = tochter_m;
  json["FlagTUStellglied"] = FlagTUStellglied;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
     Serial.println("failed to open config file for writing");
  }

  json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();
}

void ConfigButton(void){
  if(digitalRead(CONFIGPB) == LOW){
    if(FlagButtonPresed){
      if(CounterButtonPressed == 0){
        ts.remove(0);
        while(digitalRead(CONFIGPB) == LOW){
          yield();
          ts.update();
          Serial.print(".");
          delay(100);
          if(digitalRead(BLED) == HIGH){
            digitalWrite(BLED, LOW); 
          }else{
            digitalWrite(BLED, HIGH);
          }
        }

        Serial.println("erasing");
        Serial.printf("SPIFFS.remove = %d", SPIFFS.remove("/config.json"));
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
        //ESP.restart();
        delay(5000);
      } else CounterButtonPressed--;
    }else{
      FlagButtonPresed = true;
      CounterButtonPressed = 300;
      Serial.println("BP");

      digitalWrite(BLED, LOW); //Blaue LED anschalten
    }
  }else{
    // Taste wird vor dem Ablaufen des CounterButtonPressed losgelassen, dann nur Daten ins Flash sichern
    if(FlagButtonPresed){
      FlagButtonPresed = false;
      CounterButtonPressed = 0;
      digitalWrite(BLED,  HIGH);

      Serial.println("Saving Data to File...");

      DataSaving();
      
      ts.add(0, 10, tick);
      ts.enable(0);
    }
  }
}

void LCD_ClearBuffer(void){
  // Buffer          "1   5    10   15  19"
  strcpy(SBuffer[0], "                    ");
  strcpy(SBuffer[1], "                    ");
  strcpy(SBuffer[2], "                    ");
  strcpy(SBuffer[3], "                    ");
}

void LCD_Init(void){
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

int LCD_Data(int data){
  Wire.beginTransmission(I2C_ADRESS); // transmit to device #44 (0x2c)
                                      // device address is specified in datasheet
  Wire.write(0x40);                   // sends value byte
  Wire.write(data);
  return Wire.endTransmission();      // stop transmitting  
}

void * Displayupdate(void){

  //Display beschreiben
  //Buffer            "1   5    10   15  19"
  sprintf(SBuffer[0], "SSID: %14s", String(WiFi.SSID()).c_str());
  sprintf(SBuffer[1], "slave clock:%02i:%02i   ", tochter_h, tochter_m);
  sprintf(SBuffer[2], "model clock:%02i:%02i:%02i ", clock_h, clock_m, clock_s );
  if(clock_aktiv){
     // Buffer           "1   5    10   15  19"
     sprintf(SBuffer[3], "clock speed: 1:%s       ", c_speed);
  }
  else{
    // Buffer           "1   5    10   15  19"
    sprintf(SBuffer[3], "clock stopped!      ");
  }
  
  LCD_Refresh();
}

void LCD_Refresh(void){

  //Display Inhalt löschen
  //Wire.beginTransmission(I2C_ADRESS); // transmit to device #44 (0x2c)
                                // device address is specified in datasheet
  //Wire.write(0x80);             // sends value byte
  //Wire.write(0x01);             // Clear Display
  //Wire.write(0x00);             // sends value byte
  //Wire.write(0x02);             // Return Home
  //Wire.endTransmission();       // stop transmitting  

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

  //Serial.print("LCD_Refresh() finished! ");
  //Serial.print("Displayinhalt =");
  //Serial.print(SBuffer[0]);
  //Serial.print(SBuffer[1]);
  //Serial.print(SBuffer[2]);
  //Serial.println(SBuffer[3]);
}

int LCD_Command(int command){
  Wire.beginTransmission(I2C_ADRESS); // transmit to device #44 (0x2c)
                                      // device address is specified in datasheet
  Wire.write(0x00);                   // sends value byte
  Wire.write(command);
  return Wire.endTransmission();      // stop transmitting  
}

int LCD_2Byte_Command(int command1, int command2){
  Wire.beginTransmission(I2C_ADRESS); // transmit to device #44 (0x2c)
                                      // device address is specified in datasheet
  Wire.write(0x80);                   // sends value byte
  Wire.write(command1 | 0x00);        // continuation bit setting!
  Wire.write(command2); 
  return Wire.endTransmission();      // stop transmitting  
}

bool checkbusyflag(void){
  byte readdata[10];
  int i = 0;
  char temp[100];

  Wire.requestFrom(I2C_ADRESS+1, 2);    // request 2 bytes from slave device #2

  while(Wire.available() && i < 10){
    readdata[i++] = Wire.read();
  }

  //Serial.print("Busyflag =");
  if(readdata[0] &= 0x80) Serial.println("true");
  else //Serial.println("false");
  
  //sprintf(temp, "Bytes: %02X %02X", readdata[0], readdata[1]);
  
  //Serial.println(temp);

  if(readdata[0] &= 0x80) return true;
  else return false;
}

void ISRSaveData(void){
  Serial.println("Interrupt to save data!");
  DataSaving();
  digitalWrite(BLED, HIGH);
  Serial.println("Data Saved");
  detachInterrupt(INT);
  ESP.deepSleep(10);
  delay(100);
}
