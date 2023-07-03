// Header für TUE.cpp
// Hörlein, 17.11.21 - Neuerstellung

#ifndef FREZZE_H
#define FREZZE_H

//Routinen Intern
//callback notifying us of the need to save config
void saveConfigCallback();

// CALLBACK wird mit dem Starten der Configurationsseite aufgerufen und stellt die Uhr um zwei Ticks weiter, damit sie mit der internen Uhr synchron läuft
void APCallback(WiFiManager *myWiFiManager);

//Task zum weiterstellen der Tochteruhr
void tick();

//Task zum gezwungenen Weiterstellen der Tochteruhr
void forcetick();

// Schreibt Zeit der Tochteruhr auf die Serielle Konsole
void TochterUhr();

// Stellt das Stellglied der Tochteruhr auf die letzte Taktausgabe zur Synchronisation
void forcerevercetick(void);

//Task gibt den Stelltakt an die Tochteruhr raus.
void TochterUhrStellen();

//Routine Schreibt die Daten ins EEProm vom ESP
int DataSaving(void);

//Routine Prüft den Config Button
void ConfigButton(void);

// Löscht den Bildschirmspeicher
void LCD_ClearBuffer(void);

// Initalitation des Bildschirms 
void LCD_Init(void);

// Überträgt ein Byte auf dem I2C Bus zum Display
int LCD_Data(int data);

// Routine schreibt den Standartinhalt des Displays in den Bildschirmspeicher
void Displayupdate(void);

// Routine schreibt den Bildschirmspeicher ins Display
void LCD_Refresh(void);

// Übertragung eines Kommandos ans Display (1 Byte)
int LCD_Command(int command);

// Übertragung eines Kommandos ans Display (2 Byte)
int LCD_2Byte_Command(int command1, int command2);

// Routine liest das Busyflag vom Display zurück
bool checkbusyflag(void);

//Routine bedient das LED auf dem ESP6288 Modul
void LEDTS(void);

// Interruptroutine schreibt die aktuellen Werte ins EEPROM 
void ISRSaveData(void);

// Spezialanzeigeroutine für die LED
void LED(int);

// Schreibt die aktuelle NTP Zeit auf die Serielle Konsole.
void printTime(time_t);

//Print an integer in "00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void sPrintI00(int);

//Print an integer in ":00" format (with leading zero).
//Input value assumed to be between 0 and 99.
void sPrintDigits(int);

//Routine holt die NTP Zeit vom NTP Zeitserver
time_t getNtpTime(void);

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress *);

#endif