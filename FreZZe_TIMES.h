// FreZZe_TIMES.h
// Definitionen für Zeiten etc...

#define TAKTTIME                100     //Wie lange wird der Stelltakt ausgegeben? 1 Einheit entspricht 10 ms Taktausgabezeit
#define TAKTWAITIME             100     //So lange darf kein Takt ausgegeben werden     
#define CHARGEPUMPTIME          2       //So lange wird auf die Chargepump gewartet

#define WaittimeNTPSync         1440000 //Zeit in ms bis von MRCLOCK Telegrammempfang auf NTP umgeschalten wird. 1440000 entspricht 24h
#define WaittimeNTP1Sync        36000  
#define WaittimeIPSenderChanged 12      //Wartezeit 12 Telegramme

//defines für den LED Status
#define LEDOff            0       //LED is off
#define LEDBlinking       1       //Flashing LED (Saving Time to Flash)
#define LEDAlive          2       //TUE is alive (Blinktakt abhängig vom Ladezustand der Batterie)
#define LEDConfig         5       //TUE is in Config mode
#define LEDOn             6       //LED is on
#define LEDBlinkOnce      7       //LED macht einen Blinker
#define LEDAusVorbereitet 8       //LED macht langsamen Blinkrythmus

#define LEDalivetime      1
#define LEDalivecycle50   1000    //LED alive = jede 10s ein kurzer blauer blitz
#define LEDalivecycle25   1500    //LED alive = jede 15s ein kurzer blauer blitz
#define LEDalivecycle10   2000    //LED alive = jede 20s ein kurzer blauer blitz
#define LEDblinkingcycle  5
#define LEDAusVorbtime    2
#define LEDAusVorbcycle   5
#define LEDAusVorbperiod  1000