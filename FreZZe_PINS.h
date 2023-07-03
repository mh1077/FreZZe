// FreZZe_PINS.h
// Definitionen für die Pins

#define TAKTA            12       //SPI MISO = GPIO #12 (TaktA)
#define TAKTB            13       //GPIO #13  (TaktB)
#define CONFIGPB         16       //GPIO #16 (Input für Config Pushbutton) Bei MRC_REC = 5!
#define BLED              2       //GPIO #2 = blue LED on ESP
#define INT              14       //GPIO #14 = Interrupt für Spannungseinbruch -> Interrupt wird an der fallenden Flanke ausgelöst
#define DISP_RESET       15       //GPIO #15 Reset des Displays
#define AMPEL            0        //GPIO #0 Ampel grün/rot
