// ATMEL ATMEGA1284P
//
//                       +---\/---+
//   INT_NET (D 0) PB0  1|        |40  PA0 (AI 0 / D24) Voltage
//    SS_NET (D 1) PB1  2|        |39  PA1 (AI 1 / D25) A IN 1
//   INT_RFM (D 2) PB2  3|        |38  PA2 (AI 2 / D26) A IN 2
//    AC_OFF (D 3) PB3  4|        |37  PA3 (AI 3 / D27) A IN 3
// SS_RFM/SS (D 4) PB4  5|        |36  PA4 (AI 4 / D28) A IN 4
//      MOSI (D 5) PB5  6|        |35  PA5 (AI 5 / D29) A IN 5
//      MISO (D 6) PB6  7|        |34  PA6 (AI 6 / D30) A IN 6
//       SCK (D 7) PB7  8|        |33  PA7 (AI 7 / D31) A IN 7
//                 RST  9|        |32  AREF
//                 VCC 10|        |31  GND 
//                 GND 11|        |30  AVCC
//               XTAL2 12|        |29  PC7 (D 23) D IN 1
//               XTAL1 13|        |28  PC6 (D 22) D IN 2
//      RX0 (D 8)  PD0 14|        |27  PC5 (D 21) D IN 3
//      TX0 (D 9)  PD1 15|        |26  PC4 (D 20) D IN 4
//      RX1 (D 10) PD2 16|        |25  PC3 (D 19) D IN 5
//      TX1 (D 11) PD3 17|        |24  PC2 (D 18) BAT_OK
//      ED1 (D 12) PD4 18|        |23  PC1 (D 17) SDA
//   TAMPER (D 13) PD5 19|        |22  PC0 (D 16) SCL
//     OUT1 (D 14) PD6 20|        |21  PD7 (D 15) OUT2
//                       +--------+
//
#include <NilRTOS.h>

#include <DigitalIO.h>

#include "SPI.h"

#include "Ethernet.h"
#define WEBDUINO_FAVICON_DATA "" // diable favicon
#define WEBDUINO_SERIAL_DEBUGGING 0
#include "WebServer.h"
// CHANGE THIS TO YOUR OWN UNIQUE VALUE
static uint8_t mac[6] = { 0x02, 0xAA, 0xBB, 0xCC, 0x00, 0x22 };
// CHANGE THIS TO MATCH YOUR HOST NETWORK
static uint8_t ip[4] = { 10, 10, 10, 184 }; // area 51!
#define PREFIX ""
WebServer webserver(PREFIX, 80);
#include "html.h"
// no-cost stream operator as described at 
// http://sundial.org/arduino/?page_id=119
template<class T>
inline Print &operator <<(Print &obj, T arg)
{ obj.print(arg); return obj; }

//#define PINCHG_IRQ 1
#include <RFM12B.h>
// You will need to initialize the radio by telling it what ID it has and what network it's on
// The NodeID takes values from 1-127, 0 is reserved for sending broadcast messages (send to all nodes)
// The Network ID takes values from 0-255
// By default the SPI-SS line used is D10 on Atmega328. You can change it by calling .SetCS(pin) where pin can be {8,9,10}
#define NODEID    1 //network ID used for this unit
#define NETWORKID 1 //the network ID we are on
#define FREQUENCY RF12_868MHZ
#define ACK_TIME  50
//encryption is OPTIONAL
//to enable encryption you will need to:
// - provide a 16-byte encryption KEY (same on all nodes that talk encrypted)
// - to call .Encrypt(KEY) to start encrypting
// - to stop encrypting call .Encrypt(NULL)
uint8_t KEY[] = "ABCDABCDABCDABCD";
// Need an instance of the Radio Module
RFM12B radio;

// RTOS friendly Analog
#include <NilAnalog.h>

// TWI
#include <TwiMaster.h>
#include <Eeprom24C512.h>
#define EEPROM_ADDRESS 0x50
#define EEPROM_MESSAGE 16
#define EEPROM_PAGE_SIZE 128
static Eeprom24C512 eeprom(EEPROM_ADDRESS, EEPROM_PAGE_SIZE);
// TWI RTC
#include <RTClib.h>
RTC_DS1307 RTC;
DateTime time_now, time_started;

// RS 485
#include <NilRS485.h> 
RS485_msg RX_msg, TX_msg;

// Global configuration for in chip EEPROM
#include <avr/eeprom.h>

// Use tiny unbuffered NilRTOS NilSerial library.
//#include <NilSerial.h>

#include <NilGSM.h>
#define Serial GSM                     // redefine GSM as standard Serial 0

// FIFO
#include <NilFIFO.h>
// Type for a data record.
struct Record_t {
  char text[17];
};
NilFIFO<Record_t, 3> fifo;

//
#include <WebSerial.h>

// ADC Alarm settings refer to voltage divider and input voltage level
// values set for resistors 10k Tamper, 22k PIR
#define ALR_OK_LOW     100
#define ALR_OK         150
#define ALR_OK_HI      200
#define ALR_PIR_LOW    -50
#define ALR_PIR        0
#define ALR_PIR_HI     50
#define ALR_TAMP_LOW   -260
#define ALR_TAMP_HI    -160
#define ALR_ZONES      12        // Active zones
#define NUM_OF_KEYS    8         // max number of keys
#define KEY_LEN        8         // key 
#define NUM_OF_PHONES  8         // max number of phones
#define PHONE_LEN      16        // phone name 
#define ALR_GROUPS     16        // Groups
#define ALR_AUTH_UNITS 14        // 

struct alarm_event_t {
  uint8_t zone;
  char    type;
};
NilFIFO<alarm_event_t, 5> alarm_fifo;

struct zone_t {
  systime_t last_PIR = 0;
};
zone_t zone[ALR_ZONES];

struct group_t {  
  uint8_t set;
  uint8_t arm_delay;
};

group_t group[ALR_GROUPS];


/*
typedef struct  {  
  uint8_t address;
  char    function[];
} units_t;

#define UNITS_COUNT (sizeof(units)/sizeof(units_t))  // this will not work

extern units_t units[] = { 0, 'M' };
*/


// Global configuration in chip EEPROM
#define VERSION 100
struct config_t {
  uint16_t version;
  uint8_t  alr_time;
  uint16_t ee_pos;
  uint16_t zone[ALR_ZONES];
  uint8_t  arm_delay;
  char     tel_num[NUM_OF_PHONES][PHONE_LEN];  
  char     zone_name[ALR_ZONES][16];    
  char     key[NUM_OF_KEYS][KEY_LEN+1];
  char     key_name[NUM_OF_KEYS][16];
  char     tel_name[NUM_OF_PHONES][PHONE_LEN];
  uint16_t SMS;
  uint8_t  global_tel_num; 
  uint16_t group[ALR_GROUPS];
  char     group_name[ALR_GROUPS][16];
  uint8_t  tel[NUM_OF_PHONES];
  uint8_t  auth[ALR_AUTH_UNITS];
} conf;

// Global variables
volatile uint32_t idleCount;                  // temporary free ticks
volatile uint8_t  ACState = 0;
volatile uint16_t BatteryLevel = 460;         // 13.8V
volatile uint8_t  OUTs = 0;                   // Output pins

char tmp[17];       // for logger 
char _tmp[4];       // for logger 

// GSM modem 
uint8_t GSMisAlive = 0, GSMreg = 4, GSMstrength = 0;

// tmp
uint8_t n = 0; 


// Pins setting
DigitalPin<3>  pinAC_OFF(INPUT); // The signal turns to be "High" when the power supply turns OFF
DigitalPin<12> pinDE1(OUTPUT);
DigitalPin<13> pinTAMPER(INPUT);
DigitalPin<14> pinOUT1(OUTPUT);
DigitalPin<15> pinOUT2(OUTPUT);
DigitalPin<18> pinBAT_OK(INPUT); // The signal is "Low" when the voltage of battery is under 11V 
DigitalPin<23> pinIN1(INPUT);
DigitalPin<22> pinIN2(INPUT);
DigitalPin<21> pinIN3(INPUT);
DigitalPin<20> pinIN4(INPUT);

// Declare and initialize a semaphore for limiting access to a region.
SEMAPHORE_DECL(ADCSem, 1);   // one slot only
SEMAPHORE_DECL(TWISem, 1);   // one slot only
SEMAPHORE_DECL(GSMSem, 1);   // one slot only

SEMAPHORE_DECL(AUTHSem, 0);  // Authorization

//SEMAPHORE_DECL(ARMSem, 0);   // Armed 


// *********************************************************************************
// F U N C T I O N S                F U N C T I O N S              F U N C T I O N S      
// *********************************************************************************

// Put string into a fifo log
void fifoPut(char* what){ 
  
  Record_t* p = fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
  if (p == 0) return; // Continue if no free space.
  
  nilSemWait(&TWISem);     // wait for slot
  time_now = RTC.now();
  nilSemSignal(&TWISem);   // Exit region.
  
  p->text[0] = 0;
  strcat (p->text, time_now.timestamp());
  strcat (p->text, "?");
  strcat (p->text, what);

  //Serial.println(p->text);
  fifo.signalData();  // Signal idle thread data is available.
}

// Send a command to wired RS485 unit
uint8_t sendCmd(uint8_t unit, uint8_t cmd){ 
  TX_msg.address = unit;
  TX_msg.ctrl = FLAG_CMD;
  TX_msg.data_length = cmd;
  return RS485.msg_write(&TX_msg);
}

// *********************************************************************************
// W E B   P A G E S              W E B   P A G E S                W E B   P A G E S  
// *********************************************************************************

/*
Binary sketch size: 42132 bytes (of a 130048 byte maximum, 32.40 percent).
Estimated memory use: 7350 bytes (of a 16384 byte maximum, 44.86 percent).
...
Binary sketch size: 43408 bytes (of a 130048 byte maximum, 33.38 percent).
Estimated memory use: 5886 bytes (of a 16384 byte maximum, 35.93 percent).
*/

void webDebug(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (type == WebServer::POST) {
    // no post
  } else {  
    uint8_t data;
    server.httpSuccess();
    server.printP(htmlHead);
    
    WS.resetRead(); // show all in buffer
    server.printP(html_h1); server.printP(text_Debug); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_pre);
    while (WS.isAvailable()) {
      data = WS.read();
      server << (char)data;
    }
    server.printP(html_e_pre); server.printP(html_e_p);
    server.printP(htmlFoot);
  }
}

void webHome(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (type == WebServer::POST) {
    // no post
  } else {   
    int16_t val = 0;
    uint32_t voltage = (BatteryLevel * 3222581) / 1071429; // voltage divider, and avoiding float
    char _tmp_itoa[5];
    itoa(voltage, _tmp_itoa, 10);

    nilSemWait(&TWISem);     // wait for slot
    time_now = RTC.now();
    nilSemSignal(&TWISem);   // Exit region.

    server.httpSuccess();
    server.printP(htmlHead);
    
    
    server.printP(html_h1); server.printP(text_System); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_table); 
    server.printP(html_tr); server.printP(html_td);
    server.printP(text_Time); server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp);
    server << time_now.day(); server.printP(text_dot); server << time_now.month(); server.printP(text_dot); server << time_now.year(); server.printP(text_space);
    server << time_now.hour(); server.printP(text_semic); server << time_now.minute(); server.printP(text_semic); server << time_now.second(); server.printP(text_space);
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td);
    server.printP(text_Started); server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp);
    server << time_started.day(); server.printP(text_dot); server << time_started.month(); server.printP(text_dot); server << time_started.year(); server.printP(text_space);
    server << time_started.hour(); server.printP(text_semic); server << time_started.minute(); server.printP(text_semic); server << time_started.second(); server.printP(text_space);
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td);
    server.printP(text_Uptime); server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp);
    server << (time_now.get()-time_started.get()); server.printP(text_space); server.printP(text_sec);
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td);
    server.printP(text_PwrSp); server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp); pinAC_OFF.read() ? server.printP(text_Off) : server.printP(text_On); server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td);
    server.printP(text_Battery); server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp); pinBAT_OK.read() ? server.printP(text_OK) : server.printP(text_low); server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td);
    server << "Voltage"; server.printP(html_e_td); server.printP(html_td); 
    server.printP(text_sesp); server << _tmp_itoa[0] << _tmp_itoa[1] << "." << _tmp_itoa[2] << _tmp_itoa[3];
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td);
    server << "Armed"; server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp); 
    //***********************
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td);
    server << "Authorization"; server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp); server << nilSemGetCounter(&AUTHSem); server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_e_table);
    
    server.printP(html_h1); server.printP(text_GSM); server.printP(text_space); server.printP(text_modem); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_td);
    server.printP(text_GSM); server.printP(text_space); server.printP(text_modem); server.printP(text_space); server.printP(text_is);
    server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server << (GSMisAlive ? "OK" : "Not OK"); server.printP(html_e_td); server.printP(html_e_tr);
    
    server.printP(html_tr); server.printP(html_td);
    server.printP(text_GSM); server.printP(text_space); server.printP(text_network); server.printP(text_space); server.printP(text_is);
    server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    switch(GSMreg){
      case 0 : server.printP(text_nr); break;
      case 1 : server.printP(text_rh); break;
      case 2 : server.printP(text_nrs); break;
      case 3 : server.printP(text_rd); break;
      // case 4 : server.printP(text_unk); break;
      case 5 : server.printP(text_rr); break;
      default : server.printP(text_unk);; break;
    }
    server.printP(html_e_td); server.printP(html_e_tr);

    server.printP(html_tr); server.printP(html_td);
    server.printP(text_SS); server.printP(text_space);
    server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server << GSMstrength*3 << "%";
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_e_table); server.printP(html_e_p);

    server.printP(htmlFoot);
  }
}

uint16_t ses_eeprom_add = 0;
void webListLog(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (type == WebServer::POST) {
    bool repeat;
    char name[2], value[16];
    do {
      repeat = server.readPOSTparam(name, 2, value, 16);
      if (name[0] == 'p') ses_eeprom_add -= 320;
      if (name[0] == 'f') ses_eeprom_add += 320;
      if (name[0] == 'n') ses_eeprom_add = conf.ee_pos - 336 ;
    } while (repeat);
    server.httpSeeOther(PREFIX "/log");
  } else {  
    server.httpSuccess();
    server.printP(htmlHead);
    server.printP(html_h1); server.printP(text_Log); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_th); server.printP(text_hash);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Date); 
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Message);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_SMS);
    server.printP(html_e_th); server.printP(html_e_tr);
    for (uint8_t i = 0; i < 21; ++i){
      if ( (i & 0x01) == 0) server.printP(html_tr);
      else                  server.printP(html_tr_ev);
      server.printP(html_td);
      // Get data from extrenal EEPROM
      nilSemWait(&TWISem);     // wait for slot
      eeprom.readBytes((uint16_t)(ses_eeprom_add + (i*16)),EEPROM_MESSAGE, tmp);
      //    nilThdSleepMilliseconds(10); //??? wait for eeprom
      nilSemSignal(&TWISem);   // Exit region.
      if ((uint16_t)(ses_eeprom_add/16 + i) > 4095) 
           server << (uint16_t)(ses_eeprom_add/16 + i - 4096 + 1); 
      else server << (uint16_t)(ses_eeprom_add/16 + i + 1);
      server.printP(text_dot);
      server.printP(html_e_td); server.printP(html_td);
      // print date and time
      server << tmp[4] << tmp[5]; server.printP(text_dot);
      server << tmp[2] << tmp[3]; server.printP(text_dot);
      server << "20" << tmp[0] << tmp[1]; server.printP(text_space);
      server << tmp[6] << tmp[7]; server.printP(text_semic);
      server << tmp[8] << tmp[9]; server.printP(text_semic);
      server << tmp[10] << tmp[11]; server.printP(text_space);
      server.printP(html_e_td); server.printP(html_td);
      switch(tmp[13]){
        case 'S':
          server.printP(text_System); server.printP(text_sesp);
          switch(tmp[14]){
            case 'B': // Battery
              server.printP(text_BR);
              if (tmp[15] == 'L') server.printP(text_low);
              else server.printP(text_high);
              server.printP(text_space); server.printP(text_level);
            break;
            case 'A': // AC
              server.printP(text_PWS);
              if (tmp[15] == 'L') server.printP(text_On);
              else server.printP(text_Off);
            break;
            case 'C': server.printP(text_CS); break; // conf. saved
            case 'Z': server.printP(text_armed); server.printP(text_space); server.printP(text_group);
                      server.printP(text_space); server << conf.group_name[tmp[15]-48];
                      break; // system armed
            case 'S': server.printP(text_MS); break; // monitoring strted
            case 'X': server.printP(text_ALARM); break;  // alarm
            default:  server.printP(text_undefined); break; // unknown
          }
        break;
        case 'P': // PIR
          server.printP(text_Alarm); server.printP(text_sesp); server.printP(text_triggered); server.printP(text_space); server.printP(text_zone); server.printP(text_space);
          server << conf.zone_name[tmp[14]-48];
        break;
        case 'T': // Tamper
          server.printP(text_Tamper); server.printP(text_sesp); server.printP(text_triggered); server.printP(text_space); server.printP(text_zone); server.printP(text_space);
          server << conf.zone_name[tmp[14]-48];
        break;
        case 'D': // Damaged
          server.printP(text_Damaged); server.printP(text_sesp); server.printP(text_triggered); server.printP(text_space); server.printP(text_zone); server.printP(text_space);
          server << conf.zone_name[tmp[14]-48];
        break;
        case 'A': // Authentication
          server.printP(text_Authentication); server.printP(text_sesp); server.printP(text_key); server.printP(text_space); 
          if (tmp[14] == 'A' || tmp[14] == 'D' || tmp[14] == 'F') { server << conf.key_name[tmp[15]-48]; server.printP(text_space); }
          switch(tmp[14]){
            case 'D': server.printP(text_disarmed); break;
            case 'A': server.printP(text_armed); break;
            case 'U': server.printP(text_undefined); break;
            case 'F': server.printP(text_is); server.printP(text_space); server.printP(text_disabled); break;
            case 'a': server.printP(text_auto); server.printP(text_space); server.printP(text_armed); break;
            default: break;
          }
          /*
          if (tmp[15] != 'F') { // print no info when unknown key
            server.printP(text_space);  server.printP(text_group); server.printP(text_space); server.print(conf.group_name[tmp[16]-48]);
          }
          */
        break;    
        case 'G': //GSM modem
          server.printP(text_GSM); server.printP(text_space); server.printP(text_network); server.printP(text_sesp);
          switch(tmp[14]){
            case '0' : server.printP(text_nr); break;
            case '1' : server.printP(text_rh); break;
            case '2' : server.printP(text_nrs); break;
            case '3' : server.printP(text_rd); break;
            // case 4 : server.printP(text_unk); break;
            case '5' : server.printP(text_rr); break;
            default : server.printP(text_unk);; break;
          }
          server << ", strength " << (tmp[15]-48)*3 << "%";
        break;
        case 'U': // remote unit
          server.printP(text_Remote); server.printP(text_space); server.printP(text_unit); server.printP(text_sesp);
          server << tmp[15]; server.printP(text_space);
          switch(tmp[14]){
            case 'F' : server.printP(text_is); server.printP(text_space); server.printP(text_disabled); break;
            default : server.printP(text_unk);; break;
          }
        break;
        default:
          server.printP(text_Undefined); server.printP(text_sesp);
          server << tmp;
        break;
      }
      server.printP(text_dot);
      server.printP(html_e_td); server.printP(html_td);
      switch(tmp[12]){
        case '.': server.printP(text_MRF); break;
        case ',': server.printP(text_MNRF); break;
        case ';': server.printP(text_MNRA); break;
        case '|': server.printP(text_MNC); break;
        default:  server.printP(text_MOK); break;
      }
      server.printP(html_e_td); server.printP(html_e_tr);
    }
    server.printP(html_e_table);
    server.printP(html_form_s); server << PREFIX "/log"; server.printP(html_form_e);
    server.printP(html_F_LOG); // buttons << NOW >>
    server.printP(html_e_form); server.printP(html_e_p);
    server.printP(htmlFoot);
  }
}

uint8_t webZone = 0;
void webSetZone(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  int16_t val = 0;
  if (type == WebServer::POST) {
    bool repeat;
    char name[2], value[16];
    do {
      repeat = server.readPOSTparam(name, 2, value, 16);
      //Serial.print(name);
      //Serial.print("-");
      //Serial.println(value);
      //y = (x >> n) & 1;    // n=0..15.  stores nth bit of x in y.  y becomes 0 or 1.
      //x &= ~(1 << n);      // forces nth bit of x to be 0.  all other bits left alone.
      //x &= (1<<(n+1))-1;   // leaves alone the lowest n bits of x; all higher bits set to 0.
      //x |= (1 << n);       // forces nth bit of x to be 1.  all other bits left alone.
      //x ^= (1 << n);       // toggles nth bit of x.  all other bits left alone.
      //x = ~x;              // toggles ALL the bits in x.
      switch(name[0]){
        case 'Z':
          n = strtol(value, NULL, 10);
          if (n != webZone) {
            webZone = n;
            repeat = 0;
          }
        break;
        case 'n': // zone name
          strncpy (conf.zone_name[webZone], value, sizeof(value));
          conf.zone_name[webZone][15] = 0;
        break;
        case 'o': // enable zone
          if (value[0] == '0') conf.zone[webZone] &= ~(1 << 0);
          else conf.zone[webZone] |= (1 << 0);
        break;
        case 'a': // auto arm
          if (value[0] == '0') conf.zone[webZone] &= ~(1 << 7);
          else conf.zone[webZone] |= (1 << 7);
        break;
        case 'd': // auth. delay
          n = value[0] - 48;
          if ((n >> 0) & 1) conf.zone[webZone] |= (1 << 5);
          else conf.zone[webZone] &= ~(1 << 5);
          if ((n >> 1) & 1) conf.zone[webZone] |= (1 << 6);
          else conf.zone[webZone] &= ~(1 << 6);
        break;
        case 'g': // group
          n = strtol(value, NULL, 10);
          if ((n >> 0) & 1) conf.zone[webZone] |= (1 << 1);
          else conf.zone[webZone] &= ~(1 << 1);
          if ((n >> 1) & 1) conf.zone[webZone] |= (1 << 2);
          else conf.zone[webZone] &= ~(1 << 2);
          if ((n >> 2) & 1) conf.zone[webZone] |= (1 << 3);
          else conf.zone[webZone] &= ~(1 << 3);
          if ((n >> 3) & 1) conf.zone[webZone] |= (1 << 4);
          else conf.zone[webZone] &= ~(1 << 4);
        break;
         case 'e': // save to EEPROM
          eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration
          _tmp[0] = 'S'; _tmp[1] = 'C'; _tmp[2] = 'W'; _tmp[3] = 0; fifoPut(_tmp);
        break;          
      }
    } while (repeat);
    server.httpSeeOther(PREFIX "/zone");
  } else {
    server.httpSuccess();
    server.printP(htmlHead);
    server.printP(html_h1); server.printP(text_Zone); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1); server.printP(html_p);
    server.printP(html_form_s); server << PREFIX "/zone"; server.printP(html_form_e);

    server.printP(html_table);
    server.printP(html_tr); server.printP(html_th); server.printP(text_hash);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Type);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Name); 
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Enabled);
    server.printP(html_e_th); server.printP(html_th); server << "Auto arm";
    server.printP(html_e_th); server.printP(html_th); server << "Delay";
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Group);
    server.printP(html_e_th); server.printP(html_th); server << "Last PIR";
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Status);
    server.printP(html_e_th); server.printP(html_e_tr);
    systime_t _timeNow = nilTimeNow();
    for (uint8_t i = 0; i < ALR_ZONES; ++i) {
      if ( (i & 0x01) == 0) server.printP(html_tr);
      else                  server.printP(html_tr_ev);
      server.printP(html_td); 
      server << i+1; server.printP(text_dot); server.printP(html_e_td); server.printP(html_td);
      (conf.zone[i] >> 15) ? server.printP(text_analog) : server.printP(text_digital); server.printP(html_e_td); server.printP(html_td);
      server << conf.group_name[i]; server.printP(html_e_td); server.printP(html_td);
      (conf.zone[i] & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td); server.printP(html_td);
      ((conf.zone[i] >> 7) & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td); server.printP(html_td);
      server << ((conf.zone[i] >> 5) & B11); server.printP(html_e_td); server.printP(html_td);
      server << ((conf.zone[i] >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((conf.zone[i] >> 1) & B1111)];
      server.printP(html_e_td); server.printP(html_td);
      if ((conf.zone[i] & B1)) { server << (_timeNow - zone[i].last_PIR)/NIL_CFG_FREQUENCY; }
      else                     { server.printP(text_spdashsp); }
      server.printP(text_space); server.printP(text_sec);
      server.printP(html_e_td); server.printP(html_td);
      if (conf.zone[i] >> 15){       // Digital 0/ Analog 1
        nilSemWait(&ADCSem);          // Wait for slot
        val = nilAnalogRead(i+1);
        nilSemSignal(&ADCSem);        // Exit region.
          if ((conf.zone[i] & B1)) {
            switch((int16_t)(val-BatteryLevel)){
              case ALR_OK_LOW ... ALR_OK_HI: server.printP(text_OK); break;
              case ALR_PIR_LOW ... ALR_PIR_HI: server.printP(text_ALARM); break;
              case ALR_TAMP_LOW ... ALR_TAMP_HI: server.printP(text_tamper); break;
              default: server.printP(text_undefined); break;
            }
          } else { server.printP(text_disabled); } // disabled
      } else {
        if ((conf.zone[i] & B1)) {
          switch(i) {
            case 7: pinIN1.read() ? server.printP(text_ALARM) : server.printP(text_OK); break;
            case 8: pinIN2.read() ? server.printP(text_ALARM) : server.printP(text_OK); break;
            case 9: pinIN3.read() ? server.printP(text_ALARM) : server.printP(text_OK); break;
            case 10: pinIN4.read() ? server.printP(text_ALARM) : server.printP(text_OK); break;
            case 11: pinTAMPER.read() ? server.printP(text_ALARM) : server.printP(text_OK); break;          
            default: break;
          } 
        } else { server.printP(text_disabled); }
      }
      server.printP(html_e_td); server.printP(html_e_tr);      
    }
    server.printP(html_e_table); server.printP(html_e_p);

    server.printP(html_select); server << "Z"; server.printP(html_e_tag);
    for (uint8_t ii = 0; ii < ALR_ZONES; ++ii) {
      if (webZone == ii) 
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(text_spdashsp); server << conf.zone_name[ii]; server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(text_spdashsp); server << conf.zone_name[ii]; server.printP(html_e_option);}
    }
    server.printP(html_e_select);
    server.printP(html_F_S); server.printP(html_p);
    server.printP(text_Zone); server.printP(text_space); server << webZone+1; server.printP(text_space); server.printP(text_is); server.printP(text_space);
    ((conf.zone[webZone] >> 15) & 1) ? server.printP(text_analog) : server.printP(text_digital); server.printP(html_br);
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_td);  server.printP(text_Zone); server.printP(text_space); server.printP(text_name); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.zone_name[webZone]; server.printP(html_e_tag);
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Set zone"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("o", "1", "On", conf.zone[webZone] & B1);
    server.radioButton("o", "0", "Off", !(conf.zone[webZone] & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Auto arm"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("a", "1", "On", conf.zone[webZone] >> 7 & B1);
    server.radioButton("a", "0", "Off", !(conf.zone[webZone] >> 7 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Authentication delay"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("d", "0", "0x", !((conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
    server.radioButton("d", "1", "1x", (!(conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
    server.radioButton("d", "2", "2x", ((conf.zone[webZone] >> 6 & B1) & !(conf.zone[webZone] >> 5 & B1)));
    server.radioButton("d", "3", "3x", ((conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Group member"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.printP(html_select); server << "g"; server.printP(html_e_tag);
    for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
      if ((conf.zone[webZone] >> 1 & B1111) == ii) 
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_e_table);
    server.printP(html_e_p);    
    server.printP(html_F_A); // submit Apply
    server.printP(html_F_SA); // submit Save all
    server.printP(html_e_form);
    server.printP(htmlFoot);
  }
}

uint8_t webGroup = 0;
void webSetGroup(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (type == WebServer::POST) {
    bool repeat;
    char name[2], value[16];
    do {
      repeat = server.readPOSTparam(name, 2, value, 16);
      switch(name[0]){
        case 'G':
          n = strtol(value, NULL, 10);
          if (n != webGroup) {
            webGroup = n;
            repeat = 0;
          }
        break;
        case 'n': // zone name
          strncpy (conf.group_name[webGroup], value, sizeof(value));
          conf.group_name[webGroup][15] = 0;
        break;
        case 'o': // enable group
          if (value[0] == '0') conf.group[webGroup] &= ~(1 << 0);
          else conf.group[webGroup] |= (1 << 0);
        break;      
        case 'p': // PIR OUT1
          if (value[0] == '0') conf.group[webGroup] &= ~(1 << 4);
          else conf.group[webGroup] |= (1 << 4);
        break;
        case 'q': // PIR OUT2
          if (value[0] == '0') conf.group[webGroup] &= ~(1 << 3);
          else conf.group[webGroup] |= (1 << 3);
        break;
        case 't': // Tamper OUT1
          if (value[0] == '0') conf.group[webGroup] &= ~(1 << 2);
          else conf.group[webGroup] |= (1 << 2);
        break;
        case 'y': // Tamper OUT2
          if (value[0] == '0') conf.group[webGroup] &= ~(1 << 1);
          else conf.group[webGroup] |= (1 << 1);
        break;
        case 'e': // save to EEPROM
          eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration
          _tmp[0] = 'S'; _tmp[1] = 'C'; _tmp[2] = 'W'; _tmp[3] = 0; fifoPut(_tmp);
        break;          
      }
    } while (repeat);
    server.httpSeeOther(PREFIX "/group");
  } else {
    server.httpSuccess();
    server.printP(htmlHead);
    server.printP(html_h1); server << "Group"; server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_th); server.printP(text_hash);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Name); 
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Enabled);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Armed);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Delay);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Status);
    server.printP(html_e_th); server.printP(html_e_tr);
    for (uint8_t i = 0; i < ALR_GROUPS; ++i) {
      if ( (i & 0x01) == 0) server.printP(html_tr);
      else                  server.printP(html_tr_ev);
      server.printP(html_td); 
      server << i+1; server.printP(text_dot); server.printP(html_e_td); server.printP(html_td);
      server << conf.group_name[i]; server.printP(html_e_td); server.printP(html_td);
      (conf.group[i] & B1) ? server.printP(text_Yes) : server.printP(text_No);
      server.printP(html_e_td); server.printP(html_td);
      (group[i].set & B1) ? server.printP(text_Yes) : server.printP(text_No);
      server.printP(html_e_td); server.printP(html_td);
      server << group[i].arm_delay/4; server.printP(text_space); server.printP(text_sec);
      server.printP(html_e_td); server.printP(html_td);
      (group[i].set >> 1 & B1) ? server.printP(text_ALARM) : server.printP(text_OK);
      server.printP(html_e_td); server.printP(html_e_tr);
    }
    server.printP(html_e_table); server.printP(html_e_p);
    server.printP(html_form_s); server << PREFIX "/group"; server.printP(html_form_e);
    server.printP(html_select); server << "G"; server.printP(html_e_tag);
    for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
      if (webGroup == ii) 
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(html_e_option);}
    }
    server.printP(html_e_select);
    server.printP(html_F_S); server.printP(html_p);
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Group); server.printP(text_space); server.printP(text_name); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.group_name[webGroup]; server.printP(html_e_tag);
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Set group"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("o", "1", "On", conf.group[webGroup] & B1);
    server.radioButton("o", "0", "Off", !(conf.group[webGroup] & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Alarm triggers"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server << "OUT1"; server.printP(text_sesp);
    server.radioButton("p", "1", "On", conf.group[webGroup] >> 4 & B1);
    server.radioButton("p", "0", "Off", !(conf.group[webGroup] >> 4 & B1));
    server.printP(html_br);  server.printP(text_sesp); server << "OUT2"; server.printP(text_sesp);
    server.radioButton("q", "1", "On", conf.group[webGroup] >> 3 & B1);
    server.radioButton("q", "0", "Off", !(conf.group[webGroup] >> 3 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Tamper triggers"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server << "OUT1"; server.printP(text_sesp);
    server.radioButton("t", "1", "On", conf.group[webGroup] >> 2 & B1);
    server.radioButton("t", "0", "Off", !(conf.group[webGroup] >> 2 & B1));
    server.printP(html_br);  server.printP(text_sesp); server << "OUT2"; server.printP(text_sesp);
    server.radioButton("y", "1", "On", conf.group[webGroup] >> 1 & B1);
    server.radioButton("y", "0", "Off", !(conf.group[webGroup] >> 1 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    
    server.printP(html_e_table);
    server.printP(html_e_p);    
    server.printP(html_F_A); // submit Apply
    server.printP(html_F_SA); // submit Save all
    server.printP(html_e_form);
    server.printP(htmlFoot);
  }
}

uint8_t webKey = 0;
void webSetKey(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (type == WebServer::POST) {
    bool repeat;
    char name[2], value[17];
    do {
      repeat = server.readPOSTparam(name, 2, value, 17);
      switch(name[0]){
        case 'K':
          n = strtol(value, NULL, 10);
          if (n != webKey) {
            webKey = n;
            repeat = 0;
          }
        break;
        case 'k': // key
          for (uint8_t ii = 0; ii < KEY_LEN; ++ii) {
            if (value[ii*2] > '9') value[ii*2] = value[ii*2] - 'A' + 10;
            else value[ii*2] = value[ii*2] - '0';
            if (value[ii*2+1] > '9') value[ii*2+1] = value[ii*2+1] - 'A' + 10;
            else value[ii*2+1] = value[ii*2+1] - '0';
            //Serial.print(value[ii*2],HEX);
            //Serial.print('+');
            //Serial.print(value[ii*2+1],HEX);
            //Serial.print('=');
            conf.key[webKey][ii] = value[ii*2] << 4;
            conf.key[webKey][ii] = conf.key[webKey][ii] + value[ii*2+1];
            //Serial.println(conf.key[webKey][ii],HEX);
          } 
          conf.key[webKey][8] = 0;
        break;
        case 'n': // key
          strncpy (conf.key_name[webKey], value, 16);
          conf.key_name[webKey][15] = 0;
        break;
        case 'e': // save to EEPROM
          eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration
          _tmp[0] = 'S'; _tmp[1] = 'C'; _tmp[2] = 'W'; _tmp[3] = 0; fifoPut(_tmp);
        break;          
      }
    } while (repeat);
    server.httpSeeOther(PREFIX "/key");
  } else {
    server.httpSuccess();
    server.printP(htmlHead);
    server.printP(html_h1); server.printP(text_Key); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_form_s); server << PREFIX "/key"; server.printP(html_form_e);
    server.printP(html_select); server << "K"; server.printP(html_e_tag);
    for (uint8_t ii = 0; ii < NUM_OF_KEYS; ++ii) {
      if (webKey == ii) 
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(text_spdashsp); server << conf.key_name[ii]; server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(text_spdashsp); server << conf.key_name[ii]; server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(html_F_S); server.printP(html_p);
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Key); server.printP(text_space); server.printP(text_name); server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp); server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.key_name[webKey]; server.printP(html_e_tag);
    server.printP(html_e_td); server.printP(html_e_tr);
    for (uint8_t ii = 0; ii < KEY_LEN; ++ii) { // Format the key to nice HEX 
      tmp[ii*2]   = conf.key[webKey][ii] >> 4 & B1111;
      tmp[ii*2+1] = conf.key[webKey][ii] & B1111;
      if (tmp[ii*2] > 9) tmp[ii*2] = tmp[ii*2] + 'A' - 10; 
      else tmp[ii*2] = tmp[ii*2] + '0';
      if (tmp[ii*2+1] > 9) tmp[ii*2+1] = tmp[ii*2+1] + 'A' - 10;
      else tmp[ii*2+1] = tmp[ii*2+1] + '0';
    }
    tmp[16] = 0;
    server.printP(html_tr); server.printP(html_td); server.printP(text_Key); server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp); server.printP(html_s_tag); server << "k"; server.printP(html_m_tag); server << tmp; server.printP(html_e_tag);
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_e_table);
    server.printP(html_e_p);    
    server.printP(html_F_A); // submit Apply
    server.printP(html_F_SA); // submit Save all
    server.printP(html_e_form);
    server.printP(htmlFoot);
  }
}

uint8_t webTel = 0;
void webSetPhone(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (type == WebServer::POST) {
    bool repeat;
    char name[2], value[17];
    do {
      repeat = server.readPOSTparam(name, 2, value, 17);
      switch(name[0]){
        case 'P':
          n = strtol(value, NULL, 10);
          if (n != webTel) {
            webTel = n;
            repeat = 0;
          }
        break;
        case 'p': // number
          strncpy (conf.tel_num[webTel], value, 16);
          conf.tel_num[webTel][15] = 0;
        break;
        case 'n': // name
          strncpy (conf.tel_name[webTel], value, 16);
          conf.tel_name[webTel][15] = 0;
        break;
        case 'o': // enable tel
          if (value[0] == '0') conf.tel[webTel] &= ~(1 << 0);
          else conf.tel[webTel] |= (1 << 0);
        break;
        case 'g': // group
          n = strtol(value, NULL, 10);
          if ((n >> 0) & 1) conf.tel[webTel] |= (1 << 1);
          else conf.tel[webTel] &= ~(1 << 1);
          if ((n >> 1) & 1) conf.tel[webTel] |= (1 << 2);
          else conf.tel[webTel] &= ~(1 << 2);
          if ((n >> 2) & 1) conf.tel[webTel] |= (1 << 3);
          else conf.tel[webTel] &= ~(1 << 3);
          if ((n >> 3) & 1) conf.tel[webTel] |= (1 << 4);
          else conf.tel[webTel] &= ~(1 << 4);
        break;
        case 'e': // save to EEPROM
          eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration
          _tmp[0] = 'S'; _tmp[1] = 'C'; _tmp[2] = 'W'; _tmp[3] = 0; fifoPut(_tmp);
        break;          
      }
    } while (repeat);
    server.httpSeeOther(PREFIX "/phone");
  } else {
    server.httpSuccess();
    server.printP(htmlHead);
    server.printP(html_h1); server.printP(text_Phone); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1); server.printP(html_p);

    server.printP(html_table);
    server.printP(html_tr); server.printP(html_th); server.printP(text_hash);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Enabled);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Name);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Number);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Group);
    server.printP(html_e_th); server.printP(html_e_tr);
    for (uint8_t i = 0; i < NUM_OF_PHONES; ++i) {
      if ( (i & 0x01) == 0) server.printP(html_tr);
      else                  server.printP(html_tr_ev);
      server.printP(html_td); 
      server << i+1; server.printP(text_dot); server.printP(html_e_td); server.printP(html_td);
      (conf.tel[i] & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td); server.printP(html_td);
      server << conf.tel_name[i]; server.printP(html_e_td); server.printP(html_td);
      server << conf.tel_num[i]; server.printP(html_e_td); server.printP(html_td);
      server << ((conf.tel[i] >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((conf.tel[i] >> 1) & B1111)];
      server.printP(html_e_td); server.printP(html_e_tr);
    }
    server.printP(html_e_table); server.printP(html_e_p);

    server.printP(html_p);
    server.printP(html_form_s); server << PREFIX "/phone"; server.printP(html_form_e);
    server.printP(html_select); server << "P"; server.printP(html_e_tag);
    for (uint8_t ii = 0; ii < NUM_OF_PHONES; ++ii) {
      if (webTel == ii) 
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(text_spdashsp); server << conf.tel_name[ii]; server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(text_spdashsp); server << conf.tel_name[ii]; server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(html_F_S); server.printP(html_p);
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_td); server << "Set phone"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("o", "1", "On", conf.tel[webTel] & B1);
    server.radioButton("o", "0", "Off", !(conf.tel[webTel] & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Phone "; server.printP(text_name); server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp);
    if (webTel != 0) { server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.tel_name[webTel]; server.printP(html_e_tag); }
    else             { server << conf.tel_name[webTel]; }
    server.printP(html_e_p);
    
    server.printP(html_tr); server.printP(html_td); server << "Phone number"; server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp); server.printP(html_s_tag); server << "p"; server.printP(html_m_tag); server << conf.tel_num[webTel]; server.printP(html_e_tag);
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Group member"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.printP(html_select); server << "g"; server.printP(html_e_tag);
    for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
      if ((conf.tel[webTel] >> 1 & B1111) == ii) 
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_e_table);
    server.printP(html_e_p);    
    server.printP(html_F_A); // submit Apply
    server.printP(html_F_SA); // submit Save all
    server.printP(html_e_form);
    server.printP(htmlFoot);
  }
}

uint8_t webAuth = 0;
void webSetAuth(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (type == WebServer::POST) {
    bool repeat;
    char name[2], value[17];
    do {
      repeat = server.readPOSTparam(name, 2, value, 17);
      switch(name[0]){
        case 'P':
          n = strtol(value, NULL, 10);
          if (n != webAuth) {
            webAuth = n;
            repeat = 0;
          }
        break;
        case 'o': // enable 
          if (value[0] == '0') conf.auth[webAuth] &= ~(1 << 0);
          else conf.auth[webAuth] |= (1 << 0);
        break;
        case 'g': // group
          n = strtol(value, NULL, 10);
          if ((n >> 0) & 1) conf.auth[webAuth] |= (1 << 1);
          else conf.auth[webAuth] &= ~(1 << 1);
          if ((n >> 1) & 1) conf.auth[webAuth] |= (1 << 2);
          else conf.auth[webAuth] &= ~(1 << 2);
          if ((n >> 2) & 1) conf.auth[webAuth] |= (1 << 3);
          else conf.auth[webAuth] &= ~(1 << 3);
          if ((n >> 3) & 1) conf.auth[webAuth] |= (1 << 4);
          else conf.auth[webAuth] &= ~(1 << 4);
        break;
        case 'e': // save to EEPROM
          eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration
          _tmp[0] = 'S'; _tmp[1] = 'C'; _tmp[2] = 'W'; _tmp[3] = 0; fifoPut(_tmp);
        break;          
      }
    } while (repeat);
    server.httpSeeOther(PREFIX "/auth");
  } else {
    server.httpSuccess();
    server.printP(htmlHead);
    server.printP(html_h1); server.printP(text_Authentication); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_form_s); server << PREFIX "/auth"; server.printP(html_form_e);
    
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_th); server.printP(text_hash);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Enabled);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Group);
    server.printP(html_e_th); server.printP(html_e_tr);
    for (uint8_t i = 0; i < ALR_AUTH_UNITS; ++i) {
      if ( (i & 0x01) == 0) server.printP(html_tr);
      else                  server.printP(html_tr_ev);
      server.printP(html_td); 
      server << i+1; server.printP(text_dot); server.printP(html_e_td); server.printP(html_td);
      (conf.auth[i] & B1) ? server.printP(text_Yes) : server.printP(text_No);
      server.printP(html_e_td); server.printP(html_td);
      server << ((conf.auth[i] >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((conf.auth[i] >> 1) & B1111)];
      server.printP(html_e_td); server.printP(html_e_tr);
    }
    server.printP(html_e_table); server.printP(html_e_p);

    server.printP(html_select); server << "P"; server.printP(html_e_tag);
    for (uint8_t ii = 0; ii < ALR_AUTH_UNITS; ++ii) {
      if (webAuth == ii) 
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(html_F_S); server.printP(html_p);
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_td); server << "Set unit"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("o", "1", "On", conf.auth[webAuth] & B1);
    server.radioButton("o", "0", "Off", !(conf.auth[webAuth] & B1));
    server.printP(html_e_td); server.printP(html_e_tr); 
    server.printP(html_tr); server.printP(html_td); server << "Group member"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.printP(html_select); server << "g"; server.printP(html_e_tag);
    for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
      if ((conf.auth[webAuth] >> 1 & B1111) == ii) 
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_e_table);
    server.printP(html_e_p);    
    server.printP(html_F_A); // submit Apply
    server.printP(html_F_SA); // submit Save all
    server.printP(html_e_form);
    server.printP(htmlFoot);
  }
}

void webSetGlobal(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  char name[2], value[17];
  if (type == WebServer::POST) {
    bool repeat;
    //char name[2], value[17];
    do {
      repeat = server.readPOSTparam(name, 2, value, 17);
      switch(name[0]){
        case 'a':
          conf.alr_time = strtol(value, NULL, 10);
        break;
        case 'd':
          conf.arm_delay = strtol(value, NULL, 10) * 4; // 250*4 = 1 sec.
        break;
        case 'e': // save to EEPROM
          eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration
          _tmp[0] = 'S'; _tmp[1] = 'C'; _tmp[2] = 'W'; _tmp[3] = 0; fifoPut(_tmp);
        break;
        case '0': // Radio button
          if (value[0] == '0') conf.SMS &= ~(1 << 0);
          else conf.SMS |= (1 << 0);
        break;
        case '1': // Radio button
          if (value[0] == '0') conf.SMS &= ~(1 << 1);
          else conf.SMS |= (1 << 1);
        break;
        case '2': // Radio button
          if (value[0] == '0') conf.SMS &= ~(1 << 2);
          else conf.SMS |= (1 << 2);
        break;
        case '3': // Radio button
          if (value[0] == '0') conf.SMS &= ~(1 << 3);
          else conf.SMS |= (1 << 4);
        break;
        case '4': // Radio button
          if (value[0] == '0') conf.SMS &= ~(1 << 4);
          else conf.SMS |= (1 << 4);
        break;
        case '5': // Radio button
          if (value[0] == '0') conf.SMS &= ~(1 << 5);
          else conf.SMS |= (1 << 5);
        break;
        case '6': // Radio button
          if (value[0] == '0') conf.SMS &= ~(1 << 6);
          else conf.SMS |= (1 << 6);
        break;
        case '7': // Radio button
          if (value[0] == '0') conf.SMS &= ~(1 << 7);
          else conf.SMS |= (1 << 7);
        break;
        case '8': // Radio button
          if (value[0] == '0') conf.SMS &= ~(1 << 8);
          else conf.SMS |= (1 << 8);
        break;
        case '9': // Radio button
          if (value[0] == '0') conf.SMS &= ~(1 << 9);
          else conf.SMS |= (1 << 9);
        break;
        case 'Q': // Radio button
          if (value[0] == '0') conf.SMS &= ~(1 << 10);
          else conf.SMS |= (1 << 10);
        break;
        case 'W': // Radio button
          if (value[0] == '0') conf.SMS &= ~(1 << 11);
          else conf.SMS |= (1 << 11);
        break;
        case 's': // tel. number
          conf.global_tel_num = value[0] - 48;;
        break;
      }
    } while (repeat);
    server.httpSeeOther(PREFIX "/set");
  } else {
    server.httpSuccess();
    server.printP(htmlHead);
    server.printP(html_h1); server.printP(text_Global); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_form_s); server << PREFIX "/set"; server.printP(html_form_e);
    
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_td);server << "Authentication time"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.printP(html_select); server << "a"; server.printP(html_e_tag);
    for (uint8_t ii = 5; ii < 26; ++ii) {
      if ((conf.alr_time) == ii)  
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii; server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii; server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(text_space); server.printP(text_seconds); server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td);server << "Arm delay"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.printP(html_select); server << "d"; server.printP(html_e_tag);
    for (uint8_t ii = 10; ii < 41; ++ii) {
      if ((conf.arm_delay/4) == ii) // 250*4 = 1 sec.
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii; server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii; server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(text_space); server.printP(text_seconds); server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_e_table);
    server.printP(html_e_p);
    server.printP(html_h1); server.printP(text_SMS); server.printP(text_space); server.printP(text_alerting); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_td); server << "Default phone number"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.printP(html_select); server << "s"; server.printP(html_e_tag);
    for (uint8_t ii = 0; ii < NUM_OF_PHONES; ++ii) {
      if ((conf.global_tel_num) == ii) 
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(text_spdashsp); server << conf.tel_name[ii]; server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(text_spdashsp); server << conf.tel_name[ii]; server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Undefined Key"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("0", "1", "On", conf.SMS & B1);
    server.radioButton("0", "0", "Off", !(conf.SMS & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Disarmed"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("1", "1", "On", conf.SMS >> 1 & B1);
    server.radioButton("1", "0", "Off", !(conf.SMS >> 1 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Armed"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("2", "1", "On", conf.SMS >> 2 & B1);
    server.radioButton("2", "0", "Off", !(conf.SMS >> 2 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Undefined Alarm"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("3", "1", "On", conf.SMS >> 3 & B1);
    server.radioButton("3", "0", "Off", !(conf.SMS >> 3 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "TAMPER"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("4", "1", "On", conf.SMS >> 4 & B1);
    server.radioButton("4", "0", "Off", !(conf.SMS >> 4 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "PIR"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("5", "1", "On", conf.SMS >> 5 & B1);
    server.radioButton("5", "0", "Off", !(conf.SMS >> 5 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "ALARM - No authentication"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("6", "1", "On", conf.SMS >> 6 & B1);
    server.radioButton("6", "0", "Off", !(conf.SMS >> 6 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Monitoring started"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("7", "1", "On", conf.SMS >> 7 & B1);
    server.radioButton("7", "0", "Off", !(conf.SMS >> 7 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "System armed"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("8", "1", "On", conf.SMS >> 8 & B1);
    server.radioButton("8", "0", "Off", !(conf.SMS >> 8 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Configuration saved"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("9", "1", "On", conf.SMS >> 9 & B1);
    server.radioButton("9", "0", "Off", !(conf.SMS >> 9 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "AC state"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("Q", "1", "On", conf.SMS >> 10 & B1);
    server.radioButton("Q", "0", "Off", !(conf.SMS >> 10 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server << "Battery state"; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("W", "1", "On", conf.SMS >> 11 & B1);
    server.radioButton("W", "0", "Off", !(conf.SMS >> 11 & B1));
    server.printP(html_e_td); server.printP(html_e_tr); server.printP(html_e_table);
    server.printP(html_e_p);
    server.printP(html_F_A); // submit Apply
    server.printP(html_F_SA); // submit Save all
    server.printP(html_e_form);
    server.printP(htmlFoot);
  }
}

// *********************************************************************************
// T H R E A D S    T H R E A D S    T H R E A D S    T H R E A D S    T H R E A D S
// *********************************************************************************

// Sensor thread
//
NIL_WORKING_AREA(waSensorThread, 64);
NIL_THREAD(SensorThread, arg) {
  int16_t val = 0;
  uint8_t _group = 255;
  //char l_tmp[4]; // for logger 
  nilThdSleepSeconds(60); // Delay to allow PIR sensors to settle up
  _tmp[0] = 'S'; _tmp[1] = 'S'; _tmp[2] = 0; fifoPut(_tmp);
  WS.println(F("SensorThread started"));
  
  // Execute while loop every 0.25 seconds.
  while (TRUE) {
    nilThdSleepMilliseconds(250); // time is used also for arm delay and such ...

    for (int8_t i=0; i < ALR_GROUPS ; i++){ 
      if (group[i].arm_delay) { // wait for arm delay
        group[i].arm_delay--;
        if (!group[i].arm_delay) { 
          _tmp[0] = 'S'; _tmp[1] = 'Z'; _tmp[2] = 48+i; _tmp[3] = 0; fifoPut(_tmp);
        }
      }
    }

    nilSemWait(&ADCSem);    // wait for slot
    BatteryLevel = nilAnalogRead(0); // Sensor voltage level
    nilSemSignal(&ADCSem);  // Exit region.
    
    //WS.print(F(">"));
    //WS.print(BatteryLevel);
    for (int8_t i=0; i < ALR_ZONES ; i++){
      //WS.print(F(" | "));
      //WS.print(i);
      if (conf.zone[i] & B1){         // Zone enabled ?
        if (conf.zone[i] >> 15){       // Digital 0/ Analog 1 
          //WS.print(F("-A:"));
          nilSemWait(&ADCSem);          // Wait for slot
          val = nilAnalogRead(i+1);
          nilSemSignal(&ADCSem);        // Exit region.
          } else {
          //WS.print(F("-D:"));
            switch(i){
            case 7:
              if (pinIN1.read()) val = ALR_PIR+BatteryLevel; // Let's simulate alarm (NC)
              else val = ALR_OK+BatteryLevel;
              break;
            case 8:
              if (pinIN2.read()) val = ALR_PIR+BatteryLevel; // Let's simulate alarm (NC)
              else val = ALR_OK+BatteryLevel;
              break;
            case 9:
              if (pinIN3.read()) val = ALR_PIR+BatteryLevel; // Let's simulate alarm (NC)
              else val = ALR_OK+BatteryLevel;
              break;
            case 10:
              if (pinIN4.read()) val = ALR_PIR+BatteryLevel; // Let's simulate alarm (NC)
              else val = ALR_OK+BatteryLevel;
              break;
            case 11:
              if (pinTAMPER.read()) val = ALR_PIR+BatteryLevel; // Let's simulate alarm (NC)
              else val = ALR_OK+BatteryLevel;
              break;           
            default:
            break;
          } 
        }
        //WS.print(val);
        _group = (conf.zone[i] >> 1) & B1111; // set group
        // Decide 
        switch((int16_t)(val-BatteryLevel)){
          case ALR_OK_LOW ... ALR_OK_HI:
            // All is OK no action
            //WS.print(F(" OK"));
          break;
          case ALR_PIR_LOW ... ALR_PIR_HI:
            //WS.print(F(" PIR G:"));
            //WS.print(_group);
            //   group  armed              group not has alarm
            if ((group[_group].set & B1) && !((group[_group].set >> 1) & B1) && !group[_group].arm_delay){             
              _tmp[0] = 'P'; _tmp[1] = 48+i; _tmp[2] = 0; fifoPut(_tmp); 
              alarm_event_t* p = alarm_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
              if (p == 0) return; // Continue if no free space.   
              p->zone = i;
              p->type = 'P';
              group[_group].set |= (1 << 1); // Set alarm bit On
              alarm_fifo.signalData();   // Signal idle thread data is available.
            }
            zone[i].last_PIR = nilTimeNow();    // update current timestamp
          break;
          case ALR_TAMP_LOW ... ALR_TAMP_HI:
            //WS.print(F(" TAMPER G:"));
            //WS.print(_group);
            //  group not has alarm
            if (!((group[_group].set >> 1) & B1)){             
              _tmp[0] = 'T'; _tmp[1] = 48+i; _tmp[2] = 0; fifoPut(_tmp);
              alarm_event_t* p = alarm_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
              if (p == 0) return; // Continue if no free space.   
              p->zone = i;
              p->type = 'T';
              group[_group].set |= (1 << 1); // Set alarm bit On
              alarm_fifo.signalData();   // Signal idle thread data is available.
            }
          break;       
          default: 
            // Line is cut or short
            //WS.print(F(" UNK G:"));
            //WS.print(_group);
            //  group not has alarm
            if (!((group[_group].set >> 1) & B1)){             
              _tmp[0] = 'D'; _tmp[1] = 48+i; _tmp[2] = 0; fifoPut(_tmp);
              alarm_event_t* p = alarm_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
              if (p == 0) return; // Continue if no free space.   
              p->zone = i;
              p->type = 'D';
              group[_group].set |= (1 << 1); // Set alarm bit On
              alarm_fifo.signalData();   // Signal idle thread data is available.
            }
          break;
        }
        //Serial.print(F("| "));        
      } // Zone enabled ?
      //else WS.print(F("-DIS"));
    } // for (zones)
    //WS.println(F(""));
  } // while thread TRUE
}

uint8_t sendCmdToGrp(uint8_t grp, uint8_t cmd) {
  int8_t  _resp;
  uint8_t cnt = 0;
  // Go throug all units
  WS.print("Cmd: ");
  WS.print(cmd);
  WS.print(", unit: ");
  for (int8_t i=0; i < ALR_AUTH_UNITS ; i++){
    if (conf.auth[i] & B1) {                       // Auth. unit is enabled ?
      if (((conf.auth[i] >> 1) & B1111) == grp) {  // Auth. unit belong to group

        TX_msg.address = i+1;
        TX_msg.ctrl = FLAG_CMD;
        TX_msg.data_length = cmd;
        _resp = RS485.msg_write(&TX_msg);

        ++cnt;
        WS.print(i+1);
        WS.print(":");
        WS.print(_resp);
        WS.print(", ");
        nilThdSleepMilliseconds(5);
      }
    }
  }
  WS.println();
  return cnt;
}

//------------------------------------------------------------------------------
// Alarm Events
//
NIL_WORKING_AREA(waAEThread, 128);
NIL_THREAD(AEThread, arg) {
  uint8_t _group, _wait, _resp;
  msg_t r;
  WS.println(F("Event thread started"));
  
  while (TRUE) {
    // Check for data.  Use TIME_IMMEDIATE to prevent sleep in idle thread.
    alarm_event_t* p = alarm_fifo.waitData(TIME_INFINITE);
    if (!p) return;                           // return if no data

    _group = (conf.zone[p->zone] >> 1) & B1111;
    if (p->type == 'P') _wait = (conf.zone[p->zone] >> 5) & B11;
    else                _wait = 0;            // Tamper and others have no wait time

    if (!((group[_group].set >> 1) & B1)) return; // group has no alarm anymore

    WS.print("Zone: ");
    WS.print(p->zone);
    WS.print(", type: ");
    WS.print(p->type);
    WS.print(", group: ");
    WS.print(_group);
    WS.print(", Auth time: ");
    WS.print(_wait);
    WS.println();

    _resp = sendCmdToGrp(_group, 60 + _wait);

    if (!_wait) { // Alarm wait 0x
      // Combine alarms, so that next alarm will not disable ongoing one
      OUTs = ((((conf.group[_group] >> 4) & B1) | (OUTs >> 0) & B1) |
              (((conf.group[_group] >> 3) & B1) | (OUTs >> 1) & B1) << 1);
      // Trigger OUT 1 & 2              
      pinOUT1.write(((OUTs >> 0) & B1));
      pinOUT2.write(((OUTs >> 1) & B1));
      _tmp[0] = 'S'; _tmp[1] = 'X'; _tmp[2] = 48+_group; _tmp[3] = 0; fifoPut(_tmp); // ALARM no auth.
    } else { // Alarm wait > 0x
      do {
        if (_wait) r = nilSemWaitTimeout(&AUTHSem, conf.alr_time * 1000);// wait for AUT
        if (r != NIL_MSG_TMO) { // Auth. OK
          break; // escape do while 
        } else {
          _wait--;
          _resp = sendCmdToGrp(_group, 60 + _wait);
        }
      } while (_wait);
      if (!_wait) { // Alarm wait 0x = no auth in time
        // Combine alarms, so that next alarm will not disable ongoing one
        OUTs = ((((conf.group[_group] >> 4) & B1) | (OUTs >> 0) & B1) |
                (((conf.group[_group] >> 3) & B1) | (OUTs >> 1) & B1) << 1);
        // Trigger OUT 1 & 2              
        pinOUT1.write(((OUTs >> 0) & B1));
        pinOUT2.write(((OUTs >> 1) & B1));
        _tmp[0] = 'S'; _tmp[1] = 'X';  _tmp[2] = 48 + _group; _tmp[3] = 0;fifoPut(_tmp); // ALARM no auth.
      }
    }
    // Signal FIFO slot is free.
    alarm_fifo.signalFree();
  }
}

//------------------------------------------------------------------------------
// RS485 receiver thread
//
NIL_WORKING_AREA(waRS485RXThread, 64);
NIL_THREAD(RS485RXThread, arg) {
  uint8_t _resp = 0; 
  uint8_t _group = 255;
  WS.println(F("RS485 receiver thread started"));
  
  while (TRUE) {   
    nilWaitRS485NewMsg(); // wait for event
    
    _resp = RS485.msg_read(&RX_msg);
    
    WS.print(F("A:")); WS.print(RX_msg.address);
    WS.print(F(", C:")); WS.print(RX_msg.ctrl);
    WS.print(F(", L:")); WS.print(RX_msg.data_length); WS.println();   
    
    for (uint8_t i=0; i < RX_msg.data_length; i++){
      WS.print((uint8_t)RX_msg.buffer[i],HEX);
      WS.print(F(" "));
    }
    WS.println();
    
    // iButtons keys
    if (conf.auth[RX_msg.address-1] & B1) { // unit is enabled for authorzation
      if (RX_msg.ctrl == FLAG_DTA && RX_msg.data_length == KEY_LEN) {  // incoming message looks like a key 
        for (uint8_t i=0; i < NUM_OF_KEYS; i++){
          _resp = memcmp(RX_msg.buffer, conf.key[i], KEY_LEN); // Compare key
          if (!_resp ) { // key matched
            if (true) {  // key enabled
              _group = (conf.auth[RX_msg.address-1] >> 1) & B1111;
              //if (nilSemGetCounter(&ARMSem)) { // we are armed
              if ((group[_group].set) & B1) { // group is armed
                _tmp[0] = 'A'; _tmp[1] = 'D'; _tmp[2] = 48+i; _tmp[3] = 0; fifoPut(_tmp);
                group[_group].set &= ~(1 << 0);    // disarm group
                WS.print(F("Armed: ")); WS.print(((group[_group].set) & B1));
                WS.print(F(", G: ")); WS.println(_group);
                group[_group].arm_delay = 0;  // Reset arm delay
                _resp = sendCmdToGrp(_group, 0); // send quiet message to all units

                if ((group[_group].set) >> 1 & B1) { // we are armed and we have alarm
                  group[_group].set &= ~(1 << 1);    // set alarm off
                  nilSemSignal(&AUTHSem);  // inform alarm events
                  

                  OUTs = 0; // Reset outs  


                  pinOUT1.write(LOW); pinOUT2.write(LOW); // Turn off OUT 1 & 2
                }
              } else { // we are not armed
                if ((group[_group].set) >> 1 & B1) { // Handle Tamper and Unknown (quiet) alarms
                  group[_group].set &= ~(1 << 0);    // disarm group
                  group[_group].set &= ~(1 << 1);    // set alarm off
                  _tmp[0] = 'A'; _tmp[1] = 'D'; _tmp[2] = 48+i; _tmp[3] = 0; fifoPut(_tmp);
                } else { // Just do arm
                  group[_group].set |= 1; // arm group
                  WS.print(F("Armed: ")); WS.print(((group[_group].set) & B1));
                  WS.print(F(", G: ")); WS.println(_group);
                  nilSemReset(&AUTHSem, 0); // reset auth. *********************
                  _tmp[0] = 'A'; _tmp[1] = 'A'; _tmp[2] = 48+i; _tmp[3] = 0; fifoPut(_tmp);
                  group[_group].arm_delay = conf.arm_delay;
                }
              }
              break; // no need to try other
            } else { // key is enabled
              _tmp[0] = 'A'; _tmp[1] = 'F'; _tmp[2] = 48+i; _tmp[3] = 0; fifoPut(_tmp);
            }
          } // key matched
          if (n!=0 && i==NUM_OF_KEYS) { // may be we should log unknow keys 
            _tmp[0] = 'A'; _tmp[1] = 'U'; _tmp[2] = ' '; _tmp[3] = 0; fifoPut(_tmp);
          }
        }
      } // incoming message looks like a key
    } // unit is enabled for authorzation
    else { // log disabled remote units
      _tmp[0] = 'U'; _tmp[1] = 'F'; _tmp[2] = 48+RX_msg.address; _tmp[3] = 0; fifoPut(_tmp);
    } // iButtons keys

    // Registration
    /*
    if (RX_msg.ctrl == FLAG_DTA && RX_msg.buffer[0]=='R') {
      WS.print("units:");
      uint8_t _count = UNITS_COUNT;
      WS.print(_count);
      units[_count+1].address   = RX_msg.address;
      units[_count+1].function[0] = 's';
      WS.print("units:");
      _count = UNITS_COUNT;
      WS.print(_count);
    }
    */
  }
}

//------------------------------------------------------------------------------
// Logger thread
//
NIL_WORKING_AREA(waLoggerThread, 128);
NIL_THREAD(LoggerThread, arg) {
  uint8_t sms_send, sms_ok, t_size, at_tmp;
  uint8_t ttext[40];
  //char sms_text[128];
    
  // Initialize GSM modem
  nilThdSleepSeconds(1);
  nilSemWait(&GSMSem);    // wait for slot
  WS.println(Serial.ATsendCmd(AT_is_alive));
  WS.println(Serial.ATsendCmd(AT_set_sms_to_text));
  nilSemSignal(&GSMSem);  // Exit region.
  
  WS.println(F("Logger started"));
  
  while (TRUE) {
    // Check for data.  Use TIME_IMMEDIATE to prevent sleep in idle thread.
    Record_t* p = fifo.waitData(TIME_INFINITE);
    if (!p) return; // return if no data

    char* log_message = p->text; // Fetch and print data.
  
    // SMS handler
    if (GSMisAlive) {
      sms_send = 0; sms_ok = 1;
      nilSemWait(&GSMSem);    // wait for slot
      Serial.flushRX();
      // SMS header
      Serial.print(AT_send_sms);
      Serial.print('"');    



      //if (alarm.zone!=255) Serial.print(conf.tel_num[(conf.zone[alarm.zone] >> 5 & B1111)]); // zone custom
      //else                 Serial.print(conf.tel_num[conf.global_tel_num]);                 // global one
      Serial.print(conf.tel_num[conf.global_tel_num]);                 // global one



      Serial.print('"');
      Serial.write(13);
      Serial.ATWaitMsg();
      t_size = Serial.read(ttext);               // read serial
      //WS.print(t_size); WS.print(":"); for(uint8_t i = 0; i < t_size; i++) {WS.print((char)ttext[i]);} WS.println();
      at_tmp = memcmp(AT_send_sms, ttext, sizeof(AT_send_sms)-1);   // compare
      if (at_tmp != 0) sms_ok = 0;                                  // not received
      //else WS.println("match");

      // SMS body
      switch(log_message[13]){
        case 'S':
          //strcpy_P(sms_text, (char*)text_S);
          Serial.printP(text_System); Serial.printP(text_sesp);
          switch(log_message[14]){
            case 'B': // Battery
              if (conf.SMS >> 11 & B1) sms_send = 1;
              Serial.printP(text_BR);
              if (log_message[15] == 'L') Serial.printP(text_low);
              else Serial.printP(text_high);
              Serial.printP(text_space); Serial.printP(text_level);
            break;
            case 'A': // AC
              if (conf.SMS >> 10 & B1) sms_send = 1;
              Serial.printP(text_PWS);
              if (log_message[15] == 'L') Serial.printP(text_On);
              else Serial.printP(text_Off);
            break;
            case 'C': Serial.printP(text_CS); if (conf.SMS >> 9 & B1) sms_send = 1; break; // conf. saved
            case 'Z': Serial.printP(text_armed);  if (conf.SMS >> 8 & B1) sms_send = 1; break; // system armed
            case 'S': Serial.printP(text_MS); if (conf.SMS >> 7 & B1) sms_send = 1; break; // monitoring strted
            case 'X': Serial.printP(text_ALARM);  if (conf.SMS >> 6 & B1) sms_send = 1;
                      Serial.printP(text_space); Serial.printP(text_Group); Serial.printP(text_sesp); // extra text for SMS
                      Serial.print(conf.group_name[log_message[15]-48]); // extra text for SMS
                      break; // alarm
            default:  Serial.printP(text_undefined);  if (conf.SMS >> 9 & B1) sms_send = 1; break; // unknown
          }
        break;
        case 'P':
          if (conf.SMS >> 5 & B1) sms_send = 1;
          Serial.printP(text_Alarm); Serial.printP(text_sesp); Serial.printP(text_triggered); Serial.printP(text_zone); Serial.printP(text_space);
          Serial.print(conf.zone_name[log_message[14]-48]);
        break;
        case 'T':
          if (conf.SMS >> 4 & B1) sms_send = 1;
          Serial.printP(text_Tamper); Serial.printP(text_sesp); Serial.printP(text_triggered); Serial.printP(text_zone); Serial.printP(text_space);
          Serial.print(conf.zone_name[log_message[14]-48]);
        break;
        case 'D':
          if (conf.SMS >> 3 & B1) sms_send = 1;
          Serial.printP(text_Damaged); Serial.printP(text_sesp); Serial.printP(text_triggered); Serial.printP(text_zone); Serial.printP(text_space);
          Serial.print(conf.zone_name[log_message[14]-48]);
        break;
        case 'A':
          Serial.printP(text_Authentication); Serial.printP(text_sesp); Serial.printP(text_key); Serial.printP(text_space);
          if (tmp[14] == 'A' || tmp[14] == 'D' || tmp[14] == 'F') { Serial.print(conf.key_name[log_message[14]-48]); Serial.printP(text_space); }
          switch(log_message[15]){
            case 'D': Serial.printP(text_disarmed); if (conf.SMS >> 1 & B1) sms_send = 1; break;
            case 'A': Serial.printP(text_armed); if (conf.SMS >> 2 & B1) sms_send = 1; break;
            case 'U': Serial.printP(text_undefined); if (conf.SMS & B1) sms_send = 1; break;
            case 'F': Serial.printP(text_is); Serial.printP(text_space); Serial.printP(text_disabled); if (conf.SMS & B1) sms_send = 1; break;
            case 'a': Serial.printP(text_auto); Serial.printP(text_space); Serial.printP(text_armed);
              if (conf.SMS >> 2 & B1) sms_send = 1; break; // Auto armed
            default: break;
          }  
          Serial.printP(text_group); Serial.printP(text_space); Serial.print(conf.group_name[tmp[16]-48]);  
        break;
        case 'G': // GSM status is not send
        break;
        default:
          Serial.printP(text_Undefined); Serial.printP(text_sesp);
          Serial.print(log_message);
        break;
      }
      Serial.printP(text_dot);
      
      // End of SMS
      if (sms_send) Serial.write(26); // ctrl+z, send SMS
      else          Serial.write(27); // ESC   , mo SMS 
      
      // read reply
      Serial.ATWaitMsg();
      t_size = Serial.read(ttext);                // read serial
      at_tmp = memcmp("> ", ttext, 2);            // compare
      if (at_tmp != 0) sms_ok = 0;                // not received
      //else WS.println("> match");
      
      if (sms_send) {
        Serial.ATWaitMsg();
        at_tmp = memcmp(AT_send_sms_reply, ttext, sizeof(AT_send_sms_reply)-1);  // compare
        if (at_tmp != 0) sms_ok = 0;                                             // not received
        //else WS.println("CMGS match");

      } 
      
      Serial.ATWaitMsg();
      t_size = Serial.read(ttext);                    // read serial
      at_tmp = memcmp(AT_OK, ttext, sizeof(AT_OK)-1); // compare
      if (at_tmp != 0) sms_ok = 0;                    // OK not received
      //else WS.println("OK match");

      nilSemSignal(&GSMSem);  // Exit region.

      if (sms_send) {
        if (!sms_ok) log_message[12] = '.';
      } else {
        if (!sms_ok) log_message[12] = ',';
        else         log_message[12] = ';';
      }
    } // GSMisAlive
    else log_message[12] = '|'; // modem not connected

    // Put data into extrenal EEPROM
    nilSemWait(&TWISem);          // wait for slot
    eeprom.writeBytes(conf.ee_pos,EEPROM_MESSAGE, log_message);
    nilThdSleepMilliseconds(10);  // wait for eeprom or we lose very frequent entries
    nilSemSignal(&TWISem);        // Exit region.
    conf.ee_pos += EEPROM_MESSAGE;

    // Signal FIFO slot is free.
    fifo.signalFree();
  }
}

//------------------------------------------------------------------------------
// Service thread
//
NIL_WORKING_AREA(waServiceThread, 128);  
NIL_THREAD(ServiceThread, arg) {
  int8_t  val;
  uint8_t GSMlastStatus = 10; // get status on start
  uint8_t ttext[10];
  msg_t r;
  uint8_t OUTpins = 0;                 // used for pause in alarm

  WS.println(F("Service started"));

  while (TRUE) {
    nilThdSleepSeconds(10);

    // Auto arm
    WS.print("Zone ");
    for (int8_t i=0; i < ALR_ZONES ; i++){
      if ((conf.zone[i] & B1) && ((conf.zone[i] >> 7) & B1)){ // Zone enabled and auto arming
        WS.print(i);
        if (nilTimeNowIsWithin(zone[i].last_PIR - S2ST(8), zone[i].last_PIR + S2ST(8))) {
          WS.print('A');
          _tmp[0] = 'A'; _tmp[1] = 'a'; _tmp[2] = 48 + ((conf.zone[i] >> 1) & B1111); _tmp[3] = 0; fifoPut(_tmp); // Authorization auto arm
          group[(conf.zone[i] >> 1) & B1111].set |= 1; // arm group
        }
        WS.print(" , ");
      }
    }
    WS.println("");

    // Battery check 
    if (pinBAT_OK.read() == LOW) { // The signal is "Low" when the voltage of battery is under 11V 
      _tmp[0] = 'S'; _tmp[1] = 'B'; _tmp[2] = 'L'; _tmp[3] = 0; fifoPut(_tmp); // battery low
      _tmp[0] = 'S'; _tmp[1] = 'C'; _tmp[2] = 'P'; _tmp[3] = 0; fifoPut(_tmp); // conf saved 
      Record_t* p;
      do { 
        nilThdSleepMilliseconds(100);
      } while (p); // wait for logger thread
      eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration

      nilSysLock(); // Lock RTOS  
     
      // Battery is at low level but it might oscillate, so we wait for AC to recharge battery again.
      while (pinAC_OFF.read() == HIGH) { // The signal turns to be "High" when the power supply turns OFF
        delay(10000); // do nothing wait for power supply shutdown
      } 

      nilSysUnlock(); // in case the power is restored we goon
      _tmp[0] = 'S'; _tmp[1] = 'A'; _tmp[2] = 'L'; _tmp[3] = 0; fifoPut(_tmp); // AC ON
      _tmp[0] = 'S'; _tmp[1] = 'B'; _tmp[2] = 'H'; _tmp[3] = 0; fifoPut(_tmp); // Battery High
    }

    // AC power check
    if (pinAC_OFF.read() == LOW && ACState != LOW) { // The signal turns to be "High" when the power supply turns OFF
      ACState = LOW;
      _tmp[0] = 'S'; _tmp[1] = 'A'; _tmp[2] = 'L'; _tmp[3] = 0; fifoPut(_tmp); // AC ON
    }
    if (pinAC_OFF.read() == HIGH && ACState != HIGH) { // The signal turns to be "High" when the power supply turns OFF
      ACState = HIGH;
      _tmp[0] = 'S'; _tmp[1] = 'A'; _tmp[2] = 'H'; _tmp[3] = 0; fifoPut(_tmp); // AC OFF
    }

    // GSM modem check
    r = nilSemWaitTimeout(&GSMSem, TIME_IMMEDIATE); // look if slot is free
    if (r != NIL_MSG_TMO) { // slot is free 
      //WS.println("gsm slot");
      GSMisAlive = Serial.ATsendCmd(AT_is_alive); 
      if (GSMisAlive) {
        //WS.println("gsm alive");
        val = Serial.ATsendCmdWR(AT_registered, ttext, 3);
        GSMreg = strtol((char*)ttext, NULL, 10);
        val = Serial.ATsendCmdWR(AT_signal_strength, ttext, 2);
        GSMstrength= strtol((char*)ttext, NULL, 10);
      } else { GSMreg = 4; GSMstrength = 0; }
      nilSemSignal(&GSMSem);  // Exit region.
      //WS.println("gsm slot exit");
    } else {
      //WS.println("gsm no slot");
      //GSMisAlive = 0; GSMreg = 4; GSMstrength = 0;
    }
    if (GSMlastStatus != GSMreg) {
      GSMlastStatus = GSMreg;
      _tmp[0] = 'G'; _tmp[1] = 48+GSMreg; _tmp[2] = 48+GSMstrength; _tmp[3] = 0; fifoPut(_tmp); 
      //WS.println(_tmp);
    }

    // OUT 1 & 2 handling
    if (OUTpins) {
      if (OUTpins == 1) {
        pinOUT1.write(((OUTs >> 0) & B1));
        pinOUT2.write(((OUTs >> 1) & B1));
      }
      ++OUTpins;
      if (OUTpins == 6) {
        pinOUT1.write(LOW);
        pinOUT2.write(LOW);
        OUTpins = 1;
      }
    }

  }
}
//------------------------------------------------------------------------------
// Web server thread Last one lowest priority
//
NIL_WORKING_AREA(waWebThread, 320);  
NIL_THREAD(WebThread, arg) {
  WS.println(F("WebThread started"));    
  while (TRUE) {
    nilThdSleepMilliseconds(50);
    //nilSemWait(&WEBSem);    // wait for slot
    //Serial.println(F("W+")); 
    webserver.processConnection();
    //nilSemSignal(&WEBSem);  // Exit region.
    
  }
}

// wait a few milliseconds for proper ACK to me, return true if indeed received
static bool waitForAck(byte theNodeID) {
  long now = millis();
  while (millis() - now <= ACK_TIME) {
    if (radio.ACKReceived(theNodeID))
      return true;
    nilThdSleepMilliseconds(5);
  }
  return false;
}

//------------------------------------------------------------------------------
// RFM12B thread 
//
NIL_WORKING_AREA(waRadioThread, 128);  
NIL_THREAD(RadioThread, arg) {
  uint16_t bad_crc = 0;
  uint16_t no_ack = 0;
  uint32_t received = 0;
  WS.println(F("RadioThread started"));    
  while (TRUE) {
    nilThdSleepMilliseconds(50);
    if (radio.ReceiveComplete()) {
      ++received;
        if (radio.CRCPass()) {
          WS.print(F("received[")); WS.print(radio.GetSender(), DEC);WS.print(F("] "));
          for (byte i = 0; i < *radio.DataLen; i++)
            WS.print((char)radio.Data[i]);

          if (radio.ACKRequested())
          {
            byte theNodeID = radio.GetSender();
            radio.SendACK();
            //when a node requests an ACK, respond to the ACK and also send a packet requesting an ACK
            //This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
            WS.print(F(" - ACK sent. Sending packet to node "));
            WS.print(theNodeID);
            nilThdSleepMilliseconds(10);
            radio.Send(theNodeID, "ACK MAIN", 8, true);
            WS.print(F(" - waiting for ACK..."));
            if (waitForAck(theNodeID)) WS.print(F("OK!"));
            else {
              ++no_ack;
              WS.print(F("nothing"));
            }
          }
          nilThdSleepMilliseconds(5);
        }
        else {
          ++bad_crc;
          WS.print(F("BAD-CRC"));
        }
        WS.print(F("Received: ")); WS.print(received); WS.print(F(", Bad CRC: ")); WS.print(bad_crc); WS.print(F(", Not ACKed: ")); WS.print(no_ack); 
        WS.println();
      }
  }
}

//------------------------------------------------------------------------------
// 
NIL_WORKING_AREA(waThread9, 128);  
NIL_THREAD(Thread9, arg) {
  uint8_t tt = 0;
  WS.println(F("Debug started"));  
  
  while (TRUE) {
    nilThdSleepSeconds(10);    
   
    //WS.print(nilTimeNow());
    //WS.print(F(", Bogo:"));
    //WS.println(idleCount);
    //WS.print(F(", "));
    //nilPrintStackSizes(&WS);
    //nilPrintUnusedStack(&WS);
    
    //idleCount = sendCmdToGrp(0, tt);
    //++tt; if (tt==64) tt = 0;
    idleCount = 0; // reset idle
  }
}

//------------------------------------------------------------------------------
/*
 * Threads static table, one entry per thread.  A thread's priority is
 * determined by its position in the table with highest priority first.
 *
 * These threads start with a null argument.  A thread's name is also
 * null to save RAM since the name is currently not used.
 */
 NIL_THREADS_TABLE_BEGIN()
 NIL_THREADS_TABLE_ENTRY(NULL, SensorThread, NULL, waSensorThread, sizeof(waSensorThread))
 NIL_THREADS_TABLE_ENTRY(NULL, AEThread, NULL, waAEThread, sizeof(waAEThread))
 NIL_THREADS_TABLE_ENTRY(NULL, RS485RXThread, NULL, waRS485RXThread, sizeof(waRS485RXThread))
 NIL_THREADS_TABLE_ENTRY(NULL, LoggerThread, NULL, waLoggerThread, sizeof(waLoggerThread))
 NIL_THREADS_TABLE_ENTRY(NULL, ServiceThread, NULL, waServiceThread, sizeof(waServiceThread))
 NIL_THREADS_TABLE_ENTRY(NULL, WebThread, NULL, waWebThread, sizeof(waWebThread))
 NIL_THREADS_TABLE_ENTRY(NULL, RadioThread, NULL, waRadioThread, sizeof(waRadioThread))
 NIL_THREADS_TABLE_ENTRY(NULL, Thread9, NULL, waThread9, sizeof(waThread9))

 NIL_THREADS_TABLE_END()

void setDefault(){
  conf.version = VERSION;
  conf.alr_time = 10;
  conf.arm_delay = 80;
  
  conf.tel_num[0][0] = '-';conf.tel_num[0][1] = 0;
  conf.tel_num[1][0] = '-';conf.tel_num[1][1] = 0;
  conf.tel_num[2][0] = '-';conf.tel_num[2][1] = 0;
  conf.tel_num[3][0] = '-';conf.tel_num[3][1] = 0;
  conf.tel_num[4][0] = '-';conf.tel_num[4][1] = 0;
  conf.tel_num[5][0] = '-';conf.tel_num[5][1] = 0;
  conf.tel_num[6][0] = '-';conf.tel_num[6][1] = 0;
  conf.tel_num[7][0] = '-';conf.tel_num[7][1] = 0;
  conf.tel_name[0][0] = 'D'; conf.tel_name[0][1] = 'e'; conf.tel_name[0][2] = 'f'; conf.tel_name[0][3] = 'a'; conf.tel_name[0][4] = 'u';conf.tel_name[0][5] = 'l';conf.tel_name[0][6] = 't';conf.tel_name[0][7] = 0;
  conf.tel_name[1][0] = '-';conf.tel_name[1][1] = 0;
  conf.tel_name[2][0] = '-';conf.tel_name[2][1] = 0;
  conf.tel_name[3][0] = '-';conf.tel_name[3][1] = 0;
  conf.tel_name[4][0] = '-';conf.tel_name[4][1] = 0;
  conf.tel_name[5][0] = '-';conf.tel_name[5][1] = 0;
  conf.tel_name[6][0] = '-';conf.tel_name[6][1] = 0;
  conf.tel_name[7][0] = '-';conf.tel_name[7][1] = 0;
  conf.zone_name[ 0][0] = '-';conf.zone_name[0][1] = 0;
  conf.zone_name[ 1][0] = '-';conf.zone_name[1][1] = 0;
  conf.zone_name[ 2][0] = '-';conf.zone_name[2][1] = 0;
  conf.zone_name[ 3][0] = '-';conf.zone_name[3][1] = 0;
  conf.zone_name[ 4][0] = '-';conf.zone_name[4][1] = 0;
  conf.zone_name[ 5][0] = '-';conf.zone_name[5][1] = 0;
  conf.zone_name[ 6][0] = '-';conf.zone_name[6][1] = 0;
  conf.zone_name[ 7][0] = '-';conf.zone_name[7][1] = 0;
  conf.zone_name[ 8][0] = '-';conf.zone_name[8][1] = 0;
  conf.zone_name[ 9][0] = '-';conf.zone_name[9][1] = 0;
  conf.zone_name[10][0] = '-';conf.zone_name[10][1] = 0;
  conf.zone_name[11][0] = '-';conf.zone_name[11][1] = 0;
  conf.key[0][0] = '-';conf.key[0][1] = 0;
  conf.key[1][0] = '-';conf.key[1][1] = 0;
  conf.key[2][0] = '-';conf.key[2][1] = 0;
  conf.key[3][0] = '-';conf.key[3][1] = 0;
  conf.key[4][0] = '-';conf.key[4][1] = 0;
  conf.key[5][0] = '-';conf.key[5][1] = 0;
  conf.key[6][0] = '-';conf.key[6][1] = 0;
  conf.key[7][0] = '-';conf.key[7][1] = 0;
  conf.key_name[0][0] = '-';conf.key_name[0][1] = 0;
  conf.key_name[1][0] = '-';conf.key_name[1][1] = 0;
  conf.key_name[2][0] = '-';conf.key_name[2][1] = 0;
  conf.key_name[3][0] = '-';conf.key_name[3][1] = 0;
  conf.key_name[4][0] = '-';conf.key_name[4][1] = 0;
  conf.key_name[5][0] = '-';conf.key_name[5][1] = 0;
  conf.key_name[6][0] = '-';conf.key_name[6][1] = 0;
  conf.key_name[7][0] = '-';conf.key_name[7][1] = 0;
// Zones setup
//                 |- Digital 0/ Analog 1
//                 ||- Free
//                 |||- Free
//                 ||||- Free
//                 |||||- Free
//                 ||||||- Free
//                 |||||||- Free
//                 ||||||||- Free         
//                 ||||||||         |- Auto arm zone
//                 ||||||||         |||- Auth time
//                 ||||||||         |||- 0-3x the default time
//                 ||||||||         |||||||- Group number
//                 ||||||||         |||||||- 0 .. 15
//                 ||||||||         |||||||-  
//                 ||||||||         |||||||- 
//                 ||||||||         ||||||||-  Enabled   
//                B10000000         00000000
  conf.zone[ 0] = B10111111 << 8 | B00010000; // Analog sensor 1
  conf.zone[ 1] = B10100000 << 8 | B00011000; // Analog sensor 2
  conf.zone[ 2] = B10010000 << 8 | B00001000; // Analog sensor 3
  conf.zone[ 3] = B10000000 << 8 | B00001000; // Analog sensor 4
  conf.zone[ 4] = B10100000 << 8 | B00001000; // Analog sensor 5
  conf.zone[ 5] = B10100000 << 8 | B00001000; // Analog sensor 6
  conf.zone[ 6] = B10100000 << 8 | B00001000; // Analog sensor 7
  conf.zone[ 7] = B00100000 << 8 | B00001000; // Digital sensor 1
  conf.zone[ 8] = B00100000 << 8 | B00001000; // Digital sensor 2
  conf.zone[ 9] = B00100000 << 8 | B00001000; // Digital sensor 3
  conf.zone[10] = B00100000 << 8 | B00001000; // Digital sensor 4
  conf.zone[11] = B00000000 << 8 | B00001000; // Tamper

//                  |||||- Address of authentization unit
//                  |||||  0  = master/not used
//                  |||||  1  .. 14 wired
//                  |||||  15 .. 31 wireless
//                  |||||  32 = all/any units  
//                  |||||||| - Tel. # to inform
//                  ||||||||   0 = no #
//                  ||||||||   7 = all #    
//                  ||||||||         |- Free
//                  ||||||||         ||- Free
//                  ||||||||         |||- Free
//                  ||||||||         ||||- PIR signal output 1 
//                  ||||||||         |||||- PIR signal output 2
//                  ||||||||         ||||||-  Tamper signal output 1 
//                  ||||||||         |||||||-  Tamper signal output 2
//                  ||||||||         ||||||||-  Enabled 
//                 B10000000         00000000
  conf.group[ 0] = B00000000 << 8 | B00000000; 
  conf.group[ 1] = B00000000 << 8 | B00000000; 
  conf.group[ 2] = B00000000 << 8 | B00000000; 
  conf.group[ 3] = B00000000 << 8 | B00000000; 
  conf.group[ 4] = B00000000 << 8 | B00000000; 
  conf.group[ 5] = B00000000 << 8 | B00000000; 
  conf.group[ 6] = B00000000 << 8 | B00000000; 
  conf.group[ 7] = B00000000 << 8 | B00000000; 
  conf.group[ 8] = B00000000 << 8 | B00000000; 
  conf.group[ 9] = B00000000 << 8 | B00000000; 
  conf.group[10] = B00000000 << 8 | B00000000; 
  conf.group[11] = B00000000 << 8 | B00000000; 
  conf.group[12] = B00000000 << 8 | B00000000; 
  conf.group[13] = B00000000 << 8 | B00000000; 
  conf.group[14] = B00000000 << 8 | B00000000; 
  conf.group[15] = B00000000 << 8 | B00000000; 


  conf.ee_pos = 0;

// SMS settings 
//            |- 
//            ||- 
//            |||- 
//            ||||- 
// System     |||||- Battery state
// System     ||||||- AC state
// System     |||||||- Configration saved
// System     ||||||||- System armed
// System     ||||||||         
// System     ||||||||         |- Monitoring started
// System     ||||||||         ||- ALARM - No authentication
// Alarm      ||||||||         |||- PIR
// Alarm      ||||||||         ||||- TAMPER
// Alarm      ||||||||         |||||- Undefined Alarm
// Keys       ||||||||         ||||||- Armed/Auto armed
// Keys       ||||||||         |||||||- Disarmed
// Keys       ||||||||         ||||||||- Undefined Key
  conf.SMS = B11000100 << 8 | B01000001;
  conf.global_tel_num = 0;
}  


//------------------------------------------------------------------------------
void setup() {
  
  // On start set groups
  for (int8_t i=0; i < ALR_GROUPS ; i++){
    group[i].set       = 0;
    group[i].arm_delay = 0;
  }
  // On start set zones
  for (int8_t i=0; i < ALR_ZONES ; i++){
    zone[i].last_PIR   = 0;
  }

  // RFM12B radio
  radio.Initialize(NODEID, FREQUENCY, NETWORKID);
  radio.Encrypt(KEY); //comment this out to disable encryption

  //delay(1000);
  Serial.begin(9600); // GSM modem
  WS.println(F("Start"));
  
  RS485.begin(19200, 0);
    
  eeprom_read_block((void*)&conf, (void*)0, sizeof(conf)); // Read current configuration
    if (conf.version != VERSION) setDefault();
  
  //conf.ee_pos = 0;

  // ADC speed for 16MHz, faster readings less time for idle
  // ADC_PS_64 - 250kHz
  nilAnalogPrescalar(ADC_PS_64); 
  
  // TWI 
  //twi.begin(I2C_400KHZ); 
  Wire.begin();
  Wire.speed(I2C_400KHZ);


  // Web server
  Ethernet.begin(mac, ip);
  webserver.begin();
  webserver.setDefaultCommand(&webHome);
  webserver.addCommand("form", &webHome);
  webserver.addCommand("log", &webListLog);
  webserver.addCommand("zone", &webSetZone);
  webserver.addCommand("group", &webSetGroup);
  webserver.addCommand("set", &webSetGlobal);
  webserver.addCommand("phone", &webSetPhone);
  webserver.addCommand("debug", &webDebug);
  webserver.addCommand("auth", &webSetAuth);
  webserver.addCommand("key", &webSetKey);

  // start kernel
  nilSysBegin();
  
  time_started = RTC.now();
  //RTC.adjust(DateTime(__DATE__, __TIME__));
}
//------------------------------------------------------------------------------
// Loop is the idle thread.  The idle thread must not invoke any
// kernel primitive able to change its state to not runnable.
void loop() {
  while (1) {
    // Disable interrupts to insure increment is atomic.
    nilSysLock();
    idleCount++;
    nilSysUnlock();
  }
}
//------------------------------------------------------------------------------
// Balast