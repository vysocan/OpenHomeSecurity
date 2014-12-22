// ATMEL ATMEGA1284P main board PCB v. 1.5
//
//                       +---\/---+
//    D IN 7 (D 0) PB0  1|        |40  PA0 (AI 0 / D24) Voltage
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
//               XTAL2 12|        |29  PC7 (D 23) D IN 6
//               XTAL1 13|        |28  PC6 (D 22) D IN 5
//      RX0 (D 8)  PD0 14|        |27  PC5 (D 21) D IN 4
//      TX0 (D 9)  PD1 15|        |26  PC4 (D 20) D IN 3
//      RX1 (D 10) PD2 16|        |25  PC3 (D 19) D IN 2
//      TX1 (D 11) PD3 17|        |24  PC2 (D 18) D IN 1
//      ED1 (D 12) PD4 18|        |23  PC1 (D 17) SDA
//   BAT OK (D 13) PD5 19|        |22  PC0 (D 16) SCL
//     OUT1 (D 14) PD6 20|        |21  PD7 (D 15) OUT2
//                       +--------+
//
#include <NilRTOS.h>

#include <DigitalIO.h>

#include "SPI.h"

#include "Ethernet.h"
static uint8_t mac[6] = { 0x02, 0xAA, 0xBB, 0xCC, 0x00, 0x22 };  // CHANGE THIS TO YOUR OWN UNIQUE VALUE
IPAddress  our_ip( 10,  10,  10, 184);  // CHANGE THIS TO MATCH YOUR HOST NETWORK
IPAddress gateway( 10,  10,  10, 254);  // CHANGE THIS TO MATCH YOUR HOST NETWORK
IPAddress  subnet(255, 255, 255,   0);  // CHANGE THIS TO MATCH YOUR HOST NETWORK
// NTP
IPAddress  timeIP( 81,   0, 239, 181);  // ntp.globe.cz 
#include <EthernetUdp.h>                      //
#define NTP_LOCAL_PORT         9999           // local port to listen for UDP packets
#define NTP_PACKET_SIZE        48             // NTP time stamp is in the first 48 bytes of the message
#define NTP_SECS_YR_1900_2000 (3155673600UL)
#define NTP_TIME_ZONE          2              // Central European Time
#define NTP_USELESS_BYTES      40             // Set useless to 32 for speed; set to 40 for accuracy.
#define NTP_POLL_INTV          100            // poll response this many ms
#define NTP_POLL_MAX           20             // poll response up to this many times
const long ntpFirstFourBytes = 0xEC0600E3;    // NTP request header
EthernetUDP udp;                              // A UDP instance to let us send and receive packets over UDP
// WebServer
#define WEBDUINO_FAVICON_DATA     "" // disable favicon
#define WEBDUINO_SERIAL_DEBUGGING 0
#define PREFIX                    ""
#include "WebServer.h"
WebServer webserver(PREFIX, 80);
#include "html.h"
// no-cost stream operator as described at 
// http://sundial.org/arduino/?page_id=119
template<class T>
inline Print &operator <<(Print &obj, T arg)
{ obj.print(arg); return obj; }

// MQTT 
#include <PubSubClient.h>
byte MQTTIP[]     = { 10,  10,  10, 133};
char str_MQTT_dev[] = "OHS"; // device name
EthernetClient MQTTClient;

#include <RFM12B.h>
// You will need to initialize the radio by telling it what ID it has and what network it's on
// The RADIO_NODEID takes values from 1-127, 0 is reserved for sending broadcast messages (send to all nodes)
// The Network ID takes values from 0-255
// By default the SPI-SS line used is D10 on Atmega328. You can change it by calling .SetCS(pin) where pin can be {8,9,10}
#define RADIO_NODEID      1             //network ID used for this unit
#define RADIO_NETWORKID   255           //the network ID we are on
#define RADIO_FREQUENCY   RF12_868MHZ
#define RADIO_ACK_TIME    20
#define RADIO_UNIT_OFFSET 15
// Need an instance of the Radio Module
RFM12B radio;

// RTOS friendly Analog
#include <NilAnalog.h>

// TWI
#include <TwiMaster.h>
#include <Eeprom24C512.h>
#define EEPROM_ADDRESS   0x50
#define EEPROM_MESSAGE   16
#define EEPROM_PAGE_SIZE 128
#define EEPROM_SIZE      65536 // 512kib
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

#include <NilGSM.h>
#define Serial GSM  // redefine GSM as standard Serial 0
char sms_text[100];

// NilRTOS FIFO
#include <NilFIFO.h>
// Type for a data record.
struct log_event_t {
  char text[17];
};
NilFIFO<log_event_t, 9> log_fifo;

#define WEB_SERIAL_DEBUGGING 2
#if WEB_SERIAL_DEBUGGING 
#include <WebSerial.h>
#endif 

// Global configuration and default setting
#include <conf.h>

struct alarm_event_t {
  uint8_t zone;
  char    type;
};
NilFIFO<alarm_event_t, 6> alarm_fifo;

struct zone_t {
  systime_t last_PIR = 0;
  systime_t last_OK  = 0;
};
zone_t zone[ALR_ZONES];

struct group_t {  
  //       |- Free
  //       ||- Free
  //       |||- Free
  //       ||||- Free
  //       |||||- Free
  //       ||||||-  Waiting for authorization
  //       |||||||-  Alarm
  //       ||||||||-  Armed
  //      B00000000
  uint8_t setting;
  uint8_t arm_delay;
};
group_t group[ALR_GROUPS];

// Dynamic sensors
struct sensor_t {
  uint8_t address;
  char    type;
  uint8_t number;
//          |- MQTT publish
//          ||- Free    
//          |||- Free
//          |||||||- Group number
//          |||||||- 0 .. 15
//          |||||||-  
//          |||||||- 
//          ||||||||-  Enabled   
//          00000000
  uint8_t setting;
  float   value;
  systime_t last_OK = 0;
};
#define SENSORS 16
sensor_t sensor[SENSORS];
volatile uint8_t sensors = 0;
NilFIFO<sensor_t, 5> sensor_fifo;

// Float conversion 
union u_tag {
  byte b[4]; 
  float fval;
} u;

// Dynamic Authorization units
struct unit_t {
  uint8_t address;
  char    type;
  uint8_t number;
//          |- Free
//          ||- Free
//          |||- Free
//          |||||||- Group number
//          |||||||- 0 .. 15
//          |||||||-  
//          |||||||- 
//          ||||||||-  Enabled   
//          00000000
  uint8_t setting;
};
#define AUTH_UNITS 16
unit_t unit[AUTH_UNITS];
volatile uint8_t units = 0;

// Global variables
volatile uint32_t idleCount;                  // temporary free ticks
volatile uint8_t  ACState = 0;
volatile uint16_t BatteryLevel = 460;         // 13.8V
volatile uint8_t  OUTs = 0;                   // Output pins
volatile uint8_t  MQTTState = 0;
char     last_key[KEY_LEN+1] = "UUUUUUUU";

char tmp[17];       // for logger 
char _tmp[4];       // for logger 

// GSM modem 
uint8_t GSMisAlive = 0, GSMreg = 4, GSMstrength = 0, GSMsetSMS = 0;

// tmp
uint8_t n = 0; 

// Pins setting
DigitalPin<18> pinIN1(INPUT);
DigitalPin<19> pinIN2(INPUT);
DigitalPin<20> pinIN3(INPUT);
DigitalPin<21> pinIN4(INPUT);
DigitalPin<22> pinIN5(INPUT);
DigitalPin<23> pinTAMPER(INPUT);    // Tamper
DigitalPin<0>  pinIN7(INPUT);
DigitalPin<3>  pinAC_OFF(INPUT); // The signal turns to be "High" when the power supply turns OFF
DigitalPin<12> pinDE1(OUTPUT);
DigitalPin<14> pinOUT1(OUTPUT);
DigitalPin<15> pinOUT2(OUTPUT);
DigitalPin<13> pinBAT_OK(INPUT); // The signal is "Low" when the voltage of battery is under 11V 

// Declare and initialize a semaphore for limiting access to a region.
SEMAPHORE_DECL(ADCSem, 1);   // one slot only
SEMAPHORE_DECL(TWISem, 1);   // one slot only
SEMAPHORE_DECL(GSMSem, 1);   // one slot only
SEMAPHORE_DECL(RFMSem, 1);   // one slot only


// *********************************************************************************
// F U N C T I O N S                F U N C T I O N S              F U N C T I O N S      
// *********************************************************************************

// Put string into a fifo log
void pushToLog(char *what){ 
  log_event_t* p = log_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
  if (p == 0) return; // Continue if no free space.
  
  nilSemWait(&TWISem);     // wait for slot
  time_now = RTC.now();
  nilSemSignal(&TWISem);   // Exit region.
  
  p->text[0] = 0;
  strcat (p->text, time_now.timestamp());
  strcat (p->text, "?");
  strcat (p->text, what);

  //Serial.println(p->text);
  log_fifo.signalData();  // Signal idle thread data is available.
}

// wait a few milliseconds for proper ACK to me, return true if indeed received
static bool waitForAck(byte theRADIO_NODEID) {
  uint8_t _counter = 0;
  do {
    _counter++; WS.print('.');
    nilThdSleepMilliseconds(5);
  } while (!(radio.ACKReceived(theRADIO_NODEID)) && _counter < RADIO_ACK_TIME);
  if (_counter < RADIO_ACK_TIME) return true;     
  else return false;
}

// Send a command to wired RS485 unit
uint8_t sendCmd(uint8_t unit, uint8_t cmd){ 
  uint8_t _resp = 0;
  if (unit <= RADIO_UNIT_OFFSET) {
    TX_msg.address = unit;
    TX_msg.ctrl = FLAG_CMD;
    TX_msg.data_length = cmd;
    _resp = RS485.msg_write(&TX_msg);
  }
  if (unit >= RADIO_UNIT_OFFSET) { // allow command to be send to radio 0/broadcast
    char _cmd[2];
    _cmd[0] = 'C'; _cmd[1] = cmd;
    nilSemWait(&RFMSem); 
    if (unit-RADIO_UNIT_OFFSET == 0) {
      radio.Send(unit-RADIO_UNIT_OFFSET, _cmd, 2, false);
      _resp = 1;
    }
    else {
      radio.Send(unit-RADIO_UNIT_OFFSET, _cmd, 2, true);
      _resp = waitForAck(unit-RADIO_UNIT_OFFSET);
    }
    nilSemSignal(&RFMSem);  // Exit region.
  }
  return _resp;
}

// Send data to wired RS485 unit
uint8_t sendData(uint8_t unit, char *data, uint8_t length){ 
  uint8_t _resp = 0;
  #if WEB_SERIAL_DEBUGGING 
    WS.print(F("Data to unit: ")); WS.print(unit);
    WS.print(F(", len: ")); WS.println(length);
    for (uint8_t i=0; i < length; i++){
      //WS.print((uint8_t)*data++,HEX);
      WS.print((uint8_t)data[i],HEX);
      WS.print(F(" "));
    }
    WS.println();
  #endif
  if (unit <= RADIO_UNIT_OFFSET) {
    TX_msg.address = unit;
    TX_msg.ctrl = FLAG_DTA;
    TX_msg.data_length = length;
    memcpy(TX_msg.buffer, data, length);
    return RS485.msg_write(&TX_msg);
  } else {
    nilSemWait(&RFMSem); 
    radio.Send(unit-RADIO_UNIT_OFFSET, data, length, true);
    _resp = waitForAck(unit-RADIO_UNIT_OFFSET);
    nilSemSignal(&RFMSem);  // Exit region.
    return _resp;
  }
}

// Send a command to all members of a group
uint8_t sendCmdToGrp(uint8_t grp, uint8_t cmd) {
  int8_t  _resp, _try;
  uint8_t _cnt = 0;
  char _cmd[2];
  _cmd[0] = 'C'; _cmd[1] = cmd;
  // Go throug all units
  //WS.print("Grp cmd: ");
  //WS.print(cmd);
  //WS.print(", unit: ");
  for (int8_t i=0; i < units; i++){
    if (unit[i].setting & B1) {                       // Auth. unit is enabled ?
      if (((unit[i].setting >> 1) & B1111) == grp) {  // Auth. unit belong to group
        if (unit[i].address <= RADIO_UNIT_OFFSET) {
          TX_msg.address = unit[i].address;
          TX_msg.ctrl = FLAG_CMD;
          TX_msg.data_length = cmd;
          _try = 0;
          do {
            _resp = RS485.msg_write(&TX_msg);
            _try++;
            nilThdSleepMilliseconds(10);
          } while (_resp != 1 || _try < 5);
          if (_resp) _cnt++;
          //WS.print(unit[i].address); WS.print(":"); WS.print(_resp); WS.print(", ");
        } else {
          nilSemWait(&RFMSem);
          radio.Send(unit[i].address-RADIO_UNIT_OFFSET, _cmd, 2, true);
          if (waitForAck(unit[i].address-RADIO_UNIT_OFFSET)) _cnt++;
          nilSemSignal(&RFMSem);  // Exit region.
        }
      }
    }
  }
  //WS.println();
  return _cnt;
}

// Clear EEPROM log
void clearLog(){
  //nilSysLock(); // Lock RTOS  
  nilSemWait(&TWISem);     // wait for slot
  for (uint16_t i = 0; i < (EEPROM_SIZE/EEPROM_MESSAGE); i++){
    eeprom.writeBytes(i*EEPROM_MESSAGE,EEPROM_MESSAGE, "000101000000|XXX");
    nilThdSleepMilliseconds(5);  // wait for eeprom or we lose very frequent entries
  }
  conf.ee_pos = 0;
  nilSemSignal(&TWISem);   // Exit region.
  //nilSysUnlock(); // unlock RTOS
  eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration
}

void saveConf(){
  eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration
  _tmp[0] = 'S'; _tmp[1] = 'C'; _tmp[2] = 'W'; _tmp[3] = 0; pushToLog(_tmp);
}

unsigned long GetNTPTime(UDP &udp){
  //udpInited = udp.begin(NTP_LOCAL_PORT); // open socket on arbitrary port
  if (!udp.begin(NTP_LOCAL_PORT)) return 0; // Fail if WiFiUdp.begin() could not init a socket
  udp.flush();              // Clear received data from possible stray received packets
  
  // Send an NTP request
  if (! (udp.beginPacket(timeIP, 123)      // 123 is the NTP port
    && udp.write((byte *)&ntpFirstFourBytes, 48) == 48
      && udp.endPacket())) {
        udp.stop();
        return 0;       // sending request failed
    }

  // Wait for response
  uint8_t _pktLen, _pool;
  for (_pool=0; _pool<NTP_POLL_MAX; _pool++) {
    if ((_pktLen = udp.parsePacket()) == 48) break;
    nilThdSleepMilliseconds(NTP_POLL_INTV);
  }
  if (_pktLen != 48) {
    udp.stop();
    return 0;       // no correct packet received
  }

  // Read and discard the first useless bytes
  for (byte i = 0; i < NTP_USELESS_BYTES; ++i) {
    udp.read();
  }

  // Read the integer part of sending time
  unsigned long time  = (unsigned long)udp.read() << 24;
                time |= (unsigned long)udp.read() << 16;
                time |= (unsigned long)udp.read() << 8;
                time |= (unsigned long)udp.read();

  // Round to the nearest second if we want accuracy
  // The fractionary part is the next byte divided by 256: if it is
  // greater than 500ms we round to the next second; we also account
  // for an assumed network delay of 50ms, and (0.5-0.05)*256=115;
  // additionally, we account for how much we delayed reading the packet
  // since its arrival, which we assume on average to be pollIntv/2.
  time += (udp.read() > 115 - _pool/2);
  udp.flush();  // Discard the rest of the packet
  udp.stop();
  return time - NTP_SECS_YR_1900_2000 + NTP_TIME_ZONE * 3600; // convert NTP time to seconds after 2000
}


// MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}
PubSubClient client(MQTTIP, 1883, callback, MQTTClient);


// *********************************************************************************
// W E B   P A G E S              W E B   P A G E S                W E B   P A G E S  
// *********************************************************************************

#if WEB_SERIAL_DEBUGGING 
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
#endif

void webHome(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (type == WebServer::POST) {
    bool repeat;
    char name[2], value[16];
    do {
      repeat = server.readPOSTparam(name, 2, value, 16);
      if (name[0] == 'T') { // NTP Sync
        time_now = GetNTPTime(udp);  
        if (time_now.get() > 0) RTC.adjust(time_now.get());
      }
    } while (repeat);
    server.httpSeeOther(PREFIX "/");
  } else {   
    uint32_t voltage = (BatteryLevel * 3222581) / 1071429; // voltage divider, and avoiding float
    if (voltage < 100) voltage = 0; // no sense to messuer lower voltage
    char _tmp_itoa[5]; itoa(voltage, _tmp_itoa, 10); 
    nilSemWait(&TWISem);     // wait for slot
    time_now = RTC.now();
    nilSemSignal(&TWISem);   // Exit region.

    server.httpSuccess();
    server.printP(htmlHead);
    server.printP(html_h1); server.printP(text_System); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_table); 
    server.printP(html_tr); server.printP(html_td);
    server.printP(text_Time); server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp); server.print((char*)time_now.formatedDateTime());
    server.printP(html_e_td); server.printP(html_e_tr);

    server.printP(html_tr); server.printP(html_td); server.printP(html_e_td); server.printP(html_td);
    server.printP(html_form_s); server << PREFIX "/"; server.printP(html_form_e);
    server.printP(html_F_GetNTP); // Clear
    server.printP(html_e_form);
    server.printP(html_e_td); server.printP(html_e_tr);

    server.printP(html_tr); server.printP(html_td);
    server.printP(text_Started); server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp); server.print((char*)time_started.formatedDateTime());        
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td);
    server.printP(text_Uptime); server.printP(html_e_td); server.printP(html_td);
    time_now = (time_now.get()-time_started.get());
    server.printP(text_sesp); server.print((char*)time_now.formatedUpTime());
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td);
    server.printP(text_PwrSp); server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp); pinAC_OFF.read() ? server.printP(text_Off) : server.printP(text_On); server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td);
    server.printP(text_Battery); server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp); pinBAT_OK.read() ? server.printP(text_OK) : server.printP(text_low); server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td);
    server.printP(text_Voltage); server.printP(html_e_td); server.printP(html_td); 
    server.printP(text_sesp); 
    if (voltage > 1000) { server << _tmp_itoa[0] << _tmp_itoa[1] << "." << _tmp_itoa[2] << _tmp_itoa[3];}
    else                { server << _tmp_itoa[0] << "." << _tmp_itoa[1] << _tmp_itoa[2] << _tmp_itoa[3];}
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_e_table);
    
    server.printP(html_h1); server.printP(text_GSM); server.printP(text_space); server.printP(text_modem); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_td);
    server.printP(text_GSM); server.printP(text_space); server.printP(text_modem); server.printP(text_space); server.printP(text_is); server.printP(text_space); server.printP(text_connected); 
    server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    GSMisAlive ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td); server.printP(html_e_tr);
    
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
    server << (GSMstrength*3); server.printP(text_percent); 
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_e_table);


    server.printP(html_e_p);
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
      if (name[0] == 'C') clearLog();
    } while (repeat);
    server.httpSeeOther(PREFIX "/log");
  } else {  
    server.httpSuccess();
    server.printP(htmlHead);
    server.printP(html_h1); server.printP(text_Log); server.printP(html_e_h1); server.printP(html_p);
    server.printP(html_form_s); server << PREFIX "/log"; server.printP(html_form_e);
    server.printP(html_F_Clear); // Clear
    server.printP(html_e_p); server.printP(html_p);
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_th); server.printP(text_hash);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Date); 
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Message);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_SMS);
    server.printP(html_e_th); server.printP(html_e_tr);
    for (uint8_t i = 0; i < 21; ++i){
      if ( (i & 0x01) == 0) server.printP(html_tr_od);
      else                  server.printP(html_tr_ev);
      server.printP(html_td);
      // Get data from extrenal EEPROM
      nilSemWait(&TWISem);     // wait for slot
      eeprom.readBytes((uint16_t)(ses_eeprom_add + (i*EEPROM_MESSAGE)),EEPROM_MESSAGE, tmp);
      //    nilThdSleepMilliseconds(10); //??? wait for eeprom
      nilSemSignal(&TWISem);   // Exit region.
      if ((uint16_t)(ses_eeprom_add/EEPROM_MESSAGE + i) > (EEPROM_SIZE/EEPROM_MESSAGE) - 1) 
       server << (uint16_t)(ses_eeprom_add/EEPROM_MESSAGE + i - (EEPROM_SIZE/EEPROM_MESSAGE) + 1); 
     else server << (uint16_t)(ses_eeprom_add/EEPROM_MESSAGE + i + 1);
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
        case 'S': // System
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
            case 'C': server.printP(text_Configuration); server.printP(text_space); server.printP(text_saved);break; // conf. saved
            case 'Z': server.printP(text_group); server.printP(text_space);
            server << tmp[15]-48+1; server.printP(text_spdashsp); server << conf.group_name[tmp[15]-48];
            server.printP(text_space); server.printP(text_monitoring); server.printP(text_space); server.printP(text_started);
                      break; // system armed
            case 'S': server.printP(text_Monitoring); server.printP(text_space); server.printP(text_started); break; // monitoring strted
            case 's': server.printP(text_started); break;   // boot
            case 'X': server.printP(text_ALARM); break;     // alarm
            case 'M': 
              server.printP(text_MQTT); server.printP(text_space);
              switch(tmp[15]){
                case 'F': server.printP(text_network); server.printP(text_space); server.printP(text_failed); break;
                case 'O': server.printP(text_network); server.printP(text_space); server.printP(text_OK); break;
                default:  server.printP(text_undefined); break; // unknown  
              }
            break;
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
        case 'A': // Authentication
          server.printP(text_Authentication); server.printP(text_sesp); 
          if (tmp[14] <  'a') { server.printP(text_key); server.printP(text_space); }
          if (tmp[14] == 'A' || tmp[14] == 'D' || tmp[14] == 'F') { server << conf.key_name[tmp[15]-48]; server.printP(text_space); }
          switch(tmp[14]){
            case 'D': server.printP(text_disarmed); break;
            case 'A': server.printP(text_armed); break;
            case 'U': server.printP(text_undefined); break;
            case 'F': server.printP(text_is); server.printP(text_space); server.printP(text_disabled); break;
            case 'a': server.printP(text_auto); server.printP(text_space); server.printP(text_armed);
            server.printP(text_space); server.printP(text_group); server.printP(text_space); server << conf.zone_name[tmp[15]-48];
            break;
            case 'o': server.printP(text_zone); server.printP(text_space); server << conf.zone_name[tmp[15]-48];
            server.printP(text_space); server.printP(text_is); server.printP(text_space); server.printP(text_open);
            break;
            default: break;
          }
            /*
            if (tmp[15] != 'F') { // print no info when unknown key
              server.printP(text_space);  server.printP(text_group); server.printP(text_space); server.print(conf.group_name[tmp[16]-48]);
            }
            */
        break;    
        case 'M': //GSM modem
          server.printP(text_GSM); server.printP(text_space); server.printP(text_network); server.printP(text_sesp);
          switch(tmp[14]){
            case '0' : server.printP(text_nr); break;
            case '1' : server.printP(text_rh); break;
            case '2' : server.printP(text_nrs); break;
            case '3' : server.printP(text_rd); break;
              // case 4 : server.printP(text_unk); break;
            case '5' : server.printP(text_rr); break;
            default : server.printP(text_unk); break;
          }
          server << ", strength " << (tmp[15]-48)*3 << "%";
        break;
        case 'U': // remote unit
          server.printP(text_Remote); server.printP(text_space); server.printP(text_unit); server.printP(text_sesp);
          server.printP(text_address); server.printP(text_space); server << tmp[15]-48; server.printP(text_space);
          switch(tmp[14]){
            case 'F' : server.printP(text_is); server.printP(text_space); server.printP(text_disabled); break;
            case 'R' : server.printP(text_registered); break;
            case 'r' : server.printP(text_re); server.printP(text_registered); break;
            default : server.printP(text_unk); break;
          }
        break;
        case 'a' ... 'x': // remote sensors
          switch(tmp[13]){
            case 't': server.printP(text_Temperature); break;
            case 'h': server.printP(text_Humidity); break;
            case 'p': server.printP(text_Pressure); break;
            default : server.printP(text_unk); break;
          }
          server.printP(text_space); server.printP(text_sensor); server.printP(text_sesp);
          server.printP(text_address); server.printP(text_space); server << tmp[15]-48; server.printP(text_space);
          switch(tmp[14]){
            case 'F' : server.printP(text_is); server.printP(text_space); server.printP(text_disabled); break;
            case 'R' : server.printP(text_registered); break;
            case 'r' : server.printP(text_re); server.printP(text_registered); break;
            default : server.printP(text_unk); break;
          }
        break;
        case 'G': // Groups
          server.printP(text_Group); server.printP(text_sesp);
          server << tmp[15]-48+1; server.printP(text_spdashsp); server << conf.group_name[tmp[15]-48]; server.printP(text_space);
          switch(tmp[14]){
            case 'F' : server.printP(text_is); server.printP(text_space); server.printP(text_disabled); break;
            default : server.printP(text_unk); break;
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
      //Serial.print(name);zone[i].last_PIR = nilTimeNow();    // update current timestamp
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
        case 's': // still open alarm
          if (value[0] == '0') conf.zone[webZone] &= ~(1 << 8);
          else conf.zone[webZone] |= (1 << 8);
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
        case 'e': saveConf(); break; // save to EEPROM
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
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Auto); server.printP(text_space); server.printP(text_arm);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Open); server.printP(text_space); server.printP(text_alarm);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Delay);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Group);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Last); server.printP(text_space); server.printP(text_alarm);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Last); server.printP(text_space); server.printP(text_OK);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Status);
      server.printP(html_e_th); server.printP(html_e_tr);
      for (uint8_t i = 0; i < ALR_ZONES; ++i) {
        if ( (i & 0x01) == 0) server.printP(html_tr_od);
        else                  server.printP(html_tr_ev);
        server.printP(html_td); 
        server << i+1; server.printP(text_dot); server.printP(html_e_td); server.printP(html_td);
        (conf.zone[i] >> 15) ? server.printP(text_analog) : server.printP(text_digital); server.printP(html_e_td); server.printP(html_td);
        server << conf.zone_name[i]; server.printP(html_e_td); server.printP(html_td);
        (conf.zone[i] & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td); server.printP(html_td);
        ((conf.zone[i] >> 7) & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td); server.printP(html_td);
        ((conf.zone[i] >> 8) & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td); server.printP(html_td);
        server << (((conf.zone[i] >> 5) & B11)*conf.alr_time); server.printP(text_space); server.printP(text_sec); server.printP(html_e_td); server.printP(html_td);
        server << ((conf.zone[i] >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((conf.zone[i] >> 1) & B1111)];
        server.printP(html_e_td); server.printP(html_td);
        if ((conf.zone[i] & B1)) { server << (nilTimeNow() - zone[i].last_PIR)/NIL_CFG_FREQUENCY; }
        else                     { server.printP(text_spdashsp); }
        server.printP(text_space); server.printP(text_sec);
        server.printP(html_e_td); server.printP(html_td);
        if ((conf.zone[i] & B1)) { server << (nilTimeNow() - zone[i].last_OK)/NIL_CFG_FREQUENCY; }
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
              case 7:  pinIN1.read() ? server.printP(text_ALARM) : server.printP(text_OK); break;
              case 8:  pinIN2.read() ? server.printP(text_ALARM) : server.printP(text_OK); break;
              case 9:  pinIN3.read() ? server.printP(text_ALARM) : server.printP(text_OK); break;
              case 10: pinIN4.read() ? server.printP(text_ALARM) : server.printP(text_OK); break;
              case 11: pinIN5.read() ? server.printP(text_ALARM) : server.printP(text_OK); break;
              case 12: pinTAMPER.read() ? server.printP(text_ALARM) : server.printP(text_OK); break;
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
      server.printP(html_tr); server.printP(html_td); server.printP(text_Zone); server.printP(text_space); server.printP(text_is); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
      server.radioButton("o", text_1, text_On, conf.zone[webZone] & B1);
      server.radioButton("o", text_0, text_Off, !(conf.zone[webZone] & B1));
      server.printP(html_e_td); server.printP(html_e_tr);
      server.printP(html_tr); server.printP(html_td); server.printP(text_Auto); server.printP(text_space); server.printP(text_arm); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
      server.radioButton("a", text_1, text_On, conf.zone[webZone] >> 7 & B1);
      server.radioButton("a", text_0, text_Off, !(conf.zone[webZone] >> 7 & B1));
      server.printP(html_e_td); server.printP(html_e_tr);
      server.printP(html_tr); server.printP(html_td); server.printP(text_Open); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
      server.radioButton("s", text_1, text_On, conf.zone[webZone] >> 8 & B1);
      server.radioButton("s", text_0, text_Off, !(conf.zone[webZone] >> 8 & B1));
      server.printP(html_e_td); server.printP(html_e_tr);
      server.printP(html_tr); server.printP(html_td); server.printP(text_Authentication); server.printP(text_space); server.printP(text_delay); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
      server.radioButton("d", text_0, text_0, !((conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
      server.radioButton("d", text_1, text_1, (!(conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
      server.radioButton("d", text_2, text_2, ((conf.zone[webZone] >> 6 & B1) & !(conf.zone[webZone] >> 5 & B1)));
      server.radioButton("d", text_3, text_3, ((conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
      server.printP(html_e_td); server.printP(html_e_tr);
      server.printP(html_tr); server.printP(html_td); server.printP(text_Group); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
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
        case 'e': saveConf(); break; // save to EEPROM         
        }
      } while (repeat);
      server.httpSeeOther(PREFIX "/group");
    } else {
      server.httpSuccess();
      server.printP(htmlHead);
      server.printP(html_h1); server.printP(text_Group); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  server.printP(html_p);
      server.printP(html_table);
      server.printP(html_tr); server.printP(html_th); server.printP(text_hash);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Name); 
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Enabled);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Armed);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Authentication);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Delay);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Zone); server.printP(text_s);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Authentication); server.printP(text_space); server.printP(text_unit); server.printP(text_s);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Sensor); server.printP(text_s);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Phone); server.printP(text_s);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Alarm);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Tamper);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Status);
      server.printP(html_e_th); server.printP(html_e_tr);
      for (uint8_t i = 0; i < ALR_GROUPS; ++i) {
        if ( (i & 0x01) == 0) server.printP(html_tr_od);
        else                  server.printP(html_tr_ev);
        server.printP(html_td); 
        server << i+1; server.printP(text_dot); server.printP(html_e_td); server.printP(html_td);
        server << conf.group_name[i]; server.printP(html_e_td); server.printP(html_td);
        (conf.group[i] & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td); server.printP(html_td);
        (group[i].setting & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td); server.printP(html_td);
        ((group[i].setting >> 2) & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td); server.printP(html_td);
        server << group[i].arm_delay/4; server.printP(text_space); server.printP(text_sec); server.printP(html_e_td); server.printP(html_td);
        n = 0;
        for (uint8_t ii = 0; ii < ALR_ZONES; ++ii) {
          if ((((conf.zone[ii] >> 1) & B1111) == i) && (conf.zone[ii] & B1)) {
            if (n) { server.printP(text_comma); server.printP(text_space); }
            server << ii+1; n = 1;
          }
        }
        server.printP(html_e_td); server.printP(html_td);
        n = 0;
        for (uint8_t ii = 0; ii < units; ++ii) {      
          if ((((unit[ii].setting >> 1) & B1111) == i) && (unit[ii].setting & B1)) { 
            if (n) { server.printP(text_comma); server.printP(text_space); }
            server << unit[ii].type; server.printP(text_semic);
            if (unit[ii].address < RADIO_UNIT_OFFSET) { server << "W:" << unit[ii].address; }
            else                                      { server << "R:" << unit[ii].address-RADIO_UNIT_OFFSET; }
            server.printP(text_semic); server << unit[ii].number; n = 1;
          }
        }
        server.printP(html_e_td); server.printP(html_td);
        n = 0;
        for (uint8_t ii = 0; ii < SENSORS; ++ii) {      
          if (((sensor[ii].setting >> 1) & B1111) == i) { 
            if (sensor[ii].address && (sensor[ii].setting & B1)) {
              if (n) { server.printP(text_comma); server.printP(text_space); }
              server << sensor[ii].type; server.printP(text_semic);
              if (sensor[ii].address < RADIO_UNIT_OFFSET) { server << "W:" << sensor[ii].address; }
              else                                        { server << "R:" << sensor[ii].address-RADIO_UNIT_OFFSET; }
              server.printP(text_semic); server << sensor[ii].number;
              n = 1;
            }
          }
        }
        server.printP(html_e_td); server.printP(html_td);
        n = 0;
        for (uint8_t ii = 0; ii < NUM_OF_PHONES; ++ii) {
          if ((((conf.tel[ii] >> 1) & B1111) == i) && (conf.tel[ii] & B1)){
            if (n) { server.printP(text_comma); server.printP(text_space); }
            server << ii+1; server.printP(text_semic); server << conf.tel_name[ii];
            n = 1;
          }
        }
        server.printP(html_e_td); server.printP(html_td);        
        if ((conf.group[i] >> 4) & B1) { server.printP(text_OUT1); server.printP(text_space); }
        if ((conf.group[i] >> 3) & B1) server.printP(text_OUT2);
        server.printP(html_e_td); server.printP(html_td);
        if ((conf.group[i] >> 2) & B1) { server.printP(text_OUT1); server.printP(text_space); }
        if ((conf.group[i] >> 1) & B1) server.printP(text_OUT2);
        server.printP(html_e_td); server.printP(html_td);
        (group[i].setting >> 1 & B1) ? server.printP(text_ALARM) : server.printP(text_OK);
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
      server.printP(html_tr); server.printP(html_td);
      server.printP(text_Group); server.printP(text_space); server.printP(text_is); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
      server.radioButton("o", text_1, text_On, conf.group[webGroup] & B1);
      server.radioButton("o", text_0, text_Off, !(conf.group[webGroup] & B1));
      server.printP(html_e_td); server.printP(html_e_tr);
      server.printP(html_tr); server.printP(html_td);
      server.printP(text_Alarm); server.printP(text_space); server.printP(text_trigger); server.printP(text_s); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
      server.printP(text_OUT1); server.printP(text_sesp);
      server.radioButton("p", text_1, text_On, conf.group[webGroup] >> 4 & B1);
      server.radioButton("p", text_0, text_Off, !(conf.group[webGroup] >> 4 & B1));
      server.printP(html_br);  server.printP(text_sesp); server.printP(text_OUT2); server.printP(text_sesp);
      server.radioButton("q", text_1, text_On, conf.group[webGroup] >> 3 & B1);
      server.radioButton("q", text_0, text_Off, !(conf.group[webGroup] >> 3 & B1));
      server.printP(html_e_td); server.printP(html_e_tr);
      server.printP(html_tr); server.printP(html_td);
      server.printP(text_Tamper); server.printP(text_space); server.printP(text_trigger); server.printP(text_s); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
      server.printP(text_OUT1); server.printP(text_sesp);
      server.radioButton("t", text_1, text_On, conf.group[webGroup] >> 2 & B1);
      server.radioButton("t", text_0, text_Off, !(conf.group[webGroup] >> 2 & B1));
      server.printP(html_br);  server.printP(text_sesp);
      server.printP(text_OUT2); server.printP(text_sesp);
      server.radioButton("y", text_1, text_On, conf.group[webGroup] >> 1 & B1);
      server.radioButton("y", text_0, text_Off, !(conf.group[webGroup] >> 1 & B1));
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
            conf.key[webKey][ii] = value[ii*2] << 4;
            conf.key[webKey][ii] = conf.key[webKey][ii] + value[ii*2+1];
          } 
          conf.key[webKey][8] = 0;
        break;
        case 'n': // key
          strncpy (conf.key_name[webKey], value, 16);
          conf.key_name[webKey][15] = 0;
        break;
        case 'e': saveConf(); break; // save to EEPROM          
        }
      } while (repeat);
      server.httpSeeOther(PREFIX "/key");
    } else {
      server.httpSuccess();
      server.printP(htmlHead);
      server.printP(html_h1); server.printP(text_Key); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1); server.printP(html_p);

      server.printP(html_table);
      server.printP(html_tr); server.printP(html_th); server.printP(text_hash);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Name);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Key);
      server.printP(html_e_th); server.printP(html_e_tr);
      for (uint8_t i = 0; i < NUM_OF_KEYS; ++i) {
        if ( (i & 0x01) == 0) server.printP(html_tr_od);
        else                  server.printP(html_tr_ev);
        server.printP(html_td); 
        server << i+1; server.printP(text_dot); server.printP(html_e_td); server.printP(html_td);
        server << conf.key_name[i];
        server.printP(html_e_td); server.printP(html_td);
      for (uint8_t ii = 0; ii < KEY_LEN; ++ii) { // Format the key to nice HEX 
        tmp[ii*2]   = conf.key[i][ii] >> 4 & B1111;
        tmp[ii*2+1] = conf.key[i][ii] & B1111;
        if (tmp[ii*2] > 9) tmp[ii*2] = tmp[ii*2] + 'A' - 10; 
        else tmp[ii*2] = tmp[ii*2] + '0';
        if (tmp[ii*2+1] > 9) tmp[ii*2+1] = tmp[ii*2+1] + 'A' - 10;
        else tmp[ii*2+1] = tmp[ii*2+1] + '0';
      }
      tmp[16] = 0; server.printP(html_pre); server << tmp; server.printP(html_e_pre);
      server.printP(html_e_td); server.printP(html_e_tr);
    }
    server.printP(html_e_table); server.printP(html_e_p);

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
    server.printP(html_tr); server.printP(html_td); server.printP(text_Last); server.printP(text_space); server.printP(text_key); server.printP(html_e_td); server.printP(html_td);
    for (uint8_t ii = 0; ii < KEY_LEN; ++ii) { // Format the key to nice HEX 
      tmp[ii*2]   = last_key[ii] >> 4 & B1111;
      tmp[ii*2+1] = last_key[ii] & B1111;
      if (tmp[ii*2] > 9) tmp[ii*2] = tmp[ii*2] + 'A' - 10; 
      else tmp[ii*2] = tmp[ii*2] + '0';
      if (tmp[ii*2+1] > 9) tmp[ii*2+1] = tmp[ii*2+1] + 'A' - 10;
      else tmp[ii*2+1] = tmp[ii*2+1] + '0';
    }
    tmp[16] = 0;
    server.printP(text_sesp); server << tmp; server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_e_table); server.printP(html_e_p);    
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
        case 'a': // enable tel
          if (value[0] == '0') conf.tel[webTel] &= ~(1 << 5);
          else conf.tel[webTel] |= (1 << 5);
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
        case 'e': saveConf(); break; // save to EEPROM         
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
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Global);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Group);
      server.printP(html_e_th); server.printP(html_e_tr);
      for (uint8_t i = 0; i < NUM_OF_PHONES; ++i) {
        if ( (i & 0x01) == 0) server.printP(html_tr_od);
        else                  server.printP(html_tr_ev);
        server.printP(html_td); 
        server << i+1; server.printP(text_dot); server.printP(html_e_td); server.printP(html_td);
        (conf.tel[i] & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td); server.printP(html_td);
        server << conf.tel_name[i]; server.printP(html_e_td); server.printP(html_td);
        server << conf.tel_num[i]; server.printP(html_e_td); server.printP(html_td);        
        (conf.tel[i] >> 5 & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td); server.printP(html_td);
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
      server.printP(html_tr); server.printP(html_td); server.printP(text_Phone); server.printP(text_space); server.printP(text_is); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
      server.radioButton("o", text_1, text_On, conf.tel[webTel] & B1);
      server.radioButton("o", text_0, text_Off, !(conf.tel[webTel] & B1));
      server.printP(html_e_td); server.printP(html_e_tr);
      server.printP(html_tr); server.printP(html_td); server.printP(text_Phone); server.printP(text_space); server.printP(text_name); server.printP(html_e_td); server.printP(html_td);
      server.printP(text_sesp);
    //if (webTel != 0) { server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.tel_name[webTel]; server.printP(html_e_tag); }
    //else             { server << conf.tel_name[webTel]; }
      server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.tel_name[webTel]; server.printP(html_e_tag);
      server.printP(html_tr); server.printP(html_td); server.printP(text_Phone); server.printP(text_space); server.printP(text_number); server.printP(html_e_td); server.printP(html_td);
      server.printP(text_sesp); server.printP(html_s_tag); server << "p"; server.printP(html_m_tag); server << conf.tel_num[webTel]; server.printP(html_e_tag);
      server.printP(html_e_td); server.printP(html_e_tr);
      server.printP(html_tr); server.printP(html_td); server.printP(text_Global); server.printP(text_space); server.printP(text_phone); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
      server.radioButton("a", text_1, text_Yes, conf.tel[webTel] >> 5 & B1);
      server.radioButton("a", text_0, text_No, !(conf.tel[webTel] >> 5 & B1));
      server.printP(html_e_td); server.printP(html_e_tr);
      server.printP(html_tr); server.printP(html_td); server.printP(text_Group); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
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
        case 'R':
          n = sendCmd(15,1);
        break;
        case 'A': // Apply
          value[0] = 'R';
          value[1] = 'K';
          value[2] = unit[webAuth].type;
          value[3] = unit[webAuth].number;
          value[4] = (char)unit[webAuth].setting;
          sendData(unit[webAuth].address, value, 5);
        break;
        case 'o': // enable 
          if (value[0] == '0') unit[webAuth].setting &= ~(1 << 0);
          else unit[webAuth].setting |= (1 << 0);
        break;
        case 'g': // group
          n = strtol(value, NULL, 10);
          if ((n >> 0) & 1) unit[webAuth].setting |= (1 << 1);
          else unit[webAuth].setting &= ~(1 << 1);
          if ((n >> 1) & 1) unit[webAuth].setting |= (1 << 2);
          else unit[webAuth].setting &= ~(1 << 2);
          if ((n >> 2) & 1) unit[webAuth].setting |= (1 << 3);
          else unit[webAuth].setting &= ~(1 << 3);
          if ((n >> 3) & 1) unit[webAuth].setting |= (1 << 4);
          else unit[webAuth].setting &= ~(1 << 4);
        break;
        case 'e': saveConf(); break; // save to EEPROM         
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
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Address);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Type);
      server.printP(html_e_th); server.printP(html_th); server.printP(text_Group);
      server.printP(html_e_th); server.printP(html_e_tr);
      for (uint8_t i = 0; i < units; ++i) {
        if ( (i & 0x01) == 0) server.printP(html_tr_od);
        else                  server.printP(html_tr_ev);
        server.printP(html_td); 
        server << i+1; server.printP(text_dot); server.printP(html_e_td); server.printP(html_td);
        (unit[i].setting & B1) ? server.printP(text_Yes) : server.printP(text_No);
        server.printP(html_e_td); server.printP(html_td);
        if (unit[i].address < RADIO_UNIT_OFFSET) { server << "W:" << unit[i].address; }
        else                                     { server << "R:" << unit[i].address-RADIO_UNIT_OFFSET; }
        server.printP(text_semic); server << unit[i].number; server.printP(html_e_td); server.printP(html_td);
        switch(unit[i].type){
          case 'i': server.printP(text_iButton); break;
          default: server.printP(text_undefined); break;
        }
        server.printP(html_e_td); server.printP(html_td);
        server << ((unit[i].setting >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((unit[i].setting >> 1) & B1111)];
        server.printP(html_e_td); server.printP(html_e_tr);
      }
      server.printP(html_e_table); server.printP(html_e_p);

      server.printP(html_select); server << "P"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < units; ++ii) {
        if (webAuth == ii) 
          { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(html_e_option); }
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(html_e_option); }
      }
      server.printP(html_e_select);
      server.printP(html_F_S); server.printP(html_p);
      server.printP(html_table);
      server.printP(html_tr); server.printP(html_td); 
      server.printP(text_Address); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
      server << unit[webAuth].address; server.printP(text_semic); server << unit[webAuth].number; server.printP(html_e_td); server.printP(html_e_tr);
      server.printP(html_tr); server.printP(html_td); 
      server.printP(text_Type); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
      switch(unit[webAuth].type){
        case 'i': server.printP(text_iButton); break;
        default: server.printP(text_undefined); break;
      }
      server.printP(html_e_td); server.printP(html_e_tr);
      server.printP(html_tr); server.printP(html_td); server.printP(text_Unit); server.printP(text_space); server.printP(text_is); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
      server.radioButton("o", text_1, text_On, unit[webAuth].setting & B1);
      server.radioButton("o", text_0, text_Off, !(unit[webAuth].setting & B1));
      server.printP(html_e_td); server.printP(html_e_tr); 
      server.printP(html_tr); server.printP(html_td); server.printP(text_Group); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
      server.printP(html_select); server << "g"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
        if ((unit[webAuth].setting >> 1 & B1111) == ii) 
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
    server.printP(html_F_RR); // submit Reregister
    server.printP(html_e_form);
    server.printP(htmlFoot);
  }
}

uint8_t webSens = 0;
void webSetSens(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  char name[2], value[17];
  if (type == WebServer::POST) {
    bool repeat;
    do {
      repeat = server.readPOSTparam(name, 2, value, 17);
      switch(name[0]){
        case 'P':
          n = strtol(value, NULL, 10);
          if (n != webSens) {
            webSens = n;
            repeat = 0;
          }
        break;
        case 'R':
          n = sendCmd(15,1);
        break;
        case 'A': // Apply
          value[0] = 'R';
          value[1] = 'S';
          value[2] = sensor[webSens].type;
          value[3] = sensor[webSens].number;
          value[4] = (char)sensor[webSens].setting;
          sendData(sensor[webSens].address,value, 5);
        break;
        case 'o': // enable 
          if (value[0] == '0') sensor[webSens].setting &= ~(1 << 0);
          else sensor[webSens].setting |= (1 << 0);
        break;
        case 'p': // MQtt publish
          if (value[0] == '0') sensor[webSens].setting &= ~(1 << 7);
          else sensor[webSens].setting |= (1 << 7);
        break;
        case 'g': // group
          n = strtol(value, NULL, 10);
          if ((n >> 0) & 1) sensor[webSens].setting |= (1 << 1);
          else sensor[webSens].setting &= ~(1 << 1);
          if ((n >> 1) & 1) sensor[webSens].setting |= (1 << 2);
          else sensor[webSens].setting &= ~(1 << 2);
          if ((n >> 2) & 1) sensor[webSens].setting |= (1 << 3);
          else sensor[webSens].setting &= ~(1 << 3);
          if ((n >> 3) & 1) sensor[webSens].setting |= (1 << 4);
          else sensor[webSens].setting &= ~(1 << 4);
        break;
        case 'e': saveConf(); break; // save to EEPROM          
        }
      } while (repeat);
    server.httpSeeOther(PREFIX "/sens");
  } else {
    server.httpSuccess();
    server.printP(htmlHead);
    server.printP(html_h1); server.printP(text_Sensor); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_form_s); server << PREFIX "/sens"; server.printP(html_form_e);    
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_th); server.printP(text_hash);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Enabled);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Address);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_MQTT);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Last); server.printP(text_space); server.printP(text_OK);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Type);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Value);
    server.printP(html_e_th); server.printP(html_th); server.printP(text_Group);
    server.printP(html_e_th); server.printP(html_e_tr);
    for (uint8_t i = 0; i < sensors; ++i) {
      if ( (i & 0x01) == 0) server.printP(html_tr_od);
      else                  server.printP(html_tr_ev);
      server.printP(html_td); 
      server << i+1; server.printP(text_dot); server.printP(html_e_td); server.printP(html_td);
      (sensor[i].setting & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td); server.printP(html_td);
      if (sensor[i].address < RADIO_UNIT_OFFSET) { server << "W:" << sensor[i].address; }
      else                                 { server << "R:" << sensor[i].address-RADIO_UNIT_OFFSET; }
      server.printP(text_semic); server << sensor[i].number; server.printP(html_e_td); server.printP(html_td);
      ((sensor[i].setting >> 7) & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td); server.printP(html_td);
      server << (nilTimeNow() - sensor[i].last_OK)/NIL_CFG_FREQUENCY; 
      server.printP(text_space); server.printP(text_sec); server.printP(html_e_td); server.printP(html_td);
      switch(sensor[i].type){
        case 'T': server.printP(text_Temperature); break;
        case 'H': server.printP(text_Humidity); break;
        case 'P': server.printP(text_Pressure); break;
        default: server.printP(text_undefined); break;
      }
      server.printP(html_e_td); server.printP(html_td);
      dtostrf(sensor[i].value, 6, 2, value);
      server << value;
      switch(sensor[i].type){
        case 'T': server.printP(text_degC); break;
        case 'H': server.printP(text_space); server.printP(text_percent); break;
        case 'P': server.printP(text_mBar); break;
        default: break;
      }
      server.printP(html_e_td); server.printP(html_td);
      server << ((sensor[i].setting >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((sensor[i].setting >> 1) & B1111)];
      server.printP(html_e_td); server.printP(html_e_tr);
    }
    server.printP(html_e_table); server.printP(html_e_p);
    server.printP(html_select); server << "P"; server.printP(html_e_tag);
    for (uint8_t ii = 0; ii < sensors; ++ii) {
      if (webSens == ii) 
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(html_F_S); server.printP(html_p);
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_td); 
    server.printP(text_Address); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    if (sensor[webSens].address < RADIO_UNIT_OFFSET) { server << "W:" << sensor[webSens].address; }
    else                                             { server << "R:" << sensor[webSens].address-RADIO_UNIT_OFFSET; }
    server.printP(text_semic); server << sensor[webSens].number; server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); 
    server.printP(text_Type); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    switch(sensor[webSens].type){
      case 'T': server.printP(text_Temperature); break;
      case 'H': server.printP(text_Humidity); break;
      case 'P': server.printP(text_Pressure); break;
      default: server.printP(text_undefined); break;
    }
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Sensor); server.printP(text_space); server.printP(text_is); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("o", text_1, text_On, sensor[webSens].setting & B1);
    server.radioButton("o", text_0, text_Off, !(sensor[webSens].setting & B1));
    server.printP(html_e_td); server.printP(html_e_tr); 
    server.printP(html_tr); server.printP(html_td); server.printP(text_MQTT); server.printP(text_space); server.printP(text_publish); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("p", text_1, text_On, (sensor[webSens].setting >> 7) & B1);
    server.radioButton("p", text_0, text_Off, !((sensor[webSens].setting >> 7) & B1));
    server.printP(html_e_td); server.printP(html_e_tr); 
    server.printP(html_tr); server.printP(html_td); server.printP(text_Group); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.printP(html_select); server << "g"; server.printP(html_e_tag);
    for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
      if ((sensor[webSens].setting >> 1 & B1111) == ii) 
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
    server.printP(html_F_RR); // submit Reregister
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
        case 'a': conf.alr_time = strtol(value, NULL, 10); break;
        case 'b': conf.auto_arm = strtol(value, NULL, 10); break;
        case 'c': conf.open_alarm = strtol(value, NULL, 10);  break;
        case 'd': conf.arm_delay = strtol(value, NULL, 10) * 4;  break;
        case 'e': saveConf(); break; // save to EEPROM
        case 'k': // key
          strncpy ((char*)conf.radioKey, value, 16);
          conf.radioKey[16] = 0; // extra byte for null
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
        case 'E': // Radio button
          if (value[0] == '0') conf.SMS &= ~(1 << 12);
          else conf.SMS |= (1 << 12);
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
    server.printP(html_tr); server.printP(html_td); server.printP(text_Authentication); server.printP(text_space); server.printP(text_time);server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.printP(html_select); server << "a"; server.printP(html_e_tag);
    for (uint8_t ii = 5; ii < 26; ++ii) {
      if ((conf.alr_time) == ii)  
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii; server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii; server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(text_space); server.printP(text_seconds); server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Arm); server.printP(text_space); server.printP(text_delay); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.printP(html_select); server << "d"; server.printP(html_e_tag);
    for (uint8_t ii = 10; ii < 41; ++ii) {
      if ((conf.arm_delay/4) == ii) // 250*4 = 1 sec.
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii; server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii; server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(text_space); server.printP(text_seconds); server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Auto); server.printP(text_space); server.printP(text_arm); server.printP(text_space); server.printP(text_delay); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.printP(html_select); server << "b"; server.printP(html_e_tag);
    for (uint8_t ii = 10; ii < 241; ii+=5) {
      if ((conf.auto_arm) == ii) {
        server.printP(html_option); server << ii; server.printP(html_selected); 
        //if (ii>59) { server << "0" << ii/60 << ":" << ii%60;}
        //else       { server << ii;}
        server << ii;
        server.printP(html_e_option);
      }
      else { 
        server.printP(html_option); server << ii; server.printP(html_e_tag);
        //if (ii>59) { server << "0" << ii/60 << ":" << ii%60;}
        //else       { server << ii;}
        server << ii;
        server.printP(html_e_option);
      }
    }
    server.printP(html_e_select);
    server.printP(text_space); server.printP(text_minutes); server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Open); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.printP(html_select); server << "c"; server.printP(html_e_tag);
    for (uint8_t ii = 10; ii < 241; ii+=5) {
      if ((conf.open_alarm) == ii) {
        server.printP(html_option); server << ii; server.printP(html_selected); 
    //if (ii>59) { server << "0" << ii/60 << ":" << ii%60;}
    //else       { server << ii;}
        server << ii;
        server.printP(html_e_option); }
        else { 
          server.printP(html_option); server << ii; server.printP(html_e_tag);
    //if (ii>59) { server << "0" << ii/60 << ":" << ii%60;}
    //else       { server << ii;}
          server << ii;
          server.printP(html_e_option);
        }
    }
    server.printP(html_e_select);
    server.printP(text_space); server.printP(text_minutes); server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_e_table);

    server.printP(html_h1); server.printP(text_Radio); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Radio); server.printP(text_space); server.printP(text_key); server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp);
    server.printP(html_s_tag); server << "k"; server.printP(html_m_tag); server << (char*)conf.radioKey; server.printP(html_e_tag);
    server.printP(html_e_table);

    server.printP(html_h1); server.printP(text_MQTT); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_table);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Radio); server.printP(text_space); server.printP(text_key); server.printP(html_e_td); server.printP(html_td);
    server.printP(text_sesp);
    server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.tel_name[webTel]; server.printP(html_e_tag);
    server.printP(html_e_table);

    server.printP(html_h1); server.printP(text_SMS); server.printP(text_space); server.printP(text_alerting); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_table);
    /*
    server.printP(html_tr); server.printP(html_td); server.printP(text_Global); server.printP(text_space); server.printP(text_phone); server.printP(text_space); server.printP(text_number); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.printP(html_select); server << "s"; server.printP(html_e_tag);
    for (uint8_t ii = 0; ii < NUM_OF_PHONES; ++ii) {
      if ((conf.global_tel_num) == ii) 
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(text_spdashsp); server << conf.tel_name[ii]; server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(text_spdashsp); server << conf.tel_name[ii]; server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(html_e_td); server.printP(html_e_tr);
    */
    server.printP(html_tr); server.printP(html_td); server.printP(text_Undefined); server.printP(text_space); server.printP(text_key);; server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("0", text_1, text_On, conf.SMS & B1);
    server.radioButton("0", text_0, text_Off, !(conf.SMS & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_System); server.printP(text_space); server.printP(text_disarmed); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("1", text_1, text_On, conf.SMS >> 1 & B1);
    server.radioButton("1", text_0, text_Off, !(conf.SMS >> 1 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_System); server.printP(text_space); server.printP(text_armed); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("2", text_1, text_On, conf.SMS >> 2 & B1);
    server.radioButton("2", text_0, text_Off, !(conf.SMS >> 2 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Open); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("E", text_1, text_On, conf.SMS >> 12 & B1);
    server.radioButton("E", text_0, text_Off, !(conf.SMS >> 12 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Damaged); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("3", text_1, text_On, conf.SMS >> 3 & B1);
    server.radioButton("3", text_0, text_Off, !(conf.SMS >> 3 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Tamper); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("4", text_1, text_On, conf.SMS >> 4 & B1);
    server.radioButton("4", text_0, text_Off, !(conf.SMS >> 4 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Alarm); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("5", text_1, text_On, conf.SMS >> 5 & B1);
    server.radioButton("5", text_0, text_Off, !(conf.SMS >> 5 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_ALARM); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("6", text_1, text_On, conf.SMS >> 6 & B1);
    server.radioButton("6", text_0, text_Off, !(conf.SMS >> 6 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Monitoring); server.printP(text_space); server.printP(text_started); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("7", text_1, text_On, conf.SMS >> 7 & B1);
    server.radioButton("7", text_0, text_Off, !(conf.SMS >> 7 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_System); server.printP(text_space); server.printP(text_armed); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("8", text_1, text_On, conf.SMS >> 8 & B1);
    server.radioButton("8", text_0, text_Off, !(conf.SMS >> 8 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Configuration); server.printP(text_space); server.printP(text_saved); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("9", text_1, text_On, conf.SMS >> 9 & B1);
    server.radioButton("9", text_0, text_Off, !(conf.SMS >> 9 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Power); server.printP(text_space); server.printP(text_state); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("Q", text_1, text_On, conf.SMS >> 10 & B1);
    server.radioButton("Q", text_0, text_Off, !(conf.SMS >> 10 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_tr); server.printP(html_td); server.printP(text_Battery); server.printP(text_space); server.printP(text_state); server.printP(html_e_td); server.printP(html_td); server.printP(text_sesp);
    server.radioButton("W", text_1, text_On, conf.SMS >> 11 & B1);
    server.radioButton("W", text_0, text_Off, !(conf.SMS >> 11 & B1));
    server.printP(html_e_td); server.printP(html_e_tr);
    server.printP(html_e_table);
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

// Zone thread
//
NIL_WORKING_AREA(waZoneThread, 64);
NIL_THREAD(ZoneThread, arg) {
  int16_t val = 0;
  uint8_t _group = 255;
  //char l_tmp[4]; // for logger 
  nilThdSleepSeconds(60); // Delay to allow PIR sensors to settle up
  _tmp[0] = 'S'; _tmp[1] = 'S'; _tmp[2] = 0; pushToLog(_tmp);
  WS.println(F("ZoneThread started"));
  
  // Execute while loop every 0.25 seconds.
  while (TRUE) {
    nilThdSleepMilliseconds(250); // time is used also for arm delay and such ...

    for (int8_t i=0; i < ALR_GROUPS ; i++){ 
      if (group[i].arm_delay) { // wait for arm delay
        group[i].arm_delay--;
        if (!group[i].arm_delay) { 
          _tmp[0] = 'S'; _tmp[1] = 'Z'; _tmp[2] = 48+i; _tmp[3] = 0; pushToLog(_tmp);
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
          //WS.print(val);
        } else {
          //WS.print(F("-D:"));
          switch(i) {
            case 7:  pinIN1.read() ? val = ALR_PIR+BatteryLevel : val = ALR_OK+BatteryLevel; break;
            case 8:  pinIN2.read() ? val = ALR_PIR+BatteryLevel : val = ALR_OK+BatteryLevel; break;
            case 9:  pinIN3.read() ? val = ALR_PIR+BatteryLevel : val = ALR_OK+BatteryLevel; break;
            case 10: pinIN4.read() ? val = ALR_PIR+BatteryLevel : val = ALR_OK+BatteryLevel; break;
            case 11: pinIN5.read() ? val = ALR_PIR+BatteryLevel : val = ALR_OK+BatteryLevel; break;
            case 12: pinTAMPER.read() ? val = ALR_PIR+BatteryLevel : val = ALR_OK+BatteryLevel; break;
            default: break;
          } 
        }
        _group = (conf.zone[i] >> 1) & B1111; // set group
        // Decide 
        //WS.print(F("dec: ")); WS.println((int16_t)(val-BatteryLevel));
        switch((int16_t)(val-BatteryLevel)){
          case ALR_OK_LOW ... ALR_OK_HI:
            // All is OK no action
            //WS.print(F(" OK"));
            zone[i].last_OK = nilTimeNow();    // update current timestamp
          break;
          case ALR_PIR_LOW ... ALR_PIR_HI:
            //WS.print(F(" PIR G:"));
            //WS.print(_group);
            //   group  armed                      group not has alarm
            if ((group[_group].setting & B1) && !((group[_group].setting >> 1) & B1) && !group[_group].arm_delay){             
              _tmp[0] = 'P'; _tmp[1] = 48+i; _tmp[2] = 0; pushToLog(_tmp); 
              alarm_event_t* p = alarm_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
              if (p == 0) return; // Continue if no free space.   
              p->zone = i;
              p->type = 'P';
              group[_group].setting |= (1 << 1); // Set alarm bit On
              alarm_fifo.signalData();   // Signal idle thread data is available.
              // if group not enabled log error to log. 
              if (!(conf.group[_group] & B1)) {_tmp[0] = 'G'; _tmp[1] = 'F'; _tmp[2] = 48+_group; _tmp[3] = 0; pushToLog(_tmp);}
            }
            zone[i].last_PIR = nilTimeNow();    // update current timestamp
          break;
          default: 
            // Line is cut or short or tamper, no diffrence to alarm
            //WS.print(F(" UNK G:"));
            //WS.print(_group);
            //  group not has alarm
            if (!((group[_group].setting >> 1) & B1)){             
              _tmp[0] = 'D'; _tmp[1] = 48+i; _tmp[2] = 0; pushToLog(_tmp);
              alarm_event_t* p = alarm_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
              if (p == 0) return; // Continue if no free space.   
              p->zone = i;
              p->type = 'D';
              group[_group].setting |= (1 << 1); // Set alarm bit On
              alarm_fifo.signalData();   // Signal idle thread data is available.
              // if group not enabled log error to log. 
              if (!(conf.group[_group] & B1)) {_tmp[0] = 'G'; _tmp[1] = 'F'; _tmp[2] = 48+_group; _tmp[3] = 0; pushToLog(_tmp);}
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

//------------------------------------------------------------------------------
// Alarm Events
//
NIL_WORKING_AREA(waAEThread1, 64);
NIL_WORKING_AREA(waAEThread2, 64);
NIL_WORKING_AREA(waAEThread3, 64);
NIL_THREAD(thdFcn, name) {
  uint8_t _group, _wait, _resp, _cnt;
  msg_t _nil_r;
  WS.print(F("Event thread started "));
  WS.println((char*)name);
  
  while (TRUE) {
    // Check for data.  Use TIME_IMMEDIATE to prevent sleep in idle thread.
    alarm_event_t* p = alarm_fifo.waitData(TIME_INFINITE);
    if (!p) return;                           // return if no data

    _group = (conf.zone[p->zone] >> 1) & B1111;
    if (p->type == 'P') _wait = (conf.zone[p->zone] >> 5) & B11;
    else                _wait = 0;            // Tamper and others have no wait time

    if (!((group[_group].setting >> 1) & B1)) return; // group has no alarm anymore

    group[_group].setting |= (1 << 2); // Set auth bit On

    WS.print((char*)name);
    WS.print(" zone: "); WS.print(p->zone);
    WS.print(", type: "); WS.print(p->type);
    WS.print(", group: "); WS.print(_group);
    WS.print(", Auth time: "); WS.println(_wait);

    _resp = sendCmdToGrp(_group, 11 + _wait);

    if (!_wait) { // Alarm wait 0x
      // Combine alarms, so that next alarm will not disable ongoing one
      OUTs = ((((conf.group[_group] >> 4) & B1) | (OUTs >> 0) & B1) |
        (((conf.group[_group] >> 3) & B1) | (OUTs >> 1) & B1) << 1);
      // Trigger OUT 1 & 2              
      pinOUT1.write(((OUTs >> 0) & B1));
      pinOUT2.write(((OUTs >> 1) & B1));
      _tmp[0] = 'S'; _tmp[1] = 'X'; _tmp[2] = 48+_group; _tmp[3] = 0; pushToLog(_tmp); // ALARM no auth.
    } else { // Alarm wait > 0x
      do {
        _cnt = 0;
        while ((group[_group].setting >> 2 & B1) && _cnt < (10*conf.alr_time)) {
          nilThdSleepMilliseconds(100);
          _cnt++;
        }
        if (!(group[_group].setting >> 2 & B1)) { // Auth. OK
          break; // escape do while 
        } else {
          _wait--;
          _resp = sendCmdToGrp(_group, 11 + _wait);
        }
      } while (_wait);
      if (!_wait) { // Alarm wait 0x = no auth in time
        // Combine alarms, so that next alarm will not disable ongoing one
        OUTs = ((((conf.group[_group] >> 4) & B1) | (OUTs >> 0) & B1) |
          (((conf.group[_group] >> 3) & B1) | (OUTs >> 1) & B1) << 1);
        // Trigger OUT 1 & 2              
        pinOUT1.write(((OUTs >> 0) & B1));
        pinOUT2.write(((OUTs >> 1) & B1));
        _tmp[0] = 'S'; _tmp[1] = 'X';  _tmp[2] = 48 + _group; _tmp[3] = 0;pushToLog(_tmp); // ALARM no auth.
      }
    }
    // Signal FIFO slot is free.
    alarm_fifo.signalFree();
  }
}

//------------------------------------------------------------------------------
// RS485 receiver thread
//
NIL_WORKING_AREA(waRS485RXThread, 128);
NIL_THREAD(RS485RXThread, arg) {
  uint8_t _resp = 0; 
  uint8_t _group = 255;
  uint8_t _pos = 0;
  uint8_t _unit;  

  WS.println(F("RS485 receiver thread started"));
  
  while (TRUE) {   
    nilWaitRS485NewMsg(); // wait for event
    
    _resp = RS485.msg_read(&RX_msg);

    /*
    WS.print(F("> A:")); WS.print(RX_msg.address);
    WS.print(F(", C:")); WS.print(RX_msg.ctrl);
    WS.print(F(", L:")); WS.print(RX_msg.data_length); WS.print(F(" | "));   
    for (uint8_t i=0; i < RX_msg.data_length; i++){
      WS.print((uint8_t)RX_msg.buffer[i],HEX);
      WS.print(F(" "));
    }; WS.println();
    */
    
    // iButtons keys
    if (RX_msg.ctrl == FLAG_DTA && RX_msg.data_length == KEY_LEN && RX_msg.buffer[0]!='R') {  // incoming message looks like a key 
      if (unit[RX_msg.address-1].setting & B1) { // unit is enabled for authorzation
        for (uint8_t i=0; i < NUM_OF_KEYS; i++){
          _resp = memcmp(RX_msg.buffer, conf.key[i], KEY_LEN); // Compare key
          if (!_resp) { // key matched
            if (true) { // key enabled
              _group = (unit[RX_msg.address-1].setting >> 1) & B1111;
              //     we have alarm                      group is armed
              if  (((group[_group].setting) >> 1 & B1) || ((group[_group].setting) & B1)) { 
                if ((group[_group].setting) >> 1 & B1) { // we have alarm
                  group[_group].setting &= ~(1 << 1);    // set alarm off
                  OUTs = 0; // Reset outs  
                  pinOUT1.write(LOW); pinOUT2.write(LOW); // Turn off OUT 1 & 2
                }
                group[_group].setting &= ~(1 << 0);    // disarm group
                group[_group].setting &= ~(1 << 2);    // set auth bit off
                group[_group].arm_delay = 0;       // Reset arm delay
                _resp = sendCmdToGrp(_group, 20);  // send quiet message to all units
                _tmp[0] = 'A'; _tmp[1] = 'D'; _tmp[2] = 48+i; _tmp[3] = 0; pushToLog(_tmp);
              } else { // Just do arm
                _tmp[0] = 'A'; _tmp[1] = 'A'; _tmp[2] = 48+i; _tmp[3] = 0; pushToLog(_tmp);
                // if group enabled arm group or log error to log. 
                if (conf.group[_group] & B1) { 
                  group[_group].setting |= 1;                   // arm group
                  group[_group].arm_delay = conf.arm_delay; // set arm delay
                  _resp = sendCmdToGrp(_group, 10);  // send arm message to all units
                } 
                else { _tmp[0] = 'G'; _tmp[1] = 'F'; _tmp[2] = 48+_group; _tmp[3] = 0; pushToLog(_tmp); }
              }
              break; // no need to try other
            } else { // key is not enabled
              _tmp[0] = 'A'; _tmp[1] = 'F'; _tmp[2] = 48+i; _tmp[3] = 0; pushToLog(_tmp);
            }
          } // key matched
          if (_resp!=0 && i==NUM_OF_KEYS-1) { // may be we should log unknow keys 
            _tmp[0] = 'A'; _tmp[1] = 'U'; _tmp[2] = ' '; _tmp[3] = 0; pushToLog(_tmp);
            memcpy(last_key, RX_msg.buffer, KEY_LEN); // store last unknown key
            last_key[16] = 0;
          }
        } // for
      } // unit is enabled for authorzation
      else { // log disabled remote units
        _tmp[0] = 'U'; _tmp[1] = 'F'; _tmp[2] = 48 + RX_msg.address; _tmp[3] = 0; pushToLog(_tmp);
      } 
    } // incoming message looks like a key

    // Registration
    if (RX_msg.ctrl == FLAG_DTA && RX_msg.buffer[0]=='R') {
      _pos = 1;
      do {
        switch(RX_msg.buffer[_pos]){
          case 'K': // Key
          _resp = 1;
          for (_unit=0; _unit < units; _unit++) {
            if (unit[_unit].address == RX_msg.address && unit[_unit].type == RX_msg.buffer[_pos+1] && unit[_unit].number == (uint8_t)RX_msg.buffer[_pos+2]) {
              _resp = 0;
              break;
            }
          }
            if (_resp) { // if unit not present already
              units++;
              unit[units-1].address = RX_msg.address;
              unit[units-1].type    = RX_msg.buffer[_pos+1];
              unit[units-1].number  = (uint8_t)RX_msg.buffer[_pos+2];
              unit[units-1].setting = (uint8_t)RX_msg.buffer[_pos+3];
              _tmp[0] = 'U'; _tmp[1] = 'R'; _tmp[2] = 48 + unit[units-1].address; _tmp[3] = 0; pushToLog(_tmp);
            } else {
              unit[_unit].address = RX_msg.address;
              unit[_unit].type    = RX_msg.buffer[_pos+1];
              unit[_unit].number  = (uint8_t)RX_msg.buffer[_pos+2];
              unit[_unit].setting = (uint8_t)RX_msg.buffer[_pos+3];
              _tmp[0] = 'U'; _tmp[1] = 'r'; _tmp[2] = 48 + unit[_unit].address; _tmp[3] = 0; pushToLog(_tmp);
            }
            _pos += 4;
            break;
          case 'S': // Sensor
          _resp = 1;
          for (_unit=0; _unit < sensors; _unit++) {
            if (sensor[_unit].address == RX_msg.address && sensor[_unit].type == RX_msg.buffer[_pos+1] && sensor[_unit].number == (uint8_t)RX_msg.buffer[_pos+2]) {
              _resp = 0;
              break;
            }
          }
            if (_resp) { // if unit not present already
              sensors++;
              sensor[sensors-1].address = RX_msg.address;
              sensor[sensors-1].type    = RX_msg.buffer[_pos+1];
              sensor[sensors-1].number  = (uint8_t)RX_msg.buffer[_pos+2];
              sensor[sensors-1].setting = (uint8_t)RX_msg.buffer[_pos+3];
              sensor[sensors-1].value   = 0;
              sensor[sensors-1].last_OK = nilTimeNow();    // update current timestamp
              _tmp[0] = sensor[sensors-1].type + 32; _tmp[1] = 'R'; _tmp[2] = 48 + sensor[sensors-1].address; _tmp[3] = 0; pushToLog(_tmp);
            } else {
              sensor[_unit].address = RX_msg.address;
              sensor[_unit].type    = RX_msg.buffer[_pos+1];
              sensor[_unit].number  = (uint8_t)RX_msg.buffer[_pos+2];
              sensor[_unit].setting = (uint8_t)RX_msg.buffer[_pos+3];
              sensor[_unit].value   = 0;
              sensor[_unit].last_OK = nilTimeNow();    // update current timestamp
              _tmp[0] = sensor[_unit].type + 32; _tmp[1] = 'r'; _tmp[2] = 48 + sensor[_unit].address; _tmp[3] = 0; pushToLog(_tmp);
            }
          _pos += 4;
          break;
          default: 
            _pos++;
            WS.println(F("Reg. error"));
          break;
        }
      } while (_pos < RX_msg.data_length);
    }
    // Sensors
    if (RX_msg.ctrl == FLAG_DTA && RX_msg.buffer[0]=='S') {
      _pos = 1;
      do {
        sensor_t *p = sensor_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
        if (p == 0) return; // Continue if no free space.   
        p->address = RX_msg.address;
        p->type    = RX_msg.buffer[1];
        p->number  = (uint8_t)RX_msg.buffer[2];
        u.b[0] = RX_msg.buffer[3]; u.b[1] = RX_msg.buffer[4]; u.b[2] = RX_msg.buffer[5]; u.b[3] = RX_msg.buffer[6];
        p->value   = u.fval;
        p->last_OK = nilTimeNow();    // update current timestamp
        sensor_fifo.signalData();     // Signal idle thread data is available.
        _pos+=6;
      } while (_pos < RX_msg.data_length);
    } // if 'S'

  }
}

//------------------------------------------------------------------------------
// Logger thread
//
NIL_WORKING_AREA(waLoggerThread, 128);
NIL_THREAD(LoggerThread, arg) {
  uint8_t sms_send, sms_ok, _resp, _group;
  char    _text[10]; // MQTT out string

  // Initialize GSM modem
  nilSemWait(&GSMSem);    // wait for slot
  GSMisAlive = Serial.ATsendCmd(AT_is_alive);
  if (GSMisAlive) {
    GSMsetSMS = Serial.ATsendCmd(AT_CLIP_ON); 
    GSMsetSMS = Serial.ATsendCmd(AT_set_sms_to_text);
  }
  nilSemSignal(&GSMSem);  // Exit region.

  nilThdSleepSeconds(1); // Sleep before start service thread
  
  WS.println(F("Logger thread started"));
  
  while (TRUE) {
    // Check for data.  Use TIME_IMMEDIATE to prevent sleep in idle thread.
    log_event_t* p = log_fifo.waitData(TIME_INFINITE);
    if (!p) return; // return if no data

    char* log_message = p->text; // Fetch and print data.

    // SMS handler
    if (GSMisAlive) {
      sms_send = 0; sms_text[0] = 0;
      _group = 255;
      // SMS body
      switch(log_message[13]){
        case 'S':
          strcat_P(sms_text, (char*)text_System); strcat_P(sms_text, (char*)text_sesp);
          switch(log_message[14]){
            case 'B': // Battery
              if (conf.SMS >> 11 & B1) sms_send = 1;
              strcat_P(sms_text, (char*)text_BR);
              if (log_message[15] == 'L') strcat_P(sms_text, (char*)text_low);
              else strcat_P(sms_text, (char*)text_high);
              strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_level);
            break;
            case 'A': // AC
              if (conf.SMS >> 10 & B1) sms_send = 1;
              strcat_P(sms_text, (char*)text_PWS);
              if (log_message[15] == 'L') strcat_P(sms_text, (char*)text_On);
              else strcat_P(sms_text, (char*)text_Off);
            break;
            case 'C': strcat_P(sms_text, (char*)text_Configuration); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_saved); if (conf.SMS >> 9 & B1) sms_send = 1; break; // conf. saved
            case 'Z': strcat_P(sms_text, (char*)text_armed);  if (conf.SMS >> 8 & B1) sms_send = 1; break; // system armed
            case 'S': strcat_P(sms_text, (char*)text_Monitoring); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_started); if (conf.SMS >> 7 & B1) sms_send = 1; break; // monitoring strted
            case 'X': strcat_P(sms_text, (char*)text_ALARM);  if (conf.SMS >> 6 & B1) sms_send = 1;
                      strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_Group); strcat_P(sms_text, (char*)text_sesp);
                      strcat(sms_text, conf.group_name[log_message[15]-48]); // extra text for SMS
                      _group = log_message[15]-48;
                      break; // alarm
            default:  strcat_P(sms_text, (char*)text_undefined);  if (conf.SMS >> 9 & B1) sms_send = 1; break; // unknown
          }
        break;
        case 'P':
          if (conf.SMS >> 5 & B1) sms_send = 1;
          strcat_P(sms_text, (char*)text_Alarm); strcat_P(sms_text, (char*)text_sesp); strcat_P(sms_text, (char*)text_triggered); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_zone); strcat_P(sms_text, (char*)text_space);
          strcat(sms_text, conf.zone_name[log_message[14]-48]);
          _group = (conf.zone[log_message[14]-48] >> 1) & B1111;
        break;
        case 'T':
        if (conf.SMS >> 4 & B1) sms_send = 1;
          strcat_P(sms_text, (char*)text_Tamper); strcat_P(sms_text, (char*)text_sesp); strcat_P(sms_text, (char*)text_triggered); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_zone); strcat_P(sms_text, (char*)text_space);
          strcat(sms_text, conf.zone_name[log_message[14]-48]);
          _group = (conf.zone[log_message[14]-48] >> 1) & B1111;
        break;
        case 'D':
        if (conf.SMS >> 3 & B1) sms_send = 1;
          strcat_P(sms_text, (char*)text_Damaged); strcat_P(sms_text, (char*)text_sesp); strcat_P(sms_text, (char*)text_triggered); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_zone); strcat_P(sms_text, (char*)text_space);
          strcat(sms_text, conf.zone_name[log_message[14]-48]);
          _group = (conf.zone[log_message[14]-48] >> 1) & B1111;
        break;
        case 'A': // Authentication
          strcat_P(sms_text, (char*)text_Authentication); strcat_P(sms_text, (char*)text_sesp); 
          if (log_message[14] < 'a') { strcat_P(sms_text, (char*)text_key); strcat_P(sms_text, (char*)text_space); }
          if (log_message[14] == 'A' || log_message[14] == 'D' || log_message[14] == 'F') { strcat(sms_text, conf.key_name[log_message[15]-48]); strcat_P(sms_text, (char*)text_space); }  
          switch(log_message[14]){
            case 'D': strcat_P(sms_text, (char*)text_disarmed); if (conf.SMS >> 1 & B1) sms_send = 1; break;
            case 'A': strcat_P(sms_text, (char*)text_armed); if (conf.SMS >> 2 & B1) sms_send = 1; break;
            case 'U': strcat_P(sms_text, (char*)text_undefined); if (conf.SMS & B1) sms_send = 1; break;
            case 'F': strcat_P(sms_text, (char*)text_is); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_disabled); break;
            case 'a': strcat_P(sms_text, (char*)text_auto); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_armed);
                      strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_zone); strcat_P(sms_text, (char*)text_space); strcat(sms_text, conf.zone_name[log_message[15]-48]);
                      if (conf.SMS >> 2 & B1) sms_send = 1;
                      _group = (conf.zone[log_message[15]-48] >> 1) & B1111; break; // Auto armed
            case 'o': strcat_P(sms_text, (char*)text_zone); strcat_P(sms_text, (char*)text_space); strcat(sms_text, conf.zone_name[log_message[15]-48]);
                      strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_is); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_open);
                      if (conf.SMS >> 12 & B1) sms_send = 1;
                      _group = (conf.zone[log_message[15]-48] >> 1) & B1111; break; // Open alarm              
            default: break;
          }
        break;
        default: // do nothing
        break;
      }
      strcat_P(sms_text, (char*)text_dot);

      if (sms_send) {
        for (uint8_t i = 0; i < NUM_OF_PHONES; ++i) {
          //  phone enabled          specific group     global tel.
          if ((conf.tel[i] & B1) && ((_group != 255) || (conf.tel[i] >> 5 & B1))) { 
            nilSemWait(&GSMSem);    // wait for slot
            sms_ok = Serial.ATsendSMSBegin(conf.tel_num[i]);
            if (sms_ok) sms_ok = Serial.ATsendSMSEnd(sms_text, sms_send);
            WS.println(sms_ok);
            nilSemSignal(&GSMSem);  // Exit region.
          }
        }
      }

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
    conf.ee_pos += EEPROM_MESSAGE;// Will overflow by itself as size is uint16_t = EEPROM_SIZE

    // MQTT
    
    if (client.connected()) {
      _text[0] = 0;
      strcat_P(_text, (char*)text_OHS); strcat_P(_text, (char*)text_slash); strcat_P(_text, (char*)text_Log);
      client.publish(_text, log_message);
    } else {
      if (MQTTState) {
        _tmp[0] = 'S'; _tmp[1] = 'M'; _tmp[2] = 'F'; _tmp[3] = 0; pushToLog(_tmp);
        MQTTState = 0;
      }
    }
    
    // Signal FIFO slot is free.
    log_fifo.signalFree();
  }
}

//------------------------------------------------------------------------------
// Service thread
//
NIL_WORKING_AREA(waServiceThread, 160);  
NIL_THREAD(ServiceThread, arg) {
  int8_t  _resp;
  uint8_t _GSMlastStatus = 10; // get status on start
  uint8_t _text[10];
  msg_t   _smpR;
  uint8_t _counter = 0;         // used for pause in alarm

  nilThdSleepSeconds(5); // Sleep before start service thread
  // NTP Sync
  time_now = GetNTPTime(udp);  
  if (time_now.get() > 0) RTC.adjust(time_now.get());
  
  nilThdSleepSeconds(1); // Sleep before start service thread
  // Call for registration of units
  _resp = sendCmd(15,1);        nilThdSleepMilliseconds(200);
  //radio.Send(0, "C", 1, false); nilThdSleepMilliseconds(200);

  WS.println(F("Service thread started"));
  while (TRUE) {
    nilThdSleepSeconds(10);
    _counter++; 

    // Auto arm
    for (int8_t i=0; i < ALR_ZONES ; i++){
      if ((conf.zone[i] & B1) && ((conf.zone[i] >> 7) & B1)){ // Zone enabled and auto arming
        if (nilTimeNowIsWithin(zone[i].last_PIR + S2ST((conf.auto_arm * 60)-8), zone[i].last_PIR + S2ST((conf.auto_arm * 60)+8))) {
          uint8_t _group = (conf.zone[i] >> 1) & B1111;
          _tmp[0] = 'A'; _tmp[1] = 'a'; _tmp[2] = 48 + _group; _tmp[3] = 0; pushToLog(_tmp); // Authorization auto arm
          // if group is enabled arm zone else log error to log
          if (conf.group[_group] & B1) { group[(conf.zone[i] >> 1) & B1111].setting |= 1; } // arm group
          else  {_tmp[0] = 'G'; _tmp[1] = 'F'; _tmp[2] = 48+_group; _tmp[3] = 0; pushToLog(_tmp);}
        }
      }
    }
    // Open alarm
    for (int8_t i=0; i < ALR_ZONES ; i++){
      if ((conf.zone[i] & B1) && ((conf.zone[i] >> 8) & B1)){ // Zone enabled and still open alarm enabled
        if (nilTimeNowIsWithin(zone[i].last_OK + S2ST((conf.open_alarm * 60)-8), zone[i].last_OK + S2ST((conf.open_alarm * 60)+8))) {
          _tmp[0] = 'A'; _tmp[1] = 'o'; _tmp[2] = 48 + i; _tmp[3] = 0; pushToLog(_tmp); // Authorization still open alarm
          zone[i].last_OK = nilTimeNow();    // update current timestamp
        }
      }
    }

    // Battery check 
    if (pinBAT_OK.read() == LOW) { // The signal is "Low" when the voltage of battery is under 11V 
      _tmp[0] = 'S'; _tmp[1] = 'B'; _tmp[2] = 'L'; _tmp[3] = 0; pushToLog(_tmp); // battery low
      _tmp[0] = 'S'; _tmp[1] = 'C'; _tmp[2] = 'P'; _tmp[3] = 0; pushToLog(_tmp); // conf saved 
      log_event_t *p;
      do { 
        nilThdSleepMilliseconds(100);
      } while (p); // wait for logger thread
      eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration

      nilSysLock(); // Lock RTOS  

      // Battery is at low level but it might oscillate, so we wait for AC to recharge battery again.
      while (pinAC_OFF.read() == HIGH) { // The signal turns to be "High" when the power supply turns OFF
        delay(1000); // do nothing wait for power supply shutdown
      } 

      nilSysUnlock(); // in case the power is restored we goon
      _tmp[0] = 'S'; _tmp[1] = 'A'; _tmp[2] = 'L'; _tmp[3] = 0; pushToLog(_tmp); // AC ON
      _tmp[0] = 'S'; _tmp[1] = 'B'; _tmp[2] = 'H'; _tmp[3] = 0; pushToLog(_tmp); // Battery High
    }

    // AC power check
    if (pinAC_OFF.read() == LOW && ACState != LOW) { // The signal turns to be "High" when the power supply turns OFF
      ACState = LOW;
      _tmp[0] = 'S'; _tmp[1] = 'A'; _tmp[2] = 'L'; _tmp[3] = 0; pushToLog(_tmp); // AC ON
    }
    if (pinAC_OFF.read() == HIGH && ACState != HIGH) { // The signal turns to be "High" when the power supply turns OFF
      ACState = HIGH;
      _tmp[0] = 'S'; _tmp[1] = 'A'; _tmp[2] = 'H'; _tmp[3] = 0; pushToLog(_tmp); // AC OFF
    }

    if ((_counter%12) == 0) { // GSM modem check every 2 minutes
      _smpR = nilSemWaitTimeout(&GSMSem, TIME_IMMEDIATE); // look if slot is free
      if (_smpR != NIL_MSG_TMO) { // slot is free 
        //WS.println("gsm slot");
        GSMisAlive = Serial.ATsendCmd(AT_is_alive); 
        if (GSMisAlive) {
          //WS.println("gsm alive");
          if (!GSMsetSMS) {
            //_resp = Serial.ATsendCmd(AT_CLIP_ON);             // CLI On
            GSMsetSMS = Serial.ATsendCmd(AT_set_sms_to_text); // set modem to text SMS format
          }
          _resp = Serial.ATsendCmdWR(AT_registered, _text, 3);
          GSMreg = strtol((char*)_text, NULL, 10);
          _resp = Serial.ATsendCmdWR(AT_signal_strength, _text, 2);
          GSMstrength= strtol((char*)_text, NULL, 10);
        } else { GSMreg = 4; GSMstrength = 0; GSMsetSMS = 0;}
        nilSemSignal(&GSMSem);  // Exit region.
        //WS.println("gsm slot exit");
        if (_GSMlastStatus != GSMreg) { // if modem registration changes log it
          _GSMlastStatus = GSMreg;
          _tmp[0] = 'M'; _tmp[1] = 48+GSMreg; _tmp[2] = 48+GSMstrength; _tmp[3] = 0; pushToLog(_tmp); 
        }
      }
    }

    // OUT 1 & 2 handling
    if ((_counter%6) == 0) { // cycle every minute
      pinOUT1.write(LOW);
      pinOUT2.write(LOW);
    } else {
      pinOUT1.write(((OUTs >> 0) & B1));
      pinOUT2.write(((OUTs >> 1) & B1));
    }

    //   MQTT connected
    if (!client.connected()) {
      if (!client.connect(str_MQTT_dev)) {       
        // Ethernet
        Ethernet.begin(mac, our_ip, gateway, gateway, subnet);
      } else {
        if (!MQTTState){
          _tmp[0] = 'S'; _tmp[1] = 'M'; _tmp[2] = 'O'; _tmp[3] = 0; pushToLog(_tmp);
          MQTTState = 1;
        }
      }
    }
    
    if (Serial.isMsg()) {
      _resp = Serial.read((uint8_t*)sms_text);                     // read serial
      WS.print(_resp);
      for(uint8_t i = 0; i < _resp; i++) {
        WS.print((char)sms_text[i]);
      }
      WS.println();
    }
    
  }
}

//------------------------------------------------------------------------------
// RFM12B thread 
//
NIL_WORKING_AREA(waRadioThread, 128);  
NIL_THREAD(RadioThread, arg) {
  uint16_t bad_crc = 0;
  uint16_t no_ack = 0;
  uint32_t received = 0;

  uint8_t  _pos, _unit, _resp;

  WS.println(F("RadioThread started"));    
  while (TRUE) {
    nilThdSleepMilliseconds(50);
    if (radio.ReceiveComplete()) {
      nilSemWait(&RFMSem); 
      ++received;
      if (radio.CRCPass()) {
        /*
        WS.print(F("received[")); WS.print(radio.GetSender(), DEC);WS.print(F("] "));
        for (byte i = 0; i < *radio.DataLen; i++)
          WS.print((char)radio.Data[i]);
        */
        // Registration
        if ((char)radio.Data[0] == 'R') {
          WS.print(F("Radio: "));
          _pos = 1;
          do {
            switch((char)radio.Data[_pos]){
              case 'K': // Key
                _resp = 1;
                for (_unit=0; _unit < units; _unit++) {
                  if (unit[_unit].address == radio.GetSender()+RADIO_UNIT_OFFSET && unit[_unit].type == radio.Data[_pos+1] && unit[_unit].number == (uint8_t)radio.Data[_pos+2]) {
                    _resp = 0;
                    break;
                  }
                }
                if (_resp) { // if unit not present already
                  units++;
                  unit[units-1].address = radio.GetSender()+RADIO_UNIT_OFFSET;
                  unit[units-1].type    = radio.Data[_pos+1];
                  unit[units-1].number  = (uint8_t)radio.Data[_pos+2];
                  unit[units-1].setting = (uint8_t)radio.Data[_pos+3];
                  _tmp[0] = 'U'; _tmp[1] = 'R'; _tmp[2] = 48 + unit[units-1].address; _tmp[3] = 0; pushToLog(_tmp);
                  /*
                  WS.print(F("Key reg: ")); WS.println(units);
                  WS.print(unit[units-1].address);
                  WS.print(unit[units-1].type);
                  WS.println(unit[units-1].number);
                  WS.println(unit[units-1].setting,BIN);
                  */
                } else {
                  unit[_unit].address = radio.GetSender()+RADIO_UNIT_OFFSET;
                  unit[_unit].type    = radio.Data[_pos+1];
                  unit[_unit].number  = (uint8_t)radio.Data[_pos+2];
                  unit[_unit].setting = (uint8_t)radio.Data[_pos+3];
                  _tmp[0] = 'U'; _tmp[1] = 'r'; _tmp[2] = 48 + unit[_unit].address; _tmp[3] = 0; pushToLog(_tmp);
                  /*
                  WS.print(F("Key rereg: ")); WS.println(_unit+1);
                  WS.print(unit[_unit].address);
                  WS.print(unit[_unit].type);
                  WS.println(unit[_unit].number);
                  WS.println(unit[_unit].setting,BIN);
                  */
                }
                _pos += 4;
                break;
              case 'S': // Sensor
                _resp = 1;
                for (_unit=0; _unit < sensors; _unit++) {
                  if (sensor[_unit].address == radio.GetSender()+RADIO_UNIT_OFFSET && sensor[_unit].type == radio.Data[_pos+1] && sensor[_unit].number == (uint8_t)radio.Data[_pos+2]) {
                    _resp = 0;
                    break;
                  }
                }
                if (_resp) { // if unit not present already
                  sensors++;
                  sensor[sensors-1].address = radio.GetSender()+RADIO_UNIT_OFFSET;
                  sensor[sensors-1].type    = radio.Data[_pos+1];
                  sensor[sensors-1].number  = (uint8_t)radio.Data[_pos+2];
                  sensor[sensors-1].setting = (uint8_t)radio.Data[_pos+3];
                  sensor[sensors-1].value   = 0;
                  sensor[sensors-1].last_OK = nilTimeNow();    // update current timestamp
                  _tmp[0] = sensor[sensors-1].type + 32; _tmp[1] = 'R'; _tmp[2] = 48 + sensor[sensors-1].address; _tmp[3] = 0; pushToLog(_tmp);
                  /*
                  WS.print(F("Sens reg: ")); WS.println(sensors);
                  WS.print(sensor[sensors-1].address);
                  WS.print(sensor[sensors-1].type);
                  WS.println(sensor[sensors-1].number);
                  WS.println(sensor[sensors-1].setting,BIN);
                  WS.println(sensor[sensors-1].value);
                  */
                } else {
                  sensor[_unit].address = radio.GetSender()+RADIO_UNIT_OFFSET;
                  sensor[_unit].type    = radio.Data[_pos+1];
                  sensor[_unit].number  = (uint8_t)radio.Data[_pos+2];
                  sensor[_unit].setting = (uint8_t)radio.Data[_pos+3];
                  sensor[_unit].value   = 0;
                  sensor[_unit].last_OK = nilTimeNow();    // update current timestamp
                  _tmp[0] = sensor[_unit].type + 32; _tmp[1] = 'r'; _tmp[2] = 48 + sensor[_unit].address; _tmp[3] = 0; pushToLog(_tmp);
                  /*
                  WS.print(F("Sens rereg: ")); WS.println(_unit+1);
                  WS.print(sensor[_unit].address);
                  WS.print(sensor[_unit].type);
                  WS.println(sensor[_unit].number);
                  WS.println(sensor[_unit].setting,BIN);
                  WS.println(sensor[_unit].value);
                  */
                }
                _pos += 4;
              break;
              default: 
                _pos++;
                WS.println(F("Reg. error"));
              break;
            }
          } while (_pos < *radio.DataLen);
        }

        // Sensors
        if ((char)radio.Data[0] == 'S') {
          _pos = 1;
          do {
            sensor_t *p = sensor_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
            if (p == 0) return; // Continue if no free space.   
            p->address = radio.GetSender()+RADIO_UNIT_OFFSET;
            p->type    = radio.Data[1];
            p->number  = (uint8_t)radio.Data[2];
            u.b[0] = radio.Data[3]; u.b[1] = radio.Data[4]; u.b[2] = radio.Data[5]; u.b[3] = radio.Data[6];
            p->value   = u.fval;
            p->last_OK = nilTimeNow();    // update current timestamp
            sensor_fifo.signalData();   // Signal idle thread data is available.
            _pos+=6;
          } while (_pos < RX_msg.data_length);
        } // if 'S'

        if (radio.ACKRequested()) radio.SendACK();
      }
      else {
        ++bad_crc;
        WS.print(F("BAD-CRC"));
      }
      nilSemSignal(&RFMSem);  // Exit region.
      WS.print(F("Received: ")); WS.print(received); WS.print(F(", Bad CRC: ")); WS.print(bad_crc); WS.print(F(", Not ACKed: ")); WS.print(no_ack); 
      WS.println();
    }
  }
}

//------------------------------------------------------------------------------
// Sensor thread 
//
NIL_WORKING_AREA(waSensorThread, 128);  
NIL_THREAD(SensorThread, arg) {
  uint8_t _unit;
  char _value[7];
  char _text[40];
  WS.println(F("SensorThread started"));    
  while (TRUE) {
    // Check for data.  Use TIME_IMMEDIATE to prevent sleep in idle thread.
    sensor_t* p = sensor_fifo.waitData(TIME_INFINITE);
    if (!p) return; // return if no data

    for (_unit=0; _unit < sensors; _unit++) {
      if (sensor[_unit].address == p->address && sensor[_unit].type == p->type && sensor[_unit].number == p->number) {
        sensor[_unit].value   = p->value;
        sensor[_unit].last_OK = p->last_OK;
        
        //   MQTT connected           MQTT sensor enabled
        if ((client.connected()) && ((sensor[_unit].setting >> 7) & B1)) {
          _text[0] = 0;
          // Create MQTT string
          strcat_P(_text, (char*)text_OHS); strcat_P(_text, (char*)text_slash);
          strcat(_text, conf.group_name[(sensor[_unit].setting >> 1) & B1111]); strcat_P(_text, (char*)text_slash);
          strcat_P(_text, (char*)text_Sensor); strcat_P(_text, (char*)text_slash);
          switch(sensor[_unit].type){
            case 'T': strcat_P(_text, (char*)text_Temperature); break;
            case 'H': strcat_P(_text, (char*)text_Humidity); break;
            case 'P': strcat_P(_text, (char*)text_Pressure); break;
            default : strcat_P(_text, (char*)text_Undefined); break;
          }
          strcat_P(_text, (char*)text_slash);
          itoa(sensor[_unit].number, _value, 10); strcat(_text, _value);

          dtostrf(sensor[_unit].value, 6, 2, _value); // value to string
          client.publish(_text,_value);
        }
        if (!client.connected()) {
          if (MQTTState) {
            _tmp[0] = 'S'; _tmp[1] = 'M'; _tmp[2] = 'F'; _tmp[3] = 0; pushToLog(_tmp);
            MQTTState = 0;
          }
        }    
      } // if address 
    } // for
    
    // Signal FIFO slot is free.
    sensor_fifo.signalFree();
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
    webserver.processConnection();
    //nilSemSignal(&WEBSem);  // Exit region.
    
    nilThdSleepMilliseconds(10);
    client.loop(); // MQTT
    //if (!client.loop()) WS.println(F("MQTT loop NC")); // MQTT
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
 NIL_THREADS_TABLE_ENTRY(NULL, ZoneThread, NULL, waZoneThread, sizeof(waZoneThread))
 NIL_THREADS_TABLE_ENTRY(NULL, thdFcn, (void*)"AET 1", waAEThread1, sizeof(waAEThread1))
 NIL_THREADS_TABLE_ENTRY(NULL, thdFcn, (void*)"AET 2", waAEThread2, sizeof(waAEThread2))
 NIL_THREADS_TABLE_ENTRY(NULL, thdFcn, (void*)"AET 3", waAEThread3, sizeof(waAEThread3))
 NIL_THREADS_TABLE_ENTRY(NULL, RS485RXThread, NULL, waRS485RXThread, sizeof(waRS485RXThread))
 NIL_THREADS_TABLE_ENTRY(NULL, LoggerThread, NULL, waLoggerThread, sizeof(waLoggerThread))
 NIL_THREADS_TABLE_ENTRY(NULL, ServiceThread, NULL, waServiceThread, sizeof(waServiceThread))
 NIL_THREADS_TABLE_ENTRY(NULL, RadioThread, NULL, waRadioThread, sizeof(waRadioThread))
 NIL_THREADS_TABLE_ENTRY(NULL, SensorThread, NULL, waSensorThread, sizeof(waSensorThread))
 NIL_THREADS_TABLE_ENTRY(NULL, WebThread, NULL, waWebThread, sizeof(waWebThread))
 NIL_THREADS_TABLE_ENTRY(NULL, Thread9, NULL, waThread9, sizeof(waThread9))

 NIL_THREADS_TABLE_END()

//------------------------------------------------------------------------------
 void setup() {

  // On start set groups
  for (int8_t i=0; i < ALR_GROUPS ; i++){
    group[i].setting       = 0;
    group[i].arm_delay = 0;
  }
  // On start set zones
  for (int8_t i=0; i < ALR_ZONES ; i++){
    zone[i].last_PIR   = 0;
    zone[i].last_OK    = 0;
  }

  // Read current configuration
  eeprom_read_block((void*)&conf, (void*)0, sizeof(conf)); 
  if (conf.version != VERSION) setDefault();

  Serial.begin(9600); // GSM modem
  WS.println(F("Start"));

  // RFM12B radio
  radio.Initialize(RADIO_NODEID, RADIO_FREQUENCY, RADIO_NETWORKID);
  radio.Encrypt(conf.radioKey); //comment this out to disable encryption
 
  RS485.begin(19200, 0); // RS485 network, speed, our address  

  // ADC_PS_64 - 250kHz, ADC speed for 16MHz, faster readings less time for idle
  nilAnalogPrescalar(ADC_PS_64); 
  
  // TWI 
  Wire.begin(); Wire.speed(I2C_400KHZ);

  // Ethernet
  Ethernet.begin(mac, our_ip, gateway, gateway, subnet);
  // UDP for NTP
  // moved to function // udpInited = udp.begin(NTP_LOCAL_PORT); // open socket on arbitrary port
  // MQTT connect
  client.connect(str_MQTT_dev);

  // Web server
  webserver.begin();
  webserver.setDefaultCommand(&webHome);
  webserver.addCommand("form", &webHome);
  webserver.addCommand("log", &webListLog);
  webserver.addCommand("zone", &webSetZone);
  webserver.addCommand("group", &webSetGroup);
  webserver.addCommand("set", &webSetGlobal);
  webserver.addCommand("phone", &webSetPhone);
  #if WEB_SERIAL_DEBUGGING 
  webserver.addCommand("debug", &webDebug);
  #endif
  webserver.addCommand("auth", &webSetAuth);
  webserver.addCommand("sens", &webSetSens);
  webserver.addCommand("key", &webSetKey);

  // start kernel
  nilSysBegin();
  time_started = RTC.now();
  _tmp[0] = 'S'; _tmp[1] = 's'; _tmp[2] = 0; pushToLog(_tmp);
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