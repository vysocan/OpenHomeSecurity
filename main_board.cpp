// OpenHomeSecurity
// ----------------
// ATMEL ATMEGA1284P gateway PCB v. 1.7
// W5500, RFM69HW, 24C512, SIM900, DS1338
//
//                       +---\/---+
//    BAT OK (D 0) PB0  1|        |40  PA0 (AI 0 / D24) A IN 1
//    SS_NET (D 1) PB1  2|        |39  PA1 (AI 1 / D25) A IN 2
//   INT_RFM (D 2) PB2  3|        |38  PA2 (AI 2 / D26) A IN 3
//    AC_OFF (D 3) PB3  4|        |37  PA3 (AI 3 / D27) A IN 4
// SS_RFM/SS (D 4) PB4  5|        |36  PA4 (AI 4 / D28) A IN 5
//      MOSI (D 5) PB5  6|        |35  PA5 (AI 5 / D29) A IN 6
//      MISO (D 6) PB6  7|        |34  PA6 (AI 6 / D30) A IN 7
//       SCK (D 7) PB7  8|        |33  PA7 (AI 7 / D31) A IN 8
//                 RST  9|        |32  AREF
//                 VCC 10|        |31  GND 
//                 GND 11|        |30  AVCC
//               XTAL2 12|        |29  PC7 (D 23) GSM_ON
//               XTAL1 13|        |28  PC6 (D 22) D IN 5 TAMPER
//      RX0 (D 8)  PD0 14|        |27  PC5 (D 21) D IN 4
//      TX0 (D 9)  PD1 15|        |26  PC4 (D 20) D IN 3
//      RX1 (D 10) PD2 16|        |25  PC3 (D 19) D IN 2
//      TX1 (D 11) PD3 17|        |24  PC2 (D 18) D IN 1
//      ED1 (D 12) PD4 18|        |23  PC1 (D 17) SDA
//  GSM_PWR (D 13) PD5 19|        |22  PC0 (D 16) SCL
//     OUT1 (D 14) PD6 20|        |21  PD7 (D 15) OUT2
//                       +--------+
//
#include <stdlib.h>
#include <NilRTOS.h>
#include <NilTimer1.h>
#include <DigitalIO.h>
#include <SPI.h>

//#define TEST 1

#include <Ethernet.h>
#ifdef TEST
static uint8_t mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xAD };  // CHANGE THIS TO YOUR OWN UNIQUE VALUE
#else
static uint8_t mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xFD };  // CHANGE THIS TO YOUR OWN UNIQUE VALUE
#endif

// Global configuration and default setting
#include "conf.h"

// NTP
#include <EthernetUdp.h>                      //
#define NTP_LOCAL_PORT         9999           // local port to listen for UDP packets
#define NTP_PACKET_SIZE        48             // NTP time stamp is in the first 48 bytes of the message
#define NTP_SECS_YR_1900_2000 (3155673600UL)
#define NTP_SECS_YR_1970_2000 (946684800UL)
#define NTP_USELESS_BYTES      40             // Set useless to 32 for speed; set to 40 for accuracy.
#define NTP_POLL_INTV          100            // poll response this many ms
#define NTP_POLL_MAX           40             // poll response up to this many times
#define SECS_PER_DAY           86400
#define SECS_PER_HOUR          3600
#define SECS_PER_MIN           60
const uint32_t ntpFirstFourBytes = 0xEC0600E3;// NTP request header
EthernetUDP udp;                              // A UDP instance to let us send and receive packets over UDP

// WebServer
#define WEBDUINO_FAVICON_DATA     "" // disable favicon
#define WEBDUINO_AUTH_REALM       "OHS"
#define WEBDUINO_SERIAL_DEBUGGING 0
#define PREFIX                    ""
#include "WebServer.h"
WebServer webserver(PREFIX, 80);
#include "html.h"

// MQTT 
#include <PubSubClient.h>
// Callback function header
void callback(char* topic, byte* payload, unsigned int length);

#ifdef TEST
char str_MQTT_clientID[]  = "OHS2";
char str_MQTT_Subscribe[] = "OHS2/In/#";
#else
char str_MQTT_clientID[]  = "OHS";
char str_MQTT_Subscribe[] = "OHS/In/#";
#endif
EthernetClient MQTTethClient;

PubSubClient client(MQTTethClient);

//SMTP
#include <Client.h>
EthernetClient SMTPethClient;

#include <RFM69.h>
#include <RFM69_ATC.h>
// KEY is stored in EEPROM
#define RADIO_NODEID      1
#define RADIO_NETWORKID   100
// RADIO_FREQUENCY is stored in EEPROM //#define RADIO_FREQUENCY   RF69_868MHZ //Match this with the version of your Radio!
#define RADIO_ACK_TIME    5     // # of LOOPS to wait for an ack
#define RADIO_RETRY       5     // # of retry 
#define ENABLE_ATC              // comment out this line to disable AUTO TRANSMISSION CONTROL
#define RADIO_UNIT_OFFSET 15    // Radio nodes start at local address 17, since they share local addresses with wired nodes 
//#define ATC_RSSI          -20
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

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
DateTime time_temp, time_started, timestamp, time_dst, time_std; // Global datetime variables

// RS 485
#include <NilRS485.h> 
RS485_msg RX_msg, TX_msg;

// Global configuration for in chip EEPROM
#include <avr/eeprom.h>

#include <NilGSM.h>
//#define Serial GSM  // redefine GSM as standard Serial 0

char sms_text[60];

// NilRTOS FIFO
#include <NilFIFO.h>

// Logs, alerts
#define LOG_MESSAGE 11
struct log_event_t {
  char     text[EEPROM_MESSAGE];
  //uint32_t time;
  //uint8_t  type; 
};
NilFIFO<log_event_t, 9> log_fifo;

/*
struct alert_event_t {
  char    text[EEPROM_MESSAGE];
  uint8_t type;
};
*/
NilFIFO<log_event_t, 5> alert_fifo;

#define WEB_DEBUGGING 1
#ifdef WEB_DEBUGGING
#include <WebSerial.h>
#endif 

// Global configuration and default setting
#include "conf.h"

struct alarm_event_t {
  uint8_t zone;
  char    type;
};
NilFIFO<alarm_event_t, 3> alarm_fifo;  // Queue must be equal to # of AET 

struct zone_t {
  uint32_t last_PIR = 0;
  uint32_t last_OK  = 0;
  char last_event   = 'N';
  //                   |- Full fifo queue flag
  //                   ||- Free
  //                   |||- Free
  //                   ||||- Free
  //                   |||||- Free
  //                   ||||||- Free 
  //                   |||||||- Alarm 
  //                   ||||||||- Free
  //                   76543210
  uint8_t setting   = B00000000;  
};
volatile zone_t zone[ALR_ZONES];

struct group_t {  
  //                   |- Disabled group log once flag
  //                   ||- Free
  //                   |||- Free
  //                   ||||- Free
  //                   |||||- Free
  //                   ||||||-  Waiting for authorization
  //                   |||||||-  Alarm
  //                   ||||||||-  Armed
  //                   76543210
  uint8_t setting   = B00000000;
  uint8_t arm_delay = 0;
};
volatile group_t group[ALR_GROUPS];

// Dynamic nodes
struct node_t {
  uint8_t address  = 0;
  char    function = ' ';
  char    type     = ' ';
  uint8_t number   = 0;
//                    |- MQTT publish
//                    ||- Free    
//                    |||- Free
//                    |||||||- Group number
//                    |||||||- 0 .. 15
//                    |||||||-  
//                    |||||||- 
//                    ||||||||-  Enabled   
//                    76543210
  uint16_t setting = B00011110;  // 2 bytes to store zone setting
  float    value   = 0;
  uint32_t last_OK = 0;
  char name[NAME_LEN] = "";  
  uint8_t  queue   = 255; // No queue
//  uint16_t failed  = 0;
//  int8_t      RSSI = -127;
};
#define NODES 50
node_t node[NODES];
volatile uint8_t nodes = 0;

// Message queue for sleeping nodes
struct node_queue_t {
  uint8_t  address      = 0;
  //char     function     = ' ';
  //char     type         = ' ';
  //uint8_t  number       = 0;
  uint8_t  index        = 0;
  uint32_t expire       = 0;
  char     msg[REG_LEN] = "";  
  uint8_t  length       = 0;
};
#define NODE_QUEUE 16
node_queue_t node_queue[NODE_QUEUE];

// Sensor fifo
struct sensor_t {
  uint8_t address  = 0;
  char    function = ' ';
  char    type     = ' ';
  uint8_t number   = 0;
  float   value    = 0;
};
NilFIFO<node_t, 15> sensor_fifo;

// Registration fifo
struct register_t {
  uint8_t  address  = 0;
  char     node     = ' ';
  char     type     = ' ';
  uint8_t  number   = 0;
  uint16_t setting  = 0;
  char     name[NAME_LEN] = "";
};
NilFIFO<register_t, 6> reg_fifo;

// Float conversion 
union u_tag {
  byte  b[4]; 
  float fval;
} u;

// uint32_t conversion 
union l_tag {
  byte     b[4]; 
  uint32_t lval;
} l;

// Triggers 
struct trigger_t {
  uint8_t address      = 0;
  char    type         = 'U';
  uint8_t number       = 0;
//                          |- Off_time: 0 Secods, 01 Minutes,
//                          ||-          10 Hours, 11 Days 
//                          |||- 
//                          ||||- 
//                          |||||- 
//                          ||||||- 
//                          |||||||- 
//                          ||||||||- Pass off timer
//                          ||||||||         |- Pass negative once
//                          ||||||||         ||- Logging enabled 
//                          ||||||||         |||- Is triggered
//                          ||||||||         ||||- Pass once / pass always
//                          ||||||||         |||||- Passed
//                          ||||||||         ||||||- Pass
//                          ||||||||         |||||||- Pass value or constant
//                          ||||||||         ||||||||- Enabled   
//                          54321098         76543210
  uint16_t setting       = B00000000 << 8 | B00000000;
  uint8_t symbol         = 1;
  float   value          = 0;
  float   constant_on    = 1;
  float   constant_off   = 0;
  uint8_t to_address     = 0;
  char    to_type        = 'U';
  uint8_t to_number      = 0;
  uint8_t off_time       = 1;
  uint32_t next_off      = 0; 
  char    name[NAME_LEN] = "-";
  float   hysteresis     = 0;
};
#define TRIGGERS 10
trigger_t trigger[TRIGGERS];
#define TRIGGER_SYMBOLS 5
char trigger_symbol[] = "A=!<>";

// TIMERS
struct timer_t {
//                         |- Run type: 0 Secods, 01 Minutes,
//                         ||-          10 Hours, 11 Days 
//                         |||- Period type: 0 Secods, 01 Minutes,
//                         ||||-             10 Hours, 11 Days 
//                         |||||- 
//                         ||||||- 
//                         |||||||- Triggered
//                         ||||||||- Sunday
//                         ||||||||         |- Monday
//                         ||||||||         ||- Tuesday
//                         ||||||||         |||- Wednesday
//                         ||||||||         ||||- Thursday
//                         ||||||||         |||||- Friday
//                         ||||||||         ||||||- Saturday
//                         ||||||||         |||||||- Calendar / Period
//                         ||||||||         ||||||||-  Enabled   
//                         54321098         76543210
  uint16_t setting      = B00000000 << 8 | B00000000;
  uint8_t  period       = 1; // period interval
  uint16_t start_time   = 0; // for calendar timer in minutes 0 - 1440
  uint8_t  run_time     = 1; // runtime interval
  int16_t  constant_on  = 1; // value to pass 
  int16_t  constant_off = 0; // value to pass 
  uint8_t  to_address   = 0; 
  char     to_type      = 'U';
  uint8_t  to_number    = 0;
  uint32_t next_on      = 0;
  uint32_t next_off     = 0;
  char     name[NAME_LEN] = "-";
  uint8_t  asc_trigger  = 0; // 0 = none; 1 > triggers
};
#define TIMERS 10
timer_t timer[TIMERS];



// Free ticks
#define idleSlots                  10
volatile uint32_t idleCount[idleSlots];           
volatile uint8_t  idlePointer    = 0;
// Global variables
volatile uint8_t  ACState        = 0;
volatile uint8_t  OUTs           = 0;         // Output pins
volatile uint8_t  MQTTState      = 0;
volatile uint32_t radio_no_ack   = 0;
volatile uint32_t radio_received = 0;
volatile uint32_t radio_sent     = 0;
volatile uint32_t radio_failed   = 0;


char last_key[KEY_LEN+1] = "\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF";
char _tmp[LOG_MESSAGE];   // for logger 
char b64_text[EMAIL_LEN]; // encode text
// GSM modem 
int8_t  GSMisAlive = 0, GSMsetSMS = 0;
uint8_t GSMreg = 4, GSMstrength = 0;
// 
uint8_t n = 0; 

// Pins setting
DigitalPin<0>  pinBAT_OK(INPUT); // The signal is "Low" when the voltage of battery is under 11V 
DigitalPin<3>  pinAC_OFF(INPUT); // The signal turns to be "High" when the power supply turns OFF
DigitalPin<12> pinDE1(OUTPUT);
DigitalPin<13> pinGSM_PWR(OUTPUT);
DigitalPin<14> pinOUT1(OUTPUT);
DigitalPin<15> pinOUT2(OUTPUT);
DigitalPin<18> pinIN1(INPUT);
DigitalPin<19> pinIN2(INPUT);
DigitalPin<20> pinIN3(INPUT);
DigitalPin<21> pinIN4(INPUT);
DigitalPin<22> pinIN5(INPUT);
DigitalPin<23> pinGSM_ON(INPUT);

// Declare and initialize a semaphore for limiting access to a region.
SEMAPHORE_DECL(ADCSem, 1);   // one slot only
SEMAPHORE_DECL(TWISem, 1);   // one slot only
SEMAPHORE_DECL(GSMSem, 1);   // one slot only
SEMAPHORE_DECL(RFMSem, 1);   // one slot only
SEMAPHORE_DECL(ETHSem, 1);   // one slot only

// *********************************************************************************
// F U N C T I O N S                F U N C T I O N S              F U N C T I O N S      
// *********************************************************************************

// Put string into a fifo log
void pushToLog(char *what, uint8_t size = 0){ 
  //uint8_t _pos = 5;
  log_event_t* p = log_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
  if (p == 0) return; // return if no free space.
  
  memset (p->text, '\0', EEPROM_MESSAGE);
  if (size == 0) size = strlen(what);

  l.lval = timestamp.get(); 
  memcpy(p->text, l.b, 4);         // copy timestamp, p->text[4] = 0 already set 
  memmove(p->text+5, what, size);  // copy "what"

  /* Replaced by memcpy and memmove
  p->text[0] = l.b[0]; p->text[1] = l.b[1]; p->text[2] = l.b[2]; p->text[3] = l.b[3];
  p->text[4] = 0; 
  while ((_pos-5 < size) && (_pos < EEPROM_MESSAGE)) {
    p->text[_pos] = what[_pos-5];
    //WS.print(p->text[_pos]);WS.print(F("-")); WS.print(p->text[_pos], DEC);WS.print(F(", "));
    _pos++;
  }
  */
  log_fifo.signalData();  // Signal idle thread data is available.
}

// wait a few milliseconds for proper ACK to me, return true if indeed received
static bool waitForAck(uint8_t nodeID) {
  uint8_t _try = 0;
  while ( _try < RADIO_ACK_TIME) {
    nilThdSleepMilliseconds(5);
    if (radio.ACKReceived(nodeID)) return true;
    _try++;
  } 
  return false;
}

// Send a command to node
uint8_t sendCmd(uint8_t node, uint8_t cmd){ 
  uint8_t _resp = 0;
  WS.print(F("Cmd to node: ")); 
  // Wired
  if (node <= RADIO_UNIT_OFFSET) {
    TX_msg.address = node;
    TX_msg.ctrl = FLAG_CMD;
    TX_msg.data_length = cmd;
    _resp = RS485.msg_write(&TX_msg);
    WS.print(node); WS.print(F(",")); 
    WS.print(F(" W:")); WS.println(_resp);
  }
  // Radio
  if (node == RADIO_UNIT_OFFSET) { // allow command to be send to radio broadcast
    char _cmd[2];
    _cmd[0] = 'C'; _cmd[1] = cmd;
    nilSemWait(&RFMSem); 
    radio.send(255, _cmd, 2, false);
    nilSemSignal(&RFMSem);  // Exit region.
    radio_sent++;
    _resp = 1;
    WS.print(F(" R:B")); WS.println();
  }
  if (node > RADIO_UNIT_OFFSET) { 
    char _cmd[2];
    _cmd[0] = 'C'; _cmd[1] = cmd;
    nilSemWait(&RFMSem); 
    radio.send(node-RADIO_UNIT_OFFSET, _cmd, 2, true);
    radio_sent++;
    _resp = waitForAck(node-RADIO_UNIT_OFFSET);
    if (!_resp) ++radio_no_ack;
    nilSemSignal(&RFMSem);  // Exit region.
    WS.print(node-RADIO_UNIT_OFFSET); WS.print(F(",")); 
    WS.print(F(" RC:")); WS.println(_resp);
  }
  return _resp;
}

// Send data to node
uint8_t sendData(uint8_t node, char *data, uint8_t length){ 
  uint8_t _resp = 0;
  uint8_t _try = 0;
  #ifdef WEB_DEBUGGING
  WS.print(F("Data to node: ")); WS.print(node); WS.print(F(", len:")); WS.print(length);
  for (uint8_t i=0; i < length; i++){
    WS.print(F("-")); WS.print((uint8_t)data[i],HEX); 
  }
  WS.print(F(" RC: "));
  #endif
  if (node <= RADIO_UNIT_OFFSET) {
    TX_msg.address = node;
    TX_msg.ctrl = FLAG_DTA;
    TX_msg.data_length = length;
    memcpy(TX_msg.buffer, data, length);
    do {
      _resp = RS485.msg_write(&TX_msg);
      _try++;
    } while ((_try < RADIO_RETRY) && (!_resp));
  } else {
    nilSemWait(&RFMSem); 
    do {
      radio.send(node-RADIO_UNIT_OFFSET, data, length, true);
      radio_sent++;
      _resp = waitForAck(node-RADIO_UNIT_OFFSET);
      if (!_resp) ++radio_no_ack;
      _try++;
    } while ((_try < RADIO_RETRY) && (!_resp));
    if (!_resp) ++radio_failed;
    nilSemSignal(&RFMSem);  // Exit region.
  }
  WS.println(_resp);
  return _resp;
}

// Send a command to all members of a group
uint8_t sendCmdToGrp(uint8_t grp, uint8_t cmd, char type = 'K') {
  int8_t  _resp, _try;
  uint8_t _cnt = 0;
  char _cmd[2];
  _cmd[0] = 'C'; _cmd[1] = cmd;
  // Go through all nodes
  WS.print(F("Group:")); WS.print(grp); WS.print(F(" cmd:")); WS.print(cmd); WS.print(F(", node:"));
  for (int8_t i=0; i < nodes; i++){
    if (node[i].setting & B1) {                       // node is enabled ?
      // Auth. node belong to group                     function of node = 'Key' ?
      if ((((node[i].setting >> 1) & B1111) == grp) && (type == node[i].function)) {  
        nilThdSleepMilliseconds(10); // give transmitter some time to finish previous packet
        if (node[i].address <= RADIO_UNIT_OFFSET) {
          TX_msg.address = node[i].address;
          TX_msg.ctrl = FLAG_CMD;
          TX_msg.data_length = cmd;
          _try = 0;
          do {
            _resp = RS485.msg_write(&TX_msg);
            _try++;
            //WS.println(); WS.print(F("try:")); WS.print(_try);
          } while ((_try < RADIO_RETRY) && (!_resp));
          if (!_resp) ++radio_failed;
          if (_resp) _cnt++;
          WS.print(node[i].address); WS.print(":"); WS.print(_resp); WS.print(", ");
        } else {
          nilSemWait(&RFMSem);
          _try = 0;
          do {
            radio.send(node[i].address-RADIO_UNIT_OFFSET, _cmd, 2, true);
            radio_sent++;
            _resp = waitForAck(node[i].address-RADIO_UNIT_OFFSET);
            _try++;
            if (_resp) _cnt++;
            else       radio_no_ack++;
          } while ((_try < RADIO_RETRY) && (!_resp));
          nilSemSignal(&RFMSem);  // Exit region.
          if (!_resp) ++radio_failed;
          WS.print(node[i].address-RADIO_UNIT_OFFSET); WS.print(":"); WS.print(_resp); WS.print(", ");
        }
      }
    }
  }
  WS.println();
  return _cnt;
}

void saveConf(uint8_t log = 1){
  // Save current configuration
  eeprom_update_block((const void*)&conf,    (void*)0           , sizeof(conf)); 
  eeprom_update_block((const void*)&trigger, (void*)sizeof(conf), sizeof(trigger));
  eeprom_update_block((const void*)&timer,   (void*)sizeof(conf) + sizeof(trigger), sizeof(timer)); 
  if (log) {
    pushToLog("SCW");
  }
}

// Clear EEPROM log
void clearLog(){
  //nilSysLock(); // Lock RTOS  
  nilSemWait(&TWISem);     // wait for slot
  for (uint16_t i = 0; i < (EEPROM_SIZE/EEPROM_MESSAGE); i++){
    eeprom.writeBytes(i*EEPROM_MESSAGE,EEPROM_MESSAGE, "\x00\x00\x00\x00\x00- Empty - \x00");
    nilThdSleepMilliseconds(5);  // wait for eeprom or we lose very frequent entries
  }
  nilSemSignal(&TWISem);   // Exit region.
  conf.ee_pos = 0;
  //nilSysUnlock(); // unlock RTOS
  pushToLog("SL");
  saveConf();
}

//Year = 20**
//0=First, Second, Third, Fourth, or Last week of the month
//day of week, 0=Sun, 1=Mon, ... 6=Sat
//1=Jan, 2=Feb, ... 12=Dec
//0-23 
DateTime CalculateDST(uint16_t year, uint8_t week, uint8_t dow, uint8_t month, uint8_t hour){
  uint8_t _m = month, _w = week;  //temp copies of month and week
  
  if (_w == 0) {          //Last week = 0
    if (_m++ > 12) {      //for "Last", go to the next month
        _m = 1;
        year++;
    }
    _w = 1;               //and treat as first week of next month, subtract 7 days later
  }

  DateTime _t(year, _m, 1, hour, 0, 0); //first day of the month, or first day of next month for "Last" rules
  //WS.println(_t.formatedDateTime());
  _t.set(_t.get() + (7 * (_w - 1) + (dow - _t.dayOfWeek() + 7) % 7) * SECS_PER_DAY);
  //WS.println(_t.formatedDateTime());
  if (week == 0) _t = _t.get() - (7 * SECS_PER_DAY);    //back up a week if this is a "Last" rule
  //WS.println(_t.formatedDateTime());
  return _t;
}

DateTime GetNTPTime(UDP &udp){
  if (!udp.begin(NTP_LOCAL_PORT)) return 0; // Fail if WiFiUdp.begin() could not init a socket
  udp.flush();              // Clear received data from possible stray received packets
  
  // Send an NTP request 
  if (!(udp.beginPacket(conf.ntp_ip, 123)      // 123 is the NTP port
    && (udp.write((byte *)&ntpFirstFourBytes, NTP_PACKET_SIZE) == NTP_PACKET_SIZE)
    && udp.endPacket() )) {
    udp.stop();
    return 0;       // sending request failed
  }

  // Wait for response
  uint8_t _pktLen, _pool;
  for (_pool=0; _pool<NTP_POLL_MAX; _pool++) {    
    _pktLen = udp.parsePacket();
    if (_pktLen == NTP_PACKET_SIZE) break;
    nilThdSleepMilliseconds(NTP_POLL_INTV);
  }
  if (_pktLen != NTP_PACKET_SIZE) {
    udp.stop();
    return 0;       // no correct packet received
  }
  // Read and discard the first useless bytes
  for (byte i = 0; i < NTP_USELESS_BYTES; ++i) {
    udp.read();
  }
  // Read the integer part of sending time
  uint32_t time = (uint32_t)udp.read() << 24;
  time |= (uint32_t)udp.read() << 16;
  time |= (uint32_t)udp.read() << 8;
  time |= (uint32_t)udp.read();
  // Round to the nearest second if we want accuracy
  // The fractionary part is the next byte divided by 256: if it is
  // greater than 500ms we round to the next second; we also account
  // for an assumed network delay of 50ms, and (0.5-0.05)*256=115;
  // additionally, we account for how much we delayed reading the packet
  // since its arrival, which we assume on average to be pollIntv/2.
  time += (udp.read() > 115 - _pool/2);
  udp.flush();  // Discard the rest of the packet
  udp.stop();

  time -= NTP_SECS_YR_1900_2000;

  //time_temp.set(time);
  //WS.print(F("UTC: ")); WS.println(time_temp.formatedDateTime());
  time_temp.set(time + (conf.time_std_offset * SECS_PER_MIN));
  //WS.print(F("Local: ")); WS.println(time_temp.formatedDateTime());

  time_dst = CalculateDST(time_temp.year(), conf.time_dst_week, conf.time_dst_dow,
                           conf.time_dst_month, conf.time_dst_hour);
  time_std = CalculateDST(time_temp.year(), conf.time_std_week, conf.time_std_dow,
                          conf.time_std_month, conf.time_std_hour);
  //WS.print(F("DST End: ")); WS.println(time_std.formatedDateTime());  
  //WS.print(F("DST Start: ")); WS.println(time_dst.formatedDateTime());  
  
  if ((time_temp.get() >= time_dst.get()) && (time_temp.get() <= time_std.get())) {
    conf.setting |= (1 << 1); // switch ON DST flag
    time_temp.set(time_temp.get() + ((conf.time_dst_offset - conf.time_std_offset) * SECS_PER_MIN));
    //WS.print(F("DST flag On"));
  } else {
    //WS.print(F("DST flag Off"));
    conf.setting &= ~(1 << 1); // switch OFF DST flag
  }

  //WS.print(F("Local + DST: ")); WS.println(time_temp.formatedDateTime());
  //return time - NTP_SECS_YR_1900_2000 + ((conf.time_zone + ((conf.setting >> 1) & B1)) * SECS_PER_MIN); // convert NTP time to seconds after 2000
  return time_temp;
}

void PublishNode(uint8_t _node){
  char _text[40];
  char _value[10];
  //   MQTT connected           MQTT node enabled                   MQTT publish
  if ((client.connected()) && ((node[_node].setting >> 7) & B1) && (conf.mqtt & B1)) {
    _text[0] = 0;
    // Create MQTT string
    strcat(_text, str_MQTT_clientID); strcat(_text, "/");
    switch(node[_node].function){
      case 'I': strcat_P(_text, (char*)text_Input); break;
      default : strcat_P(_text, (char*)text_Sensor); break;
    }
    strcat(_text, "/");
    //strcat(_text, conf.group_name[(node[_node].setting >> 1) & B1111]); strcat(_text, "/");
    strcat(_text, node[_node].name); strcat(_text, "/");
    switch(node[_node].type){
      case 'T': strcat_P(_text, (char*)text_Temperature); break;
      case 'H': strcat_P(_text, (char*)text_Humidity); break;
      case 'P': strcat_P(_text, (char*)text_Pressure); break;
      case 'I': strcat_P(_text, (char*)text_Input); break;
      case 'V': strcat_P(_text, (char*)text_Voltage); break;
      case 'D': strcat_P(_text, (char*)text_Digital); break;
      case 'A': strcat_P(_text, (char*)text_Analog); break;
      case 'F': strcat_P(_text, (char*)text_Float); break;
      case 'X': strcat_P(_text, (char*)text_TX_Power); break;
      default : strcat_P(_text, (char*)text_Undefined); break;
    }
    strcat(_text, "/");
    itoa(node[_node].number, _value, 10); strcat(_text, _value);
    // For Digital nodes we publish 0 or 1 not float string
    if (node[_node].type == 'D') dtostrf(node[_node].value, 1, 0, _value); // value to string
    else                         dtostrf(node[_node].value, 6, 2, _value); // value to string
    
    nilSemWait(&ETHSem);    // wait for slot
    client.publish(_text,_value, true);
    nilSemSignal(&ETHSem);  // Exit region.
  }
  if (!client.connected()) {
    if (MQTTState) { pushToLog("SMF"); MQTTState = 0; }
  } 
}

void PublishGroup(uint8_t _group, char state){
  char _text[40];
  char _value[10];
  //   MQTT connected           MQTT publish group
  if ((client.connected()) && ((conf.mqtt >> 3) & B1)) {
    _text[0] = 0;
    // Create MQTT string
    strcat(_text, str_MQTT_clientID); strcat(_text, "/");
    strcat_P(_text, (char*)text_Group); strcat(_text, "/");
    strcat(_text, conf.group_name[_group]); strcat(_text, "/");
    strcat_P(_text, (char*)text_state); 
    _value[0] = 0;
    switch(state){
      case 'A': strcat_P(_value, (char*)text_armed_away); break;
      case 'D': strcat_P(_value, (char*)text_disarmed); break;
      case 'T': strcat_P(_value, (char*)text_triggered); break;
      case 'P': strcat_P(_value, (char*)text_pending); break;
      default : strcat_P(_value, (char*)text_undefined); break;
    }
    nilSemWait(&ETHSem);    // wait for slot
    client.publish(_text,_value, true);
    nilSemSignal(&ETHSem);  // Exit region.
  }
  if (!client.connected()) {
    if (MQTTState) { pushToLog("SMF"); MQTTState = 0; }
  } 
}


// MQTT Callback function
void callback(char* topic, byte* payload, unsigned int length) {
  uint8_t _resp, _found, _number, _i, _update_node;
  char * _pch;
  char _text[40];
  char _message[6];
  // In order to republish this payload, a copy must be made
  // as the orignal payload buffer will be overwritten whilst
  // constructing the PUBLISH packet.
  
  if (length >= 40) length = 39;
  strncpy(_text, (char*)payload, length); _text[length] = 0;
  //WS.print(topic); WS.print(F(" >")); WS.print(_text); WS.print(F("< len:")); WS.println(length);

  // get topic
  _pch = strtok(topic,"/"); // Set pointer
  _pch = strtok(NULL, "/"); // OHS
  _pch = strtok(NULL, "/"); // In
  // Main tree
  if (_pch != NULL) {
    WS.print(_pch); WS.print(F(":"));
    // Pass to Input node
    if ((strcmp(_pch, "Input") == 0) && (conf.mqtt >> 1) & B1)  {
      _pch = strtok(NULL, "/");
      WS.print(_pch); WS.print(F(":"));
      if (_pch != NULL) {
        // Get node name
        _resp = 1;
        for (_i = 0; _i < nodes; ++_i) {
          _resp = strcmp(_pch, node[_i].name); 
          //WS.print(F("_i:")); WS.print(_i); WS.print(F("_resp:")); WS.println(_resp);
          if (_resp == 0) break;
        }
        //WS.print(F(":_i=")); WS.print(_i);
        // We have name match
        if (_resp == 0) {
          _pch = strtok(NULL, "/");
          // Node local number
          if (_pch != NULL) {
            _number = (uint8_t)_pch[0]-48;
            WS.print(_number);
            // Select input node to update its value and timestamp
            _found = 0;
            for (_update_node = 0; _update_node < nodes; _update_node++) {
              /*
              WS.println();
              WS.print(node[_update_node].address); WS.print(F(":"));
              WS.print(node[_update_node].function); WS.print(F(":"));
              WS.print(node[_update_node].number); WS.print(F(":"));
              WS.print(node[_update_node].setting & B1); WS.print(F(":"));
              */
              if (node[_update_node].address  == node[_i].address &&
                  node[_update_node].function == 'I' &&
                  node[_update_node].number   == _number &&
                  node[_update_node].setting & B1) {
                _found = 1;
                WS.print(F(":Found "));
                break;
              }
            }
            if (_found) {
              u.fval = atof(_text);                         // convert payload to float
              _message[0] = 'I'; // 'I'nput only
              _message[1] = _number;
              _message[2] = u.b[0]; _message[3] = u.b[1]; _message[4] = u.b[2]; _message[5] = u.b[3];
              if (sendData(node[_update_node].address, _message, 6)) {
                //WS.print(F(":SD"));
                node[_update_node].last_OK = timestamp.get(); // update receiving node current timestamp
                node[_update_node].value   = u.fval;          // update receiving node value
              } else {
                //WS.print(F(":NS"));
              }
            }
            else {
              //WS.print(F(" NF"));
            }
          }
        }
      }
    }
    // Send SMS
    else if ((strcmp(_pch, "SMS") == 0) && (conf.mqtt >> 2) & B1) {

    }
  }
  /*while (_pch != NULL){
    WS.println(_pch);

    _pch = strtok(NULL, "/");
  }*/
}

const char PROGMEM b64_alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                    "abcdefghijklmnopqrstuvwxyz"
                                    "0123456789+/";

inline void a3_to_a4(unsigned char * a4, unsigned char * a3) {
  a4[0] =  (a3[0] & 0xfc) >> 2;
  a4[1] = ((a3[0] & 0x03) << 4) + ((a3[1] & 0xf0) >> 4);
  a4[2] = ((a3[1] & 0x0f) << 2) + ((a3[2] & 0xc0) >> 6);
  a4[3] =  (a3[2] & 0x3f);
}

//uint8_t base64_encode(char *output, char *input, uint8_t inputLen) {
void base64_encode(char *output, char *input, uint8_t inputLen) {  
  uint8_t i = 0, j = 0, encLen = 0;
  unsigned char a3[3];
  unsigned char a4[4];
  while(inputLen--) {
    a3[i++] = *(input++);
    if(i == 3) {
      a3_to_a4(a4, a3);
      for(i = 0; i < 4; i++) {
        output[encLen++] = pgm_read_byte(&b64_alphabet[a4[i]]);
      }
      i = 0;
    }
  }

  if(i) {
    for(j = i; j < 3; j++) { a3[j] = 0; }
    a3_to_a4(a4, a3);
    for(j = 0; j < i + 1; j++) {
      output[encLen++] = pgm_read_byte(&b64_alphabet[a4[j]]);
    }
    while((i++ < 3)) {
      output[encLen++] = '=';
    }
  }
  output[encLen] = 0;
//  return encLen;
}

void set_timer(uint8_t _timer, uint8_t _restart = 1) {
  uint8_t _day, _found;
  uint32_t _add, _tmpt;
  time_temp = timestamp; // .get();

  // Calendar
  if (timer[_timer].setting >> 1 & B1) {
    WS.print(F("CAL# ")); WS.print(_timer);
    _day = time_temp.dayOfWeek(); _found = 0; _add = 0;
    WS.print(F(" Today: ")); WS.print(_day);
    for (uint8_t i = 0; i < 8; i++) {
      // _day = 1 = Monday, _day = 0 = Sunday
      if ((timer[_timer].setting >> 8-_day) & B1) {
        if (_day != time_temp.dayOfWeek()) {_found = 1; break; }
        else {
          // Today start_time has passed already?
          WS.print(F(" Time: ")); WS.print(time_temp.hour()*60 + time_temp.minute()); WS.print(F("~")); WS.print(timer[_timer].start_time);
          if (((time_temp.hour()*60 + time_temp.minute()) > timer[_timer].start_time) &&
            _add == 0) _add++;
          else { _found = 1; break; }
        }
      } else _add++;
      _day++; 
      if (_day == 7) _day = 0;
      WS.print(F(" _day: ")); WS.print(_day);
    }
    if (_found) {
      WS.print(F(" _add: ")); WS.print(_add);
      // Calculate On time
      timer[_timer].next_on = time_temp.get() - ((uint32_t)time_temp.hour()*SECS_PER_HOUR + (uint32_t)time_temp.minute()*60 + (uint32_t)time_temp.second()) + 
        ((uint32_t)timer[_timer].start_time*60) + ((uint32_t)_add * SECS_PER_DAY);
    } else timer[_timer].next_on = 0;
    WS.println();
  // Period
  } else {
    WS.print(F("Period# ")); WS.print(_timer);
    switch(timer[_timer].setting >> 12 & B11){
      case 0:  _add = (uint32_t)timer[_timer].period; break;
      case 1:  _add = (uint32_t)timer[_timer].period*SECS_PER_MIN; break;
      case 2:  _add = (uint32_t)timer[_timer].period*SECS_PER_HOUR; break;
      default: _add = (uint32_t)timer[_timer].period*SECS_PER_DAY; break;
    }
    WS.print(F(" _add: ")); WS.print(_add);
    if (_add > 0) { 
      // request come from Web interface, recalculate next_on 
      if (_restart) {
        timer[_timer].next_on = time_temp.get() - ((uint32_t)time_temp.hour()*SECS_PER_HOUR + (uint32_t)time_temp.minute()*SECS_PER_MIN + (uint32_t)time_temp.second()) + 
          ((uint32_t)timer[_timer].start_time*SECS_PER_MIN);
        WS.print(F(" next_on: ")); WS.print(timer[_timer].next_on);
        // if next_on is in past calculate new next_on
        if (time_temp.get() > timer[_timer].next_on) {
          _tmpt = (time_temp.get() - timer[_timer].next_on) / _add; 
          WS.print(F(" _tmpt: ")); WS.print(_tmpt);   
          timer[_timer].next_on += (_tmpt + 1) * _add;
        }
      } else {
        timer[_timer].next_on += _add;
      } 
      WS.print(F(" next_on: ")); WS.print(timer[_timer].next_on);
      WS.println();
    }
  }
  // Set Off time
  timer[_timer].next_off = timer[_timer].next_on;
  switch(timer[_timer].setting >> 14 & B11){
    case 0:  timer[_timer].next_off += (uint32_t)timer[_timer].run_time; break;
    case 1:  timer[_timer].next_off += (uint32_t)timer[_timer].run_time*SECS_PER_MIN; break;
    case 2:  timer[_timer].next_off += (uint32_t)timer[_timer].run_time*SECS_PER_HOUR; break;
    default: timer[_timer].next_off += (uint32_t)timer[_timer].run_time*SECS_PER_DAY; break;
  }
  timer[_timer].setting &= ~(1 << 9); // switch OFF Is triggered
}

void process_triggers(uint8_t _address, char _type, uint8_t _number, float _value) {
  uint8_t _trigger, _found, _update_node;
  uint32_t _add;
  float _to_compare;
  char _message[6];
  for (_trigger = 0; _trigger < TRIGGERS; _trigger++) {
  //  trigger enabled
  if (trigger[_trigger].setting & B1) { 
    if (trigger[_trigger].address == _address && 
        trigger[_trigger].type    == _type &&
        trigger[_trigger].number  == _number) {
      // check value
      _found = 0;
      switch(trigger[_trigger].symbol){
        //case 0: _found = 1; break; // Always
        case 1: if (_value == trigger[_trigger].value) _found = 1; break;
        case 2: if (_value != trigger[_trigger].value) _found = 1; break;
        case 3: 
          if ((trigger[_trigger].setting) >> 5 & B1) _to_compare = trigger[_trigger].value + trigger[_trigger].hysteresis;
          else                                       _to_compare = trigger[_trigger].value - trigger[_trigger].hysteresis;
          if (_value < _to_compare) _found = 1;
          //WS.print(F("Trigger ")); WS.print(_value); WS.print(F(" < ")); WS.println(_to_compare); 
          break;
        case 4: 
          if ((trigger[_trigger].setting) >> 5 & B1) _to_compare = trigger[_trigger].value - trigger[_trigger].hysteresis;
          else                                       _to_compare = trigger[_trigger].value + trigger[_trigger].hysteresis;
          if (_value > _to_compare) _found = 1;
          //WS.print(F("Trigger ")); WS.print(_value); WS.print(F(" > ")); WS.println(_to_compare); 
          break;
        default: _found = 1; break; // Always
      }
      if (_found) {
        //WS.println(F("trigger on"));
        //    Logging enabled                          not triggered
        if (((trigger[_trigger].setting) >> 6 & B1) && !((trigger[_trigger].setting) >> 5 & B1)) {
          //WS.println(F("trigger log"));
          _tmp[0] = 'R'; _tmp[1] = 'A'; _tmp[2] = _trigger; _tmp[3] = 0; pushToLog(_tmp);
        }
        trigger[_trigger].setting |= (1 << 5); // switch ON Is triggered
        //WS.println(F("trigger on set"));
        //   Pass
        if ((trigger[_trigger].setting) >> 2 & B1) {
          // Select input node to update its value and timestamp
          _found = 0;
          for (_update_node = 0; _update_node < nodes; _update_node++) {
            if (node[_update_node].address  == trigger[_trigger].to_address &&
                node[_update_node].function == 'I' &&
                node[_update_node].number   == trigger[_trigger].to_number &&
                node[_update_node].setting & B1) { 
              //WS.print("_update_node: "); WS.println(_update_node);
              _found = 1;
              break;
            }
          }
          if (_found) {
            //WS.print(" pass");
            //   off time
            if ((trigger[_trigger].setting) >> 8 & B1) {
              //WS.print(F("Off_time # ")); WS.print(_trigger);
              switch(trigger[_trigger].setting >> 14 & B11){
                case 0:  _add = (uint32_t)trigger[_trigger].off_time; break;
                case 1:  _add = (uint32_t)trigger[_trigger].off_time*SECS_PER_MIN; break;
                case 2:  _add = (uint32_t)trigger[_trigger].off_time*SECS_PER_HOUR; break;
                default: _add = (uint32_t)trigger[_trigger].off_time*SECS_PER_DAY; break;
              }
              //WS.print(F(" _add: ")); WS.print(_add); WS.print(F(" , "));
              trigger[_trigger].next_off = timestamp.get() + _add;
              //WS.print(timestamp.get()); WS.print(F(" next_off: ")); WS.print(trigger[_trigger].next_off);
              //WS.println();
            }
            //     passed 
            if (!((trigger[_trigger].setting) >> 3 & B1)) {
              _message[0] = 'I'; // 'I'nput only
              _message[1] = trigger[_trigger].to_number;
              if ((trigger[_trigger].setting) >> 1 & B1) { u.fval = _value; }
              else                                       { u.fval = trigger[_trigger].constant_on; }
              _message[2] = u.b[0]; _message[3] = u.b[1]; _message[4] = u.b[2]; _message[5] = u.b[3];
              if (sendData(trigger[_trigger].to_address, _message, 6)) {
                node[_update_node].last_OK = timestamp.get(); // update receiving node current timestamp
                // update receiving node value
                if ((trigger[_trigger].setting) >> 1 & B1) { node[_update_node].value = _value; }
                else                { node[_update_node].value = trigger[_trigger].constant_on; }
                //   pass only once
                if ((trigger[_trigger].setting) >> 4 & B1) {
                  trigger[_trigger].setting |= (1 << 3); // switch ON passed
                }
                PublishNode(_update_node); // MQTT
              }
            }
          } //_found
        }  
      } else { // Off trigger
        //WS.println(F("trigger off"));
        //    Logging enabled                            triggered
        if (((trigger[_trigger].setting) >> 6 & B1) && ((trigger[_trigger].setting) >> 5 & B1)) {
          //WS.println(F("trigger log"));
          _tmp[0] = 'R'; _tmp[1] = 'D'; _tmp[2] = _trigger; _tmp[3] = 0; pushToLog(_tmp);
        }
        //    Pass Off enabled                           Is alerted/triggered
        if (((trigger[_trigger].setting) >> 7 & B1) && ((trigger[_trigger].setting) >> 5 & B1)) {
          // Select input node to update its value and timestamp
          _found = 0;
          for (_update_node = 0; _update_node < nodes; _update_node++) {
            if (node[_update_node].address  == trigger[_trigger].to_address &&
                node[_update_node].function == 'I' &&
                node[_update_node].number   == trigger[_trigger].to_number &&
                node[_update_node].setting & B1) { 
              //WS.print("F_update_node: "); WS.println(_update_node);
              _found = 1;
              break;
            }
          }
          if (_found) {
            // Pass Off value
            //WS.print(" pass Off");
            _message[0] = 'I'; // 'I'nput only
            _message[1] = trigger[_trigger].to_number;
            if ((trigger[_trigger].setting) >> 1 & B1) { u.fval = _value; }
            else                                       { u.fval = trigger[_trigger].constant_off; }
            _message[2] = u.b[0]; _message[3] = u.b[1]; _message[4] = u.b[2]; _message[5] = u.b[3];
            if (sendData(trigger[_trigger].to_address, _message, 6)) {
              node[_update_node].last_OK = timestamp.get(); // update receiving node current timestamp
              // update receiving node value  
              if ((trigger[_trigger].setting) >> 1 & B1) { node[_update_node].value = _value; }
              else               { node[_update_node].value = trigger[_trigger].constant_off; }
              trigger[_trigger].setting &= ~(1 << 5); // switch OFF Is alerted/triggered
              trigger[_trigger].setting &= ~(1 << 3); // switch OFF passed
              PublishNode(_update_node); // MQTT
            }
          } //_found
        } // if 
        //    Pass Off NOT enabled                       Is alerted/triggered
        if (!((trigger[_trigger].setting) >> 7 & B1) && ((trigger[_trigger].setting) >> 5 & B1)) {
          // Turn OFF trigger switches
          trigger[_trigger].setting &= ~(1 << 5); // switch OFF Is alerted/triggered
          trigger[_trigger].setting &= ~(1 << 3); // switch OFF passed
          //WS.println(F("trigger off set"));
        }
      } // else            
    } // trigger match
  } // trigger enabled
  } // for (trigger)
}

void formatKey(char* in, char* out) { // Format the key value to nice HEX string
  for (uint8_t ii = 0; ii < KEY_LEN; ++ii) { 
    out[ii*2]   = in[ii] >> 4 & B1111;
    out[ii*2+1] = in[ii] & B1111;
    if (out[ii*2] > 9) out[ii*2] = out[ii*2] + 'A' - 10; 
    else out[ii*2] = out[ii*2] + '0';
    if (out[ii*2+1] > 9) out[ii*2+1] = out[ii*2+1] + 'A' - 10;
    else out[ii*2+1] = out[ii*2+1] + '0';
  }
  out[16] = 0;
}

// Include HTTP SERVER FUNCTIONS
#include "hsf.h"

// *********************************************************************************
// T H R E A D S    T H R E A D S    T H R E A D S    T H R E A D S    T H R E A D S
// *********************************************************************************

// Zone thread
//
NIL_WORKING_AREA(waZoneThread, 128);
NIL_THREAD(ZoneThread, arg) {
  int16_t val;
  uint8_t _group = 255;
  uint8_t _resp;
  
  nilThdSleepSeconds(SECS_PER_MIN); // Delay to allow PIR to settle up
  pushToLog("SS");
  #ifdef WEB_DEBUGGING
  WS.println(F("Zone thread started"));
  #endif
  
  // Execute while loop every 0.25 seconds.
  while (TRUE) {
    nilThdSleepMilliseconds(250); // time is used also for arm delay and others ...

    for (int8_t i=0; i < ALR_GROUPS ; i++){ 
      if (group[i].arm_delay) { // wait for arm delay
        group[i].arm_delay--;
        if (!group[i].arm_delay) { 
          _resp = sendCmdToGrp(i, 15);  // send arm message to all nodes
          _tmp[0] = 'G'; _tmp[1] = 'S'; _tmp[2] = i;  pushToLog(_tmp, 3);
        }
      }
    }
   
    //WS.print(F("Zone: "));
    for (int8_t i=0; i < ALR_ZONES ; i++){  
      if (conf.zone[i] & B1){           // Zone enabled ?
        //WS.print(i+1); WS.print(F(":"));
        if (conf.zone[i] >> 15){        // Digital 0/ Analog 1 
          nilSemWait(&ADCSem);          // Wait for slot
          val = nilAnalogRead(i);
          nilSemSignal(&ADCSem);        // Exit region.
        } else {
          switch(i) {
            case 8:  pinIN1.read() ? val = ALR_PIR : val = ALR_OK; break;
            case 9:  pinIN2.read() ? val = ALR_PIR : val = ALR_OK; break;
            case 10: pinIN3.read() ? val = ALR_PIR : val = ALR_OK; break;
            case 11: pinIN4.read() ? val = ALR_PIR : val = ALR_OK; break;
            case 12: pinIN5.read() ? val = ALR_PIR : val = ALR_OK; break;
            default: break;
          } 
        }
        //    alarm as tamper              is PIR                                    make it tmaper
        if (((conf.zone[i] >> 9) & B1) && (val >= ALR_PIR_LOW && val <= ALR_PIR_HI)) val += ALR_PIR;
        //WS.print(val); WS.print(F(", "));
        //WS.print(i); WS.print(F(","));
        _group = (conf.zone[i] >> 1) & B1111; // set group
        // Decide 
        switch((int16_t)(val)){
          case ALR_OK_LOW ... ALR_OK_HI:
            //WS.print(F("O,"));
            // All is OK no action
            zone[i].last_event = 'O';
            zone[i].last_OK = timestamp.get();    // update current timestamp
            break;
          case ALR_PIR_LOW ... ALR_PIR_HI:
            //WS.println(); WS.print(F("Zone:")); WS.print(i);
            //WS.print(F(",Gr:")); WS.print(_group); WS.print(F(", ")); 
            //     zone not have alarm              group delay is 0
            if (!((zone[i].setting >> 1) & B1) && (group[_group].arm_delay == 0)){
              //WS.print(F("+ "));
              // if group not enabled log error to log. 
              if (!(conf.group[_group] & B1)) {
                if (!((group[_group].setting >> 7) & B1)) {
                  group[_group].setting |= (1 << 7); // Set logged disabled bit On
                  _tmp[0] = 'G'; _tmp[1] = 'F'; _tmp[2] = _group;  pushToLog(_tmp, 3);
                }
              } else {
                //WS.print(F(", armed:")); WS.print(group[_group].setting & B1);
                //WS.print(F(", last:")); WS.print(zone[i].last_event);
                //   group armed
                if ((group[_group].setting & B1) && (zone[i].last_event == 'P')) {
                  //WS.print(F(" = PIR"));
                  alarm_event_t* p = alarm_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
                  if (p == 0) {
                    if (!((zone[i].setting >> 7) & B1)) {
                      _tmp[0] = 'Z'; _tmp[1] = 'P'; _tmp[2] = i;  pushToLog(_tmp, 3); 
                      pushToLog("FA"); // Alarm queue is full
                    }
                    zone[i].setting |= (1 << 7); // Set On Alarm queue is full  
                    continue; // Continue if no free space.
                  }
                  _tmp[0] = 'Z'; _tmp[1] = 'P'; _tmp[2] = i;  pushToLog(_tmp, 3); 
                  zone[i].setting |=  (1 << 1); // Set alarm bit On
                  zone[i].setting &= ~(1 << 7); // Set Off Alarm queue is full
                  p->zone = i; p->type = zone[i].last_event; alarm_fifo.signalData(); // Signal idle thread data is available.
                }
              }
            }
            zone[i].last_event = 'P';
            zone[i].last_PIR = timestamp.get();    // update current timestamp
            break;
          default: // Line is cut or short or tamper, no difference to alarm event
            //WS.print(F("T,"));
            //  zone not have alarm
            if (!((zone[i].setting >> 1) & B1)){
              //WS.print(F("N,"));
              // if group not enabled log error to log. 
              if (!(conf.group[_group] & B1)) {
                if (!((group[_group].setting >> 7) & B1)) {
                  group[_group].setting |= (1 << 7); // Set logged disabled bit On
                  _tmp[0] = 'G'; _tmp[1] = 'F'; _tmp[2] = _group;  pushToLog(_tmp, 3);
                }
              } else {
                if (zone[i].last_event == 'T') {
                  //WS.println(F(">T"));
                  WS.print(F("Zone:")); WS.print(i); WS.print(F("T:")); WS.print(val);
                  alarm_event_t* p = alarm_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
                  if (p == 0) {
                    if (!((zone[i].setting >> 7) & B1)) {
                      _tmp[0] = 'Z'; _tmp[1] = 'T'; _tmp[2] = i;  pushToLog(_tmp, 3);
                      pushToLog("FA"); // Alarm queue is full
                    }
                    zone[i].setting |= (1 << 7); // Set On Alarm queue is full                    
                    continue; // Continue if no free space.
                  }
                  _tmp[0] = 'Z'; _tmp[1] = 'T'; _tmp[2] = i;  pushToLog(_tmp, 3);
                  zone[i].setting |=  (1 << 1); // Set alarm bit On
                  zone[i].setting &= ~(1 << 7); // Set Off Alarm queue is full
                  p->zone = i; p->type = zone[i].last_event; alarm_fifo.signalData(); // Signal idle thread data is available.
                }
              }
            }
            zone[i].last_event = 'T';
            zone[i].last_PIR = timestamp.get();    // update current timestamp
            break;
          }    
        
        // Triggers
        if (zone[i].last_event == 'O') val = 0;
        else val = 1;
        process_triggers(0, 'Z', i, (float)val);
      } // zone enabled ?
    } // for (zones)
    //WS.println();
    //watchdog |= (1 << 0); // Flag watchdog for this thread
  } // while thread TRUE
}

//------------------------------------------------------------------------------
// Time thread
//
NIL_WORKING_AREA(waTimeThread, 96);
NIL_THREAD(TimeThread, arg) {
  uint16_t _counter = 0, _year;
  nilTimer1Start(1000000);  
  #ifdef WEB_DEBUGGING
  WS.println(F("Time thread started"));
  #endif

  // Get time
  nilSemWait(&TWISem);     // wait for slot
  timestamp = RTC.now();
  nilSemSignal(&TWISem);   // Exit region.
  // Set started timestamp
  time_started = timestamp;
  // Set DST
  time_dst = CalculateDST(timestamp.year(), conf.time_dst_week, conf.time_dst_dow,
                          conf.time_dst_month, conf.time_dst_hour);
  time_std = CalculateDST(timestamp.year(), conf.time_std_week, conf.time_std_dow,
                          conf.time_std_month, conf.time_std_hour);
  _year = timestamp.year();
  //WS.print(F("DST End: ")); WS.println(time_std.formatedDateTime());
  //WS.print(F("DST Start: ")); WS.println(time_dst.formatedDateTime());  
  // Set DST flag
  if ((timestamp.get() >= time_dst.get()) && (timestamp.get() <= time_std.get())) {
    conf.setting |= (1 << 1); // switch ON DST flag
  } else {
    conf.setting &= ~(1 << 1); // switch OFF DST flag
  }
  
  // Execute while loop at every second.
  while (TRUE) {
    nilTimer1Wait();
    timestamp.set(timestamp.get() + 1);
    _counter++;

    // Daylight Saving Time START
    if ((timestamp.get() == time_dst.get()) && !((conf.setting >> 1) & B1)) {
      conf.setting |= (1 << 1); // switch ON DST flag
      //WS.print(F("DST flag On"));
      timestamp.set(timestamp.get() + ((conf.time_dst_offset - conf.time_std_offset) * SECS_PER_MIN));
      nilSemWait(&TWISem);     // wait for slot
      RTC.adjust(timestamp);
      nilSemSignal(&TWISem);   // Exit region.
    }
    // Daylight Saving Time STOP
    if ((timestamp.get() == time_std.get()) && ((conf.setting >> 1) & B1)) {
      conf.setting &= ~(1 << 1); // switch OFF DST flag
      //WS.print(F("DST flag Off"));
      timestamp.set(timestamp.get() - ((conf.time_dst_offset - conf.time_std_offset) * SECS_PER_MIN));
      nilSemWait(&TWISem);     // wait for slot
      RTC.adjust(timestamp);
      nilSemSignal(&TWISem);   // Exit region.
    }
    // Sync with RTC every uint16_t overfolow
    if (_counter == 0) {
      nilSemWait(&TWISem);     // wait for slot
      timestamp = RTC.now();
      nilSemSignal(&TWISem);   // Exit region.
      if (_year != timestamp.year()) {
        time_dst = CalculateDST(timestamp.year(), conf.time_dst_week, conf.time_dst_dow,
                                conf.time_dst_month, conf.time_dst_hour);
        time_std = CalculateDST(timestamp.year(), conf.time_std_week, conf.time_std_dow,
                                conf.time_std_month, conf.time_std_hour);
        _year = timestamp.year();
      }
    }
    //watchdog |= (1 << 1); // Flag watchdog for this thread
  }
}

//------------------------------------------------------------------------------
// Alarm Events
//
NIL_WORKING_AREA(waAEThread1, 90);
NIL_WORKING_AREA(waAEThread2, 90);
NIL_WORKING_AREA(waAEThread3, 90);
NIL_THREAD(thdFcn, name) {
  uint8_t _group, _wait, _resp, _cnt;

  #ifdef WEB_DEBUGGING
  WS.print((char*)name);  WS.println(F(" started"));
  #endif
  
  while (TRUE) {
    // Check for data.  Use TIME_IMMEDIATE to prevent sleep in idle thread.
    alarm_event_t* p = alarm_fifo.waitData(TIME_INFINITE);

    _group = (conf.zone[p->zone] >> 1) & B1111;
    //WS.print((char*)name);
    //   Group has alarm already nothing to do!
    if ((group[_group].setting >> 1) & B1) {
      //WS.println(F(" stop"));
      alarm_fifo.signalFree();
      continue;
    } 
    //WS.print(F(" go"));

    group[_group].setting |= (1 << 2); // Set Authentication On

    if (p->type == 'P') _wait = (conf.zone[p->zone] >> 5) & B11;
    else                _wait = 0;            // Tamper have no wait time
    /*
    WS.print(F(" zone: ")); WS.print(p->zone); WS.print(F(", type: ")); WS.print(p->type);
    WS.print(F(", group: ")); WS.print(_group); WS.print(F(", Auth time: ")); WS.println(_wait);
    */
    //       wait > 0     NOT group has alarm already              Authentication On   
    while ((_wait > 0) && !((group[_group].setting >> 1) & B1) && (group[_group].setting >> 2 & B1)) {
      _resp = sendCmdToGrp(_group, 11 + _wait);
      WS.print((char*)name); WS.print(F(" w:")); WS.println(_wait);
      _cnt = 0;
      //       Authentication On                   time of one alarm period      NOT group has alarm already
      while ((group[_group].setting >> 2 & B1) && (_cnt < (10*conf.alr_time)) && !((group[_group].setting >> 1) & B1)) {
        nilThdSleepMilliseconds(100);
        _cnt++;
      }
      //  Authentication On          
      if (group[_group].setting >> 2 & B1) _wait--;
    }
    //   wait = 0   NOT group has alarm already  
    if ((!_wait) && !((group[_group].setting >> 1) & B1)) { 
      group[_group].setting |= (1 << 1); // Set alarm bit On
      _resp = sendCmdToGrp(_group, 11);  // ALARM COMMAND
      // Combine alarms, so that next alarm will not disable ongoing one
      if (p->type == 'P') {
        OUTs = ((((conf.group[_group] >> 4) & B1) | (OUTs >> 0) & B1) |
         (((conf.group[_group] >> 3) & B1) | (OUTs >> 1) & B1) << 1);
      } else {
        OUTs = ((((conf.group[_group] >> 2) & B1) | (OUTs >> 0) & B1) |
         (((conf.group[_group] >> 1) & B1) | (OUTs >> 1) & B1) << 1);
      }
      // Trigger OUT 1 & 2              
      pinOUT1.write(((OUTs >> 0) & B1));
      pinOUT2.write(((OUTs >> 1) & B1));
      _tmp[0] = 'S'; _tmp[1] = 'X';  _tmp[2] = _group;  pushToLog(_tmp, 3); // ALARM no auth.
    }
    //WS.print((char*)name); WS.println(F(" end"));
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
  uint8_t _pos = 0;
  uint8_t _node, _found;

  #ifdef WEB_DEBUGGING
  WS.println(F("RS485 thread started"));
  #endif
  
  while (TRUE) {   
    nilWaitRS485NewMsg(); // wait for event
    _resp = RS485.msg_read(&RX_msg);
    
    #ifdef WEB_DEBUGGING
    WS.print(F(">Wire A:")); WS.print(RX_msg.address); WS.print(F(", C:")); WS.print(RX_msg.ctrl);
    WS.print(F(", L:")); WS.print(RX_msg.data_length); WS.print(F(", "));   
    for (uint8_t i=0; i < RX_msg.data_length; i++){
      WS.print((uint8_t)RX_msg.buffer[i],HEX); WS.print(F("-"));
    }; WS.println();
    #endif
      
    // iButtons keys
    if (RX_msg.ctrl == FLAG_DTA && RX_msg.data_length == KEY_LEN) {  // incoming message looks like a key 
      _found = 0;
      // search node with given address but only Keys
      for (_node=0; _node < nodes; _node++) {
        if ((node[_node].address  == RX_msg.address) &&
            (node[_node].function == 'K')) {
          _found = 1; break;
        }
      }
      // address found
      if (_found) {
        node[_node].last_OK = timestamp.get(); // Update timestamp
        //  enabled for authorzation       
        if (node[_node].setting & B1) { 
          for (uint8_t i=0; i < NUM_OF_KEYS; i++){
            _resp = memcmp(RX_msg.buffer, conf.key[i], KEY_LEN); // Compare key
            if (!_resp) { // key matched
              _group = (node[_node].setting >> 1) & B1111;
              //  key enabled && (group = key_group || key = global)
              if ((conf.key_setting[i] & B1) &&
                 ((_group == ((conf.key_setting[i] >> 1) & B1111)) || ((conf.key_setting[i] >> 5) & B1))) {
                //     we have alarm                     or   group is armed
                if  (((group[_group].setting) >> 1 & B1) || ((group[_group].setting) & B1)) { 
                  // we have alarm
                  if ((group[_group].setting) >> 1 & B1) { 
                    group[_group].setting &= ~(1 << 1);    // set group alarm off
                    /* *** ToDo: add bitwise reset of OUTs instead of full reset ? */
                    OUTs = 0; // Reset outs  
                    pinOUT1.write(LOW); pinOUT2.write(LOW); // Turn off OUT 1 & 2
                  }
                  // set each member zone of this group as alarm off
                  for (uint8_t j=0; j < ALR_ZONES; j++){
                    if (((conf.zone[j] >> 1) & B1111) == _group) zone[j].setting &= ~(1 << 1); 
                  }
                  group[_group].setting &= ~(1 << 0);    // disarm group
                  group[_group].setting &= ~(1 << 2);    // set auth bit off
                  group[_group].arm_delay = 0;       // Reset arm delay
                  _resp = sendCmdToGrp(_group, 20);  // send quiet message to all nodes
                  _tmp[0] = 'A'; _tmp[1] = 'D'; _tmp[2] = i;  pushToLog(_tmp, 3); // Key
                  _tmp[0] = 'G'; _tmp[1] = 'D'; _tmp[2] = _group;   pushToLog(_tmp, 3); // Group
                } else { // Just do arm
                  _tmp[0] = 'A'; _tmp[1] = 'A'; _tmp[2] = i; _tmp[3] = 0; pushToLog(_tmp);
                  // if group enabled arm group or log error to log. 
                  if (conf.group[_group] & B1) { 
                    group[_group].setting |= (1 << 0);   // arm group
                    group[_group].arm_delay = conf.arm_delay; // set arm delay
                    _resp = sendCmdToGrp(_group, 10);  // send arming message to all nodes
                  } 
                  else { _tmp[0] = 'G'; _tmp[1] = 'F'; _tmp[2] = _group;  pushToLog(_tmp, 3); }
                }
                break; // no need to try other
              } else { // key is not enabled
                _tmp[0] = 'A'; _tmp[1] = 'F'; _tmp[2] = i;  pushToLog(_tmp, 3);
              }
            } // key matched
            if (_resp!=0 && i==NUM_OF_KEYS-1) { // may be we should log unknow keys 
              //pushToLog("AU");
              _tmp[0] = 'A'; _tmp[1] = 'U'; for (uint8_t ii = 0; ii < KEY_LEN; ++ii) {_tmp[2+ii] = RX_msg.buffer[ii];}
              pushToLog(_tmp, 10);
              memcpy(last_key, RX_msg.buffer, KEY_LEN); // store last unknown key
              last_key[KEY_LEN+1] = 0;
            }
          } // for
        } // node is enabled for authorization
        else { // log disabled remote nodes
          _tmp[0] = 'N'; _tmp[1] = 'F'; _tmp[2] = RX_msg.address; _tmp[3] = 'i';  pushToLog(_tmp, 4);
        } 
      } else { // node not found
        // call this address to register
        nilThdSleepMilliseconds(10);  
        _resp = sendCmd(RX_msg.address,1);
      }
    } // incoming message looks like a key

    // Registration
    if (RX_msg.ctrl == FLAG_DTA && RX_msg.buffer[0]=='R') {
      _pos = 0;
      do {
        _pos++; // Skip 'R'
        register_t *p = reg_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
        if (p == 0) {
          pushToLog("FR"); // Registration queue is full
          continue; // Continue if no free space.
        }
        // node setting
        p->node    = RX_msg.buffer[_pos];    
        p->address = RX_msg.address;
        p->type    = RX_msg.buffer[_pos+1];
        p->number  = (uint8_t)RX_msg.buffer[_pos+2];
        p->setting = ((uint8_t)RX_msg.buffer[_pos+3] << 8) | ((uint8_t)RX_msg.buffer[_pos+4]);
        for (uint8_t _t=0; _t < NAME_LEN; _t++) {p->name[_t] = RX_msg.buffer[_pos+5+_t];}
        reg_fifo.signalData();   // Signal idle thread data is available.
        _pos+=REG_LEN;
      } while (_pos < RX_msg.data_length);
    }
    // Sensors & Inputs feedback
    if ((RX_msg.ctrl == FLAG_DTA) && ((RX_msg.buffer[0]=='S') || (RX_msg.buffer[0]=='I'))){
      _pos = 1;
      do {
        node_t *p = sensor_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
        if (p == 0) {
          pushToLog("FS"); // node queue is full
          continue; // Continue if no free space.
        }
        p->function = RX_msg.buffer[0];
        p->address  = RX_msg.address;
        p->type     = RX_msg.buffer[_pos];
        p->number   = (uint8_t)RX_msg.buffer[_pos+1];
        u.b[0] = RX_msg.buffer[_pos+2]; u.b[1] = RX_msg.buffer[_pos+3]; u.b[2] = RX_msg.buffer[_pos+4]; u.b[3] = RX_msg.buffer[_pos+5];
        p->value    = u.fval;
        sensor_fifo.signalData();     // Signal idle thread data is available.
        _pos+=6;
      } while (_pos < RX_msg.data_length);
    } // if 'S'
  }
}

//------------------------------------------------------------------------------
// Logger thread
//
NIL_WORKING_AREA(waLoggerThread, 64);
NIL_THREAD(LoggerThread, arg) {
  uint8_t alert_type;
  
  #ifdef WEB_DEBUGGING
  WS.println(F("Logger thread started"));
  #endif
  
  while (TRUE) {
    // Check for data.  Use TIME_IMMEDIATE to prevent sleep in idle thread.
    log_event_t* p = log_fifo.waitData(TIME_INFINITE);

    char* log_message = p->text; // Fetch and print data.
    alert_type = 0;
  
    switch(log_message[5]){
      case 'S': // System
        switch(log_message[6]){
          case 'B': // Battery
            if (conf.alerts[alert_SMS] >> ALERT_BATERY_STATE & B1) alert_type |= (1 << alert_SMS); // Set On
            if (conf.alerts[alert_email] >> ALERT_BATERY_STATE & B1) alert_type |= (1 << alert_email); // Set On
            if (conf.alerts[alert_page] >> ALERT_BATERY_STATE & B1) alert_type |= (1 << alert_page); // Set On
          break;
          case 'A': // AC
            if (conf.alerts[alert_SMS] >> ALERT_AC_STATE & B1) alert_type |= (1 << alert_SMS); // Set On
            if (conf.alerts[alert_email] >> ALERT_AC_STATE & B1) alert_type |= (1 << alert_email); // Set On
            if (conf.alerts[alert_page] >> ALERT_AC_STATE & B1) alert_type |= (1 << alert_page); // Set On
          break;
          case 'C':
            if (conf.alerts[alert_SMS] >> ALERT_CONF_SAVED & B1) alert_type |= (1 << alert_SMS); // Set On
            if (conf.alerts[alert_email] >> ALERT_CONF_SAVED & B1) alert_type |= (1 << alert_email); // Set On
            if (conf.alerts[alert_page] >> ALERT_CONF_SAVED & B1) alert_type |= (1 << alert_page); // Set On
          break;
          case 'S': 
            if (conf.alerts[alert_SMS] >> ALERT_MONITORING_STARTED & B1) alert_type |= (1 << alert_SMS); // Set On
            if (conf.alerts[alert_email] >> ALERT_MONITORING_STARTED & B1) alert_type |= (1 << alert_email); // Set On
            if (conf.alerts[alert_page] >> ALERT_MONITORING_STARTED & B1) alert_type |= (1 << alert_page); // Set On
          break;
          case 'X': 
            if (conf.alerts[alert_SMS] >> ALERT_ALARM & B1) alert_type |= (1 << alert_SMS); // Set On
            if (conf.alerts[alert_email] >> ALERT_ALARM & B1) alert_type |= (1 << alert_email); // Set On
            if (conf.alerts[alert_page] >> ALERT_ALARM & B1) alert_type |= (1 << alert_page); // Set On
          break;
        }
      break;
      case 'Z': // Zone
        switch(log_message[6]){
          case 'P': 
            if (conf.alerts[alert_SMS] >> ALERT_PIR & B1) alert_type |= (1 << alert_SMS); // Set On
            if (conf.alerts[alert_email] >> ALERT_PIR & B1) alert_type |= (1 << alert_email); // Set On
            if (conf.alerts[alert_page] >> ALERT_PIR & B1) alert_type |= (1 << alert_page); // Set On
          break;
          case 'T': 
            if (conf.alerts[alert_SMS] >> ALERT_TAMPER & B1) alert_type |= (1 << alert_SMS); // Set On
            if (conf.alerts[alert_email] >> ALERT_TAMPER & B1) alert_type |= (1 << alert_email); // Set On
            if (conf.alerts[alert_page] >> ALERT_TAMPER & B1) alert_type |= (1 << alert_page); // Set On
          break;
          case 'O':
            if (conf.alerts[alert_SMS] >> ALERT_OPEN & B1) alert_type |= (1 << alert_SMS); // Set On
            if (conf.alerts[alert_email] >> ALERT_OPEN & B1) alert_type |= (1 << alert_email); // Set On
            if (conf.alerts[alert_page] >> ALERT_OPEN & B1) alert_type |= (1 << alert_page); // Set On
          break;
        }
      break;
      case 'A': // Authentication
        switch(log_message[6]){
          case 'U': 
          case 'F': 
            if (conf.alerts[alert_SMS] >> ALERT_FALSE_KEY & B1) alert_type |= (1 << alert_SMS); // Set On
            if (conf.alerts[alert_email] >> ALERT_FALSE_KEY & B1) alert_type |= (1 << alert_email); // Set On
            if (conf.alerts[alert_page] >> ALERT_FALSE_KEY & B1) alert_type |= (1 << alert_page); // Set On
          break;
        }
      break;    
      case 'G': // Groups
       switch(log_message[6]){
          case 'D': 
            if (conf.alerts[alert_SMS] >> ALERT_DISARMED & B1) alert_type |= (1 << alert_SMS); // Set On
            if (conf.alerts[alert_email] >> ALERT_DISARMED & B1) alert_type |= (1 << alert_email); // Set On
            if (conf.alerts[alert_page] >> ALERT_DISARMED & B1) alert_type |= (1 << alert_page); // Set On
          break;
          case 'A': 
            if (conf.alerts[alert_SMS] >> ALERT_ARMED & B1) alert_type |= (1 << alert_SMS); // Set On
            if (conf.alerts[alert_email] >> ALERT_ARMED & B1) alert_type |= (1 << alert_email); // Set On
            if (conf.alerts[alert_page] >> ALERT_ARMED & B1) alert_type |= (1 << alert_page); // Set On
          break;
        }
      break;
      case 'F': // Fifo      
        if (conf.alerts[alert_SMS] >> ALERT_FIFO & B1) alert_type |= (1 << alert_SMS); // Set On
        if (conf.alerts[alert_email] >> ALERT_FIFO & B1) alert_type |= (1 << alert_email); // Set On
        if (conf.alerts[alert_page] >> ALERT_FIFO & B1) alert_type |= (1 << alert_page); // Set On  
      break;
      case 'R': // Trigger
        if (conf.alerts[alert_SMS] >> ALERT_TRIGGER & B1) alert_type |= (1 << alert_SMS); // Set On
        if (conf.alerts[alert_email] >> ALERT_TRIGGER & B1) alert_type |= (1 << alert_email); // Set On
        if (conf.alerts[alert_page] >> ALERT_TRIGGER & B1) alert_type |= (1 << alert_page); // Set On  
      break;
      case 'C': // SMS commands
        alert_type |= (1 << alert_SMS); // Set always On
      break;
    }
    log_message[4] = alert_type;

    // Put data into external EEPROM
    nilSemWait(&TWISem);          // wait for slot
    eeprom.writeBytes(conf.ee_pos,EEPROM_MESSAGE, log_message);
    nilSemSignal(&TWISem);        // Exit region.
    nilThdSleepMilliseconds(5);   // wait for EEPROM or we lose frequent entries, but out of TWI region
    
    conf.ee_pos += EEPROM_MESSAGE;// Will overflow by itself as size is uint16_t = EEPROM_SIZE

    log_fifo.signalFree(); // Signal FIFO slot is free.

    // Send data to alert thread
    if (alert_type) {
      log_event_t* q = alert_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
      if (q == 0) {
        pushToLog("FL"); // Alert queue is full
        continue; // Continue if no free space.
      }  
      //strncpy(q->text, log_message, EEPROM_MESSAGE-1); q->text[EEPROM_MESSAGE-1] = 0;
      memcpy(q->text, log_message, EEPROM_MESSAGE);
      alert_fifo.signalData();  // Signal idle thread data is available.
    }
  }
}

uint8_t readLine(char *line, uint8_t maxLen) {
  systime_t _time_now;
  systime_t _time_start = nilTimeNow();
  uint8_t  count = 0;
  uint8_t  cr    = 0;
  int16_t  c     = -1;

  nilSemSignal(&ETHSem);  // Exit region.
  //WS.print("RL");
  while (c == -1) {
    _time_now = nilTimeNow();
    if (_time_now - _time_start > 2000) { 
      nilSemWait(&ETHSem);    // wait for slot
      return 0; } // If no response from server longer then # seconds
    nilSemWait(&ETHSem);    // wait for slot
    c = SMTPethClient.read();
    nilSemSignal(&ETHSem);  // Exit region.
    if (c == -1) nilThdSleepMilliseconds(25);
  }
  nilSemWait(&ETHSem);    // wait for slot
  while (true) {
    //WS.print((char)c);
    if (count < maxLen)  { line[count++] = c; }
    if (cr && c == '\n') { break; }
    if (c == '\r') { cr = 1; }
    else           { cr = 0; }
    c = SMTPethClient.read();
  }

  if (count == maxLen - 1) { line[count - 1] = '\0'; }
  else                     { line[count] = '\0';     }
  return count;
}

int16_t readStatus() {
  char    line[4];
  uint8_t result;
  while(true) {
    result = readLine(line, 4);
    if (result >= 4 && (line[3] != '-')) { break; }
  }
  if (result < 3) { return 0; }
  line[3] = 0;
  //WS.print("RS:");WS.println(line); 
  return strtol(line, NULL, 10);
}

void smtp_escape(uint8_t _state) {
  SMTPethClient.stop();
  nilSemSignal(&ETHSem);  // Exit region.
  WS.println(F("Email End"));
  if (_state > 0) { _tmp[0] = 'T'; _tmp[1] = 'E'; _tmp[2] = _state;  pushToLog(_tmp, 3);}
}

//------------------------------------------------------------------------------
// Alert thread
//
NIL_WORKING_AREA(waAlertThread, 192);
NIL_THREAD(AlertThread, arg) {
  uint8_t _group, _smtp_go, _resp;
  int8_t _status;
  //uint8_t _text[10];

  nilThdSleepSeconds(1); // Sleep before start thread

  // Initialize GSM modem
  nilSemWait(&GSMSem);    // wait for slot
  GSMisAlive = GSM.ATsendCmd(AT_is_alive);
  if (GSMisAlive == 1) {
    //GSMsetSMS = GSM.ATsendCmd(AT_CLIP_ON); 
    GSMsetSMS = GSM.ATsendCmd(AT_set_sms_to_text);
    if (GSMsetSMS) GSMsetSMS = GSM.ATsendCmd(AT_set_sms_receive);
  }
  nilSemSignal(&GSMSem);  // Exit region.
 
  #ifdef WEB_DEBUGGING
  WS.println(F("Alert thread started"));
  #endif
  
  while (TRUE) {
    // Check for data.  
    log_event_t* p = alert_fifo.waitData(TIME_INFINITE);

    char* alert_message = p->text;
    _group      = ALR_GROUPS + 1; // no group
    sms_text[0] = 0;

    WS.print(F("Alert: "));
    // Compose text
    switch(alert_message[5]){
      case 'S': // System
        _group = 100; // any group
        strcat_P(sms_text, (char*)text_System); strcat(sms_text, " ");
        switch(alert_message[6]){
          case 'B': // Battery
            strcat_P(sms_text, (char*)text_battery); strcat(sms_text, " ");
            strcat_P(sms_text, (char*)text_is); strcat(sms_text, " ");
            if (alert_message[7] == 'L') strcat_P(sms_text, (char*)text_low);
            else strcat_P(sms_text, (char*)text_OK);
          break;
          case 'A': // AC
            strcat_P(sms_text, (char*)text_power); strcat(sms_text, " ");
            strcat_P(sms_text, (char*)text_is); strcat(sms_text, " ");
            if (alert_message[7] == 'L') strcat_P(sms_text, (char*)text_On);
            else strcat_P(sms_text, (char*)text_Off);
          break;
          case 'C':
            strcat_P(sms_text, (char*)text_configuration); strcat(sms_text, " ");
            switch(alert_message[7]){
              case 'W': strcat_P(sms_text, (char*)text_saved); break; // conf. saved
              case 'P': strcat_P(sms_text, (char*)text_saved); strcat_P(sms_text, (char*)text_cosp); strcat_P(sms_text, (char*)text_system);
                strcat(sms_text, " "); strcat_P(sms_text, (char*)text_disabled);break; // conf. saved
              case 'L': strcat_P(sms_text, (char*)text_loaded); break; // conf. saved
              case 'R': strcat_P(sms_text, (char*)text_reset); break; // conf. saved 
            }
          break;
          case 'S': 
            strcat_P(sms_text, (char*)text_monitoring); strcat(sms_text, " "); strcat_P(sms_text, (char*)text_started);
          break;
          case 'X': 
            strcat_P(sms_text, (char*)text_ALARM);
          break;
        }
      break;
      case 'Z': // Zone
        _group = ((conf.zone[alert_message[7]] >> 1) & B1111); // only specific group
        strcat_P(sms_text, (char*)text_Zone); strcat(sms_text, " ");
        //strcat(sms_text, (uint8_t)alert_message[7]+1); strcat_P(sms_text, (char*)text_spdashsp);
        strcat(sms_text, conf.zone_name[alert_message[7]]); strcat(sms_text, " ");
        switch(alert_message[6]){
          case 'P': strcat_P(sms_text, (char*)text_trigger); strcat_P(sms_text, (char*)text_ed);
            strcat(sms_text, " "); strcat_P(sms_text, (char*)text_alarm);
          break;
          case 'T': strcat_P(sms_text, (char*)text_tamper); strcat_P(sms_text, (char*)text_ed);
          break;
          case 'O': strcat_P(sms_text, (char*)text_is); strcat(sms_text, " ");
            strcat_P(sms_text, (char*)text_open);
          break;
        }
      break;
      case 'A': // Authenticatn
        _group = ((conf.key_setting[alert_message[7]] >> 1) & B1111); // only specific group
        strcat_P(sms_text, (char*)text_Authentication); strcat(sms_text, " ");
        strcat_P(sms_text, (char*)text_key); strcat(sms_text, " "); 
        if (alert_message[6] != 'U') { strcat(sms_text, conf.key_name[alert_message[7]]); strcat(sms_text, " "); }
        switch(alert_message[6]){
          case 'U': strcat_P(sms_text, (char*)text_unk); 
          break;
          case 'F': strcat_P(sms_text, (char*)text_is); strcat(sms_text, " ");
            strcat_P(sms_text, (char*)text_disabled);
          break;
        }
      break;    
      case 'G': // Groups
        _group = alert_message[7]; // only specific group
        strcat_P(sms_text, (char*)text_Group); strcat(sms_text, " ");
        //server << (uint8_t)alert_message[7]+1; strcat_P(sms_text, (char*)text_spdashsp);
        strcat(sms_text, conf.group_name[alert_message[7]]); strcat(sms_text, " ");
        switch(alert_message[6]){
          case 'D': strcat_P(sms_text, (char*)text_is); strcat(sms_text, " ");
            strcat_P(sms_text, (char*)text_disarmed);
          break;
          case 'A': strcat_P(sms_text, (char*)text_is); strcat(sms_text, " ");
            strcat_P(sms_text, (char*)text_auto); strcat(sms_text, " ");
            strcat_P(sms_text, (char*)text_armed);
          break;
          case 'S': strcat_P(sms_text, (char*)text_is); strcat(sms_text, " ");
            strcat_P(sms_text, (char*)text_armed);
          break;
          default : strcat_P(sms_text, (char*)text_unk); break;
        }
      break;
      case 'F': // Fifo
        _group = 100; // any group      
        switch(alert_message[6]){
          case 'A' : strcat_P(sms_text, (char*)text_Alarm); break;
          case 'L' : strcat_P(sms_text, (char*)text_Alert); break;
          case 'S' : strcat_P(sms_text, (char*)text_Node); break;
          case 'R' : strcat_P(sms_text, (char*)text_Registration); break;
          default : strcat_P(sms_text, (char*)text_unk); break;
        }
        strcat(sms_text, " "); strcat_P(sms_text, (char*)text_queue);
        strcat(sms_text, " "); strcat_P(sms_text, (char*)text_is);
        strcat(sms_text, " "); strcat_P(sms_text, (char*)text_full);
      break;
      case 'R': // Trigger
        _group = ((trigger[alert_message[7]].setting >> 1) & B1111); // only specific group
        strcat_P(sms_text, (char*)text_Trigger); strcat(sms_text, " ");
        //strcat_P(sms_text, (char*)text_number); strcat(sms_text, " ");
        //server << (uint8_t)alert_message[6]+1; strcat(sms_text, " ");
        strcat(sms_text, trigger[alert_message[7]].name); strcat(sms_text, " ");
        strcat_P(sms_text, (char*)text_activated);
        switch(alert_message[6]){
          case 'A' : strcat_P(sms_text, (char*)text_activated); break;
          case 'D' : strcat_P(sms_text, (char*)text_de); strcat_P(sms_text, (char*)text_activated); break;
          default : strcat_P(sms_text, (char*)text_unk); break;
        } 
      break;
      case 'C': // SMS commands
        _group = ((trigger[alert_message[7]].setting >> 1) & B1111); // only specific group
        strcat_P(sms_text, (char*)text_Command); strcat_P(sms_text, (char*)text_sesp); 
        switch(alert_message[7]){
          case 'G' : strcat_P(sms_text, (char*)text_Group); strcat(sms_text, " ");
            strcat(sms_text, conf.group_name[alert_message[8]]); strcat_P(sms_text, (char*)text_sesp);
            (conf.group[alert_message[8]] & B1) ? strcat_P(sms_text, (char*)text_On) : strcat_P(sms_text, (char*)text_Off);
            strcat_P(sms_text, (char*)text_cosp);
            (group[alert_message[8]].setting & B1) ? strcat_P(sms_text, (char*)text_Armed) : strcat_P(sms_text, (char*)text_Disarmed);
            strcat_P(sms_text, (char*)text_cosp);
            (group[alert_message[8]].setting >> 1 & B1) ? strcat_P(sms_text, (char*)text_Alarm) : strcat_P(sms_text, (char*)text_OK);
            break;
          case 'I' : strcat_P(sms_text, (char*)text_Input);
            strcat_P(sms_text, (char*)text_state); strcat(sms_text, " ");

            break;
          default : strcat_P(sms_text, (char*)text_unk); break;
        } 
      break;
    }
    strcat(sms_text, ".");

    WS.println(sms_text);

    // Send SMS
    if (((uint8_t)alert_message[4] >> alert_SMS) & B1) { 
      for (uint8_t i = 0; i < NUM_OF_PHONES; ++i) {
        //  phone enabled              specific group                       or global tel.
        if ((conf.tel[i] & B1) && ((((_group == conf.tel[i] >> 1) & B1111)) || (conf.tel[i] >> 5 & B1))) { 
          nilSemWait(&GSMSem);    // wait for slot
          _status = GSM.ATsendSMSBegin(conf.tel_num[i]);
          WS.print(F("SMS begin: ")); WS.println(_status);
          if (_status == 1) {
            _status = GSM.ATsendSMSEnd(sms_text);
            WS.print(F("SMS sent: ")); WS.println(_status);  
          }
          nilSemSignal(&GSMSem);  // Exit region.
        }
      }
    }

    // Send email
    if (((uint8_t)alert_message[4] >> alert_email) & B1) {
      _smtp_go = 1;
      WS.print(F("Email "));
      // Add timestamp to email
      strcat(sms_text, " "); 
      l.b[0] = alert_message[0]; l.b[1] = alert_message[1]; l.b[2] = alert_message[2]; l.b[3] = alert_message[3];
      time_temp.set(l.lval);
      strcat(sms_text, (char*)time_temp.formatedDateTime()); strcat(sms_text, ".");
      // Send Email
      nilSemWait(&ETHSem);    // wait for slot
      SMTPethClient.connect("mail.smtp2go.com", 2525);
      if (readStatus() != 220 ) { smtp_escape(1); _smtp_go = 0; }
      if (_smtp_go) { 
        SMTPethClient.println(F("EHLO"));
        if (readStatus() != 250 ) { smtp_escape(2); _smtp_go = 0; }
      }
      if (_smtp_go) {
        SMTPethClient.println(F("auth login"));
        if (readStatus() != 334 ) { smtp_escape(3); _smtp_go = 0; }
      }
      if (_smtp_go) { 
        b64_text[0] = 0; base64_encode(b64_text, conf.SMTP_user, strlen(conf.SMTP_user));
        SMTPethClient.println(b64_text);
        if (readStatus() != 334 ) { smtp_escape(4); _smtp_go = 0; }
      }
      if (_smtp_go) { 
        b64_text[0] = 0; base64_encode(b64_text, conf.SMTP_password, strlen(conf.SMTP_password));
        SMTPethClient.println(b64_text);
        if (readStatus() != 235 ) { smtp_escape(5); _smtp_go = 0; }
      }
      if (_smtp_go) { 
        SMTPethClient.print(F("MAIL FROM:<")); SMTPethClient.print(conf.SMTP_user); SMTPethClient.println(F(">"));
        if (readStatus() != 250 ) { smtp_escape(6); _smtp_go = 0; }
      }
      for (uint8_t i = 0; i < NUM_OF_PHONES; ++i) {
        //  phone enabled              specific group                       or global tel.
        if ((conf.tel[i] & B1) && ((((_group == conf.tel[i] >> 1) & B1111)) || (conf.tel[i] >> 5 & B1))) {  
          if (_smtp_go) { 
            SMTPethClient.print(F("RCPT TO:<")); SMTPethClient.print(conf.email[i]); SMTPethClient.println(F(">"));
            // OK status is 25*
            if ((readStatus()/10) != 25 ) { smtp_escape(7); _smtp_go = 0; }
          }
        }
      }    
      if (_smtp_go) {   
        SMTPethClient.println(F("DATA"));
        if (readStatus() != 354 ) { smtp_escape(8); _smtp_go = 0; }
      }
      if (_smtp_go) { 
        SMTPethClient.print(F("From: ")); SMTPethClient.println(conf.SMTP_user);
        SMTPethClient.print(F("To: "));
        for (uint8_t i = 0; i < NUM_OF_PHONES; ++i) {
          //  phone enabled              specific group                       or global tel.
          if ((conf.tel[i] & B1) && ((((_group == conf.tel[i] >> 1) & B1111)) || (conf.tel[i] >> 5 & B1))) { 
            SMTPethClient.print(conf.email[i]); SMTPethClient.print(F(", "));
          }
        } 
        SMTPethClient.println();
        SMTPethClient.print(F("Subject: ")); SMTPethClient.println(sms_text);
        SMTPethClient.println(); SMTPethClient.println(F("."));
        if (readStatus() != 250 ) { smtp_escape(9); _smtp_go = 0; }
      }
      if (_smtp_go) {
        WS.println(F("OK"));
        smtp_escape(0);
      }
    }

    // Page number
    if (((uint8_t)alert_message[4] >> alert_page) & B1) {
      for (uint8_t i = 0; i < NUM_OF_PHONES; ++i) {
        //  phone enabled              specific group                       or global tel.
        if ((conf.tel[i] & B1) && ((((_group == conf.tel[i] >> 1) & B1111)) || (conf.tel[i] >> 5 & B1))) { 
          // Prepare ATD+ command
          sms_text[0] = 0;
          strcat(sms_text, AT_D); strcat(sms_text, conf.tel_num[i]); strcat(sms_text, ";");
          nilSemWait(&GSMSem);    // wait for slot
          _status = GSM.ATsendCmd(sms_text); 
          WS.print(F("Page begin: ")); WS.println(_status);
          if (_status < 1) {
            nilSemSignal(&GSMSem);  // Exit region.
            break; 
          }
          nilThdSleepSeconds(20); // RING ... RING ...
          _status = GSM.ATsendCmd(AT_H); 
          WS.print(F("Page end: "));  WS.println(_status);
          nilSemSignal(&GSMSem);  // Exit region.
        }
      }
    }

    WS.println(F("Alert end"));
    alert_fifo.signalFree(); // Signal FIFO slot is free.
  }
}

//------------------------------------------------------------------------------
// Service thread
//
NIL_WORKING_AREA(waServiceThread, 190);  
NIL_THREAD(ServiceThread, arg) {
  int8_t  _resp;
  uint8_t _update_node;
  uint8_t _GSMlastStatus = 10; // get status on start
  char    _message[6];
  uint8_t _text[10];
  uint8_t _counter = 0; // used for pause in alarm
  char *  _pch;
  uint32_t group_aa;

  nilThdSleepSeconds(1); // Sleep before start service thread
  // Set timers on start
  for (int8_t i=0; i < TIMERS ; i++){
    //  timer enabled
    if (timer[i].setting & B1) set_timer(i);
  }

  nilThdSleepSeconds(1); // Sleep before start service thread
  // Call for registration of nodes
  _resp = sendCmd(15,1); nilThdSleepSeconds(1);

  #ifdef WEB_DEBUGGING
  WS.println(F("Service thread started"));
  #endif  

  while (TRUE) {
    nilThdSleepSeconds(1);
    //count to 70 seconds, for delays in relay OUT
    _counter++; if (_counter == 70) {_counter = 0;}

    /*
        // Auto arm
    for (int8_t i=0; i < ALR_ZONES ; i++){
      //   Zone enabled            auto arm                     
      if ((conf.zone[i] & B1) && ((conf.zone[i] >> 7) & B1)){ 
        if ( timestamp.get() >= (zone[i].last_PIR + (conf.auto_arm * SECS_PER_MIN))) {
          uint8_t _group = (conf.zone[i] >> 1) & B1111;
          // Only if group not armed already
          if (!(group[_group].setting & B1)) {
            _tmp[0] = 'G'; _tmp[1] = 'A'; _tmp[2] = _group; pushToLog(_tmp, 3); // Authorization auto arm
            // if group is enabled arm zone's group else log error to log
            if (conf.group[_group] & B1) { 
              group[_group].setting |= (1 << 0); // arm group
              group[_group].arm_delay = 0;       // set arm delay off
              _resp = sendCmdToGrp(_group, 15);  // send arm message to all nodes
            } else {
              if (!((group[_group].setting >> 7) & B1)) {
                group[_group].setting |= (1 << 7); // Set logged disabled bit On
                _tmp[0] = 'G'; _tmp[1] = 'F'; _tmp[2] = _group; pushToLog(_tmp, 3);
              }
            }
          }
        }
      }
    }
    */
    // New group aware auto arm   
    for (int8_t i=0; i < ALR_GROUPS ; i++){
      //   Group enabled    
      if (conf.group[i] & B1){ 
        //WS.print(F("Group: ")); WS.print(i);
        group_aa = 0;
        for (int8_t j=0; j < ALR_ZONES ; j++){
          //   Zone enabled            auto arm                       group matches
          if ((conf.zone[j] & B1) && ((conf.zone[j] >> 7) & B1) && (((conf.zone[j] >> 1) & B1111) == i)){
            //WS.print(F(", zone: ")); WS.print(j);
            //WS.print(F("=")); WS.print((zone[j].last_PIR + (conf.auto_arm * SECS_PER_MIN)));
            //  Get latest PIR  
            if ((zone[j].last_PIR + (conf.auto_arm * SECS_PER_MIN)) > group_aa) {
              //WS.print(F(" AA"));
              group_aa = (zone[j].last_PIR + (conf.auto_arm * SECS_PER_MIN));
            }
          }
        } // list through zones
        // auto arm group
        //   Group has at leas one autoarm and time has passed
        if ((group_aa != 0) && (group_aa <= timestamp.get())) {
          //WS.print(F(" | IF")); 
          //WS.print(F(" AA:")); WS.print(group_aa);
          //WS.print(F(" TS:")); WS.print(timestamp.get());
          // Only if group not armed already
          if (!(group[i].setting & B1)) {
            _tmp[0] = 'G'; _tmp[1] = 'A'; _tmp[2] = i; pushToLog(_tmp, 3); // Authorization auto arm
            // No need to check if group is enabled again, or log error to log
            group[i].setting |= (1 << 0); // arm group
            group[i].arm_delay = 0;       // set arm delay off
            _resp = sendCmdToGrp(i, 15);  // send arm message to all nodes
          }
        }
        //WS.println(); 
      } 
    }

    // Zone open alarm
    for (int8_t i=0; i < ALR_ZONES ; i++){
      //   Zone enabled      and   open alarm enabled
      if ((conf.zone[i] & B1) && ((conf.zone[i] >> 8) & B1)) { 
        if ( timestamp.get() >= (zone[i].last_OK + (conf.open_alarm * SECS_PER_MIN))) {
          _tmp[0] = 'Z'; _tmp[1] = 'O'; _tmp[2] = i; pushToLog(_tmp, 3); // Authorization still open alarm
          zone[i].last_OK = timestamp.get();    // update current timestamp
        }
      }
    }
    // Timers
    time_temp = timestamp;
    for (int8_t i=0; i < TIMERS ; i++){
      //   Timer enabled              Timer next on is set
      if ((timer[i].setting & B1) && (timer[i].next_on > 0)){ 
        //   start time has passed                   NOT triggerd yet
        if ((time_temp.get() >= timer[i].next_on) && !((timer[i].setting >> 9) & B1)) {
          //   Assoc. triger is none or assoc. trigger is trigerred.
          if ((timer[i].asc_trigger == 0) ||
             ((timer[i].asc_trigger > 0) && (trigger[timer[i].asc_trigger-1].setting >> 5) & B1))  {
            _resp = 0;
            for (_update_node = 0; _update_node < nodes; _update_node++) {
              if (node[_update_node].address  == timer[i].to_address &&
                  node[_update_node].function == 'I' &&
                  node[_update_node].number   == timer[i].to_number &&
                  node[_update_node].setting & B1) { 
                _resp = 1;
                break;
              }
            }
            if (_resp) {
              _message[0] = 'I'; // 'I'nput only
              _message[1] = timer[i].to_number;
              u.fval = timer[i].constant_on;
              _message[2] = u.b[0]; _message[3] = u.b[1]; _message[4] = u.b[2]; _message[5] = u.b[3];
              if (sendData(timer[i].to_address, _message, 6)) {
                timer[i].setting |= (1 << 9); // Set triggered On
                node[_update_node].last_OK = timestamp.get(); // update receiving node current timestamp
                node[_update_node].value   = timer[i].constant_on; // update receiving node value
                PublishNode(_update_node); // MQTT
              }
            }
          }
        }
        //   end time has passed
        if (time_temp.get() >= timer[i].next_off) { // reschedule even if not triggered //&& ((timer[i].setting >> 9) & B1)) {
          _resp = 0;
          for (_update_node = 0; _update_node < nodes; _update_node++) {
            if (node[_update_node].address  == timer[i].to_address &&
                node[_update_node].function == 'I' &&
                node[_update_node].number   == timer[i].to_number &&
                node[_update_node].setting & B1) { 
              _resp = 1;
              break;
            }
          }
          //               is triggered
          if ((_resp) && ((timer[i].setting >> 9) & B1)) {
            _message[0] = 'I'; // 'I'nput only
            _message[1] = timer[i].to_number;
            u.fval = timer[i].constant_off;
            _message[2] = u.b[0]; _message[3] = u.b[1]; _message[4] = u.b[2]; _message[5] = u.b[3];
            if (sendData(timer[i].to_address, _message, 6)) {
              timer[i].setting &= ~(1 << 9); // Set triggered Off
              node[_update_node].last_OK = timestamp.get(); // update receiving node current timestamp
              node[_update_node].value   = timer[i].constant_off; // update receiving node value
              PublishNode(_update_node); // MQTT
            }
          }
          set_timer(i, 0); // set next start time for this timer
        }
      }
    }
    //Trigger off timer
    for (int8_t i=0; i < TRIGGERS ; i++){
      //   Trigger enabled               Off timer enabled                  next off is set
      if ((trigger[i].setting & B1) && ((trigger[i].setting >> 8) & B1) && (trigger[i].next_off > 0)){ 
        //   off time has passed                
        if (time_temp.get() >= trigger[i].next_off) {
          _resp = 0;
          for (_update_node = 0; _update_node < nodes; _update_node++) {
            if (node[_update_node].address  == trigger[i].to_address &&
                node[_update_node].function == 'I' &&
                node[_update_node].number   == trigger[i].to_number &&
                node[_update_node].setting & B1) { 
              _resp = 1;
              break;
            }
          }
          if (_resp) {
            _message[0] = 'I'; // 'I'nput only
            _message[1] = trigger[i].to_number;
            u.fval = trigger[i].constant_off;
            _message[2] = u.b[0]; _message[3] = u.b[1]; _message[4] = u.b[2]; _message[5] = u.b[3];
            if (sendData(trigger[i].to_address, _message, 6)) {
              trigger[i].setting &= ~(1 << 5); // switch OFF Is alerted/triggered
              trigger[i].setting &= ~(1 << 3); // switch OFF passed
              trigger[i].next_off = 0; // Clear off timer
              node[_update_node].last_OK = timestamp.get(); // update receiving node current timestamp
              node[_update_node].value   = trigger[i].constant_off; // update receiving node value
              PublishNode(_update_node); // MQTT
            }
          }
        }
      }
    }
    // Message queue, delete expired packets
    for (int8_t i=0; i < NODE_QUEUE ; i++){
      if ((node_queue[i].address != 0) && (time_temp.get() > node_queue[i].expire)) {
        /*
        WS.print(F("QN ")); WS.print(i);
        WS.print(F(" time ")); WS.print(time_temp.get()); WS.print(F("/")); WS.print(node_queue[i].expire);
        WS.print(F(" node ")); WS.print(node_queue[i].index);
        WS.println();
        */
        node_queue[i].expire  = 0; // mark as empty
        node_queue[i].address = 0; // mark as empty
        node[node_queue[i].index].queue = 255; // Clear node queued flag
      }
    }
    // Battery check 
    if (pinBAT_OK.read() == LOW) { // The signal is "Low" when the voltage of battery is under 11V 
      pushToLog("SBL"); // battery low
      pushToLog("SCP"); // conf saved 
      log_event_t *p;
      do { 
        nilThdSleepMilliseconds(100);
      } while (p); // wait for logger thread
      conf.setting |= (1 << 0); // flag that we are out of power
      saveConf(0); // Save conf, no log
      eeprom_update_block((const void*)&group, (void*)sizeof(conf)+sizeof(trigger)+sizeof(timer), sizeof(group)); // Save current configuration      

      nilSysLock(); // Lock RTOS  

      // Battery is at low level but it might oscillate, so we wait for AC to recharge battery again.
      while (pinAC_OFF.read() == HIGH) { // The signal turns to be "High" when the power supply turns OFF
        delay(1000); // do nothing, wait for power supply shutdown
      } 

      conf.setting &= ~(1 << 0); // un-flag that we are out of power
      saveConf(0); // Save conf, no log

      nilSysUnlock(); // Power is restored we go on      
      pushToLog("SBH"); // Battery High
    }
    // AC power check
    if (pinAC_OFF.read() == LOW && ACState != LOW) { // The signal turns to be "High" when the power supply turns OFF
      ACState = LOW;
      pushToLog("SAL"); // AC ON
    }
    if (pinAC_OFF.read() == HIGH && ACState != HIGH) { // The signal turns to be "High" when the power supply turns OFF
      ACState = HIGH;
      pushToLog("SAH"); // AC OFF
    }
    // GSM modem check 
    if (_counter == 0) { 
      // look if GSM is free
      if (nilSemWaitTimeout(&GSMSem, TIME_IMMEDIATE) != NIL_MSG_TMO) { 
        //WS.println("gsm free");
        GSMisAlive = GSM.ATsendCmd(AT_is_alive);
        if (GSMisAlive == 1) {
          if (!GSMsetSMS) {
            //GSMsetSMS = GSM.ATsendCmd(AT_CLIP_ON);             // CLI On
            GSMsetSMS = GSM.ATsendCmd(AT_set_sms_to_text); // set modem to text SMS format
            //if (GSMsetSMS) GSMsetSMS = GSM.ATsendCmd(AT_set_sms_receive);
          }
          _resp = GSM.ATsendCmdWR(AT_registered, _text, 3); 
          GSMreg = strtol((char*)_text, NULL, 10);
          _resp = GSM.ATsendCmdWR(AT_signal_strength, _text, 2);
          GSMstrength = (strtol((char*)_text, NULL, 10)) * 3; 
          //GSM.print("AT&F\r");
          //GSM.print("AT+CBAND=\"ALL_BAND\"\r");
          //GSM.print("AT+CBAND?\r");
        } else { GSMreg = 4; GSMstrength = 0; GSMsetSMS = 0;}
        nilSemSignal(&GSMSem);  // Exit region.
        //WS.println("gsm slot exit");
        if (_GSMlastStatus != GSMreg) { // if modem registration changes log it
          _GSMlastStatus = GSMreg;
          _tmp[0] = 'M'; _tmp[1] = GSMreg; _tmp[2] = GSMstrength;  pushToLog(_tmp, 3); 
        }
      }
    }

    // OUT 1 & 2 handling
    if ((_counter) >= 60) { // cycle every minute
      pinOUT1.write(LOW);
      pinOUT2.write(LOW);
    } else {
      pinOUT1.write(((OUTs >> 0) & B1));
      pinOUT2.write(((OUTs >> 1) & B1));
    }

    //   MQTT connected
    if (_counter == 0) { 
      nilSemWait(&ETHSem);    // wait for slot
      if (!client.connected()) {
        //client.disconnect(); // This have problems with Teensy Eth.
        client.setServer(conf.mqtt_ip, conf.mqtt_port);
        WS.print(F(">Eth. down"));
        if (!client.connect(str_MQTT_clientID)) {
          WS.print(F(", state: ")); WS.print(client.state());
          Ethernet.begin(mac, conf.ip, conf.gw, conf.gw, conf.mask); // Ethernet
          WS.println(F(", restart."));
        } else {
          WS.println(F(", renew."));
          if (!MQTTState){
            client.subscribe(str_MQTT_Subscribe);
            pushToLog("SMO");
            MQTTState = 1;
          }
        }
      }
      nilSemSignal(&ETHSem);  // Exit region.
    }

    // Read GSM incomming messages
    _update_node = 255; // Used here to indicate incomming SMS
    while(GSM.isMsg()) {
      _resp = GSM.read((uint8_t*)sms_text);                     // read serial
      #if WEB_DEBUGGING
      WS.print(F(">GSM len: ")); WS.print(_resp); WS.print('>');
      for(uint8_t i = 0; i < _resp; i++) {
        WS.print((char)sms_text[i]);
      }
      WS.println(F("<"));
      #endif
      // response is not empty string
      if (_resp > 0) {
        if (_update_node == 255) {
          // Example: +CMT: "+420731435556","","17/05/29,15:31:47+08"
          _pch = strtok(sms_text, ":");
          if (_pch != NULL) {
            if (strcmp(_pch, "+CMT") == 0) {
              _pch = strtok(NULL, "\""); _pch = strtok(NULL, "\""); // extracat tel. number
              WS.print(F(">SMS from: ")); WS.print(_pch); 
              for (uint8_t i = 0; i < NUM_OF_PHONES; ++i) {
                //  phone enabled          number does match
                if ((conf.tel[i] & B1) && (strcmp(_pch, conf.tel_num[i]) == 0)) { 
                  WS.print(F(" OK"));
                  _update_node = i;
                }
              }
              WS.println();
            }
          }
        } else {
          _pch = strtok(sms_text, " ");
          if (_pch != NULL) {
            WS.print(F(">SMS command: ")); WS.print(_pch); 
            if (strcmp(_pch, "Group") == 0) {
              _pch = strtok(NULL, " ");
              WS.print(F(": ")); WS.print(_pch); 
              for (uint8_t i = 0; i < ALR_GROUPS; ++i) {
                //  name does match
                if (strcmp(_pch, conf.group_name[i]) == 0) { 
                  _pch = strtok(NULL, " ");
                  WS.print(F(" Cmd: ")); WS.print(_pch); 
                  // Get status
                  if (strcmp(_pch, "state") == 0) {
                    _tmp[0] = 'C'; _tmp[1] = _update_node; _tmp[2] = 'G'; _tmp[3] = i; _tmp[4] = 'S'; pushToLog(_tmp, 5);
                  } else
                  if (strcmp(_pch, "arm") == 0) {
                    // if group enabled arm group or log error to log. 
                    if (conf.group[i] & B1) { 
                      group[i].setting |= (1 << 0);   // arm group
                      group[i].arm_delay = conf.arm_delay; // set arm delay
                      _resp = sendCmdToGrp(i, 10);  // send arming message to all nodes
                    } 
                    else { _tmp[0] = 'G'; _tmp[1] = 'F'; _tmp[2] = i;  pushToLog(_tmp, 3); }
                    _tmp[0] = 'C'; _tmp[1] = _update_node; _tmp[2] = 'G'; _tmp[3] = i; _tmp[4] = 'A'; pushToLog(_tmp, 5);
                  } else
                  if (strcmp(_pch, "disarm") == 0) {
                    _tmp[0] = 'C'; _tmp[1] = _update_node; _tmp[2] = 'G'; _tmp[3] = i; _tmp[4] = 'D'; pushToLog(_tmp, 5);
                  } 
                  break; // no need to try other
                }
              } // for  
            } // = group 
            else if (strcmp(_pch, "Input") == 0) {
              _pch = strtok(NULL, " ");
              WS.print(F(": ")); WS.print(_pch); 
              for (uint8_t i = 0; i < ALR_GROUPS; ++i) {

              }
            }
            WS.println();
          }
          _update_node = 255; // reset 
        } // Else line is SMS command
      }
    } // End GSM incoming
    
    //watchdog |= (1 << 2); // Flag watchdog for this thread
  }
}

//------------------------------------------------------------------------------
// RFM69 thread 
//
NIL_WORKING_AREA(waRadioThread, 128);  
NIL_THREAD(RadioThread, arg) {
  uint8_t  _pos;

  #ifdef WEB_DEBUGGING
  WS.println(F("Radio thread started"));    
  #endif

  while (TRUE) {
    nilThdSleepMilliseconds(20);
    if(nilSemWaitTimeout(&RFMSem, TIME_IMMEDIATE) != NIL_MSG_TMO) {
      //WS.print('@');
      if (radio.receiveDone()) {
        ++radio_received;
        #ifdef WEB_DEBUGGING
        WS.print(F(">Radio A:")); WS.print(radio.SENDERID);
        WS.print(F(",Data "));WS.print(radio.DATALEN); WS.print(F(","));
        for (uint8_t _t=0; _t < radio.DATALEN; _t++) {
          WS.print(radio.DATA[_t], HEX); WS.print(F("-"));
        }
        WS.print(F("RSSI:")); WS.println(radio.RSSI);
        #endif
        // Registration
        if ((char)radio.DATA[0] == 'R') {
          _pos = 0;
          do {
            _pos++; // Skip 'R'
            register_t *p = reg_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
            if (p == 0) {
              pushToLog("FR"); // Registration queue is full
              continue; // Continue if no free space.
            }
            // node setting
            p->node    = (char)radio.DATA[_pos];    
            p->address = radio.SENDERID+RADIO_UNIT_OFFSET;
            p->type    = radio.DATA[_pos+1];
            p->number  = (uint8_t)radio.DATA[_pos+2];
            p->setting = ((uint8_t)radio.DATA[_pos+3] << 8) | (uint8_t)radio.DATA[_pos+4];
            for (uint8_t _t=0; _t < NAME_LEN; _t++) {p->name[_t] = (char)radio.DATA[_pos+5+_t];}
            reg_fifo.signalData();   // Signal idle thread data is available.
            _pos+=REG_LEN;
          } while (_pos < radio.DATALEN);
        }
        // Sensors & Inputs feedback
        if (((char)radio.DATA[0] == 'S') || ((char)radio.DATA[0] == 'I')) {
          _pos = 1;
          do {
            node_t *p = sensor_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
            if (p == 0) {
              pushToLog("FS"); // Sensor queue is full
              continue; // Continue if no free space.
            }    
            p->function = (char)radio.DATA[0];
            p->address  = radio.SENDERID+RADIO_UNIT_OFFSET;
            p->type     = radio.DATA[_pos];
            p->number   = (uint8_t)radio.DATA[_pos+1];
            u.b[0] = radio.DATA[_pos+2]; u.b[1] = radio.DATA[_pos+3]; u.b[2] = radio.DATA[_pos+4]; u.b[3] = radio.DATA[_pos+5];
            p->value    = u.fval;
            sensor_fifo.signalData();   // Signal idle thread data is available.
            _pos+=6;
          } while (_pos < radio.DATALEN);
        } // if 'S'
        if (radio.ACKRequested()) radio.sendACK();
      }
      nilSemSignal(&RFMSem);  // Exit region.
      // Message queue
      for (_pos = 0; _pos < NODE_QUEUE; ++_pos) {
        if((node_queue[_pos].address == radio.SENDERID+RADIO_UNIT_OFFSET) && (node_queue[_pos].expire != 0)) {
          if (sendData(node_queue[_pos].address, node_queue[_pos].msg, node_queue[_pos].length)) {
            node_queue[_pos].expire  = 0; // mark as empty
            node_queue[_pos].address = 0; // mark as empty
            node[node_queue[_pos].index].queue = 255; // Clear node queued flag 
          }
          break; // we can send only one queued message to single address as we are in reciever thread
        }
      }
    }
    //watchdog |= (1 << 3); // Flag watchdog for this thread
  }
}

//------------------------------------------------------------------------------
// Sensor thread 
//
NIL_WORKING_AREA(waSensorThread, 160);  
NIL_THREAD(SensorThread, arg) {
  uint8_t _node, _found, _lastNode = 0;
  uint32_t _lastNodeTime = 0;
  //char _text[40];
  //char _value[10];

  #ifdef WEB_DEBUGGING
  WS.println(F("Sensor thread started"));    
  #endif

  while (TRUE) {
    // Check for data.  Use TIME_IMMEDIATE to prevent sleep in idle thread.
    node_t* p = sensor_fifo.waitData(TIME_INFINITE);

    _found = 0;
    for (_node = 0; _node < nodes; _node++) { 
      /*
      WS.print(node[_node].function); WS.print(':'); WS.println(p->function);
      WS.print(node[_node].address); WS.print(':'); WS.println(p->address);
      WS.print(node[_node].type); WS.print(':'); WS.println(p->type);
      WS.print(node[_node].number); WS.print(':'); WS.println(p->number);
      WS.println(F("---"));
      */
      if ((node[_node].function == p->function) && 
          (node[_node].address  == p->address) && 
          (node[_node].type     == p->type) &&
          (node[_node].number   == p->number)) { 
        _found = 1;        
        //  node enabled          
        if (node[_node].setting & B1) { 
          node[_node].value   = p->value;
          node[_node].last_OK = timestamp.get();
          PublishNode(_node); // MQTT
          // Triggers
          process_triggers(node[_node].address, node[_node].type, node[_node].number, node[_node].value);
        } // node enbled
        break; // no need to look for other node
      } // if address 
    } // for
    if (!_found) {
      // Let's call same unknown node for reregistrtion only once a while or we send many packets if multiple sensor data come in
      if ((timestamp.get() > _lastNodeTime) || (_lastNode != p->address)) {
        nilThdSleepMilliseconds(5);  // Thid is needed for sleeping batery nodes !!! Or they wont see reg. command.
        _node = sendCmd(p->address, 1); // call this address to register
        _lastNode = p->address;
        _lastNodeTime = timestamp.get() + 1; // add 1-2 second(s)
      }
    }
    
    // Signal FIFO slot is free.
    sensor_fifo.signalFree();
  }
}

//------------------------------------------------------------------------------
// Registration thread 
//
NIL_WORKING_AREA(waRegThread, 96);  
NIL_THREAD(RegThread, arg) {
  uint8_t _node, _resp;

  #ifdef WEB_DEBUGGING
  WS.println(F("Registration thread started"));    
  #endif

  while (TRUE) {
    // Check for data.
    register_t* p = reg_fifo.waitData(TIME_INFINITE);

    switch(p->node){
      case 'K': 
      case 'S':
      case 'I':
        _resp = 0;
        for (_node=0; _node < nodes; _node++) {
          if (node[_node].address  == p->address &&
              node[_node].type     == p->type &&
              node[_node].number   == p->number &&
              node[_node].function == p->node) {
            _resp = 1; break;
          }
        }
        if (!_resp) { // if node not present already
          node[nodes].address  = p->address;
          node[nodes].function = p->node;
          node[nodes].type     = p->type;
          node[nodes].number   = p->number;
          node[nodes].setting  = p->setting;
          node[nodes].last_OK  = timestamp.get();
          strncpy(node[nodes].name, p->name, NAME_LEN-1); node[nodes].name[NAME_LEN-1] = 0;
          _tmp[0] = 'N'; _tmp[1] = 'R'; _tmp[2] = p->address; _tmp[3] = p->number; _tmp[4] = p->node; _tmp[5] = p->type;  pushToLog(_tmp, 6);
          // WS.print(F("Reg: ")); WS.println(nodes); WS.print(node[nodes].address); WS.print(node[nodes].type);
          // WS.println(node[nodes].number); WS.println(node[nodes].setting,BIN); 
          nodes++;
        } else {
          // there is match already
          node[_node].setting  = p->setting;
          node[_node].last_OK  = timestamp.get();
          strncpy(node[_node].name, p->name, NAME_LEN-1); node[_node].name[NAME_LEN-1] = 0;
          _tmp[0] = 'N'; _tmp[1] = 'r'; _tmp[2] = p->address; _tmp[3] = p->number; _tmp[4] = p->node; _tmp[5] = p->type; pushToLog(_tmp, 6);
          // WS.print(F("Rereg: ")); WS.println(_node+1); WS.print(node[_node].address); WS.print(node[_node].type);
          // WS.println(node[_node].number); WS.println(node[_node].setting,BIN); 
        }
        break;
        default:
          _tmp[0] = 'N'; _tmp[1] = 'E'; _tmp[2] = p->address; _tmp[3] = p->number; _tmp[4] = p->node; _tmp[5] = p->type; pushToLog(_tmp, 6);
        break;
    }
    // Signal FIFO slot is free.
    reg_fifo.signalFree();
  }
}

//------------------------------------------------------------------------------
// Web server thread Last one lowest priority
//
NIL_WORKING_AREA(waWebThread, 320);  
NIL_THREAD(WebThread, arg) {
  
  #ifdef WEB_DEBUGGING
  WS.println(F("Web thread started"));    
  #endif

  while (TRUE) {
    nilThdSleepMilliseconds(30);
    
    if (nilSemWaitTimeout(&ETHSem, TIME_IMMEDIATE) != NIL_MSG_TMO) {
      //WS.print('#');    
      webserver.processConnection();  // WebServer
      nilSemSignal(&ETHSem);          // Exit region.
    }
    
    nilThdSleepMilliseconds(10);
    
    if (nilSemWaitTimeout(&ETHSem, TIME_IMMEDIATE) != NIL_MSG_TMO) {
      client.loop();                  // MQTT
      nilSemSignal(&ETHSem);          // Exit region.
    }
    
    //watchdog |= (1 << 4); // Flag watchdog for this thread
  }
}

//------------------------------------------------------------------------------
// 
NIL_WORKING_AREA(waDebugThread, 96);  
NIL_THREAD(DebugThread, arg) {

  #ifdef WEB_DEBUGGING
  WS.println(F("Debug started"));  
  #endif
  
  while (TRUE) {
    nilThdSleepSeconds(1);    
    idlePointer++;
    if (idlePointer==idleSlots) {
      idlePointer=0;
      nilPrintUnusedStack(&WS);
      //nilPrintStackSizes(&WS);
    }
    idleCount[idlePointer] = 0; // reset idle

    //watchdog |= (1 << 5); // Flag watchdog for this thread
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
 NIL_THREADS_TABLE_ENTRY(NULL, TimeThread, NULL, waTimeThread, sizeof(waTimeThread))
 NIL_THREADS_TABLE_ENTRY(NULL, thdFcn, (void*)"AET 1", waAEThread1, sizeof(waAEThread1))
 NIL_THREADS_TABLE_ENTRY(NULL, thdFcn, (void*)"AET 2", waAEThread2, sizeof(waAEThread2))
 NIL_THREADS_TABLE_ENTRY(NULL, thdFcn, (void*)"AET 3", waAEThread3, sizeof(waAEThread3))
 NIL_THREADS_TABLE_ENTRY(NULL, RS485RXThread, NULL, waRS485RXThread, sizeof(waRS485RXThread))
 NIL_THREADS_TABLE_ENTRY(NULL, LoggerThread, NULL, waLoggerThread, sizeof(waLoggerThread))
 NIL_THREADS_TABLE_ENTRY(NULL, AlertThread, NULL, waAlertThread, sizeof(waAlertThread))
 NIL_THREADS_TABLE_ENTRY(NULL, ServiceThread, NULL, waServiceThread, sizeof(waServiceThread))
 NIL_THREADS_TABLE_ENTRY(NULL, RadioThread, NULL, waRadioThread, sizeof(waRadioThread))
 NIL_THREADS_TABLE_ENTRY(NULL, SensorThread, NULL, waSensorThread, sizeof(waSensorThread))
 NIL_THREADS_TABLE_ENTRY(NULL, RegThread, NULL, waRegThread, sizeof(waRegThread))
 NIL_THREADS_TABLE_ENTRY(NULL, WebThread, NULL, waWebThread, sizeof(waWebThread))
 NIL_THREADS_TABLE_ENTRY(NULL, DebugThread, NULL, waDebugThread, sizeof(waDebugThread))
 NIL_THREADS_TABLE_END()

//------------------------------------------------------------------------------
 void setup() {
  timestamp.set(0); time_started.set(0); // Set time stamps to 0 = 1.1.2000

  // Turn ON GSM
  delay(200);
  while (pinGSM_ON.read() == HIGH) {
    pinGSM_PWR.write(LOW); delay(1000); pinGSM_PWR.write(HIGH); delay(2000);
  }
  

  // Read current configuration
  eeprom_read_block((void*)&conf, (void*)0, sizeof(conf)); 
  if (conf.version != VERSION) { 
    setDefault();
    // encode default user pass 
    char _text[20]; _text[0] = 0;
    strcat (_text, conf.user); strcat (_text, ":"); strcat (_text, conf.password);
    base64_encode(conf.user_pass, _text, strlen(_text));
  } else {
    // If not fresh install read other configuration
    eeprom_read_block((void*)&trigger, (void*)sizeof(conf), sizeof(trigger));
    eeprom_read_block((void*)&timer,   (void*)sizeof(conf) + sizeof(trigger), sizeof(timer));
    // Reset trigger  dynamic values to 0
    for (uint8_t _trigger = 0; _trigger < TRIGGERS; _trigger++) {
      trigger[_trigger].setting &= ~(1 << 3); // Passed
      trigger[_trigger].setting &= ~(1 << 5); // Is triggered
    }
  }
  // Read group state/setting if power was lost
  if ((conf.setting >> 0) & B1) {
    eeprom_read_block((void*)&group, (void*)sizeof(conf)+sizeof(trigger)+sizeof(timer), sizeof(group));
    conf.setting |= (1 << 0); // Set it Off
  }
  
  GSM.begin(115200); // GSM modem or serial out
  #ifdef WEB_DEBUGGING
    WS.println(F("Start"));
  #endif

  // RFM69 radio
  radio.initialize((((conf.setting >> 7) & B1) ? RF69_915MHZ : RF69_868MHZ), RADIO_NODEID, RADIO_NETWORKID);
  radio.setHighPower(); //uncomment only for RFM69HW!
  radio.encrypt((char*)conf.radioKey); //comment this out to disable encryption

  // RS485 network, speed, our address 
  RS485.begin(19200, 0); 

  // ADC_PS_64 - 250kHz, ADC speed for 16MHz, faster readings less time for idle
  //  - ADC_PS_128: Arduino default (125 kHz on a 16 MHz CPU)
  //  - ADC_PS_64: (250 kHz on a 16 MHz CPU)
  //  - ADC_PS_32: (500 kHz on a 16 MHz CPU)
  nilAnalogPrescalar(ADC_PS_64); 

  // TWI 
  Wire.begin(); Wire.speed(I2C_400KHZ);

  Ethernet.init(1); // Teensy Etherent Init
  // Ethernet
  if ((conf.version != VERSION) || (conf.ip[0] == 0)) {
    GSM.print(F("DHCP: "));
    GSM.println(Ethernet.begin(mac)); // DHCP
    GSM.print(F("IP  : ")); GSM.println(Ethernet.localIP());
    GSM.print(F("GW  : ")); GSM.println(Ethernet.gatewayIP());
    GSM.print(F("Mask: ")); GSM.println(Ethernet.subnetMask());
    conf.ip   = Ethernet.localIP();
    conf.gw   = Ethernet.gatewayIP();
    conf.mask = Ethernet.subnetMask();
  } else {
    client.setServer(conf.mqtt_ip, conf.mqtt_port);
    client.setCallback(callback);
    Ethernet.begin(mac, conf.ip, conf.gw, conf.gw, conf.mask); // Loaded IP setting
  }


  // MQTT connect
  if (client.connect(str_MQTT_clientID)) {
    client.subscribe(str_MQTT_Subscribe);
  }

  // Web server
  webserver.begin();
  webserver.setDefaultCommand(&webHome);
  webserver.addCommand("h", &webHome);
  webserver.addCommand("l", &webListLog);
  webserver.addCommand("z", &webSetZone);
  webserver.addCommand("g", &webSetGroup);
  webserver.addCommand("r", &webSetGlobal);
  webserver.addCommand("c", &webSetPhone);
  webserver.addCommand("i", &webSetTimers);
  webserver.addCommand("s", &webSetSens);
  webserver.addCommand("k", &webSetKey);
  webserver.addCommand("m", &webSetMQTT);
  webserver.addCommand("t", &webSetTriggers);
  #ifdef WEB_DEBUGGING
  webserver.addCommand("d", &webDebug);
  #endif

  // Start kernel
  nilSysBegin();
 
  // On start set zones
  for (int8_t i=0; i < ALR_ZONES ; i++){
    zone[i].last_PIR = time_started.get();
    zone[i].last_OK  = time_started.get();
  }

  pushToLog("Ss"); // Issue start message
}
//------------------------------------------------------------------------------
// Loop is the idle thread.  The idle thread must not invoke any
// kernel primitive able to change its state to not runnable.
void loop() {
  while (1) {
    // Disable interrupts to insure increment is atomic.
    nilSysLock();
    idleCount[idlePointer]++;
    nilSysUnlock();
  }
}