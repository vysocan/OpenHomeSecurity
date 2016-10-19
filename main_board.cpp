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

#include <Ethernet.h>
static uint8_t mac[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  // CHANGE THIS TO YOUR OWN UNIQUE VALUE
IPAddress  our_ip( 10,  10,  10, 200);  // CHANGE THIS TO MATCH YOUR HOST NETWORK
IPAddress gateway( 10,  10,  10, 254);  // CHANGE THIS TO MATCH YOUR HOST NETWORK
IPAddress  subnet(255, 255, 255,   0);  // CHANGE THIS TO MATCH YOUR HOST NETWORK

// NTP
IPAddress  timeIP(195, 113, 144, 201);  // tik.cesnet.cz CHANGE THIS TO YOUE NEAREST NTP SERVER
#include <EthernetUdp.h>                      //
#define NTP_LOCAL_PORT         9999           // local port to listen for UDP packets
#define NTP_PACKET_SIZE        48             // NTP time stamp is in the first 48 bytes of the message
#define NTP_SECS_YR_1900_2000 (3155673600UL)
#define NTP_TIME_ZONE          2              // Central European Time
#define NTP_USELESS_BYTES      40             // Set useless to 32 for speed; set to 40 for accuracy.
#define NTP_POLL_INTV          100            // poll response this many ms
#define NTP_POLL_MAX           40             // poll response up to this many times
const long ntpFirstFourBytes = 0xEC0600E3;    // NTP request header
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
IPAddress  MQTTIP( 10,  10,  10, 126);
char str_MQTT_dev[] = "OHS2"; // device name
EthernetClient MQTTethClient;
PubSubClient client(MQTTIP, 1883, callback, MQTTethClient);

//SMTP
//#include <Client.h>
//#include <Mail.h>
//#include <SMTPClient.h>
//EthernetClient SMTPethClient;
//SMTPClient SMTPclient(SMTPethClient, "mail.smtp2go.com", 2525);
//Mail mail;

#include <RFM69.h>
#include <RFM69_ATC.h>
#define RADIO_NODEID      1
#define RADIO_NETWORKID   100
#define RADIO_FREQUENCY   RF69_868MHZ //Match this with the version of your Moteino! (others: RF69_433MHZ, RF69_868MHZ)
//#define KEY               "ABCDABCDABCDABCD" //has to be same 16 characters/bytes on all nodes, not more not less!
#define RADIO_ACK_TIME    10     // # of LOOPS to wait for an ack
#define ENABLE_ATC            //comment out this line to disable AUTO TRANSMISSION CONTROL
#define RADIO_UNIT_OFFSET 15
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
DateTime time_temp, time_started, timestamp;

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
#include "conf.h"

struct alarm_event_t {
  uint8_t zone;
  char    type;
};
NilFIFO<alarm_event_t, 3> alarm_fifo;  // Queue must be equal to # of AET 

struct zone_t {
  uint32_t last_PIR = 0;
  uint32_t last_OK  = 0;
  char last_event    = 'N';
  //       |- Full fifo queue flag
  //       ||- Free
  //       |||- Free
  //       ||||- Free
  //       |||||- Free
  //       ||||||- Alarm 
  //       |||||||- Free 
  //       ||||||||- 
  //      B00000000
  uint8_t setting   = 0;  
};
volatile zone_t zone[ALR_ZONES];

struct group_t {  
  //       |- Disabled group log once flag
  //       ||- Free
  //       |||- Free
  //       ||||- Free
  //       |||||- Free
  //       ||||||-  Waiting for authorization
  //       |||||||-  Alarm
  //       ||||||||-  Armed
  //      B00000000
  uint8_t setting   = 0;
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
//                    00000000
  uint16_t setting = B00011110;  // 2 bytes to store zone setting
  float    value   = 0;
  uint32_t last_OK = 0;
  int8_t      RSSI = -127;
};
#define NODES 32
node_t node[NODES];
volatile uint8_t nodes = 0;
NilFIFO<node_t, 6> node_fifo;

// node names
struct node_name_t {
  uint8_t address  = 0;
  char    name[16] = "";
};
#define node_NAMES 8
node_name_t node_name[node_NAMES];
volatile uint8_t node_names = 0;

// Float conversion 
union u_tag {
  byte  b[4]; 
  float fval;
} u;

// Triggers 
struct trigger_t {
  uint8_t address      = 0;
  char    type         = 'U';
  uint8_t number       = 0;
//                        |- Pass negative once
//                        ||- Logging enabled 
//                        |||- Is triggered
//                        ||||- Pass once / pass always
//                        |||||- Passed
//                        ||||||- Pass
//                        |||||||- Pass value or constant
//                        ||||||||- Enabled   
//                        00000000     
  uint8_t setting      = B00000000;
  uint8_t symbol       = 1;
  float   value        = 0;
  float   constant_on  = 1;
  float   constant_off = 0;
  uint8_t to_address   = 0;
  char    to_type      = 'U';
  uint8_t to_number    = 0;
  char    name[16]     = "";
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
//                         ||||||||- Monday
//                         ||||||||         |- Tuesday
//                         ||||||||         ||- Wednesday
//                         ||||||||         |||- Thursday
//                         ||||||||         ||||- Friday
//                         ||||||||         |||||- Saturday
//                         ||||||||         ||||||- Sunday
//                         ||||||||         |||||||- Calendar / Period
//                         ||||||||         ||||||||-  Enabled   
//                        B10000000         00000000
  uint16_t setting      = B00000000 << 8 | B00000000;
  uint8_t  period       = 0; // number of intervals
  uint16_t start_time   = 0; // for calendar timer in minutes 0 - 1440
  uint8_t  run_time     = 0; // runtime interval
  int16_t  constant_on  = 1; // value to pass 
  int16_t  constant_off = 0; // value to pass 
  uint8_t  to_address   = 0; 
  char     to_type      = 'U';
  uint8_t  to_number    = 0;
  uint32_t next_on      = 0;
  uint32_t next_off     = 0;
  char     name[16]     = "";
};
#define TIMERS 10
timer_t timer[TIMERS];

// Registration
struct register_t {
  char    node     = ' ';
  uint8_t address  = 0;
  char    type     = ' ';
  uint8_t number   = 0;
  uint8_t setting  = 0;
  char    name[16] = "";
};
NilFIFO<register_t, 8> reg_fifo;

// Free ticks
#define idleSlots                 10
volatile uint32_t idleCount[idleSlots];           
volatile uint8_t  idlePointer    = 0;
// Global variables
volatile uint8_t  ACState        = 0;
volatile uint8_t  OUTs           = 0;         // Output pins
volatile uint8_t  MQTTState      = 0;
volatile uint16_t radio_no_ack   = 0;
volatile uint32_t radio_received = 0;
volatile uint32_t radio_sent     = 0;

char last_key[KEY_LEN+1] = "UUUUUUUU";

char tmp[17];       // for logger 
char _tmp[4];       // for logger 

char  b64_text[30]; // encode text

// GSM modem 
uint8_t GSMisAlive = 0, GSMreg = 4, GSMstrength = 0, GSMsetSMS = 0;

// tmp
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
void pushToLog(char *what){ 
  log_event_t* p = log_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
  if (p == 0) return; // return if no free space.
  
  time_temp = timestamp.get(); 
  p->text[0] = 0;
  strcat (p->text, time_temp.timestamp());
  strcat (p->text, "?");
  strcat (p->text, what);

  log_fifo.signalData();  // Signal idle thread data is available.
}

// wait a few milliseconds for proper ACK to me, return true if indeed received
static bool waitForAck(uint8_t nodeID) {
  uint8_t _counter = 0;
  while ( _counter < RADIO_ACK_TIME) {
    nilThdSleepMilliseconds(5);
    if (radio.ACKReceived(nodeID)) return true;
    _counter++;
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
    WS.print(F(" R:")); WS.println(_resp);
  }
  return _resp;
}

// Send data to node
uint8_t sendData(uint8_t node, char *data, uint8_t length){ 
  uint8_t _resp = 0;
  #if WEB_SERIAL_DEBUGGING 
  WS.print(F("Data to node: ")); WS.print(node); WS.print(F(", len:")); WS.print(length);
  for (uint8_t i=0; i < length; i++){
    WS.print(F("-")); WS.print((uint8_t)data[i],HEX); 
  }
  WS.println();
  #endif
  if (node <= RADIO_UNIT_OFFSET) {
    TX_msg.address = node;
    TX_msg.ctrl = FLAG_DTA;
    TX_msg.data_length = length;
    memcpy(TX_msg.buffer, data, length);
    return RS485.msg_write(&TX_msg);
  } else {
    nilSemWait(&RFMSem); 
    radio.send(node-RADIO_UNIT_OFFSET, data, length, true);
    radio_sent++;
    _resp = waitForAck(node-RADIO_UNIT_OFFSET);
    if (!_resp) ++radio_no_ack;
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
  // Go throug all nodes
  WS.print("Grp cmd:"); WS.print(cmd); WS.print(", node:");
  for (int8_t i=0; i < nodes; i++){
    if (node[i].setting & B1) {                       // Auth. node is enabled ?
      if (((node[i].setting >> 1) & B1111) == grp) {  // Auth. node belong to group
        if (node[i].address <= RADIO_UNIT_OFFSET) {
          TX_msg.address = node[i].address;
          TX_msg.ctrl = FLAG_CMD;
          TX_msg.data_length = cmd;
          _try = 0;
          do {
            _resp = RS485.msg_write(&TX_msg);
            _try++;
            nilThdSleepMilliseconds(10);
          } while (_resp != 1 || _try < 5);
          if (_resp) _cnt++;
          WS.print(node[i].address); WS.print(":"); WS.print(_resp); WS.print(", ");
        } else {
          nilSemWait(&RFMSem);
          radio.send(node[i].address-RADIO_UNIT_OFFSET, _cmd, 2, true);
          radio_sent++;
          if (waitForAck(node[i].address-RADIO_UNIT_OFFSET)) {_cnt++;}
          else {++radio_no_ack;}
          nilSemSignal(&RFMSem);  // Exit region.
          WS.print(node[i].address-RADIO_UNIT_OFFSET); WS.print(":"); WS.print(_resp); WS.print(", ");
        }
      }
    }
  }
  WS.println();
  return _cnt;
}

void saveConf(uint8_t log = 1){
  eeprom_update_block((const void*)&conf,    (void*)0, sizeof(conf)); // Save current configuration
  eeprom_update_block((const void*)&trigger, (void*)sizeof(conf), sizeof(trigger)); // Save current configuration
  eeprom_update_block((const void*)&timer,   (void*)sizeof(conf) + sizeof(trigger), sizeof(timer)); 
  if (log) _tmp[0] = 'S'; _tmp[1] = 'C'; _tmp[2] = 'W'; _tmp[3] = 0; pushToLog(_tmp);
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
  _tmp[0] = 'S'; _tmp[1] = 'L'; _tmp[2] = 0; pushToLog(_tmp);
  saveConf();
}



unsigned long GetNTPTime(UDP &udp){
  //udpInited = udp.begin(NTP_LOCAL_PORT); // open socket on arbitrary port
  if (!udp.begin(NTP_LOCAL_PORT)) return 0; // Fail if WiFiUdp.begin() could not init a socket
  udp.flush();              // Clear received data from possible stray received packets
  
  // Send an NTP request 
  if (!(udp.beginPacket(timeIP, 123)      // 123 is the NTP port
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

// MQTT Callback function
void callback(char* topic, byte* payload, unsigned int length) {
  uint8_t _resp, _found, _number, _i, _update_node;
  char * _pch;
  char _text[40];
  char _message[6];
  // In order to republish this payload, a copy must be made
  // as the orignal payload buffer will be overwritten whilst
  // constructing the PUBLISH packet.
  
  strncpy(_text,(char*)payload, length); _text[length] = 0;

  WS.print(topic); WS.print(F(":")); WS.print(_text); WS.print(F(":")); WS.println(length);

  // get topic
  _pch = strtok(topic,"/"); // Set pointer
  _pch = strtok(NULL, "/"); // OHS
  _pch = strtok(NULL, "/"); // In
  // Main tree
  if (_pch != NULL) {
    WS.print(_pch); WS.print(F(":"));
    // Pass to Input node
    if (strcmp(_pch, "Input") == 0)  {
      _pch = strtok(NULL, "/");
      WS.print(_pch); WS.print(F(":"));
      if (_pch != NULL) {
        // Get node name
        _resp = 1;
        for (_i = 0; _i < node_NAMES; ++_i) {
          _resp = strcmp(_pch, node_name[_i].name); 
          if (_resp == 0) break;
        }
        // We have name
        if (_resp == 0) {
          _pch = strtok(NULL, "/");
          // Node local number
          if (_pch != NULL) {
            _number = (uint8_t)_pch[0]-48;
            WS.println(_number);
            // Select input node to update its value and timestamp
            _found = 0;
            for (_update_node = 0; _update_node < nodes; _update_node++) {
              if (node[_update_node].address  == node_name[_i].address &&
                  node[_update_node].function == 'I' &&
                  node[_update_node].number   == _number &&
                  node[_update_node].setting & B1) {
                _found = 1;
                break;
              }
            }
            if (_found) {
              node[_update_node].last_OK = timestamp.get(); // update receiving node current timestamp
              u.fval = atof(_text);                         // convert payload to float
              node[_update_node].value   = u.fval;          // update receiving node value
              _message[0] = 'I'; // 'I'nput only
              _message[1] = _number;
              _message[2] = u.b[0]; _message[3] = u.b[1]; _message[4] = u.b[2]; _message[5] = u.b[3];
              sendData(node[_update_node].address, _message, 6);
            }
          }
        }
      }
    }
    // Send SMS
    else if (strcmp(_pch, "SMS") == 0)  {

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

int base64_encode(char *output, char *input, uint8_t inputLen) {
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
  return encLen;
}

void set_timer(uint8_t _timer, uint8_t _restart = 1) {
  uint8_t _day, _found;
  uint32_t _add, _tmp;
  time_temp = timestamp.get();

  // Calendar
  if (timer[_timer].setting >> 1 & B1) {
    WS.print(F("CAL: ")); WS.print(_timer);
    // look for next day
    _day = time_temp.dayOfWeek(); _found = 0; _add = 0;
    // Today start_time has passed already?
    //if ((time_temp.hour()*60 + time_temp.minute()) > timer[_timer].start_time) _day++;
    //if (_day > 7) _day = 1;

    WS.print(F(" Today: ")); WS.print(_day);
    for (uint8_t i = 0; i < 8; i++) {
      // _day = 1 = Monday, _day = 7 = Saturday
      if ((timer[_timer].setting >> 9-_day) & B1) {
        if (_day != time_temp.dayOfWeek()) {_found = 1; break; }
        else {
          if (((time_temp.hour()*60 + time_temp.minute()) < timer[_timer].start_time) ||
             _add > 0) {_found = 1; break; }
          else _add++;
        }
      } else _add++;
      if (_day == 7) _day = 0;
      _day++; 
      WS.print(F(" _day: ")); WS.print(_day);
    }
    if (_found) {
      WS.print(F(" F_day: ")); WS.print(_day);
      WS.print(F(" _add: ")); WS.print(_add);
      // Today start_time has passed already?
      //if (((time_temp.hour()*60 + time_temp.minute()) > timer[_timer].start_time) && _add == 0) _add = 7;
      // Calculate On time
      timer[_timer].next_on = time_temp.get() - ((uint32_t)time_temp.hour()*3600 + (uint32_t)time_temp.minute()*60 + (uint32_t)time_temp.second()) + 
        ((uint32_t)timer[_timer].start_time*60) + ((uint32_t)_add * 86400);
    } else timer[_timer].next_on = 0;
  // Period
  } else {
    WS.print(F("Period: ")); WS.print(_timer);
    switch(timer[_timer].setting >> 12 & B11){
      case 0:  _add = (uint32_t)timer[_timer].period; break;
      case 1:  _add = (uint32_t)timer[_timer].period*60; break;
      case 2:  _add = (uint32_t)timer[_timer].period*3600; break;
      default: _add = (uint32_t)timer[_timer].period*86400; break;
    }
    WS.print(F(" _add: ")); WS.print(_add);
    // request come from Web interface
    if (_restart) {
      timer[_timer].next_on = time_temp.get() - ((uint32_t)time_temp.hour()*3600 + (uint32_t)time_temp.minute()*60 + (uint32_t)time_temp.second()) + 
        ((uint32_t)timer[_timer].start_time*60);
      WS.print(F(" next_on: ")); WS.print(timer[_timer].next_on);
      // if next_on is in past calculate new next_on
      if (time_temp.get() > timer[_timer].next_on) {
        _tmp = (time_temp.get() - timer[_timer].next_on) / _add; 
        WS.print(F(" _tmp: ")); WS.print(_tmp);   
        timer[_timer].next_on += (_tmp + 1) * _add;
      }
    } else {
      timer[_timer].next_on += _add;
    } 
    WS.print(F(" next_on: ")); WS.print(timer[_timer].next_on);
    WS.println();
  }
  // Set Off time
  timer[_timer].next_off = timer[_timer].next_on;
  switch(timer[_timer].setting >> 14 & B11){
    case 0:  timer[_timer].next_off += (uint32_t)timer[_timer].run_time; break;
    case 1:  timer[_timer].next_off += (uint32_t)timer[_timer].run_time*60; break;
    case 2:  timer[_timer].next_off += (uint32_t)timer[_timer].run_time*3600; break;
    default: timer[_timer].next_off += (uint32_t)timer[_timer].run_time*86400; break;
  }
}

void process_triggers(uint8_t _address, char _type, uint8_t _number, float _value) {
  uint8_t _trigger, _found, _update_node;
  char _message[6];
  for (_trigger = 0; _trigger < TRIGGERS; _trigger++) {
  //  trigger enabled
  if (trigger[_trigger].setting & B1) { 
    /*
    WS.print(">");WS.print(trigger[_trigger].address);
    WS.print(">");WS.print(trigger[_trigger].type);
    WS.print(">");WS.print(trigger[_trigger].number);
    WS.print(">");WS.print(trigger[_trigger].setting, BIN);
    WS.println();
    */
    if (trigger[_trigger].address == _address && 
        trigger[_trigger].type    == _type &&
        trigger[_trigger].number  == _number) {
      // check value
      _found = 0;
      switch(trigger[_trigger].symbol){
        case 0: _found = 1; break; // Always
        case 1: if (_value == trigger[_trigger].value) _found = 1; break;
        case 2: if (_value != trigger[_trigger].value) _found = 1; break;
        case 3: if (_value <  trigger[_trigger].value) _found = 1; break;
        case 4: if (_value >  trigger[_trigger].value) _found = 1; break;
        default : break;
      }
      if (_found) {
        //    Logging enabled                          not triggered
        if (((trigger[_trigger].setting) >> 6 & B1) && !((trigger[_trigger].setting) >> 5 & B1)) {
          _tmp[0] = 'R'; _tmp[1] = _trigger + 48; _tmp[2] = 0; pushToLog(_tmp);
        }
        trigger[_trigger].setting |= (1 << 5); // switch ON Is triggered
        //   Pass
        if ((trigger[_trigger].setting) >> 2 & B1) {
          // Select input node to update its value and timestamp
          _found = 0;
          for (_update_node = 0; _update_node < nodes; _update_node++) {
            /*
            WS.print(">");WS.print(node[_update_node].address);
            WS.print(">");WS.print(node[_update_node].type);
            WS.print(">");WS.print(node[_update_node].number);
            WS.println();
            */
            if (node[_update_node].address  == trigger[_trigger].to_address &&
                node[_update_node].function == 'I' &&
                node[_update_node].number   == trigger[_trigger].to_number) { 
              //WS.print("_update_node: "); WS.println(_update_node);
              _found = 1;
              break;
            }
          }
          if (_found) {
            node[_update_node].last_OK = timestamp.get(); // update receiving node current timestamp
            //WS.print(" pass");
            _message[0] = 'I'; // 'I'nput only
            _message[1] = trigger[_trigger].to_number;
            if ((trigger[_trigger].setting) >> 1 & B1) {
              //WS.println(" value");
              u.fval = _value;
              node[_update_node].value = _value; // update receiving node value
            } else {
              //WS.println(" const");
              u.fval = trigger[_trigger].constant_on;
              node[_update_node].value = trigger[_trigger].constant_on; // update receiving node value  
            }
            _message[2] = u.b[0]; _message[3] = u.b[1]; _message[4] = u.b[2]; _message[5] = u.b[3];
            //     passed
            if (!((trigger[_trigger].setting) >> 3 & B1)) {
              sendData(trigger[_trigger].to_address, _message, 6);
              //   pass only once
              if ((trigger[_trigger].setting) >> 4 & B1) {
                trigger[_trigger].setting |= (1 << 3); // switch ON passed
              }
            }
          } //_found
        }  
      } else { // not found
        //    Pass Off enabled                           Is alerted/triggered
        if (((trigger[_trigger].setting) >> 7 & B1) && ((trigger[_trigger].setting) >> 5 & B1))  {
          // Select input node to update its value and timestamp
          _found = 0;
          for (_update_node = 0; _update_node < nodes; _update_node++) {
            /*
            WS.print(">");WS.print(node[_update_node].address);
            WS.print(">");WS.print(node[_update_node].type);
            WS.print(">");WS.print(node[_update_node].number);
            WS.println();
            */
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
            node[_update_node].last_OK = timestamp.get(); // update receiving node current timestamp
            // Pass Off value
            //WS.print(" pass Off");
            _message[0] = 'I'; // 'I'nput only
            _message[1] = trigger[_trigger].to_number;
            if ((trigger[_trigger].setting) >> 1 & B1) {
              //WS.println(" value");
              u.fval = _value;
              node[_update_node].value = _value; // update receiving node value
            } else {
              u.fval = trigger[_trigger].constant_off;
              node[_update_node].value = trigger[_trigger].constant_off; // update receiving node value  
              //WS.println(" const");
            }
            _message[2] = u.b[0]; _message[3] = u.b[1]; _message[4] = u.b[2]; _message[5] = u.b[3];
            sendData(trigger[_trigger].to_address, _message, 6);
          } //_found
        }
        trigger[_trigger].setting &= ~(1 << 5); // switch OFF Is alerted/triggered
        trigger[_trigger].setting &= ~(1 << 3); // switch OFF passed
      }            
    } // trigger match
  } // trigger enabled
  } // for (trigger)
}

void formatKey(char* in, char* out) { // Format the key to nice HEX string
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
NIL_WORKING_AREA(waZoneThread, 64);
NIL_THREAD(ZoneThread, arg) {
  int16_t val;
  uint8_t _group = 255;
  uint8_t _resp;
  
  nilThdSleepSeconds(60); // Delay to allow PIR nodes to settle up
  _tmp[0] = 'S'; _tmp[1] = 'S'; _tmp[2] = 0; pushToLog(_tmp);
  #if WEB_SERIAL_DEBUGGING 
  WS.println(F("Zone thread started"));
  #endif
  
  // Execute while loop every 0.25 seconds.
  while (TRUE) {
    nilThdSleepMilliseconds(250); // time is used also for arm delay and others ...

    for (int8_t i=0; i < ALR_GROUPS ; i++){ 
      if (group[i].arm_delay) { // wait for arm delay
        group[i].arm_delay--;
        if (!group[i].arm_delay) { 
          _resp = sendCmdToGrp(_group, 15);  // send arm message to all nodes
          _tmp[0] = 'S'; _tmp[1] = 'Z'; _tmp[2] = 48+i; _tmp[3] = 0; pushToLog(_tmp);
        }
      }
    }
   
    //WS.print(F("Zone: "));
    for (int8_t i=0; i < ALR_ZONES ; i++){  
      if (conf.zone[i] & B1){         // Zone enabled ?
        //WS.print(i+1); WS.print(F(":"));
        if (conf.zone[i] >> 15){       // Digital 0/ Analog 1 
          nilSemWait(&ADCSem);          // Wait for slot
          val  = nilAnalogRead(i);
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
            //WS.print(F("P,"));
            //     zone not have alarm              group delay is 0
            if (!((zone[i].setting >> 1) & B1) && !(group[_group].arm_delay)){
              //WS.print(F("N,"));
              // if group not enabled log error to log. 
              if (!(conf.group[_group] & B1)) {
                if (!((group[_group].setting >> 7) & B1)) {
                  group[_group].setting |= (1 << 7); // Set logged disabled bit On
                  _tmp[0] = 'G'; _tmp[1] = 'F'; _tmp[2] = 48+_group; _tmp[3] = 0; pushToLog(_tmp);
                }
              } else {
                //   group armed
                if ((group[_group].setting & B1) && (zone[i].last_event == 'P')) {
                  //WS.println(F(">P"));
                  alarm_event_t* p = alarm_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
                  if (p == 0) {
                    if (!((zone[i].setting >> 7) & B1)) {
                      _tmp[0] = 'P'; _tmp[1] = 48+i; _tmp[2] = 0; pushToLog(_tmp); 
                      _tmp[0] = 'F'; _tmp[1] = 'A'; _tmp[2] = 0; pushToLog(_tmp); // Alarm queue is full
                    }
                    zone[i].setting |= (1 << 7); // Set On Alarm queue is full  
                    continue; // Continue if no free space.
                  }
                  _tmp[0] = 'P'; _tmp[1] = 48+i; _tmp[2] = 0; pushToLog(_tmp); 
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
                  _tmp[0] = 'G'; _tmp[1] = 'F'; _tmp[2] = 48+_group; _tmp[3] = 0; pushToLog(_tmp);
                }
              } else {
                if (zone[i].last_event == 'T') {
                  //WS.println(F(">T"));
                  alarm_event_t* p = alarm_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
                  if (p == 0) {
                    if (!((zone[i].setting >> 7) & B1)) {
                      _tmp[0] = 'T'; _tmp[1] = 48+i; _tmp[2] = 0; pushToLog(_tmp);
                      _tmp[0] = 'F'; _tmp[1] = 'A'; _tmp[2] = 0; pushToLog(_tmp); // Alarm queue is full
                    }
                    zone[i].setting |= (1 << 7); // Set On Alarm queue is full                    
                    continue; // Continue if no free space.
                  }
                  _tmp[0] = 'T'; _tmp[1] = 48+i; _tmp[2] = 0; pushToLog(_tmp);
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
  uint16_t _counter = 1;
  nilTimer1Start(1000000);  
  
  // Execute while loop at every second.
  while (TRUE) {
    nilTimer1Wait();
    if (_counter == 0) {
      nilSemWait(&TWISem);     // wait for slot
      timestamp = RTC.now();
      nilSemSignal(&TWISem);   // Exit region.
    } else {
      timestamp = timestamp.get()+1;
    }
    _counter++;
    //watchdog |= (1 << 1); // Flag watchdog for this thread
  }
}

//------------------------------------------------------------------------------
// Alarm Events
//
NIL_WORKING_AREA(waAEThread1, 64);
NIL_WORKING_AREA(waAEThread2, 64);
NIL_WORKING_AREA(waAEThread3, 64);
NIL_THREAD(thdFcn, name) {
  uint8_t _group, _wait, _resp, _cnt;

  #if WEB_SERIAL_DEBUGGING 
  WS.print((char*)name);  WS.println(F(" started"));
  #endif
  
  while (TRUE) {
    // Check for data.  Use TIME_IMMEDIATE to prevent sleep in idle thread.
    alarm_event_t* p = alarm_fifo.waitData(TIME_INFINITE);

    _group = (conf.zone[p->zone] >> 1) & B1111;
    WS.print((char*)name);
    //   Group has alarm already nothing to do!
    if ((group[_group].setting >> 1) & B1) {
      WS.println(F(" stop"));
      alarm_fifo.signalFree();
      continue;
    } 
    WS.print(F(" go"));

    group[_group].setting |= (1 << 2); // Set Authentication On

    if (p->type == 'P') _wait = (conf.zone[p->zone] >> 5) & B11;
    else                _wait = 0;            // Tamper have no wait time

    WS.print(F(" zone: ")); WS.print(p->zone); WS.print(F(", type: ")); WS.print(p->type);
    WS.print(F(", group: ")); WS.print(_group); WS.print(F(", Auth time: ")); WS.println(_wait);
    
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
      _tmp[0] = 'S'; _tmp[1] = 'X';  _tmp[2] = 48 + _group; _tmp[3] = 0;pushToLog(_tmp); // ALARM no auth.
    }
    WS.print((char*)name); WS.println(F(" end"));
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

  #if WEB_SERIAL_DEBUGGING 
  WS.println(F("RS485 receiver thread started"));
  #endif
  
  while (TRUE) {   
    nilWaitRS485NewMsg(); // wait for event
    _resp = RS485.msg_read(&RX_msg);
    
    //#if WEB_SERIAL_DEBUGGING 
    WS.print(F(">Wire A:")); WS.print(RX_msg.address); WS.print(F(", C:")); WS.print(RX_msg.ctrl);
    WS.print(F(", L:")); WS.print(RX_msg.data_length); WS.print(F(", "));   
    for (uint8_t i=0; i < RX_msg.data_length; i++){
      WS.print((uint8_t)RX_msg.buffer[i],HEX); WS.print(F(" "));
    }; WS.println();
    //#endif
      
    // iButtons keys
    if (RX_msg.ctrl == FLAG_DTA && RX_msg.data_length == KEY_LEN && RX_msg.buffer[0]!='R') {  // incoming message looks like a key 
      _found = 0;
      // search node with given address
      for (_node=0; _node < nodes; _node++) {
        if (node[_node].address == RX_msg.address) {
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
              if (true) { // key enabled
                _group = (node[_node].setting >> 1) & B1111;
                //     we have alarm                          group is armed
                if  (((group[_group].setting) >> 1 & B1) || ((group[_group].setting) & B1)) { 
                  if ((group[_group].setting) >> 1 & B1) { // we have alarm
                    group[_group].setting &= ~(1 << 1);    // set group alarm off
                    // set each member zone alarm off
                    for (uint8_t j=0; j < ALR_ZONES; j++){
                      if (((conf.zone[j] >> 1) & B1111) == _group)
                        zone[j].setting &= ~(1 << 1); 
                    }
                    OUTs = 0; // Reset outs  
                    pinOUT1.write(LOW); pinOUT2.write(LOW); // Turn off OUT 1 & 2
                  }
                  group[_group].setting &= ~(1 << 0);    // disarm group
                  group[_group].setting &= ~(1 << 2);    // set auth bit off
                  group[_group].arm_delay = 0;       // Reset arm delay
                  _resp = sendCmdToGrp(_group, 20);  // send quiet message to all nodes
                  _tmp[0] = 'A'; _tmp[1] = 'D'; _tmp[2] = 48+i; _tmp[3] = 0; pushToLog(_tmp);
                } else { // Just do arm
                  _tmp[0] = 'A'; _tmp[1] = 'A'; _tmp[2] = 48+i; _tmp[3] = 0; pushToLog(_tmp);
                  // if group enabled arm group or log error to log. 
                  if (conf.group[_group] & B1) { 
                    group[_group].setting |= 1;                   // arm group
                    group[_group].arm_delay = conf.arm_delay; // set arm delay
                    _resp = sendCmdToGrp(_group, 10);  // send arming message to all nodes
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
        } // node is enabled for authorization
        else { // log disabled remote nodes
          _tmp[0] = 'U'; _tmp[1] = 'F'; _tmp[2] = 48 + RX_msg.address; _tmp[3] = 0; pushToLog(_tmp);
        } 
      } else { // node not found
        // call this address to register
        _resp = sendCmd(RX_msg.address,1);
      }
    } // incoming message looks like a key

    // Registration
    if (RX_msg.ctrl == FLAG_DTA && RX_msg.buffer[0]=='R') {
      _pos = 1;
      do {
        register_t *p = reg_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
        if (p == 0) {
          _tmp[0] = 'F'; _tmp[1] = 'R'; _tmp[2] = 0; pushToLog(_tmp); // Registration queue is full
          continue; // Continue if no free space.
        }
        switch(RX_msg.buffer[_pos]){
          case 'N': 
            // node name
            p->node    = RX_msg.buffer[_pos];    
            p->address = RX_msg.address;
            for (uint8_t _t=0; _t < 16; _t++) {p->name[_t] = RX_msg.buffer[_pos+1+_t];}
            reg_fifo.signalData();   // Signal idle thread data is available.
            _pos+=17;
            break;
          default : 
            // node setting
            p->node    = RX_msg.buffer[_pos];    
            p->address = RX_msg.address;
            p->type    = RX_msg.buffer[_pos+1];
            p->number  = (uint8_t)RX_msg.buffer[_pos+2];
            p->setting = ((uint8_t)RX_msg.buffer[_pos+3] << 8) | ((uint8_t)RX_msg.buffer[_pos+4]);
            reg_fifo.signalData();   // Signal idle thread data is available.
            _pos+=5;
            break;
        }
      } while (_pos < RX_msg.data_length);
    }
    // Sensors
    if (RX_msg.ctrl == FLAG_DTA && RX_msg.buffer[0]=='S') {
      _pos = 1;
      do {
        node_t *p = node_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
        if (p == 0) {
          _tmp[0] = 'F'; _tmp[1] = 'S'; _tmp[2] = 0; pushToLog(_tmp); // node queue is full
          continue; // Continue if no free space.
        }
        p->address  = RX_msg.address;
        p->type     = RX_msg.buffer[_pos];
        p->number   = (uint8_t)RX_msg.buffer[_pos+1];
        u.b[0] = RX_msg.buffer[_pos+2]; u.b[1] = RX_msg.buffer[_pos+3]; u.b[2] = RX_msg.buffer[_pos+4]; u.b[3] = RX_msg.buffer[_pos+5];
        p->value    = u.fval;
        node_fifo.signalData();     // Signal idle thread data is available.
        _pos+=6;
      } while (_pos < RX_msg.data_length);
    } // if 'S'
  }
}

/*
int readLine(char *line, int maxLen) {
  uint32_t start = timestamp.get();
  uint32_t now = timestamp.get(); 
  int count = 0;
  int cr    = 0;
  int c     = -1;

  WS.print("RL");
  while (c == -1) {
    WS.print(".");
    nilThdSleepMilliseconds(25);
    now = timestamp.get();
    if (now < start || now - start > 5000) { return 0; }
    c = SMTPethClient.read();
  }

  while (true) {
    WS.print((char)c);
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


int readStatus() {
  char line[4];
  int result;
  while(true) {
    result = readLine(line, 4);
    if (result >= 4 && (line[3] != '-')) { break; }
  }

  if (result < 3) { return 0; }

  char st[4];
  strncpy(st, line, 3);
  st[3] = '\0';
  WS.print("rS:");WS.println(st); 
  return strtol(st, NULL, 10);
}

*/
//------------------------------------------------------------------------------
// Logger thread
//
NIL_WORKING_AREA(waLoggerThread, 256);
NIL_THREAD(LoggerThread, arg) {
  uint8_t sms_send, _group;
  int8_t sms_ok;
  int status;
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
  
  #if WEB_SERIAL_DEBUGGING 
  WS.println(F("Logger thread started"));
  #endif
  
  while (TRUE) {
    // Check for data.  Use TIME_IMMEDIATE to prevent sleep in idle thread.
    log_event_t* p = log_fifo.waitData(TIME_INFINITE);

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
            if (conf.alerts[alert_SMS] >> 11 & B1) sms_send = 1;
            strcat_P(sms_text, (char*)text_battery);
            if (log_message[15] == 'L') strcat_P(sms_text, (char*)text_low);
            else strcat_P(sms_text, (char*)text_OK);
            break;
          case 'A': // AC
            if (conf.alerts[alert_SMS] >> 10 & B1) sms_send = 1;
            strcat_P(sms_text, (char*)text_PWS);
            if (log_message[15] == 'L') strcat_P(sms_text, (char*)text_On);
            else strcat_P(sms_text, (char*)text_Off);
            break;
          case 'C': strcat_P(sms_text, (char*)text_Configuration); strcat_P(sms_text, (char*)text_space);
            switch(tmp[15]){
              case 'W': strcat_P(sms_text, (char*)text_saved); break; // conf. saved
              case 'P': strcat_P(sms_text, (char*)text_saved); strcat_P(sms_text, (char*)text_cosp);
                        strcat_P(sms_text, (char*)text_monitoring); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_disabled);
                        break; // conf. saved
              case 'L': strcat_P(sms_text, (char*)text_loaded); break; // conf. saved
              case 'R': strcat_P(sms_text, (char*)text_reset); break; // conf. saved
              default:  strcat_P(sms_text, (char*)text_undefined); break; // unknown  
            }
            if (conf.alerts[alert_SMS] >> 9 & B1) sms_send = 1; // conf. saved
            break;
          case 'Z': strcat_P(sms_text, (char*)text_Group); strcat_P(sms_text, (char*)text_space);
                    strcat(sms_text, conf.group_name[log_message[15]-48]); strcat_P(sms_text, (char*)text_space);
                    strcat_P(sms_text, (char*)text_armed);
                    if (conf.alerts[alert_SMS] >> 8 & B1) sms_send = 1;
                    break; // system armed
          case 'S': strcat_P(sms_text, (char*)text_Monitoring); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_started); if (conf.alerts[alert_SMS] >> 7 & B1) sms_send = 1;
                    break; // monitoring strted
          case 'X': strcat_P(sms_text, (char*)text_ALARM);  if (conf.alerts[alert_SMS] >> 6 & B1) sms_send = 1;
                    strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_Group); strcat_P(sms_text, (char*)text_sesp);
                    strcat(sms_text, conf.group_name[log_message[15]-48]); // extra text for SMS
                    _group = log_message[15]-48;
                    break; // alarm
            /* ****************************** ADD
            case 's': server.printP(text_started); break;   // boot
            case 'M': 
              server.printP(text_MQTT); server.printP(text_space);
              switch(tmp[15]){
                case 'F': server.printP(text_network); server.printP(text_space); server.printP(text_failed); break;
                case 'O': server.printP(text_network); server.printP(text_space); server.printP(text_OK); break;
                default:  server.printP(text_undefined); break; // unknown  
                }
              break;
            */
          default:  strcat_P(sms_text, (char*)text_undefined); 
            // if (conf.alerts[alert_SMS] >> 9 & B1) sms_send = 1;
            break; // unknown
          }
          break;
        case 'P':
          if (conf.alerts[alert_SMS] >> 5 & B1) sms_send = 1;
          strcat_P(sms_text, (char*)text_Alarm); strcat_P(sms_text, (char*)text_sesp); strcat_P(sms_text, (char*)text_trigger); strcat_P(sms_text, (char*)text_ed); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_zone); strcat_P(sms_text, (char*)text_space);
          strcat(sms_text, conf.zone_name[log_message[14]-48]);
          _group = (conf.zone[log_message[14]-48] >> 1) & B1111;
          break;
        case 'T':
          if (conf.alerts[alert_SMS] >> 4 & B1) sms_send = 1;
          strcat_P(sms_text, (char*)text_Tamper); strcat_P(sms_text, (char*)text_sesp); strcat_P(sms_text, (char*)text_trigger); strcat_P(sms_text, (char*)text_ed); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_zone); strcat_P(sms_text, (char*)text_space);
          strcat(sms_text, conf.zone_name[log_message[14]-48]);
          _group = (conf.zone[log_message[14]-48] >> 1) & B1111;
          break;
        case 'A': // Authentication
          strcat_P(sms_text, (char*)text_Authentication); strcat_P(sms_text, (char*)text_sesp); 
          if (log_message[14] < 'a') { strcat_P(sms_text, (char*)text_key); strcat_P(sms_text, (char*)text_space); }
          if (log_message[14] == 'A' || log_message[14] == 'D' || log_message[14] == 'F') { strcat(sms_text, conf.key_name[log_message[15]-48]); strcat_P(sms_text, (char*)text_space); }  
          switch(log_message[14]){
            case 'D': strcat_P(sms_text, (char*)text_disarmed); if (conf.alerts[alert_SMS] >> 1 & B1) sms_send = 1; break;
            case 'A': strcat_P(sms_text, (char*)text_armed); if (conf.alerts[alert_SMS] >> 2 & B1) sms_send = 1; break;
            case 'U': strcat_P(sms_text, (char*)text_undefined); if (conf.alerts[alert_SMS] & B1) sms_send = 1; break;
            case 'F': strcat_P(sms_text, (char*)text_is); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_disabled); break;
            case 'a': strcat_P(sms_text, (char*)text_auto); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_armed);
                      strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_zone); strcat_P(sms_text, (char*)text_space); strcat(sms_text, conf.zone_name[log_message[15]-48]);
                      if (conf.alerts[alert_SMS] >> 2 & B1) sms_send = 1;
                      _group = (conf.zone[log_message[15]-48] >> 1) & B1111;
                      break; // Auto armed
            case 'o': strcat_P(sms_text, (char*)text_zone); strcat_P(sms_text, (char*)text_space); strcat(sms_text, conf.zone_name[log_message[15]-48]);
                      strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_is); strcat_P(sms_text, (char*)text_space); strcat_P(sms_text, (char*)text_open);
                      if (conf.alerts[alert_SMS] >> 12 & B1) sms_send = 1;
                      _group = (conf.zone[log_message[15]-48] >> 1) & B1111;
                      break; // Open alarm              
            default: break;
          }
          break;
        default: // do nothing
        break;
      }
      strcat_P(sms_text, (char*)text_dot);

      
      if (sms_send) {
        for (uint8_t i = 0; i < NUM_OF_PHONES; ++i) {
          //  phone enabled           specific group     global tel.
          if ((conf.tel[i] & B1) && ((_group != 255) || (conf.tel[i] >> 5 & B1))) { 
            nilSemWait(&GSMSem);    // wait for slot
            sms_ok = Serial.ATsendSMSBegin(conf.tel_num[i]);
            //WS.print(F("sms_ok")); WS.println(sms_ok);
            if (sms_ok == 1) sms_ok = Serial.ATsendSMSEnd(sms_text, sms_send);
            //WS.print(F("sms_ok")); WS.println(sms_ok);
            while (Serial.ATWaitMsg()) { // wait for GSM modem
              nilThdSleepMilliseconds(200);
            }
            nilSemSignal(&GSMSem);  // Exit region.
          }
        }
      }
      
      /*
      if (sms_send) {
        /*
        SMTPethClient.connect("mail.smtp2go.com", 2525);
        status = readStatus();
        SMTPethClient.println("EHLO");
        status = readStatus();
        SMTPethClient.println("auth login");
        status = readStatus();
        WS.println(conf.SMTP_user);
        b64_text[0] = 0;
        WS.print(base64_encode(b64_text, conf.SMTP_user, strlen(conf.SMTP_user)));
        WS.println(b64_text);
        SMTPethClient.println(b64_text);
        status = readStatus();
        WS.println(conf.SMTP_password);
        b64_text[0] = 0;
        WS.print(base64_encode(b64_text, conf.SMTP_password, strlen(conf.SMTP_password)));
        WS.println(b64_text);
        SMTPethClient.println(b64_text);
        status = readStatus();
        SMTPethClient.stop();

        */
        /*
        SMTPethClient.print("MAIL FROM:");
        SMTPethClient.println("<vysocan76@gmail.com>");
        status = readStatus();
        SMTPethClient.print("RCPT TO:");
        SMTPethClient.println("<vysocan76@gmail.com>");
        status = readStatus();
        SMTPethClient.println("DATA ");
        status = readStatus();
        SMTPethClient.println();
        SMTPethClient.println(".");
        status = readStatus();
        //mail.from("OHS <vysocan76@gmail.com>");
        for (uint8_t i = 0; i < NUM_OF_PHONES; ++i) {
          //  phone enabled           specific group     global tel.
          if ((conf.tel[i] & B1) && ((_group != 255) || (conf.tel[i] >> 5 & B1))) { 
            //mail.to(conf.email[i]);  
          }
        }
        //mail.subject(sms_text);
        //mail.body("I can send email from an Arduino!");
        //WS.print(">");WS.println(SMTPclient.send(&mail)); 
        SMTPethClient.stop();
      }
      */

      if (sms_send) {
        if (sms_ok != 1) log_message[12] = '.';
      } else {
        if (sms_ok != 1) log_message[12] = ',';
        else             log_message[12] = ';';
      }

    } // GSMisAlive
    else log_message[12] = '|'; // modem not connected

    // Put data into external EEPROM
    nilSemWait(&TWISem);          // wait for slot
    eeprom.writeBytes(conf.ee_pos,EEPROM_MESSAGE, log_message);
    nilThdSleepMilliseconds(5);   // wait for EEPROM or we lose very frequent entries
    nilSemSignal(&TWISem);        // Exit region.
    conf.ee_pos += EEPROM_MESSAGE;// Will overflow by itself as size is uint16_t = EEPROM_SIZE

    // MQTT
    nilSemWait(&ETHSem);    // wait for slot
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
    nilSemSignal(&ETHSem);  // Exit region.

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
  uint8_t _update_node;
  uint8_t _GSMlastStatus = 10; // get status on start
  char    _message[6];
  uint8_t _text[10];
  msg_t   _smpR;
  uint8_t _counter = 0; // used for pause in alarm

  nilThdSleepSeconds(5); // Sleep before start service thread
  // NTP Sync
  time_temp = GetNTPTime(udp);  
  if (time_temp.get() > 0) {
    RTC.adjust(time_temp.get());
    timestamp = time_temp.get();
  }
  
  nilThdSleepSeconds(1); // Sleep before start service thread
  // Call for registration of nodes
  _resp = sendCmd(15,1); nilThdSleepMilliseconds(200);

  #if WEB_SERIAL_DEBUGGING 
  WS.println(F("Service thread started"));
  #endif  

  while (TRUE) {
    nilThdSleepSeconds(1);
    //count to 70 seconds, for delays in relay OUT
    _counter++; if (_counter == 70) {_counter = 0;}

    // Auto arm
    for (int8_t i=0; i < ALR_ZONES ; i++){
      if ((conf.zone[i] & B1) && ((conf.zone[i] >> 7) & B1)){ // Zone enabled and auto arming
        if ( timestamp.get() >= (zone[i].last_PIR + (conf.auto_arm * 60))) {
        //if (nilTimeNowIsWithin(zone[i].last_PIR + S2ST((conf.auto_arm * 60)-1), zone[i].last_PIR + S2ST((conf.auto_arm * 60)+1))) {
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
        if ( timestamp.get() >= (zone[i].last_OK + (conf.open_alarm * 60))) {
        //if (nilTimeNowIsWithin(zone[i].last_OK + S2ST((conf.open_alarm * 60)-1), zone[i].last_OK + S2ST((conf.open_alarm * 60)+1))) {
          _tmp[0] = 'A'; _tmp[1] = 'o'; _tmp[2] = 48 + i; _tmp[3] = 0; pushToLog(_tmp); // Authorization still open alarm
          zone[i].last_OK = timestamp.get();    // update current timestamp
        }
      }
    }

    // Timers
    time_temp = timestamp.get();
    for (int8_t i=0; i < TIMERS ; i++){
      if (timer[i].setting & B1){ // Timer enabled 
        //   start time has passed                   NOT triggerd yet
        if ((time_temp.get() >= timer[i].next_on) && !((timer[i].setting >> 9) & B1)) {
          timer[i].setting |= (1 << 9); // Set triggered On
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
            node[_update_node].last_OK = timestamp.get(); // update receiving node current timestamp
            node[_update_node].value   = timer[i].constant_on; // update receiving node value
            _message[0] = 'I'; // 'I'nput only
            _message[1] = timer[i].to_number;
            u.fval = timer[i].constant_on;
            _message[2] = u.b[0]; _message[3] = u.b[1]; _message[4] = u.b[2]; _message[5] = u.b[3];
            sendData(timer[i].to_address, _message, 6);
          }
        }
        //   end time has passed                       is triggered
        if ((time_temp.get() >= timer[i].next_off) && ((timer[i].setting >> 9) & B1)) {
          timer[i].setting &= ~(1 << 9); // Set triggered Off
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
            node[_update_node].last_OK = timestamp.get(); // update receiving node current timestamp
            node[_update_node].value   = timer[i].constant_off; // update receiving node value
            _message[0] = 'I'; // 'I'nput only
            _message[1] = timer[i].to_number;
            u.fval = timer[i].constant_off;
            _message[2] = u.b[0]; _message[3] = u.b[1]; _message[4] = u.b[2]; _message[5] = u.b[3];
            sendData(timer[i].to_address, _message, 6);
          }
          set_timer(i, 0); // get next time
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
      conf.power_loss = 1; // flag that we are out of power
      saveConf(0); // Save conf, no log
      eeprom_update_block((const void*)&group, (void*)sizeof(conf)+sizeof(trigger)+sizeof(timer), sizeof(group)); // Save current configuration      

      nilSysLock(); // Lock RTOS  

      // Battery is at low level but it might oscillate, so we wait for AC to recharge battery again.
      while (pinAC_OFF.read() == HIGH) { // The signal turns to be "High" when the power supply turns OFF
        delay(1000); // do nothing wait for power supply shutdown
      } 

      conf.power_loss = 0; // un-flag that we are out of power
      eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration

      nilSysUnlock(); // in case the power is restored we go on
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

    if (_counter == 0) { // GSM modem check 
      _smpR = nilSemWaitTimeout(&GSMSem, TIME_IMMEDIATE); // look if slot is free
      if (_smpR != NIL_MSG_TMO) { // slot is free 
        //WS.println("gsm slot");
        GSMisAlive = Serial.ATsendCmd(AT_is_alive); 
        if (GSMisAlive) {
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
        WS.print(F(">Eth C:"));
        if (!client.connect(str_MQTT_dev)) {       
          // Ethernet
          Ethernet.begin(mac, our_ip, gateway, gateway, subnet);
          WS.print(F("R"));
        } else {
          if (!MQTTState){
            _tmp[0] = 'S'; _tmp[1] = 'M'; _tmp[2] = 'O'; _tmp[3] = 0; pushToLog(_tmp);
            MQTTState = 1;
            WS.print(F("New"));
          }
        }
      }
      nilSemSignal(&ETHSem);  // Exit region.
    }
    
    // Read GSM incomming messages
    if (Serial.isMsg()) {
      _resp = Serial.read((uint8_t*)sms_text);                     // read serial
      #if WEB_SERIAL_DEBUGGING
      WS.print(F(">GSM ")); WS.print(_resp); WS.print(':');
      for(uint8_t i = 0; i < _resp; i++) {
        WS.print((char)sms_text[i]);
      }
      WS.println();
      #endif
    }
    //watchdog |= (1 << 2); // Flag watchdog for this thread
  }
}

//------------------------------------------------------------------------------
// RFM69 thread 
//
NIL_WORKING_AREA(waRadioThread, 96);  
NIL_THREAD(RadioThread, arg) {
  uint8_t  _pos;

  #if WEB_SERIAL_DEBUGGING 
  WS.println(F("Radio thread started"));    
  #endif

  while (TRUE) {
    nilThdSleepMilliseconds(20);
    nilSemWait(&RFMSem); 
    if (radio.receiveDone()) {
      ++radio_received;

      WS.print(F(">Radio A:")); WS.print(radio.SENDERID);
      WS.print(F(",Data "));WS.print(radio.DATALEN); WS.print(F(","));
      for (uint8_t _t=0; _t < radio.DATALEN; _t++) {
        WS.print(radio.DATA[_t], HEX); WS.print(F(":"));
      }
      WS.print(F(" RSSI:"));
      WS.println(radio.RSSI);

      // Registration
      if ((char)radio.DATA[0] == 'R') {
        _pos = 1;
        do {
          register_t *p = reg_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
          if (p == 0) {
            _tmp[0] = 'F'; _tmp[1] = 'R'; _tmp[2] = 0; pushToLog(_tmp); // Registration queue is full
            continue; // Continue if no free space.
          }
          switch((char)radio.DATA[_pos]){
            case 'N': 
              // node name
              p->node    = (char)radio.DATA[_pos];    
              p->address = radio.SENDERID+RADIO_UNIT_OFFSET;
              for (uint8_t _t=0; _t < 16; _t++) {p->name[_t] = (char)radio.DATA[_pos+1+_t];}
              reg_fifo.signalData();   // Signal idle thread data is available.
              _pos+=17;
              break;
            default : 
              // node setting
              p->node    = (char)radio.DATA[_pos];    
              p->address = radio.SENDERID+RADIO_UNIT_OFFSET;
              p->type    = radio.DATA[_pos+1];
              p->number  = (uint8_t)radio.DATA[_pos+2];
              p->setting = ((uint8_t)radio.DATA[_pos+3] << 8) | (uint8_t)radio.DATA[_pos+4];
              reg_fifo.signalData();   // Signal idle thread data is available.
              _pos+=5;
              break;
          }
        } while (_pos < radio.DATALEN);
      }

      // Sensor
      if ((char)radio.DATA[0] == 'S') {
        _pos = 1;
        do {
          node_t *p = node_fifo.waitFree(TIME_IMMEDIATE); // Get a free FIFO slot.
          if (p == 0) {
            _tmp[0] = 'F'; _tmp[1] = 'S'; _tmp[2] = 0; pushToLog(_tmp); // node queue is full
            continue; // Continue if no free space.
          }    
          p->address  = radio.SENDERID+RADIO_UNIT_OFFSET;
          p->type     = radio.DATA[_pos];
          p->number   = (uint8_t)radio.DATA[_pos+1];
          u.b[0] = radio.DATA[_pos+2]; u.b[1] = radio.DATA[_pos+3]; u.b[2] = radio.DATA[_pos+4]; u.b[3] = radio.DATA[_pos+5];
          p->value    = u.fval;
          node_fifo.signalData();   // Signal idle thread data is available.
          _pos+=6;
        } while (_pos < radio.DATALEN);
      } // if 'S'

      if (radio.ACKRequested()) radio.sendACK();
    }
    nilSemSignal(&RFMSem);  // Exit region.
    //watchdog |= (1 << 3); // Flag watchdog for this thread
  }
}

//------------------------------------------------------------------------------
// Sensor thread 
//
NIL_WORKING_AREA(waSensorThread, 96);  
NIL_THREAD(SensorThread, arg) {
  uint8_t _node, _found, _lastNode = 0;
  uint32_t _lastNodeTime = 0;
  char _text[40];
  char _value[10];

  #if WEB_SERIAL_DEBUGGING 
  WS.println(F("Sensor thread started"));    
  #endif

  while (TRUE) {
    // Check for data.  Use TIME_IMMEDIATE to prevent sleep in idle thread.
    node_t* p = node_fifo.waitData(TIME_INFINITE);

    _found = 0;
    for (_node = 0; _node < nodes; _node++) {      
      if ((node[_node].address == p->address) && (node[_node].type == p->type) && (node[_node].number == p->number)) { 
        _found = 1;        
        //  node enabled          
        if (node[_node].setting & B1) { 
          node[_node].value   = p->value;
          node[_node].last_OK = timestamp.get();
          
          //   MQTT connected           MQTT node enabled
          if ((client.connected()) && ((node[_node].setting >> 7) & B1)) {
            _text[0] = 0;
            // Create MQTT string
            strcat_P(_text, (char*)text_OHS); strcat_P(_text, (char*)text_slash);
            strcat(_text, conf.group_name[(node[_node].setting >> 1) & B1111]); strcat_P(_text, (char*)text_slash);
            strcat_P(_text, (char*)text_node); strcat_P(_text, (char*)text_slash);
            switch(node[_node].type){
              case 'T': strcat_P(_text, (char*)text_Temperature); break;
              case 'H': strcat_P(_text, (char*)text_Humidity); break;
              case 'P': strcat_P(_text, (char*)text_Pressure); break;
              case 'I': strcat_P(_text, (char*)text_Input); break;
              case 'V': strcat_P(_text, (char*)text_Voltage); break;
              default : strcat_P(_text, (char*)text_Undefined); break;
            }
            strcat_P(_text, (char*)text_slash);
            itoa(node[_node].number, _value, 10); strcat(_text, _value);
            dtostrf(node[_node].value, 6, 2, _value); // value to string
            
            nilSemWait(&ETHSem);    // wait for slot
            client.publish(_text,_value);
            nilSemSignal(&ETHSem);  // Exit region.
          }
          if (!client.connected()) {
            if (MQTTState) {
              _tmp[0] = 'S'; _tmp[1] = 'M'; _tmp[2] = 'F'; _tmp[3] = 0; pushToLog(_tmp);
              MQTTState = 0;
            }
          }    
          // Triggers
          process_triggers(node[_node].address, node[_node].type, node[_node].number, node[_node].value);
        } // node enbled
        break; // no need to look for other node
      } // if address 
    } // for
    if (!_found) {
      // Let's call same unknown node for reregistrtion only once a while or we send many packets if multiple sensor data come in
      if ((timestamp.get() > _lastNodeTime) || (_lastNode != p->address)) {
        _node = sendCmd(p->address,1); // call this address to register
        _lastNode = p->address;
        _lastNodeTime = timestamp.get() + 1; // add 1-2 second(s)
      }
    }
    
    // Signal FIFO slot is free.
    node_fifo.signalFree();
  }
}

//------------------------------------------------------------------------------
// Registration thread 
//
NIL_WORKING_AREA(waRegThread, 96);  
NIL_THREAD(RegThread, arg) {
  uint8_t _node, _resp;

  #if WEB_SERIAL_DEBUGGING 
  WS.println(F("Registration thread started"));    
  #endif

  while (TRUE) {
    // Check for data.
    register_t* p = reg_fifo.waitData(TIME_INFINITE);

    switch(p->node){
      case 'N': // Name
        _resp = 1;
        for (_node=0; _node < node_names; _node++) {
          if (node_name[_node].address == p->address) {
            _resp = 0; break;
          }
        }
        if (_resp) { // if node not present already
          node_name[node_names].address = p->address;
          strncpy(node_name[node_names].name, p->name, 16);
          //WS.print(F("N Name:")); WS.println(node_name[node_names].name);
          node_names++;
        } else {
          strncpy(node_name[_node].name, p->name, 16);
          //WS.print(F("R Name:")); WS.println(node_name[_node].name);
        }
        break;
      case 'K': // Other
      case 'S':
      case 'I':
        _resp = 1;
        for (_node=0; _node < nodes; _node++) {
          if (node[_node].address == p->address && node[_node].type == p->type && node[_node].number == p->number) {
            _resp = 0; break;
          }
        }
        if (_resp) { // if node not present already
          node[nodes].address  = p->address;
          node[nodes].function = p->node;
          node[nodes].type     = p->type;
          node[nodes].number   = p->number;
          node[nodes].setting  = p->setting;
          node[nodes].last_OK  = timestamp.get(); 
          _tmp[0] = 'U'; _tmp[1] = 'R'; _tmp[2] = 48 + node[nodes].address; _tmp[3] = 0; pushToLog(_tmp);
          // WS.print(F("Key reg: ")); WS.println(nodes); WS.print(node[nodes].address); WS.print(node[nodes].type);
          // WS.println(node[nodes].number); WS.println(node[nodes].setting,BIN); 
          nodes++;
        } else {
          node[_node].address  = p->address;
          node[_node].function = p->node;
          node[_node].type     = p->type;
          node[_node].number   = p->number;
          node[_node].setting  = p->setting;
          node[_node].last_OK  = timestamp.get();
          _tmp[0] = 'U'; _tmp[1] = 'r'; _tmp[2] = 48 + node[_node].address; _tmp[3] = 0; pushToLog(_tmp);
          // WS.print(F("Key rereg: ")); WS.println(_node+1); WS.print(node[_node].address); WS.print(node[_node].type);
          // WS.println(node[_node].number); WS.println(node[_node].setting,BIN); 
        }
        break;
        default: _tmp[0] = 'U'; _tmp[1] = 'E'; _tmp[2] = 0; pushToLog(_tmp); 
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
  
  #if WEB_SERIAL_DEBUGGING 
  WS.println(F("Web thread started"));    
  #endif

  while (TRUE) {
    nilThdSleepMilliseconds(30);
    nilSemWait(&ETHSem);    // wait for slot
    webserver.processConnection();
    nilSemSignal(&ETHSem);  // Exit region.
    
    nilThdSleepMilliseconds(10);
    nilSemWait(&ETHSem);    // wait for slot
    client.loop(); // MQTT
    nilSemSignal(&ETHSem);  // Exit region.
    //if (!client.loop()) WS.println(F("MQTT loop NC")); // MQTT
    //watchdog |= (1 << 4); // Flag watchdog for this thread
  }
}

//------------------------------------------------------------------------------
// 
NIL_WORKING_AREA(waThread9, 64);  
NIL_THREAD(Thread9, arg) {

  #if WEB_SERIAL_DEBUGGING 
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
 NIL_THREADS_TABLE_ENTRY(NULL, ServiceThread, NULL, waServiceThread, sizeof(waServiceThread))
 NIL_THREADS_TABLE_ENTRY(NULL, RadioThread, NULL, waRadioThread, sizeof(waRadioThread))
 NIL_THREADS_TABLE_ENTRY(NULL, SensorThread, NULL, waSensorThread, sizeof(waSensorThread))
 NIL_THREADS_TABLE_ENTRY(NULL, RegThread, NULL, waRegThread, sizeof(waRegThread))
 NIL_THREADS_TABLE_ENTRY(NULL, WebThread, NULL, waWebThread, sizeof(waWebThread))
 NIL_THREADS_TABLE_ENTRY(NULL, Thread9, NULL, waThread9, sizeof(waThread9))
 NIL_THREADS_TABLE_END()

//------------------------------------------------------------------------------
 void setup() {
  // Turn ON GSM
  delay(200);

  while (pinGSM_ON.read() == HIGH) {
    pinGSM_PWR.write(LOW); delay(1000); pinGSM_PWR.write(HIGH); delay(2000);
  }

  /* Replaced by definition defaults ...
  // On start set groups
  for (int8_t i=0; i < ALR_GROUPS ; i++){
    group[i].setting   = 0;
    group[i].arm_delay = 0;
  }
  // On start set zones
  for (int8_t i=0; i < ALR_ZONES ; i++){
    zone[i].last_PIR   = 0;
    zone[i].last_OK    = 0;
    zone[i].last_event = 'N';
  }
  */

  // Read current configuration
  eeprom_read_block((void*)&conf, (void*)0, sizeof(conf)); 
  if (conf.version != VERSION) { 
    setDefault();
    // encode default user pass 
    char _text[20];
    _text[0] = 0;
    strcat (_text, conf.user); strcat (_text, ":"); strcat (_text, conf.password);
    base64_encode(conf.user_pass, _text, strlen(_text));
  } else {
    // If not fresh install read other configuration
    eeprom_read_block((void*)&trigger, (void*)sizeof(conf), sizeof(trigger));
    eeprom_read_block((void*)&timer, (void*)sizeof(conf) + sizeof(trigger), sizeof(timer));
    // Reset trigger  dynamic values to 0
    for (uint8_t _trigger = 0; _trigger < TRIGGERS; _trigger++) {
      trigger[_trigger].setting &= ~(1 << 3); // Passed
      trigger[_trigger].setting &= ~(1 << 5); // Is triggered
    }
  }
  // Read group setting if power was lost
  if (conf.power_loss) {
    eeprom_read_block((void*)&group, (void*)sizeof(conf)+sizeof(trigger)+sizeof(timer), sizeof(group));
    conf.power_loss = 0;
  }
  
  //Serial.begin(9600); // GSM modem
  Serial.begin(115200); // GSM modem
  #if WEB_SERIAL_DEBUGGING 
    WS.println(F("Start"));
  #endif

  // RFM69 radio
  radio.initialize(RADIO_FREQUENCY, RADIO_NODEID, RADIO_NETWORKID);
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

  // Ethernet
  Ethernet.begin(mac, our_ip, gateway, gateway, subnet);

  // MQTT connect
  if (client.connect(str_MQTT_dev)) {
    client.subscribe("OHS2/In/#");
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
  #if WEB_SERIAL_DEBUGGING 
  webserver.addCommand("d", &webDebug);
  #endif
  webserver.addCommand("i", &webSetTimers);
  webserver.addCommand("s", &webSetSens);
  webserver.addCommand("k", &webSetKey);
  webserver.addCommand("m", &webSetMQTT);
  webserver.addCommand("t", &webSetTriggers);


  // Start kernel
  nilSysBegin();
  // Get time
  nilSemWait(&TWISem);     // wait for slot
  timestamp = RTC.now();
  nilSemSignal(&TWISem);   // Exit region.
  // Set started timestamp
  time_started = timestamp;
  // On start set zones
  for (int8_t i=0; i < ALR_ZONES ; i++){
    zone[i].last_PIR   = time_started.get();
    zone[i].last_OK    = time_started.get();
  }
  // Set timers on start
  for (int8_t i=0; i < TIMERS ; i++){
    //  timer enabled
    if (timer[i].setting & B1) set_timer(i);
  }
  
  // Issue start message
  _tmp[0] = 'S'; _tmp[1] = 's'; _tmp[2] = 0; pushToLog(_tmp);
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