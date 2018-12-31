#ifndef defaults_h
#define defaults_h

#define ALR_ZONES      50        // Active zones
#define HW_ZONES       13        // Last hardware zone index in array
#define NUM_OF_KEYS    20        // max number of keys
#define KEY_LEN        8         // key 
#define NUM_OF_PHONES  10        // max number of phones
#define PHONE_LEN      16        // phone name 
#define NAME_LEN       16        // Zone, Group, Trigger ... name 
#define USER_LEN       11        // User, passwrod
#define EMAIL_LEN      30        // email address
#define ALR_GROUPS     16        // Groups

// ADC Alarm settings refer to voltage divider and input voltage level
// values set for resistors 2k2 Tamper, 2k2 PIR
// OK       0,9V - 1,6V     270 - 490  = 1k1
// PIR      1,8V - 2,9V     560 - 900  = 2k2
// TAMPER                              = everything else
#define ALR_PIR_LOW   560
#define ALR_PIR       715
#define ALR_PIR_HI    900
#define ALR_OK_LOW    270
#define ALR_OK        380
#define ALR_OK_HI     490

#define VERSION       176         // EEPROM config version

#define REG_LEN       22          // Size of one conf. element comming from nodes + 1 

typedef enum {
  alert_SMS   = 0,
  alert_email = 1,
  alert_page  = 2
} alert_t;

// Global configuration in chip EEPROM
struct config_t {
  uint16_t version;
  uint8_t  alr_time;
  uint16_t ee_pos;
  uint16_t zone[ALR_ZONES];
  uint8_t  arm_delay;
  char     tel_num[NUM_OF_PHONES][PHONE_LEN];  
  char     zone_name[ALR_ZONES][NAME_LEN];    
  char     key[NUM_OF_KEYS][KEY_LEN+1];
  char     key_name[NUM_OF_KEYS][NAME_LEN];
  uint8_t  key_setting[NUM_OF_KEYS];
  char     tel_name[NUM_OF_PHONES][PHONE_LEN];
  uint16_t group[ALR_GROUPS];
  char     group_name[ALR_GROUPS][NAME_LEN];
  uint8_t  tel[NUM_OF_PHONES];
  uint8_t  auto_arm;    // minutes
  uint8_t  open_alarm;  // minutes
  uint8_t  radioKey[17];
  IPAddress mqtt_ip;
  uint16_t  mqtt_port;
  IPAddress ip;
  IPAddress gw;
  IPAddress mask;
  IPAddress ntp_ip;
  uint8_t  mqtt; //MQTT setting
  char     user[USER_LEN];
  char     password[USER_LEN];
  char     user_pass[EMAIL_LEN]; // b64 encoded for easy access
  char     email[NUM_OF_PHONES][EMAIL_LEN];
  uint8_t  setting;
  uint16_t alerts[3]; // for alert configuration, SMS, email, page
  char     SMTP_user[EMAIL_LEN]; // user name is full email address
  char     SMTP_password[USER_LEN];
  uint8_t  time_std_week;      //First, Second, Third, Fourth, or Last week of the month
  uint8_t  time_std_dow;       //day of week, 1=Sun, 2=Mon, ... 7=Sat
  uint8_t  time_std_month;     //1=Jan, 2=Feb, ... 12=Dec
  uint8_t  time_std_hour;      //0-23
  int16_t  time_std_offset;    //offset from UTC in minutes
  uint8_t  time_dst_week;      //First, Second, Third, Fourth, or Last week of the month
  uint8_t  time_dst_dow;       //day of week, 1=Sun, 2=Mon, ... 7=Sat
  uint8_t  time_dst_month;     //1=Jan, 2=Feb, ... 12=Dec
  uint8_t  time_dst_hour;      //0-23
  int16_t  time_dst_offset;    //offset from UTC in minutes
  uint8_t  zone_address[ALR_ZONES-HW_ZONES];  // remote zone address
} conf;

void setDefault(){
  conf.version = VERSION;
  conf.alr_time = 10;
  conf.arm_delay = 80;
  for(uint8_t i = 0; i < NUM_OF_PHONES; i++) {
    conf.tel_num[i][0] = '-'; conf.tel_num[i][1] = 0;
    conf.tel_name[i][0] = '-'; conf.tel_name[i][1] = 0;
    conf.email[i][0] = '-'; conf.email[i][1] = 0;
    conf.tel[i] = 0x1E;
  }
  for(uint8_t i = 0; i < NUM_OF_KEYS; i++) {
    for(uint8_t ii = 0; ii < KEY_LEN; ii++) {
      conf.key[i][ii] = 0xFF;
    }
    // to delete - conf.key[i][KEY_LEN+1] = 0;
    //                        group 16 and disabled
    conf.key_setting[i] = B00011110;
    conf.key_name[i][0] = '-';conf.key_name[i][1] = 0;
  }
  for(uint8_t i = 0; i < ALR_ZONES; i++) {
    conf.zone_name[i][0] = '-';conf.zone_name[i][1] = 0;
      // Zones setup
      //                 |- Digital 0/ Analog 1
      //                 ||- Present - connected
      //                 |||- TWI zone
      //                 ||||- Remote zone
      //                 |||||- Battery node, they dont send OK, only PIR or Tamper.
      //                 ||||||- Free
      //                 |||||||- PIR as Tamper
      //                 ||||||||- Still open alarm         
      //                 ||||||||         |- Arm Home zone
      //                 ||||||||         |||- Auth time
      //                 ||||||||         |||- 0-3x the default time
      //                 ||||||||         |||||||- Group number
      //                 ||||||||         |||||||- 0 .. 15
      //                 ||||||||         |||||||-  
      //                 ||||||||         |||||||- 
      //                 ||||||||         ||||||||-  Enabled   
      //                 54321098         76543210
    switch(i){
      case  0 ...  7: 
         conf.zone[i] = B11000000 << 8 | B00011110; // Analog sensor
        break;
      case  8 ... 11:
         conf.zone[i] = B01000000 << 8 | B00011110; // Digital sensor 
        break;
      case  12      :
         conf.zone[i] = B01000010 << 8 | B00011110; // Tamper
        break;
      default: 
         conf.zone[i] = B00000000 << 8 | B00011110; // Other zones
         conf.zone_address[i-HW_ZONES] = 0;
        break;
    }
  }
  for(uint8_t i = 0; i < ALR_GROUPS; i++) {
    conf.group_name[i][0] = '-';conf.group_name[i][1] = 0;
  //                  ||||- disarm chain
  //                  ||||
  //                  ||||
  //                  ||||
  //                  ||||||||- arm chain
  //                  ||||||||
  //                  ||||||||
  //                  ||||||||
  //                  ||||||||         |- MQTT publish
  //                  ||||||||         ||- Free
  //                  ||||||||         |||- Auto arm
  //                  ||||||||         ||||- PIR signal output 1 
  //                  ||||||||         |||||- PIR signal output 2
  //                  ||||||||         ||||||-  Tamper signal output 1 
  //                  ||||||||         |||||||-  Tamper signal output 2
  //                  ||||||||         ||||||||-  Enabled 
  //                  54321098         76543210
   conf.group[i] = i << 12 | i << 8 | B00000000; 
  }

  conf.ee_pos = 0;
  conf.radioKey[0] = '-'; conf.radioKey[1] = 0;
  conf.mqtt_ip[0] = 10; conf.mqtt_ip[1] = 10; conf.mqtt_ip[2] = 10; conf.mqtt_ip[3] = 127;
  conf.mqtt_port = 1883;
  conf.mqtt = 0;
  conf.ip[0]      = 0;       conf.ip[1] = 0;       conf.ip[2] = 0;       conf.ip[3] = 0;
  conf.gw[0]      = 0;       conf.gw[1] = 0;       conf.gw[2] = 0;       conf.gw[3] = 0;
  conf.mask[0]    = 255;   conf.mask[1] = 255;   conf.mask[2] = 255;   conf.mask[3] = 0;
  conf.ntp_ip[0]  = 195; conf.ntp_ip[1] = 113; conf.ntp_ip[2] = 144; conf.ntp_ip[3] = 201;
  conf.user[0] = '#'; conf.user[1] = 0;
  conf.password[0] = '#'; conf.password[1] = 0;
  conf.SMTP_user[0] = '#'; conf.SMTP_user[1] = 0;
  conf.SMTP_password[0] = '#'; conf.SMTP_password[1] = 0;
  //       |- Radio 0=868/1=915 MHz
  //       ||- Free
  //       |||- Free
  //       ||||- Free
  //       |||||- Free
  //       ||||||- Free
  //       |||||||- Daylight saving flag
  //       ||||||||- Power lost
  //       76543210
  conf.setting = 0;

#define ALERT_TRIGGER             12
#define ALERT_BATERY_STATE        11
#define ALERT_AC_STATE            10    
#define ALERT_CONF_SAVED          9
#define ALERT_QUEUE               8
#define ALERT_MONITORING_STARTED  7
#define ALERT_ALARM               6    
#define ALERT_PIR                 5    
#define ALERT_TAMPER              4    
#define ALERT_OPEN                3
#define ALERT_ARMED               2
#define ALERT_DISARMED            1
#define ALERT_FALSE_KEY           0  
// Alert settings 
//                            |- 
//                            ||- 
//                            |||- 
//                            ||||- Trigger
//                 System     |||||- Battery state
//                 System     ||||||- AC state
//                 System     |||||||- Configration saved
//                 System     ||||||||- Fifo
//                 System     ||||||||         
//                 System     ||||||||         |- Monitoring started
//                 System     ||||||||         ||- ALARM - No authentication
//                 Zone       ||||||||         |||- PIR
//                 Zone       ||||||||         ||||- TAMPER
//                 Zone       ||||||||         |||||- Open Alarm
//                 Group      ||||||||         ||||||- Armed/Auto armed
//                 Group      ||||||||         |||||||- Disarmed
//                 Keys       ||||||||         ||||||||- Undefined Key
//                            54321098         76543210
  conf.alerts[alert_SMS]   = B00000000 << 8 | B00000000;
  conf.alerts[alert_email] = B00000000 << 8 | B00000000;
  conf.alerts[alert_page]  = B00000000 << 8 | B00000000;

  conf.time_std_week   = 0;      //First, Second, Third, Fourth, or Last week of the month
  conf.time_std_dow    = 0;      //day of week, 0=Sun, 1=Mon, ... 6=Sat
  conf.time_std_month  = 10;     //1=Jan, 2=Feb, ... 12=Dec
  conf.time_std_hour   = 3;      //0-23
  conf.time_std_offset = 60;     //offset from UTC in minutes
  conf.time_dst_week   = 0;      //First, Second, Third, Fourth, or Last week of the month
  conf.time_dst_dow    = 0;      //day of week, 0=Sun, 1=Mon, ... 6=Sat
  conf.time_dst_month  = 3;      //1=Jan, 2=Feb, ... 12=Dec
  conf.time_dst_hour   = 2;      //0-23
  conf.time_dst_offset = 120;    //offset from UTC in minutes
 
}  

#endif
