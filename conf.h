#ifndef defaults_h
#define defaults_h

#define ALR_ZONES      50        // Active zones
#define NUM_OF_KEYS    20        // max number of keys
#define KEY_LEN        8         // key 
#define NUM_OF_PHONES  10        // max number of phones
#define PHONE_LEN      16        // phone name 
#define EMAIL_LEN      30        // email address
#define ALR_GROUPS     16        // Groups

// ADC Alarm settings refer to voltage divider and input voltage level
// values set for resistors 2k2 Tamper, 2k2 PIR
// OK       0,9V - 1,5V     270 - 470  = 1k1
// PIR      1,8V - 2,6V     560 - 820  = 2k2
// TAMPER                              = everything else
#define ALR_PIR_LOW   560
#define ALR_PIR       690
#define ALR_PIR_HI    820

#define ALR_OK_LOW    270
#define ALR_OK        370
#define ALR_OK_HI     470
// EEPROM config version
#define VERSION 173

typedef enum {
  alert_SMS = 0,
  alert_email = 1,
  alert_page = 2
} alert_t;

// Global configuration in chip EEPROM
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
  uint8_t  key_setting[NUM_OF_KEYS];
  char     tel_name[NUM_OF_PHONES][PHONE_LEN];
  uint16_t SMS; // **** NOT USED
  uint16_t group[ALR_GROUPS];
  char     group_name[ALR_GROUPS][16];
  uint8_t  tel[NUM_OF_PHONES];
  uint8_t  auto_arm;    // minutes
  uint8_t  open_alarm;  // minutes
  uint8_t  radioKey[17];
  byte     mqtt_serv_ip[4];
  char     user[11];
  char     password[11];
  char     user_pass[31];
  char     email[NUM_OF_PHONES][EMAIL_LEN];
  uint8_t  power_loss;
  uint16_t alerts[3]; // for alert configuration, SMS, email, page
  char     SMTP_user[31];
  char     SMTP_password[11];
  //char     b64_SMTP_user[41];
  //char     b64_SMTP_password[17];
} conf;

void setDefault(){
  conf.version = VERSION;
  conf.alr_time = 10;
  conf.arm_delay = 80;
  for(uint8_t i = 0; i < NUM_OF_PHONES; i++) {
    conf.tel_num[i][0] = '-';conf.tel_num[i][1] = 0;
    conf.tel_name[i][0] = '-';conf.tel_name[i][1] = 0;
    conf.email[i][0] = '-';conf.email[i][1] = 0;
    conf.tel[i] = 0x1E;
  }
  for(uint8_t i = 0; i < NUM_OF_KEYS; i++) {
    for(uint8_t ii = 0; ii < KEY_LEN; ii++) {
      conf.key[i][ii] = 0xFF;
    }
    conf.key[i][KEY_LEN+1] = 0;
    //                        group 16 and disabled
    conf.key_setting[i] = B00011110;
    conf.key_name[i][0] = '-';conf.key_name[i][1] = 0;
  }
  for(uint8_t i = 0; i < ALR_ZONES; i++) {
    conf.zone_name[i][0] = '-';conf.zone_name[i][1] = 0;
      // Zones setup
      //                 |- Digital 0/ Analog 1
      //                 ||- Present - connected
      //                 |||- Free
      //                 ||||- Free
      //                 |||||- Free
      //                 ||||||- Free
      //                 |||||||- PIR as Tamper
      //                 ||||||||- Still open alarm         
      //                 ||||||||         |- Auto arm zone
      //                 ||||||||         |||- Auth time
      //                 ||||||||         |||- 0-3x the default time
      //                 ||||||||         |||||||- Group number
      //                 ||||||||         |||||||- 0 .. 15
      //                 ||||||||         |||||||-  
      //                 ||||||||         |||||||- 
      //                 ||||||||         ||||||||-  Enabled   
      //                B10000000         00000000
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
        break;
    }
  }
  for(uint8_t i = 0; i < ALR_GROUPS; i++) {
    conf.group_name[i][0] = '-';conf.group_name[i][1] = 0;
  //                  |- Free
  //                  ||- Free
  //                  |||- Free
  //                  ||||- Free
  //                  |||||- Free
  //                  ||||||- Free
  //                  |||||||- Free
  //                  ||||||||- Free
  //                  ||||||||         |- MQTT publish
  //                  ||||||||         ||- Free
  //                  ||||||||         |||- Free
  //                  ||||||||         ||||- PIR signal output 1 
  //                  ||||||||         |||||- PIR signal output 2
  //                  ||||||||         ||||||-  Tamper signal output 1 
  //                  ||||||||         |||||||-  Tamper signal output 2
  //                  ||||||||         ||||||||-  Enabled 
  //                 B10000000         00000000
    conf.group[ i] = B00000000 << 8 | B00000000; 
  }

  conf.ee_pos = 0;

// SMS settings 
//            |- 
//            ||- 
//            |||- 
// Keys       ||||- Open Alarm
// System     |||||- Battery state
// System     ||||||- AC state
// System     |||||||- Configration saved
// System     ||||||||- System group armed
// System     ||||||||         
// System     ||||||||         |- Monitoring started
// System     ||||||||         ||- ALARM - No authentication
// Alarm      ||||||||         |||- PIR
// Alarm      ||||||||         ||||- TAMPER
//            ||||||||         |||||- 
// Keys       ||||||||         ||||||- Armed/Auto armed
// Keys       ||||||||         |||||||- Disarmed
// Keys       ||||||||         ||||||||- Undefined Key
  conf.SMS = B00000000 << 8 | B00000000;
//encryption is OPTIONAL
//to enable encryption you will need to:
// - provide a 16-byte encryption KEY (same on all nodes that talk encrypted)
// - to call .Encrypt(KEY) to start encrypting
// - to stop encrypting call .Encrypt(NULL)
  conf.radioKey[0] = '-'; conf.radioKey[1] = 0;
  conf.mqtt_serv_ip[0] = 10; conf.mqtt_serv_ip[1] = 10; conf.mqtt_serv_ip[2] = 10; conf.mqtt_serv_ip[3] = 126;
  conf.user[0] = '#'; conf.user[1] = 0;
  conf.password[0] = '#'; conf.password[1] = 0;
  conf.SMTP_user[0] = '#'; conf.SMTP_user[1] = 0;
  conf.SMTP_password[0] = '#'; conf.SMTP_password[1] = 0;
  conf.power_loss = 0;
  conf.alerts[alert_SMS]   = 0x0000;
  conf.alerts[alert_email] = 0x0000;
  conf.alerts[alert_page]  = 0x0000;
 
}  

#endif