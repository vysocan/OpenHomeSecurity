#ifndef defaults_h
#define defaults_h

#define ALR_ZONES      13        // Active zones
#define NUM_OF_KEYS    8         // max number of keys
#define KEY_LEN        8         // key 
#define NUM_OF_PHONES  8         // max number of phones
#define PHONE_LEN      16        // phone name 
#define EMAIL_LEN      30        // email address
#define ALR_GROUPS     16        // Groups

// ADC Alarm settings refer to voltage divider and input voltage level
// values set for resistors 1.1k Tamper, 1.1k PIR
// PIR      0,9V - 1,5V     270 - 470  = 1k1
// OK       1,8V - 2,6V     560 - 820  = 2k2
// TAMPTER  ->0,9, 2,6->               = everything else
#define ALR_PIR_LOW     270
#define ALR_PIR         370
#define ALR_PIR_HI      470
#define ALR_OK_LOW    560
#define ALR_OK        690
#define ALR_OK_HI     820
// EEPROM config version
#define VERSION 100

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
  
  conf.tel_num[0][0] = '-';conf.tel_num[0][1] = 0;
  conf.tel_num[1][0] = '-';conf.tel_num[1][1] = 0;
  conf.tel_num[2][0] = '-';conf.tel_num[2][1] = 0;
  conf.tel_num[3][0] = '-';conf.tel_num[3][1] = 0;
  conf.tel_num[4][0] = '-';conf.tel_num[4][1] = 0;
  conf.tel_num[5][0] = '-';conf.tel_num[5][1] = 0;
  conf.tel_num[6][0] = '-';conf.tel_num[6][1] = 0;
  conf.tel_num[7][0] = '-';conf.tel_num[7][1] = 0;
  conf.tel_name[0][0] = '-';conf.tel_name[0][1] = 0;
  conf.tel_name[1][0] = '-';conf.tel_name[1][1] = 0;
  conf.tel_name[2][0] = '-';conf.tel_name[2][1] = 0;
  conf.tel_name[3][0] = '-';conf.tel_name[3][1] = 0;
  conf.tel_name[4][0] = '-';conf.tel_name[4][1] = 0;
  conf.tel_name[5][0] = '-';conf.tel_name[5][1] = 0;
  conf.tel_name[6][0] = '-';conf.tel_name[6][1] = 0;
  conf.tel_name[7][0] = '-';conf.tel_name[7][1] = 0;
  conf.email[0][0] = '-';conf.email[0][1] = 0;
  conf.email[1][0] = '-';conf.email[1][1] = 0;
  conf.email[2][0] = '-';conf.email[2][1] = 0;
  conf.email[3][0] = '-';conf.email[3][1] = 0;
  conf.email[4][0] = '-';conf.email[4][1] = 0;
  conf.email[5][0] = '-';conf.email[5][1] = 0;
  conf.email[6][0] = '-';conf.email[6][1] = 0;
  conf.email[7][0] = '-';conf.email[7][1] = 0;
  conf.tel[0] = 0x1E;
  conf.tel[1] = 0x1E;
  conf.tel[2] = 0x1E;
  conf.tel[3] = 0x1E;
  conf.tel[4] = 0x1E;
  conf.tel[5] = 0x1E;
  conf.tel[6] = 0x1E;
  conf.tel[7] = 0x1E;
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
  conf.zone_name[12][0] = '-';conf.zone_name[12][1] = 0;
  conf.key[0][0] = 0xFF;conf.key[0][1] = 0;
  conf.key[1][0] = 0xFF;conf.key[1][1] = 0;
  conf.key[2][0] = 0xFF;conf.key[2][1] = 0;
  conf.key[3][0] = 0xFF;conf.key[3][1] = 0;
  conf.key[4][0] = 0xFF;conf.key[4][1] = 0;
  conf.key[5][0] = 0xFF;conf.key[5][1] = 0;
  conf.key[6][0] = 0xFF;conf.key[6][1] = 0;
  conf.key[7][0] = 0xFF;conf.key[7][1] = 0;
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
  conf.zone[ 0] = B10000000 << 8 | B00011110; // Analog sensor 1
  conf.zone[ 1] = B10000000 << 8 | B00011110; // Analog sensor 2
  conf.zone[ 2] = B10000000 << 8 | B00011110; // Analog sensor 3
  conf.zone[ 3] = B10000000 << 8 | B00011110; // Analog sensor 4
  conf.zone[ 4] = B10000000 << 8 | B00011110; // Analog sensor 5
  conf.zone[ 5] = B10000000 << 8 | B00011110; // Analog sensor 6
  conf.zone[ 6] = B10000000 << 8 | B00011110; // Analog sensor 7
  conf.zone[ 7] = B10000000 << 8 | B00011110; // Analog sensor 8
  conf.zone[ 8] = B00000000 << 8 | B00011110; // Digital sensor 1
  conf.zone[ 9] = B00000000 << 8 | B00011110; // Digital sensor 2
  conf.zone[10] = B00000000 << 8 | B00011110; // Digital sensor 3
  conf.zone[11] = B00000000 << 8 | B00011110; // Digital sensor 4
  conf.zone[12] = B00000000 << 8 | B00011110; // Tamper

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
  conf.group_name[ 0][0] = '-';conf.group_name[0][1] = 0;
  conf.group_name[ 1][0] = '-';conf.group_name[1][1] = 0;
  conf.group_name[ 2][0] = '-';conf.group_name[2][1] = 0;
  conf.group_name[ 3][0] = '-';conf.group_name[3][1] = 0;
  conf.group_name[ 4][0] = '-';conf.group_name[4][1] = 0;
  conf.group_name[ 5][0] = '-';conf.group_name[5][1] = 0;
  conf.group_name[ 6][0] = '-';conf.group_name[6][1] = 0;
  conf.group_name[ 7][0] = '-';conf.group_name[7][1] = 0;
  conf.group_name[ 8][0] = '-';conf.group_name[8][1] = 0;
  conf.group_name[ 9][0] = '-';conf.group_name[9][1] = 0;
  conf.group_name[10][0] = '-';conf.group_name[10][1] = 0;
  conf.group_name[11][0] = '-';conf.group_name[11][1] = 0;
  conf.group_name[12][0] = '-';conf.group_name[12][1] = 0;
  conf.group_name[13][0] = '-';conf.group_name[13][1] = 0;
  conf.group_name[14][0] = '-';conf.group_name[14][1] = 0;
  conf.group_name[15][0] = '-';conf.group_name[15][1] = 0;

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
  conf.mqtt_serv_ip[0] = 10; conf.mqtt_serv_ip[1] = 10; conf.mqtt_serv_ip[2] = 10; conf.mqtt_serv_ip[3] = 1;
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