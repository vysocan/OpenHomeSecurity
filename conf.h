#ifndef defaults_h
#define defaults_h

#define ALR_ZONES      12        // Active zones
#define NUM_OF_KEYS    8         // max number of keys
#define KEY_LEN        8         // key 
#define NUM_OF_PHONES  8         // max number of phones
#define PHONE_LEN      16        // phone name 
#define ALR_GROUPS     16        // Groups
#define ALR_AUTH_UNITS 14        // 

#define VERSION 100
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
  uint16_t SMS;
  uint8_t  global_tel_num; 
  uint16_t group[ALR_GROUPS];
  char     group_name[ALR_GROUPS][16];
  uint8_t  tel[NUM_OF_PHONES];
  uint8_t  auth[ALR_AUTH_UNITS];
  uint8_t  auto_arm;  // minutes
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
  conf.zone[ 0] = B10000000 << 8 | B00000000; // Analog sensor 1
  conf.zone[ 1] = B10000000 << 8 | B00000000; // Analog sensor 2
  conf.zone[ 2] = B10000000 << 8 | B00000000; // Analog sensor 3
  conf.zone[ 3] = B10000000 << 8 | B00000000; // Analog sensor 4
  conf.zone[ 4] = B10000000 << 8 | B00000000; // Analog sensor 5
  conf.zone[ 5] = B10000000 << 8 | B00000000; // Analog sensor 6
  conf.zone[ 6] = B10000000 << 8 | B00000000; // Analog sensor 7
  conf.zone[ 7] = B00000000 << 8 | B00000000; // Digital sensor 1
  conf.zone[ 8] = B00000000 << 8 | B00000000; // Digital sensor 2
  conf.zone[ 9] = B00000000 << 8 | B00000000; // Digital sensor 3
  conf.zone[10] = B00000000 << 8 | B00000000; // Digital sensor 4
  conf.zone[11] = B00000000 << 8 | B00000000; // Tamper

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

#endif