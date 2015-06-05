#ifndef HSF_h
#define HSF_h

// *********************************************************************************
// W E B   P A G E S              W E B   P A G E S                W E B   P A G E S  
// *********************************************************************************

void webHome(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (server.checkCredentials(conf.user_pass)) {
    if (type == WebServer::POST) {
      bool repeat;
      char name[2], value[16];
      do {
        repeat = server.readPOSTparam(name, 2, value, 16);
        switch(name[0]){
          case 'T': // NTP Sync
            time_now = GetNTPTime(udp);  
            if (time_now.get() > 0) RTC.adjust(time_now.get());
          break;
          case 'e': saveConf(); break; // save to EEPROM
          case 'l': // load from EEPROM
            _tmp[0] = 'S'; _tmp[1] = 'C'; _tmp[2] = 'L'; _tmp[3] = 0; pushToLog(_tmp);
            //pushPToLogP(log_SCL);
            //strcpy_P(_tmp, (const char*) log_SCL); pushToLog();
            eeprom_read_block((void*)&conf, (void*)0, sizeof(conf)); // Read current configuration
          break; 
          case 'r': // reset ro default
            _tmp[0] = 'S'; _tmp[1] = 'C'; _tmp[2] = 'R'; _tmp[3] = 0; pushToLog(_tmp);
            //strcpy_P(_tmp, (const char*) log_SCR); pushToLog();
            //pushPToLogP(log_SCR);
            setDefault();
          break; 
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
      server.printP(html_table_tr_td); 
      server.printP(text_Time); server.printP(html_e_td_td_sesp);
      server.print((char*)time_now.formatedDateTime());
      server.printP(html_e_td_e_tr);

      server.printP(html_tr_td); server.printP(html_e_td_td);
      server.printP(html_form_s); server << PREFIX "/"; server.printP(html_form_e);
      server.printP(html_F_GetNTP); // Clear
      server.printP(html_e_td_e_tr);

      server.printP(html_tr_td);
      server.printP(text_Started); server.printP(html_e_td_td_sesp);
      server.print((char*)time_started.formatedDateTime());        
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td);
      server.printP(text_Uptime); server.printP(html_e_td_td_sesp);
      time_now = (time_now.get()-time_started.get());
      server.print((char*)time_now.formatedUpTime());
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td);
      server.printP(text_PwrSp); server.printP(html_e_td_td_sesp);
      pinAC_OFF.read() ? server.printP(text_Off) : server.printP(text_On); server.printP(html_e_td_e_tr);
      server.printP(html_tr_td);
      server.printP(text_Battery); server.printP(html_e_td_td_sesp);
      pinBAT_OK.read() ? server.printP(text_OK) : server.printP(text_low); server.printP(html_e_td_e_tr);
      server.printP(html_tr_td);
      server.printP(text_Voltage); server.printP(html_e_td_td_sesp); 
      if (voltage > 1000) { server << _tmp_itoa[0] << _tmp_itoa[1] << "." << _tmp_itoa[2] << _tmp_itoa[3];}
      else                { server << _tmp_itoa[0] << "." << _tmp_itoa[1] << _tmp_itoa[2] << _tmp_itoa[3];}
      server.printP(html_e_td_e_tr);
      server.printP(html_e_table);
      
      server.printP(html_h1); server.printP(text_Radio); server.printP(html_e_h1);  server.printP(html_p);
      server.printP(html_table_tr_td);
      server.printP(text_Received); server.printP(text_space); server.printP(text_packets); server.printP(html_e_td_td_sesp);
      server << radio_received; server.printP(html_e_td_e_tr);
      server.printP(html_tr_td);
      server.printP(text_Failed); server.printP(text_space); server.printP(text_packets); server.printP(html_e_td_td_sesp);
      server << radio_bad_crc; server.printP(html_e_td_e_tr);
      server.printP(html_tr_td);
      server.printP(text_Not); server.printP(text_space); server.printP(text_acknowledged); server.printP(text_space); server.printP(text_packets); server.printP(html_e_td_td_sesp);
      server << radio_no_ack; server.printP(html_e_td_e_tr);
      server.printP(html_e_table);

      server.printP(html_h1); server.printP(text_GSM); server.printP(text_space); server.printP(text_modem); server.printP(html_e_h1);  server.printP(html_p);
      server.printP(html_table_tr_td);
      server.printP(text_GSM); server.printP(text_space); server.printP(text_modem); server.printP(text_space); server.printP(text_is); server.printP(text_space); server.printP(text_connected); 
      server.printP(html_e_td_td_sesp);
      GSMisAlive ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td_e_tr);
      
      server.printP(html_tr_td);
      server.printP(text_GSM); server.printP(text_space); server.printP(text_network); server.printP(text_space); server.printP(text_is);
      server.printP(html_e_td_td_sesp);
      switch(GSMreg){
        case 0 : server.printP(text_nr); break;
        case 1 : server.printP(text_rh); break;
        case 2 : server.printP(text_nrs); break;
        case 3 : server.printP(text_rd); break;
        // case 4 : server.printP(text_unk); break;
        case 5 : server.printP(text_rr); break;
        default : server.printP(text_unk);; break;
      }
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td);
      server.printP(text_Signal); server.printP(text_space); server.printP(text_strength);
      server.printP(html_e_td_td_sesp);
      server << (GSMstrength*3); server.printP(text_percent); 
      server.printP(html_e_td_e_tr);
      server.printP(html_e_table);
      server.printP(html_e_p);
      server.printP(html_F_LA); // submit Load all
      server.printP(html_F_SA); // submit Save all
      server.printP(html_F_RD); // submit Save all
      server.printP(html_e_form);
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
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
    server.printP(html_table_tr_th);
    server.printP(text_hash);
    server.printP(html_e_th_th); server.printP(text_Date); 
    server.printP(html_e_th_th); server.printP(text_Message);
    server.printP(html_e_th_th); server.printP(text_SMS);
    server.printP(html_e_th_e_tr);
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
     server.printP(html_e_td_td);
      // print date and time
     server << tmp[4] << tmp[5]; server.printP(text_dot);
     server << tmp[2] << tmp[3]; server.printP(text_dot);
     server << "20" << tmp[0] << tmp[1]; server.printP(text_space);
     server << tmp[6] << tmp[7]; server.printP(text_semic);
     server << tmp[8] << tmp[9]; server.printP(text_semic);
     server << tmp[10] << tmp[11]; server.printP(text_space);
     server.printP(html_e_td_td);
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
            case 'C': server.printP(text_Configuration); server.printP(text_space);
              switch(tmp[15]){
                case 'W': server.printP(text_saved); break; // conf. saved
                case 'P': server.printP(text_saved); server.printP(text_cosp); server.printP(text_monitoring);
                  server.printP(text_space); server.printP(text_disabled);break; // conf. saved
                case 'L': server.printP(text_loaded); break; // conf. saved
                case 'R': server.printP(text_reset); break; // conf. saved
                default:  server.printP(text_undefined); break; // unknown  
              }
            break;
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
          server.printP(text_cosp); server.printP(text_strength); server.printP(text_space);
          server << (tmp[15]-48)*3 << "%";
        break;
        case 'U': // remote unit
          if (tmp[14] == 'E') { 
            server.printP(text_Unit); server.printP(text_space); server.printP(text_registration);
            server.printP(text_space); server.printP(text_error);
            break;
          }
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
        case 'F': // Fifo
          server.printP(text_Fifo); server.printP(text_space);          
          switch(tmp[14]){
            case 'A' : server.printP(text_alarm); break;
            case 'S' : server.printP(text_sensor); break;
            case 'R' : server.printP(text_registration); break;
            default : server.printP(text_unk); break;
          }
          server.printP(text_space); server.printP(text_is); server.printP(text_space); server.printP(text_full);
        break;
        default:
          server.printP(text_Undefined); server.printP(text_sesp);
          server << tmp;
        break;
      }
      server.printP(text_dot);
      server.printP(html_e_td_td);
      switch(tmp[12]){
        case '.': server.printP(text_requested); server.printP(text_sesp); server.printP(text_failed); break;
        case ',': server.printP(text_not); server.printP(text_space); server.printP(text_requested); server.printP(text_cosp); server.printP(text_failed); break;
        case ';': server.printP(text_not); server.printP(text_space); server.printP(text_requested); server.printP(text_cosp); server.printP(text_acknowledged); break;
        case '|': server.printP(text_not); server.printP(text_space); server.printP(text_connected); break;
        default:  server.printP(text_sent); break;
      }
      server.printP(html_e_td_e_tr);
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

      server.printP(html_table_tr_th);
      server.printP(text_hash);
      server.printP(html_e_th_th); server.printP(text_Type);
      server.printP(html_e_th_th); server.printP(text_Name); 
      server.printP(html_e_th_th); server.printP(text_Enabled);
      server.printP(html_e_th_th); server.printP(text_Auto); server.printP(text_space); server.printP(text_arm);
      server.printP(html_e_th_th); server.printP(text_Open); server.printP(text_space); server.printP(text_alarm);
      server.printP(html_e_th_th); server.printP(text_Delay);
      server.printP(html_e_th_th); server.printP(text_Group);
      server.printP(html_e_th_th); server.printP(text_Last); server.printP(text_space); server.printP(text_alarm);
      server.printP(html_e_th_th); server.printP(text_Last); server.printP(text_space); server.printP(text_OK);
      server.printP(html_e_th_th); server.printP(text_Status);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < ALR_ZONES; ++i) {
        if ( (i & 0x01) == 0) server.printP(html_tr_od);
        else                  server.printP(html_tr_ev);
        server.printP(html_td); 
        server << i+1; server.printP(text_dot); server.printP(html_e_td_td);
        (conf.zone[i] >> 15) ? server.printP(text_analog) : server.printP(text_digital); server.printP(html_e_td_td);
        server << conf.zone_name[i]; server.printP(html_e_td_td);
        (conf.zone[i] & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td_td);
        ((conf.zone[i] >> 7) & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td_td);
        ((conf.zone[i] >> 8) & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td_td);
        server << (((conf.zone[i] >> 5) & B11)*conf.alr_time); server.printP(text_space); server.printP(text_sec); server.printP(html_e_td_td);
        if ((conf.zone[i] & B1)) { server << ((conf.zone[i] >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((conf.zone[i] >> 1) & B1111)]; }
        else                     { server.printP(text_spdashsp); }        
        server.printP(html_e_td_td);
        time_now = (nilTimeNow() - zone[i].last_PIR)/NIL_CFG_FREQUENCY;
        if ((conf.zone[i] & B1)) { server.print((char*)time_now.formatedUpTime()); }
        else                     { server.printP(text_spdashsp); }
        //server.printP(text_space); server.printP(text_sec);
        time_now = (nilTimeNow() - zone[i].last_OK)/NIL_CFG_FREQUENCY;
        server.printP(html_e_td_td);
        if ((conf.zone[i] & B1)) { server.print((char*)time_now.formatedUpTime()); }
        else                     { server.printP(text_spdashsp); }
        //server.printP(text_space); server.printP(text_sec);
        server.printP(html_e_td_td);
        if ((conf.zone[i] & B1)) {
          switch(zone[i].last_event){
            case 'O': server.printP(text_OK); break;
            case 'P': server.printP(text_ALARM); break;
            case 'N': server.printP(text_Not); server.printP(text_space); server.printP(text_started); break;
            default: server.printP(text_tamper); break;
          }
        } else { server.printP(text_disabled); } // disabled
        server.printP(html_e_td_e_tr);      
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
      server.printP(html_table_tr_td);
      server.printP(text_Zone); server.printP(text_space); server << webZone+1; server.printP(text_space); server.printP(text_is); server.printP(html_e_td_td_sesp);
      ((conf.zone[webZone] >> 15) & 1) ? server.printP(text_analog) : server.printP(text_digital); server.printP(html_e_td_e_tr);
      server.printP(html_tr_td); server.printP(text_Zone); server.printP(text_space); server.printP(text_name); server.printP(html_e_td_td_sesp);
      server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.zone_name[webZone]; server.printP(html_e_tag);
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td); server.printP(text_Zone); server.printP(text_space); server.printP(text_is); server.printP(html_e_td_td_sesp);
      server.radioButton("o", text_1, text_On, conf.zone[webZone] & B1);
      server.radioButton("o", text_0, text_Off, !(conf.zone[webZone] & B1));
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td); server.printP(text_Auto); server.printP(text_space); server.printP(text_arm); server.printP(html_e_td_td_sesp);
      server.radioButton("a", text_1, text_On, conf.zone[webZone] >> 7 & B1);
      server.radioButton("a", text_0, text_Off, !(conf.zone[webZone] >> 7 & B1));
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td); server.printP(text_Open); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td_td_sesp);
      server.radioButton("s", text_1, text_On, conf.zone[webZone] >> 8 & B1);
      server.radioButton("s", text_0, text_Off, !(conf.zone[webZone] >> 8 & B1));
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td); server.printP(text_Authentication); server.printP(text_space); server.printP(text_delay); server.printP(html_e_td_td_sesp);
      server.radioButton("d", text_0, text_0, !((conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
      server.radioButton("d", text_1, text_1, (!(conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
      server.radioButton("d", text_2, text_2, ((conf.zone[webZone] >> 6 & B1) & !(conf.zone[webZone] >> 5 & B1)));
      server.radioButton("d", text_3, text_3, ((conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
      server.printP(text_x); server << conf.alr_time; server.printP(text_space); server.printP(text_sec);
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td); server.printP(text_Group); server.printP(html_e_td_td_sesp);
      server.printP(html_select); server << "g"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
        if ((conf.zone[webZone] >> 1 & B1111) == ii) 
          { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option); }
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option); }
      }
      server.printP(html_e_select);
      server.printP(html_e_td_e_tr);
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
      server.printP(html_table_tr_th);
      server.printP(text_hash);
      server.printP(html_e_th_th); server.printP(text_Name); 
      server.printP(html_e_th_th); server.printP(text_Enabled);
      server.printP(html_e_th_th); server.printP(text_Armed);
      server.printP(html_e_th_th); server.printP(text_Authentication);
      server.printP(html_e_th_th); server.printP(text_Delay);
      server.printP(html_e_th_th); server.printP(text_Zone); server.printP(text_s);
      server.printP(html_e_th_th); server.printP(text_Authentication); server.printP(text_space); server.printP(text_unit); server.printP(text_s);
      server.printP(html_e_th_th); server.printP(text_Sensor); server.printP(text_s);
      server.printP(html_e_th_th); server.printP(text_Phone); server.printP(text_s);
      server.printP(html_e_th_th); server.printP(text_Alarm);
      server.printP(html_e_th_th); server.printP(text_Tamper);
      server.printP(html_e_th_th); server.printP(text_Status);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < ALR_GROUPS; ++i) {
        if ( (i & 0x01) == 0) server.printP(html_tr_od);
        else                  server.printP(html_tr_ev);
        server.printP(html_td); 
        server << i+1; server.printP(text_dot); server.printP(html_e_td_td);
        server << conf.group_name[i]; server.printP(html_e_td_td);
        (conf.group[i] & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td_td);
        (group[i].setting & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td_td);
        ((group[i].setting >> 2) & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td_td);
        server << group[i].arm_delay/4; server.printP(text_space); server.printP(text_sec); server.printP(html_e_td_td);
        n = 0;
        for (uint8_t ii = 0; ii < ALR_ZONES; ++ii) {
          if ((((conf.zone[ii] >> 1) & B1111) == i) && (conf.zone[ii] & B1)) {
            if (n) { server.printP(text_cosp);}
            server << ii+1; n = 1;
          }
        }
        server.printP(html_e_td_td);
        n = 0;
        for (uint8_t ii = 0; ii < units; ++ii) {      
          if ((((unit[ii].setting >> 1) & B1111) == i) && (unit[ii].setting & B1)) { 
            if (n) { server.printP(text_cosp); }
            server << unit[ii].type; server.printP(text_semic);
            if (unit[ii].address < RADIO_UNIT_OFFSET) { server << "W:" << unit[ii].address; }
            else                                      { server << "R:" << unit[ii].address-RADIO_UNIT_OFFSET; }
            server.printP(text_semic); server << unit[ii].number; n = 1;
          }
        }
        server.printP(html_e_td_td);
        n = 0;
        for (uint8_t ii = 0; ii < SENSORS; ++ii) {      
          if (((sensor[ii].setting >> 1) & B1111) == i) { 
            if (sensor[ii].address && (sensor[ii].setting & B1)) {
              if (n) { server.printP(text_cosp); }
              server << sensor[ii].type; server.printP(text_semic);
              if (sensor[ii].address < RADIO_UNIT_OFFSET) { server << "W:" << sensor[ii].address; }
              else                                        { server << "R:" << sensor[ii].address-RADIO_UNIT_OFFSET; }
              server.printP(text_semic); server << sensor[ii].number;
              n = 1;
            }
          }
        }
        server.printP(html_e_td_td);
        n = 0;
        for (uint8_t ii = 0; ii < NUM_OF_PHONES; ++ii) {
          if (((((conf.tel[ii] >> 1) & B1111) == i) || (conf.tel[ii] >> 5 & B1)) && (conf.tel[ii] & B1)){
            if (n) { server.printP(text_cosp); }
            server << ii+1; server.printP(text_semic); server << conf.tel_name[ii];
            n = 1;
          }
        }
        server.printP(html_e_td_td);        
        if ((conf.group[i] >> 4) & B1) { server.printP(text_OUT1); server.printP(text_space); }
        if ((conf.group[i] >> 3) & B1) server.printP(text_OUT2);
        server.printP(html_e_td_td);
        if ((conf.group[i] >> 2) & B1) { server.printP(text_OUT1); server.printP(text_space); }
        if ((conf.group[i] >> 1) & B1) server.printP(text_OUT2);
        server.printP(html_e_td_td);

        if (conf.group[i] & B1) {
          (group[i].setting >> 1 & B1) ? server.printP(text_ALARM) : server.printP(text_OK);
        } else { server.printP(text_disabled); }
        server.printP(html_e_td_e_tr);
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
      server.printP(html_table_tr_td);
      server.printP(text_Group); server.printP(text_space); server.printP(text_name); server.printP(html_e_td_td_sesp);
      server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.group_name[webGroup]; server.printP(html_e_tag);
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td);
      server.printP(text_Group); server.printP(text_space); server.printP(text_is); server.printP(html_e_td_td_sesp);
      server.radioButton("o", text_1, text_On, conf.group[webGroup] & B1);
      server.radioButton("o", text_0, text_Off, !(conf.group[webGroup] & B1));
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td);
      server.printP(text_Alarm); server.printP(text_space); server.printP(text_trigger); server.printP(text_s); server.printP(html_e_td_td_sesp);
      server.printP(text_OUT1); server.printP(text_sesp);
      server.radioButton("p", text_1, text_On, conf.group[webGroup] >> 4 & B1);
      server.radioButton("p", text_0, text_Off, !(conf.group[webGroup] >> 4 & B1));
      server.printP(html_br);  server.printP(text_sesp); server.printP(text_OUT2); server.printP(text_sesp);
      server.radioButton("q", text_1, text_On, conf.group[webGroup] >> 3 & B1);
      server.radioButton("q", text_0, text_Off, !(conf.group[webGroup] >> 3 & B1));
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td);
      server.printP(text_Tamper); server.printP(text_space); server.printP(text_trigger); server.printP(text_s); server.printP(html_e_td_td_sesp);
      server.printP(text_OUT1); server.printP(text_sesp);
      server.radioButton("t", text_1, text_On, conf.group[webGroup] >> 2 & B1);
      server.radioButton("t", text_0, text_Off, !(conf.group[webGroup] >> 2 & B1));
      server.printP(html_br);  server.printP(text_sesp);
      server.printP(text_OUT2); server.printP(text_sesp);
      server.radioButton("y", text_1, text_On, conf.group[webGroup] >> 1 & B1);
      server.radioButton("y", text_0, text_Off, !(conf.group[webGroup] >> 1 & B1));
      server.printP(html_e_td_e_tr);

      server.printP(html_e_table);
      server.printP(html_e_p);    
    server.printP(html_F_A); // submit Apply
    server.printP(html_F_SA); // submit Save all
    server.printP(html_e_form);
    server.printP(htmlFoot);
  }
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

    server.printP(html_table_tr_th);
    server.printP(text_hash);
    server.printP(html_e_th_th); server.printP(text_Name);
    server.printP(html_e_th_th); server.printP(text_Key);
    server.printP(html_e_th_e_tr);
    for (uint8_t i = 0; i < NUM_OF_KEYS; ++i) {
      if ( (i & 0x01) == 0) server.printP(html_tr_od);
      else                  server.printP(html_tr_ev);
      server.printP(html_td); 
      server << i+1; server.printP(text_dot); server.printP(html_e_td_td);
      server << conf.key_name[i];
      server.printP(html_e_td_td);
      formatKey(conf.key[i], tmp);
      //server.printP(html_pre);
      server << tmp;
      // server.printP(html_e_pre);
      server.printP(html_e_td_e_tr);
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
    server.printP(html_table_tr_td);
    server.printP(text_Key); server.printP(text_space); server.printP(text_name); server.printP(html_e_td_td_sesp);
    server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.key_name[webKey]; server.printP(html_e_tag);
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_Key); server.printP(html_e_td_td_sesp);
    formatKey(conf.key[webKey], tmp);    
    server.printP(html_s_tag); server << "k"; server.printP(html_m_tag); server << tmp; server.printP(html_e_tag);
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_Last); server.printP(text_space); server.printP(text_key); server.printP(html_e_td_td);
    formatKey(last_key, tmp);
    server.printP(text_sesp); server << tmp; server.printP(html_e_td_e_tr);
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

      server.printP(html_table_tr_th);
      server.printP(text_hash);
      server.printP(html_e_th_th); server.printP(text_Enabled);
      server.printP(html_e_th_th); server.printP(text_Name);
      server.printP(html_e_th_th); server.printP(text_Number);
      server.printP(html_e_th_th); server.printP(text_Global);
      server.printP(html_e_th_th); server.printP(text_Group);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < NUM_OF_PHONES; ++i) {
        if ( (i & 0x01) == 0) server.printP(html_tr_od);
        else                  server.printP(html_tr_ev);
        server.printP(html_td); 
        server << i+1; server.printP(text_dot); server.printP(html_e_td_td);
        (conf.tel[i] & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td_td);
        server << conf.tel_name[i]; server.printP(html_e_td_td);
        server << conf.tel_num[i]; server.printP(html_e_td_td);        
        (conf.tel[i] >> 5 & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td_td);
        if (conf.tel[i] & B1) {
          if (conf.tel[i] >> 5 & B1) { server.printP(text_all); }
          else {
            server << ((conf.tel[i] >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((conf.tel[i] >> 1) & B1111)];
          }
        } else { server.printP(text_disabled); }
        server.printP(html_e_td_e_tr);
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
      server.printP(html_table_tr_td);
      server.printP(text_Phone); server.printP(text_space); server.printP(text_is); server.printP(html_e_td_td_sesp);
      server.radioButton("o", text_1, text_On, conf.tel[webTel] & B1);
      server.radioButton("o", text_0, text_Off, !(conf.tel[webTel] & B1));
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td); server.printP(text_Phone); server.printP(text_space); server.printP(text_name); server.printP(html_e_td_td_sesp);
    //if (webTel != 0) { server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.tel_name[webTel]; server.printP(html_e_tag); }
    //else             { server << conf.tel_name[webTel]; }
      server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.tel_name[webTel]; server.printP(html_e_tag);
      server.printP(html_tr_td); server.printP(text_Phone); server.printP(text_space); server.printP(text_number); server.printP(html_e_td_td_sesp);
      server.printP(html_s_tag); server << "p"; server.printP(html_m_tag); server << conf.tel_num[webTel]; server.printP(html_e_tag);
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td); server.printP(text_Global); server.printP(text_space); server.printP(text_phone); server.printP(html_e_td_td_sesp);
      server.radioButton("a", text_1, text_Yes, conf.tel[webTel] >> 5 & B1);
      server.radioButton("a", text_0, text_No, !(conf.tel[webTel] >> 5 & B1));
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td); server.printP(text_Group); server.printP(html_e_td_td_sesp);
      server.printP(html_select); server << "g"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
        if ((conf.tel[webTel] >> 1 & B1111) == ii) 
          { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option); }
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option); }
      }
      server.printP(html_e_select);
      server.printP(html_e_td_e_tr);
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

      server.printP(html_table_tr_th);
      server.printP(text_hash);
      server.printP(html_e_th_th); server.printP(text_Enabled);
      server.printP(html_e_th_th); server.printP(text_Address);
      server.printP(html_e_th_th); server.printP(text_Type);
      server.printP(html_e_th_th); server.printP(text_Group);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < units; ++i) {
        if ( (i & 0x01) == 0) server.printP(html_tr_od);
        else                  server.printP(html_tr_ev);
        server.printP(html_td); 
        server << i+1; server.printP(text_dot); server.printP(html_e_td_td);
        (unit[i].setting & B1) ? server.printP(text_Yes) : server.printP(text_No);
        server.printP(html_e_td_td);
        if (unit[i].address < RADIO_UNIT_OFFSET) { server << "W:" << unit[i].address; }
        else                                     { server << "R:" << unit[i].address-RADIO_UNIT_OFFSET; }
        server.printP(text_semic); server << unit[i].number; server.printP(html_e_td_td);
        switch(unit[i].type){
          case 'i': server.printP(text_iButton); break;
          default: server.printP(text_undefined); break;
        }
        server.printP(html_e_td_td);
        server << ((unit[i].setting >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((unit[i].setting >> 1) & B1111)];
        server.printP(html_e_td_e_tr);
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
      server.printP(html_table_tr_td);
      server.printP(text_Address); server.printP(html_e_td_td_sesp);
      server << unit[webAuth].address; server.printP(text_semic); server << unit[webAuth].number; server.printP(html_e_td_e_tr);
      server.printP(html_tr_td); 
      server.printP(text_Type); server.printP(html_e_td_td_sesp);
      switch(unit[webAuth].type){
        case 'i': server.printP(text_iButton); break;
        default: server.printP(text_undefined); break;
      }
      server.printP(html_e_td_e_tr);
      server.printP(html_tr_td); server.printP(text_Unit); server.printP(text_space); server.printP(text_is); server.printP(html_e_td_td_sesp);
      server.radioButton("o", text_1, text_On, unit[webAuth].setting & B1);
      server.radioButton("o", text_0, text_Off, !(unit[webAuth].setting & B1));
      server.printP(html_e_td_e_tr); 
      server.printP(html_tr_td); server.printP(text_Group); server.printP(html_e_td_td_sesp);
      server.printP(html_select); server << "g"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
        if ((unit[webAuth].setting >> 1 & B1111) == ii) 
          { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option); }
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option); }
      }
      server.printP(html_e_select);
      server.printP(html_e_td_e_tr);
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
    server.printP(html_table_tr_th);
    server.printP(text_hash);
    server.printP(html_e_th_th); server.printP(text_Enabled);
    server.printP(html_e_th_th); server.printP(text_Address);
    server.printP(html_e_th_th); server.printP(text_MQTT);
    server.printP(html_e_th_th); server.printP(text_Last); server.printP(text_space); server.printP(text_OK);
    server.printP(html_e_th_th); server.printP(text_Type);
    server.printP(html_e_th_th); server.printP(text_Value);
    server.printP(html_e_th_th); server.printP(text_Group);
    server.printP(html_e_th_e_tr);
    for (uint8_t i = 0; i < sensors; ++i) {
      if ( (i & 0x01) == 0) server.printP(html_tr_od);
      else                  server.printP(html_tr_ev);
      server.printP(html_td); 
      server << i+1; server.printP(text_dot); server.printP(html_e_td_td);
      (sensor[i].setting & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td_td);
      if (sensor[i].address < RADIO_UNIT_OFFSET) { server << "W:" << sensor[i].address; }
      else                                 { server << "R:" << sensor[i].address-RADIO_UNIT_OFFSET; }
      server.printP(text_semic); server << sensor[i].number; server.printP(html_e_td_td);
      ((sensor[i].setting >> 7) & B1) ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td_td);
      time_now = (nilTimeNow() - sensor[i].last_OK)/NIL_CFG_FREQUENCY; 
      server.print((char*)time_now.formatedUpTime()); server.printP(html_e_td_td);
      switch(sensor[i].type){
        case 'T': server.printP(text_Temperature); break;
        case 'H': server.printP(text_Humidity); break;
        case 'P': server.printP(text_Pressure); break;
        default: server.printP(text_undefined); break;
      }
      server.printP(html_e_td_td);
      dtostrf(sensor[i].value, 6, 2, value);
      server << value;
      switch(sensor[i].type){
        case 'T': server.printP(text_degC); break;
        case 'H': server.printP(text_space); server.printP(text_percent); break;
        case 'P': server.printP(text_mBar); break;
        default: break;
      }
      server.printP(html_e_td_td);
      server << ((sensor[i].setting >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((sensor[i].setting >> 1) & B1111)];
      server.printP(html_e_td_e_tr);
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
    server.printP(html_table_tr_td);
    server.printP(text_Address); server.printP(html_e_td_td_sesp);
    if (sensor[webSens].address < RADIO_UNIT_OFFSET) { server << "W:" << sensor[webSens].address; }
    else                                             { server << "R:" << sensor[webSens].address-RADIO_UNIT_OFFSET; }
    server.printP(text_semic); server << sensor[webSens].number; server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); 
    server.printP(text_Type); server.printP(html_e_td_td_sesp);
    switch(sensor[webSens].type){
      case 'T': server.printP(text_Temperature); break;
      case 'H': server.printP(text_Humidity); break;
      case 'P': server.printP(text_Pressure); break;
      default: server.printP(text_undefined); break;
    }
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_Sensor); server.printP(text_space); server.printP(text_is); server.printP(html_e_td_td_sesp);
    server.radioButton("o", text_1, text_On, sensor[webSens].setting & B1);
    server.radioButton("o", text_0, text_Off, !(sensor[webSens].setting & B1));
    server.printP(html_e_td_e_tr); 
    server.printP(html_tr_td); server.printP(text_MQTT); server.printP(text_space); server.printP(text_publish); server.printP(html_e_td_td_sesp);
    server.radioButton("p", text_1, text_On, (sensor[webSens].setting >> 7) & B1);
    server.radioButton("p", text_0, text_Off, !((sensor[webSens].setting >> 7) & B1));
    server.printP(html_e_td_e_tr); 
    server.printP(html_tr_td); server.printP(text_Group); server.printP(html_e_td_td_sesp);
    server.printP(html_select); server << "g"; server.printP(html_e_tag);
    for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
      if ((sensor[webSens].setting >> 1 & B1111) == ii) 
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(html_e_td_e_tr);
    server.printP(html_e_table);
    server.printP(html_e_p);    
    server.printP(html_F_A); // submit Apply
    server.printP(html_F_SA); // submit Save all
    server.printP(html_F_RR); // submit Reregister
    server.printP(html_e_form);
    server.printP(htmlFoot);
  }
}

void webSetMQTT(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  char name[2], value[17];
  if (type == WebServer::POST) {
    bool repeat;
    //char name[2], value[17];
    do {
      repeat = server.readPOSTparam(name, 2, value, 17);
      switch(name[0]){
        case 'e': saveConf(); break; // save to EEPROM
      }
    } while (repeat);
    server.httpSeeOther(PREFIX "/mqtt");
  } else {
    server.httpSuccess();
    server.printP(htmlHead);

    server.printP(html_h1); server.printP(text_MQTT); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_table_tr_td);

    server.printP(text_MQTT); server.printP(text_space); server.printP(text_is); server.printP(text_space); server.printP(text_connected); server.printP(html_e_td_td_sesp);
    client.connected() ? server.printP(text_Yes) : server.printP(text_No); server.printP(html_e_td_e_tr);
    /*
    server.printP(html_tr_td); server.printP(text_Radio); server.printP(text_space); server.printP(text_key); server.printP(html_e_td_td);
    server.printP(text_sesp);
    server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.tel_name[webTel]; server.printP(html_e_tag);
    */
    server.printP(html_e_table);
    server.printP(html_e_p);
    server.printP(html_F_A); // submit Apply
    server.printP(html_F_SA); // submit Save all
    server.printP(html_e_form);
    server.printP(htmlFoot);
  }
}

void webSetGlobal(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  char name[2], value[17], _text[20];
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
        case 'p': // password
          _text[0] = 0;
          strcat (_text, conf.user);
          strcat (_text, ":");
          strcat (_text, value);
          base64_encode(conf.user_pass, _text, strlen(_text));
        break;
          case 'u': // user
          _text[0] = 0;
          strcat (_text, value);
          strcat (_text, ":");
          strcat (_text, conf.password);
          base64_encode(conf.user_pass, _text, strlen(_text));
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
        /* Free Free Free Free 
        case '3': // Radio button
          if (value[0] == '0') conf.SMS &= ~(1 << 3);
          else conf.SMS |= (1 << 4);
        break;
        */
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
    server.printP(html_table_tr_td);
    server.printP(text_User); server.printP(html_e_td_td_sesp);
    server.printP(html_s_tag); server << "u"; server.printP(html_m_tag); server.printP(text_hash); ; server.printP(html_e_tag);
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td);
    server.printP(text_Password); server.printP(html_e_td_td_sesp);
    server.printP(html_s_tag); server << "p"; server.printP(html_m_tag); server.printP(text_hash); ; server.printP(html_e_tag);
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td);
    server.printP(text_Authentication); server.printP(text_space); server.printP(text_time);server.printP(html_e_td_td_sesp);
    server.printP(html_select); server << "a"; server.printP(html_e_tag);
    for (uint8_t ii = 5; ii < 26; ++ii) {
      if ((conf.alr_time) == ii)  
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii; server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii; server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(text_space); server.printP(text_seconds); server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_Arm); server.printP(text_space); server.printP(text_delay); server.printP(html_e_td_td_sesp);
    server.printP(html_select); server << "d"; server.printP(html_e_tag);
    for (uint8_t ii = 10; ii < 41; ++ii) {
      if ((conf.arm_delay/4) == ii) // 250*4 = 1 sec.
        { server.printP(html_option); server << ii; server.printP(html_selected); server << ii; server.printP(html_e_option); }
      else 
        { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii; server.printP(html_e_option); }
    }
    server.printP(html_e_select);
    server.printP(text_space); server.printP(text_seconds); server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_Auto); server.printP(text_space); server.printP(text_arm); server.printP(text_space); server.printP(text_delay); server.printP(html_e_td_td_sesp);
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
    server.printP(text_space); server.printP(text_minutes); server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_Open); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td_td_sesp);
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
    server.printP(text_space); server.printP(text_minutes); server.printP(html_e_td_e_tr);
    server.printP(html_e_table);

    server.printP(html_h1); server.printP(text_Radio); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_table_tr_td);
    server.printP(text_Radio); server.printP(text_space); server.printP(text_key); server.printP(html_e_td_td_sesp);
    server.printP(html_s_tag); server << "k"; server.printP(html_m_tag); server << (char*)conf.radioKey; server.printP(html_e_tag);
    server.printP(html_e_table);

    server.printP(html_h1); server.printP(text_SMS); server.printP(text_space); server.printP(text_alerting); server.printP(html_e_h1);  server.printP(html_p);
    server.printP(html_table_tr_td);
    server.printP(text_Undefined); server.printP(text_space); server.printP(text_key);; server.printP(html_e_td_td_sesp);
    server.radioButton("0", text_1, text_On, conf.SMS & B1);
    server.radioButton("0", text_0, text_Off, !(conf.SMS & B1));
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_System); server.printP(text_space); server.printP(text_disarmed); server.printP(html_e_td_td_sesp);
    server.radioButton("1", text_1, text_On, conf.SMS >> 1 & B1);
    server.radioButton("1", text_0, text_Off, !(conf.SMS >> 1 & B1));
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_System); server.printP(text_space); server.printP(text_armed); server.printP(html_e_td_td_sesp);
    server.radioButton("2", text_1, text_On, conf.SMS >> 2 & B1);
    server.radioButton("2", text_0, text_Off, !(conf.SMS >> 2 & B1));
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_Open); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td_td_sesp);
    server.radioButton("E", text_1, text_On, conf.SMS >> 12 & B1);
    server.radioButton("E", text_0, text_Off, !(conf.SMS >> 12 & B1));
    server.printP(html_e_td_e_tr);
    /* Free Free Free Free  
    server.printP(html_tr_td); server.printP(text_Damaged); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td_td_sesp);
    server.radioButton("3", text_1, text_On, conf.SMS >> 3 & B1);
    server.radioButton("3", text_0, text_Off, !(conf.SMS >> 3 & B1));
    server.printP(html_e_td_e_tr);
    */
    server.printP(html_tr_td); server.printP(text_Tamper); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td_td_sesp);
    server.radioButton("4", text_1, text_On, conf.SMS >> 4 & B1);
    server.radioButton("4", text_0, text_Off, !(conf.SMS >> 4 & B1));
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_Sensor); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td_td_sesp);
    server.radioButton("5", text_1, text_On, conf.SMS >> 5 & B1);
    server.radioButton("5", text_0, text_Off, !(conf.SMS >> 5 & B1));
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_System); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td_td_sesp);
    server.radioButton("6", text_1, text_On, conf.SMS >> 6 & B1);
    server.radioButton("6", text_0, text_Off, !(conf.SMS >> 6 & B1));
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_Monitoring); server.printP(text_space); server.printP(text_started); server.printP(html_e_td_td_sesp);
    server.radioButton("7", text_1, text_On, conf.SMS >> 7 & B1);
    server.radioButton("7", text_0, text_Off, !(conf.SMS >> 7 & B1));
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_System); server.printP(text_space); server.printP(text_group); server.printP(text_space); server.printP(text_armed); server.printP(html_e_td_td_sesp);
    server.radioButton("8", text_1, text_On, conf.SMS >> 8 & B1);
    server.radioButton("8", text_0, text_Off, !(conf.SMS >> 8 & B1));
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_Configuration); server.printP(text_space); server.printP(text_saved); server.printP(html_e_td_td_sesp);
    server.radioButton("9", text_1, text_On, conf.SMS >> 9 & B1);
    server.radioButton("9", text_0, text_Off, !(conf.SMS >> 9 & B1));
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_Power); server.printP(text_space); server.printP(text_state); server.printP(html_e_td_td_sesp);
    server.radioButton("Q", text_1, text_On, conf.SMS >> 10 & B1);
    server.radioButton("Q", text_0, text_Off, !(conf.SMS >> 10 & B1));
    server.printP(html_e_td_e_tr);
    server.printP(html_tr_td); server.printP(text_Battery); server.printP(text_space); server.printP(text_state); server.printP(html_e_td_td_sesp);
    server.radioButton("W", text_1, text_On, conf.SMS >> 11 & B1);
    server.radioButton("W", text_0, text_Off, !(conf.SMS >> 11 & B1));
    server.printP(html_e_td_e_tr);
    server.printP(html_e_table);
    server.printP(html_e_p);
    server.printP(html_F_A); // submit Apply
    server.printP(html_F_SA); // submit Save all
    server.printP(html_e_form);
    server.printP(htmlFoot);
  }
}

#endif