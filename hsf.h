#ifndef HSF_h
#define HSF_h

// *********************************************************************************
// W E B   P A G E S              W E B   P A G E S                W E B   P A G E S  
// *********************************************************************************
typedef enum {
  menu_Home     = 1,
  menu_Global   = 2,
  menu_Zones    = 3,
  menu_Groups   = 4,
  menu_Keys     = 5,
  menu_Contacts = 6,
  menu_Sens     = 7,
  menu_Triggers = 8,
  menu_Timers   = 9,  
  menu_MQTT     = 10,
  menu_Log      = 11,
  menu_Debug    = 12
} menu_t;

void webMenu(WebServer &server, uint8_t selected) {
  server.printP(htmlHead_s);
  server.printP(html_li_a);
  if (selected == menu_Home ) { server.printP(html_c_active); }
  server.printP(html_href); server.printP(html_menu_Home); server.printP(html_li_e_li_a);
  if (selected == menu_Global ) { server.printP(html_c_active); }
  server.printP(html_href); server.printP(html_menu_Global); server.printP(html_li_e_li_a);
  if (selected == menu_Zones ) { server.printP(html_c_active); }
  server.printP(html_href); server.printP(html_menu_Zones); server.printP(html_li_e_li_a);
  if (selected == menu_Groups ) { server.printP(html_c_active); }
  server.printP(html_href); server.printP(html_menu_Groups); server.printP(html_li_e_li_a);
  if (selected == menu_Keys ) { server.printP(html_c_active); }
  server.printP(html_href); server.printP(html_menu_Keys); server.printP(html_li_e_li_a);
  if (selected == menu_Contacts ) { server.printP(html_c_active); }
  server.printP(html_href); server.printP(html_menu_Contacts); server.printP(html_li_e_li_a);
  if (selected == menu_Sens ) { server.printP(html_c_active); }
  server.printP(html_href); server.printP(html_menu_Sens); server.printP(html_li_e_li_a);
  if (selected == menu_Triggers ) { server.printP(html_c_active); }
  server.printP(html_href); server.printP(html_menu_Triggers); server.printP(html_li_e_li_a);
  if (selected == menu_Timers) { server.printP(html_c_active); }
  server.printP(html_href); server.printP(html_menu_Timers); server.printP(html_li_e_li_a);
  if (selected == menu_MQTT ) { server.printP(html_c_active); }
  server.printP(html_href); server.printP(html_menu_MQTT); server.printP(html_li_e_li_a);
  if (selected == menu_Log) { server.printP(html_c_active); }
  server.printP(html_href); server.printP(html_menu_Log); server.printP(html_li_e_li_a);
  if (selected == menu_Debug) { server.printP(html_c_active); }
  server.printP(html_href); server.printP(html_menu_Debug); server.printP(html_li_e);
  server.printP(htmlHead_e);
}


#if WEB_SERIAL_DEBUGGING 
void webDebug(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (server.checkCredentials(conf.user_pass)) {
    if (type == WebServer::POST) {
      // no post
    } else {  
      uint8_t data;
      server.httpSuccess();
      webMenu(server, menu_Debug);
      
      WS.resetRead(); // show all in buffer
      server.printP(html_h1); server.printP(text_Debug); server.printP(html_e_h1); 
      for (uint8_t i = 0; i < idleSlots; ++i){
        if (i == idlePointer) server << "<B>" << idleCount[i] << "</B>"; 
        else                  server << idleCount[i];
        server.printP(text_spdashsp);
      }
      server << "Conf size: " << sizeof(conf)+sizeof(trigger)+sizeof(timer)+sizeof(group) << "</br>";
       
      server.printP(html_pre);
      while (WS.isAvailable()) {
        data = WS.read();
        server << (char)data;
      }
      server.printP(html_e_pre); 
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
}
#endif

void webHome(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (server.checkCredentials(conf.user_pass)) {
    if (type == WebServer::POST) {
      bool repeat;
      char name[2], value[16];
      do {
        repeat = server.readPOSTparam(name, 2, value, 16);
        switch(name[0]){
          case 'T': // NTP Sync
            time_temp = GetNTPTime(udp);  
            if (time_temp.get() > 0) {
              RTC.adjust(time_temp.get());
              timestamp = time_temp.get();
            }
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
      server.httpSuccess();
      webMenu(server, menu_Home);
      server.printP(html_h1); server.printP(text_System); server.printP(html_e_h1);  
      server.printP(html_table_tr_td); 
      server.printP(text_Time); server.printP(html_e_td_td);
      server.print((char*)timestamp.formatedDateTime());
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Start); server.printP(text_ed); server.printP(html_e_td_td);
      server.print((char*)time_started.formatedDateTime());        
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Uptime); server.printP(html_e_td_td);
      time_temp = (timestamp.get()-time_started.get());
      server.print((char*)time_temp.formatedUpTime());
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_PwrSp); server.printP(html_e_td_td);
      pinAC_OFF.read() ? server.printP(text_i_disabled) : server.printP(text_i_OK); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Battery); server.printP(html_e_td_td);
      pinBAT_OK.read() ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_e_tr);
      server.printP(html_e_table);
      
      server.printP(html_h1); server.printP(text_Radio); server.printP(html_e_h1);  
      server.printP(html_table_tr_td);
      server.printP(text_Received); server.printP(text_space); server.printP(text_packets); server.printP(html_e_td_td);
      server << radio_received; server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Sent); server.printP(text_space); server.printP(text_packets); server.printP(html_e_td_td);
      server << radio_sent; server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Not); server.printP(text_space); server.printP(text_acknowledged); server.printP(text_space); server.printP(text_packets); server.printP(html_e_td_td);
      server << radio_no_ack; server.printP(html_e_td_e_tr);
      server.printP(html_e_table);

      server.printP(html_h1); server.printP(text_GSM); server.printP(text_space); server.printP(text_modem); server.printP(html_e_h1);  
      server.printP(html_table_tr_td);
      server.printP(text_GSM); server.printP(text_space); server.printP(text_modem); server.printP(text_space); server.printP(text_is); server.printP(text_space); server.printP(text_On); server.printP(html_e_td_td);
      pinGSM_ON.read() ? server.printP(text_i_disabled) : server.printP(text_i_OK); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_GSM); server.printP(text_space); server.printP(text_modem); server.printP(text_space); server.printP(text_is); server.printP(text_space); server.printP(text_connected); server.printP(html_e_td_td);
      GSMisAlive ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_e_tr_tr_td);      
      server.printP(text_GSM); server.printP(text_space); server.printP(text_network); server.printP(text_space); server.printP(text_is);
      server.printP(html_e_td_td);
      switch(GSMreg){
        case 0 : server.printP(text_nr); break;
        case 1 : server.printP(text_i_home); break;
        case 2 : server.printP(text_i_starting); break;
        case 3 : server.printP(text_rd); break;
        // case 4 : server.printP(text_unk); break;
        case 5 : server.printP(text_rr); break;
        default : server.printP(text_i_question);; break;
      }
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Signal); server.printP(text_space); server.printP(text_strength);
      server.printP(html_e_td_td);
      server << (GSMstrength*3); server.printP(text_percent); 
      server.printP(html_e_td_e_tr);
      server.printP(html_e_table);
      
      server.printP(html_form_s); server << PREFIX "/"; server.printP(html_form_e);
      server.printP(html_F_GetNTP);
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
  if (server.checkCredentials(conf.user_pass)) {
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
      server.httpSeeOther(PREFIX "/l");
    } else {  
      server.httpSuccess();
      webMenu(server, menu_Log);
      server.printP(html_h1); server.printP(text_Log); server.printP(html_e_h1); 
      server.printP(html_form_s); server << PREFIX "/l"; server.printP(html_form_e);
      server.printP(html_F_Clear); // Clear
      server.printP(html_F_CL);
       
      server.printP(html_table_tr_th); server.printP(text_hash);
      server.printP(html_e_th_th); server.printP(text_Date); 
      server.printP(html_e_th_th); server.printP(text_Message);
      server.printP(html_e_th_th); server.printP(text_SMS);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < 21; ++i){
        server.printP(html_tr_td);
        // Get data from extrenal EEPROM
        nilSemWait(&TWISem);     // wait for slot
        eeprom.readBytes((uint16_t)(ses_eeprom_add + (i*EEPROM_MESSAGE)),EEPROM_MESSAGE, tmp);
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
            server.printP(text_System); server.printP(text_space);
            switch(tmp[14]){
              case 'B': // Battery
                server.printP(text_battery); server.printP(text_space);
                if (tmp[15] == 'L') server.printP(text_low);
                else server.printP(text_OK);
              break;
              case 'A': // AC
                server.printP(text_PWS);
                if (tmp[15] == 'L') server.printP(text_On);
                else server.printP(text_Off);
              break;
              case 'C': server.printP(text_configuration); server.printP(text_space);
                switch(tmp[15]){
                  case 'W': server.printP(text_saved); break; // conf. saved
                  case 'P': server.printP(text_saved); server.printP(text_cosp); server.printP(text_system);
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
              case 'S': server.printP(text_monitoring); server.printP(text_space); server.printP(text_started); break; // monitoring strted
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
              case 'L': server.printP(text_Log); server.printP(text_space); server.printP(text_erased);break;     // Log cleared
              default:  server.printP(text_undefined); break; // unknown
            }
          break;
          case 'P': // PIR
            server.printP(text_Zone); server.printP(text_space); server << conf.zone_name[tmp[14]-48]; server.printP(text_space); server.printP(text_trigger); server.printP(text_ed); server.printP(text_space); server.printP(text_alarm);
          break;
          case 'T': // Tamper
            server.printP(text_Zone); server.printP(text_space); server << conf.zone_name[tmp[14]-48]; server.printP(text_space); server.printP(text_is); server.printP(text_space); server.printP(text_tamper); server.printP(text_ed);
          break;
          case 'A': // Authentication
            server.printP(text_Authentication); server.printP(text_cosp); 
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
            server.printP(text_GSM); server.printP(text_space); server.printP(text_network); server.printP(text_space);
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
          case 'U': // remote node
            if (tmp[14] == 'E') { 
              server.printP(text_Node); server.printP(text_space); server.printP(text_registration);
              server.printP(text_space); server.printP(text_error);
              break;
            }
            server.printP(text_Remote); server.printP(text_space); server.printP(text_node); server.printP(text_space);
            server.printP(text_address); server.printP(text_space); server << tmp[15]-48; server.printP(text_space);
            switch(tmp[14]){
              case 'F' : server.printP(text_is); server.printP(text_space); server.printP(text_disabled); break;
              case 'R' : server.printP(text_registered); break;
              case 'r' : server.printP(text_re); server.printP(text_registered); break;
              default : server.printP(text_unk); break;
            }
          break;
          case 'a' ... 'x': // remote nodes
            switch(tmp[13]){
              case 't': server.printP(text_Temperature); break;
              case 'h': server.printP(text_Humidity); break;
              case 'p': server.printP(text_Pressure); break;
              case 'i': server.printP(text_Input); break;
              case 'v': server.printP(text_Voltage); break;
              default : server.printP(text_unk); break;
            }
            server.printP(text_space); server.printP(text_node); server.printP(text_sesp);
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
            switch(tmp[14]){
              case 'A' : server.printP(text_Alarm); break;
              case 'S' : server.printP(text_Node); break;
              case 'R' : server.printP(text_Registration); break;
              default : server.printP(text_unk); break;
            }
            server.printP(text_space); server.printP(text_queue); server.printP(text_space); server.printP(text_is); server.printP(text_space); server.printP(text_full);
          break;
          case 'R': // Trigger
            server.printP(text_Trigger); server.printP(text_space); server.printP(text_number); server.printP(text_space);
            server << tmp[14]-47; server.printP(text_space); server.printP(text_activated); server.printP(text_dot); server.printP(text_space);
            server.printP(text_node); server.printP(text_space);
            if (trigger[tmp[14]-48].address < RADIO_UNIT_OFFSET) { server << "W:" << trigger[tmp[14]-48].address; }
            else                                                 { server << "R:" << trigger[tmp[14]-48].address-RADIO_UNIT_OFFSET; }
            server.printP(text_semic); server << trigger[tmp[14]-48].number; server.printP(text_semic);
            switch(trigger[tmp[14]-48].type){
              case 'T': server.printP(text_Temperature); break;
              case 'H': server.printP(text_Humidity); break;
              case 'P': server.printP(text_Pressure); break;
              case 'I': server.printP(text_Input); break;
              case 'V': server.printP(text_Voltage); break;
              default: server.printP(text_i_question); break;
            }
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
      server.printP(html_e_form); 
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
}

uint8_t webZone = 0;
void webSetZone(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (server.checkCredentials(conf.user_pass)) {
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
          case '0' ... '9': // Handle all single radio buttons for settings
            if (value[0] == '0') conf.zone[webZone] &= ~(1 << (name[0]-48));
            else                 conf.zone[webZone] |=  (1 << (name[0]-48));
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
        server.httpSeeOther(PREFIX "/z");
      } else {
        server.httpSuccess();
        webMenu(server, menu_Zones);
        server.printP(html_h1); server.printP(text_Zone); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1); 
        server.printP(html_form_s); server << PREFIX "/z"; server.printP(html_form_e);

        server.printP(html_table_tr_th); server.printP(text_hash);
        server.printP(html_e_th_th); server.printP(text_Type);
        server.printP(html_e_th_th); server.printP(text_Name); 
        server.printP(html_e_th_th); server.printP(text_On);
        server.printP(html_e_th_th); server.printP(text_Auto); server.printP(text_space); server.printP(text_arm);
        server.printP(html_e_th_th); server.printP(text_Open); server.printP(text_space); server.printP(text_alarm);
        server.printP(html_e_th_th); server.printP(text_Alarm); server.printP(text_space); server.printP(text_as); server.printP(text_space); server.printP(text_tamper);
        server.printP(html_e_th_th); server.printP(text_Delay);
        server.printP(html_e_th_th); server.printP(text_Group);
        server.printP(html_e_th_th); server.printP(text_Last); server.printP(text_space); server.printP(text_alarm);
        server.printP(html_e_th_th); server.printP(text_Last); server.printP(text_space); server.printP(text_OK);
        server.printP(html_e_th_th); server.printP(text_Status);
        server.printP(html_e_th_e_tr);
        for (uint8_t i = 0; i < ALR_ZONES; ++i) {
          //   Zone is connected
          if ((conf.zone[i] >> 14) & B1) {
            server.printP(html_tr_td); 
            server << i+1; server.printP(text_dot); server.printP(html_e_td_td);
            (conf.zone[i] >> 15) ? server.printP(text_analog) : server.printP(text_digital); server.printP(html_e_td_td);
            server << conf.zone_name[i]; server.printP(html_e_td_td);
            (conf.zone[i] & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
            ((conf.zone[i] >> 7) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
            ((conf.zone[i] >> 8) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
            ((conf.zone[i] >> 9) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
            server << (((conf.zone[i] >> 5) & B11)*conf.alr_time); server.printP(text_space); server.printP(text_seconds); server.printP(html_e_td_td);
            if ((conf.zone[i] & B1)) { server << ((conf.zone[i] >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((conf.zone[i] >> 1) & B1111)]; }
            else                     { server.printP(text_spdashsp); }
            server.printP(html_e_td_td);
            time_temp = timestamp.get() - zone[i].last_PIR;
            if ((conf.zone[i] & B1)) { server.print((char*)time_temp.formatedUpTime()); }
            else                     { server.printP(text_spdashsp); }
            time_temp = timestamp.get() - zone[i].last_OK;
            server.printP(html_e_td_td);
            if ((conf.zone[i] & B1)) { server.print((char*)time_temp.formatedUpTime()); }
            else                     { server.printP(text_spdashsp); }
            server.printP(html_e_td_td);
            if ((conf.zone[i] & B1)) {
              switch(zone[i].last_event){
                case 'O': server.printP(text_i_OK); break;
                case 'P': server.printP(text_i_ALARM); break;
                case 'N': server.printP(text_i_starting); break;
                default: server.printP(text_tamper); break;
              }
            } else { server.printP(text_i_disabled); } // disabled
            server.printP(html_e_td_e_tr);
          } // Zone is connected
        }
        server.printP(html_e_table); 

        server.printP(html_form_s); server << PREFIX "/zone"; server.printP(html_form_e);
        server.printP(html_table_tr_td);
        server.printP(text_Zone);server.printP(html_e_td_td);
        server.printP(html_select_submit); server << "Z"; server.printP(html_e_tag);
        for (uint8_t ii = 0; ii < ALR_ZONES; ++ii) {
            if ((conf.zone[ii] >> 14) & B1) {
            server.printP(html_option); server << ii;
            if (webZone == ii) { server.printP(html_selected); }
            else               { server.printP(html_e_tag); }
            server << ii + 1; server.printP(text_spdashsp); server << conf.zone_name[ii]; server.printP(html_e_option);
          }
        }
        server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Type); server.printP(html_e_td_td);
        ((conf.zone[webZone] >> 15) & B1) ? server.printP(text_analog) : server.printP(text_digital); server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Name); server.printP(html_e_td_td);
        server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.zone_name[webZone]; server.printP(html_e_tag);
        server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Zone); server.printP(text_space); server.printP(text_is); server.printP(html_e_td_td);
        server.printP(html_radio_s);
        server.radioButton("0", text_1, text_On, conf.zone[webZone] & B1);
        server.radioButton("0", text_0, text_Off, !(conf.zone[webZone] & B1));
        server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Auto); server.printP(text_space); server.printP(text_arm); server.printP(html_e_td_td);
        server.printP(html_radio_s);
        server.radioButton("7", text_1, text_On, conf.zone[webZone] >> 7 & B1);
        server.radioButton("7", text_0, text_Off, !(conf.zone[webZone] >> 7 & B1));
        server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Open); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td_td);
        server.printP(html_radio_s);
        server.radioButton("8", text_1, text_On, conf.zone[webZone] >> 8 & B1);
        server.radioButton("8", text_0, text_Off, !(conf.zone[webZone] >> 8 & B1));
        server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Alarm); server.printP(text_space); server.printP(text_as); server.printP(text_space); server.printP(text_tamper); server.printP(html_e_td_td);
        server.printP(html_radio_s);
        server.radioButton("9", text_1, text_On, conf.zone[webZone] >> 8 & B1);
        server.radioButton("9", text_0, text_Off, !(conf.zone[webZone] >> 8 & B1));
        server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Authentication); server.printP(text_space); server.printP(text_delay); 
        server.printP(html_e_td_td);
        server.printP(html_radio_sl);
        server.radioButton("d", text_0, text_0, !((conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
        server.radioButton("d", text_1, text_1, (!(conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
        server.radioButton("d", text_2, text_2, ((conf.zone[webZone] >> 6 & B1) & !(conf.zone[webZone] >> 5 & B1)));
        server.radioButton("d", text_3, text_3, ((conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
        server.printP(html_div_e); server.printP(text_space); server.printP(text_x); server << conf.alr_time; server.printP(text_space); server.printP(text_seconds); server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Group); server.printP(html_e_td_td);
        server.printP(html_select); server << "g"; server.printP(html_e_tag);
        for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
          server.printP(html_option); server << ii;
          if ((conf.zone[webZone] >> 1 & B1111) == ii) { server.printP(html_selected); }
          else                                         { server.printP(html_e_tag); }
          server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option);
        }
        server.printP(html_e_select);
        server.printP(html_e_td_e_tr);
        server.printP(html_e_table);
          
      server.printP(html_F_A); // submit Apply
      server.printP(html_F_SA); // submit Save all
      server.printP(html_e_form);
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
}

uint8_t webGroup = 0;
void webSetGroup(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (server.checkCredentials(conf.user_pass)) {
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
          case '0' ... '7': // Handle all single radio buttons for settings
            if (value[0] == '0') conf.group[webGroup] &= ~(1 << (name[0]-48));
            else                 conf.group[webGroup] |=  (1 << (name[0]-48));
            group[webGroup].setting &= ~(1 << 7);  // Clear disable group log bit
          break;
          case 'e': saveConf(); break; // save to EEPROM         
          }
        } while (repeat);
        server.httpSeeOther(PREFIX "/g");
    } else {
      server.httpSuccess();
      webMenu(server, menu_Groups);
      server.printP(html_h1); server.printP(text_Group); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  
      server.printP(html_table_tr_th); server.printP(text_hash);
      server.printP(html_e_th_th); server.printP(text_Name); 
      server.printP(html_e_th_th); server.printP(text_On);
      server.printP(html_e_th_th); server.printP(text_Armed);
      server.printP(html_e_th_th); server.printP(text_Authentication);
      server.printP(html_e_th_th); server.printP(text_Delay);
      server.printP(html_e_th_th); server.printP(text_i_zone); server.printP(text_space); server.printP(text_Zone); server.printP(text_s);
      server.printP(html_e_th_th); server.printP(text_i_auth); server.printP(text_space);server.printP(text_Authentication); server.printP(text_space); server.printP(text_node); server.printP(text_s);
      server.printP(html_e_th_th); server.printP(text_i_sens); server.printP(text_space);server.printP(text_Sensor); server.printP(text_s);
      server.printP(html_e_th_th); server.printP(text_i_phone); server.printP(text_space);server.printP(text_Phone); server.printP(text_s);
      server.printP(html_e_th_th); server.printP(text_Alarm);
      server.printP(html_e_th_th); server.printP(text_Tamper);
      server.printP(html_e_th_th); server.printP(text_Status);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < ALR_GROUPS; ++i) {
        server.printP(html_tr_td); 
        server << i+1; server.printP(text_dot); server.printP(html_e_td_td);
        server << conf.group_name[i]; server.printP(html_e_td_td);
        (conf.group[i] & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        (group[i].setting & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((group[i].setting >> 2) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        server << group[i].arm_delay/4; server.printP(text_space); server.printP(text_seconds); server.printP(html_e_td_td);
        n = 0;
        for (uint8_t ii = 0; ii < ALR_ZONES; ++ii) {
          if ((((conf.zone[ii] >> 1) & B1111) == i) && (conf.zone[ii] & B1)) {
            if (n) { server.printP(text_cosp);}
            server << ii+1; n = 1;
          }
        }
        server.printP(html_e_td_td);
        n = 0;
        for (uint8_t ii = 0; ii < nodes; ++ii) {      
          //     group number                              Node enabled               Auth. node
          if ((((node[ii].setting >> 1) & B1111) == i) && (node[ii].setting & B1) && (node[ii].function == 'K')) { 
            if (n) { server.printP(html_br); }
            for (uint8_t j = 0; j < node_names; ++j) {
              if (node_name[j].address == node[ii].address) {
                server << node_name[j].name << ":";
                break;
              }
            }
            server << node[ii].type; server.printP(text_semic);
            if (node[ii].address < RADIO_UNIT_OFFSET) { server << "W:" << node[ii].address; }
            else                                      { server << "R:" << node[ii].address-RADIO_UNIT_OFFSET; }
            server.printP(text_semic); server << node[ii].number; n = 1;
          }
        }
        server.printP(html_e_td_td);
        n = 0;
        for (uint8_t ii = 0; ii < nodes; ++ii) {
          //     group number                              Node enabled               Sensor   
          if ((((node[ii].setting >> 1) & B1111) == i) && (node[ii].setting & B1) && (node[ii].function == 'S')) {
            if (n) { server.printP(html_br); }
            for (uint8_t j = 0; j < node_names; ++j) {
              if (node_name[j].address == node[ii].address) {
                server << node_name[j].name << ":";
                break;
              }
            }
            if (node[ii].address < RADIO_UNIT_OFFSET) { server << "W:" << node[ii].address; }
            else                                      { server << "R:" << node[ii].address-RADIO_UNIT_OFFSET; }
            server.printP(text_semic); server << node[ii].number;
            server.printP(text_semic); server << node[ii].type;
            n = 1;
          }
        }
        server.printP(html_e_td_td);
        n = 0;
        for (uint8_t ii = 0; ii < NUM_OF_PHONES; ++ii) {
          if (((((conf.tel[ii] >> 1) & B1111) == i) || (conf.tel[ii] >> 5 & B1)) && (conf.tel[ii] & B1)){
            if (n) { server.printP(html_br); }
            server << conf.tel_name[ii];
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
          (group[i].setting >> 1 & B1) ? server.printP(text_i_ALARM) : server.printP(text_i_OK);
        } else { server.printP(text_i_disabled); }
        server.printP(html_e_td_e_tr);
      }
      server.printP(html_e_table); 
      server.printP(html_form_s); server << PREFIX "/g"; server.printP(html_form_e);
      
      
      server.printP(html_table_tr_td);
      server.printP(text_Group); server.printP(html_e_td_td);
      server.printP(html_select_submit); server << "G"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
        if (webGroup == ii) 
          { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(html_e_option); }
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(html_e_option);}
      }
      server.printP(html_e_select);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Name); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.group_name[webGroup]; server.printP(html_e_tag);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Group); server.printP(text_space); server.printP(text_is); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0", text_1, text_On, conf.group[webGroup] & B1);
      server.radioButton("0", text_0, text_Off, !(conf.group[webGroup] & B1));
      server.printP(html_div_e);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Alarm); server.printP(text_space); server.printP(text_trigger); server.printP(text_s); server.printP(text_space); server.printP(text_OUT1); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("4", text_1, text_On, conf.group[webGroup] >> 4 & B1);
      server.radioButton("4", text_0, text_Off, !(conf.group[webGroup] >> 4 & B1));
      server.printP(html_div_e);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Alarm); server.printP(text_space); server.printP(text_trigger); server.printP(text_s); server.printP(text_space); server.printP(text_OUT2); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("3", text_1, text_On, conf.group[webGroup] >> 3 & B1);
      server.radioButton("3", text_0, text_Off, !(conf.group[webGroup] >> 3 & B1));
      server.printP(html_div_e);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Tamper); server.printP(text_space); server.printP(text_trigger); server.printP(text_s); server.printP(text_space); server.printP(text_OUT1); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("2", text_1, text_On, conf.group[webGroup] >> 2 & B1);
      server.radioButton("2", text_0, text_Off, !(conf.group[webGroup] >> 2 & B1));
      server.printP(html_div_e);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Tamper); server.printP(text_space); server.printP(text_trigger); server.printP(text_s); server.printP(text_space); server.printP(text_OUT2); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("1", text_1, text_On, conf.group[webGroup] >> 1 & B1);
      server.radioButton("1", text_0, text_Off, !(conf.group[webGroup] >> 1 & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr);
      server.printP(html_e_table);
          
      server.printP(html_F_A); // submit Apply
      server.printP(html_F_SA); // submit Save all
      server.printP(html_e_form);
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
}

uint8_t webKey = 0;
void webSetKey(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (server.checkCredentials(conf.user_pass)) {
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
        server.httpSeeOther(PREFIX "/k");
    } else {
      server.httpSuccess();
      webMenu(server, menu_Keys);
      server.printP(html_h1); server.printP(text_Key); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1); 

      server.printP(html_table_tr_th); server.printP(text_hash);
      server.printP(html_e_th_th); server.printP(text_Name);
      server.printP(html_e_th_th); server.printP(text_On);
      server.printP(html_e_th_th); server.printP(text_Value);
      server.printP(html_e_th_th); server.printP(text_Group);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < NUM_OF_KEYS; ++i) {
        server.printP(html_tr_td); 
        server << i+1; server.printP(text_dot); server.printP(html_e_td_td);
        server << conf.key_name[i]; server.printP(html_e_td_td);
        (conf.key_setting[i] & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        formatKey(conf.key[i], tmp); server << tmp; server.printP(html_e_td_td);
        if ((conf.key_setting[i] & B1)) { server << ((conf.key_setting[i] >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((conf.key_setting[i] >> 1) & B1111)]; }
        else                            { server.printP(text_spdashsp); }
        server.printP(html_e_td_e_tr);
      }
      server.printP(html_e_table); 

      server.printP(html_form_s); server << PREFIX "/k"; server.printP(html_form_e);      
      server.printP(html_table_tr_td);
      server.printP(text_Key); server.printP(html_e_td_td);
      server.printP(html_select_submit); server << "K"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < NUM_OF_KEYS; ++ii) {
        server.printP(html_option); server << ii;
        if (webKey == ii) { server.printP(html_selected); }
        else              { server.printP(html_e_tag); }
        server << ii + 1; server.printP(text_spdashsp); server << conf.key_name[ii]; server.printP(html_e_option);
      }
      server.printP(html_e_select);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Name); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.key_name[webKey]; server.printP(html_e_tag);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Value); server.printP(html_e_td_td);
      formatKey(conf.key[webKey], tmp);    
      server.printP(html_s_tag); server << "k"; server.printP(html_m_tag); server << tmp; server.printP(html_e_tag);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Last); server.printP(text_space); server.printP(text_key); server.printP(html_e_td_td);
      formatKey(last_key, tmp); server << tmp; server.printP(html_e_td_e_tr);
      server.printP(html_e_table);     
      server.printP(html_F_A); // submit Apply
      server.printP(html_F_SA); // submit Save all
      server.printP(html_e_form);
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
}

uint8_t webTel = 0;
void webSetPhone(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (server.checkCredentials(conf.user_pass)) {
    if (type == WebServer::POST) {
      bool repeat;
      char name[2], value[31];
      do {
        repeat = server.readPOSTparam(name, 2, value, 31);
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
          case 'm': // email
            strncpy (conf.email[webTel], value, 16);
            conf.email[webTel][15] = 0;
          break;
          case '0' ... '7': // Handle all single radio buttons for settings
            if (value[0] == '0') conf.tel[webTel] &= ~(1 << (name[0]-48));
            else                 conf.tel[webTel] |=  (1 << (name[0]-48));
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
        server.httpSeeOther(PREFIX "/c");
      } else {
        server.httpSuccess();
        webMenu(server, menu_Contacts);
        server.printP(html_h1); server.printP(text_Contact); server.printP(text_s); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1); 

        server.printP(html_table_tr_th); server.printP(text_hash);
        server.printP(html_e_th_th); server.printP(text_On);
        server.printP(html_e_th_th); server.printP(text_Name);
        server.printP(html_e_th_th); server.printP(text_Number);
        server.printP(html_e_th_th); server.printP(text_Email);
        server.printP(html_e_th_th); server.printP(text_Global);
        server.printP(html_e_th_th); server.printP(text_Group);
        server.printP(html_e_th_e_tr);
        for (uint8_t i = 0; i < NUM_OF_PHONES; ++i) {
          server.printP(html_tr_td); 
          server << i+1; server.printP(text_dot); server.printP(html_e_td_td);
          (conf.tel[i] & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
          server << conf.tel_name[i]; server.printP(html_e_td_td);
          server << conf.tel_num[i]; server.printP(html_e_td_td);
          server << conf.email[i]; server.printP(html_e_td_td);
          (conf.tel[i] >> 5 & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
          if (conf.tel[i] & B1) {
            if (conf.tel[i] >> 5 & B1) { server.printP(text_all); }
            else { server << ((conf.tel[i] >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((conf.tel[i] >> 1) & B1111)]; }
          } 
          else { server.printP(text_i_disabled); }
          server.printP(html_e_td_e_tr);
        }
        server.printP(html_e_table); 
        server.printP(html_form_s); server << PREFIX "/c"; server.printP(html_form_e);
        
        server.printP(html_table_tr_td);
        server.printP(text_Contact); server.printP(html_e_td_td);  
        server.printP(html_select_submit); server << "P"; server.printP(html_e_tag);
        for (uint8_t ii = 0; ii < NUM_OF_PHONES; ++ii) {
          server.printP(html_option); server << ii;
          if (webTel == ii) { server.printP(html_selected); }
          else              { server.printP(html_e_tag); }
          server << ii + 1; server.printP(text_spdashsp); server << conf.tel_name[ii]; server.printP(html_e_option);
        }
        server.printP(html_e_select);
        server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Contact); server.printP(text_space); server.printP(text_is); server.printP(html_e_td_td);
        server.printP(html_radio_s);
        server.radioButton("0", text_1, text_On, conf.tel[webTel] & B1);
        server.radioButton("0", text_0, text_Off, !(conf.tel[webTel] & B1));
        server.printP(html_div_e);
        server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Name); server.printP(html_e_td_td);
        server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.tel_name[webTel]; server.printP(html_e_tag);
        server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Phone); server.printP(text_space); server.printP(text_number); server.printP(html_e_td_td);
        server.printP(html_s_tag); server << "p"; server.printP(html_m_tag); server << conf.tel_num[webTel]; server.printP(html_e_tag);
        server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Email); server.printP(html_e_td_td);
        server.printP(html_s_tag); server << "m"; server.printP(html_m_tag); server << conf.email[webTel]; server.printP(html_e_tag);
        server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Global); server.printP(text_space); server.printP(text_contact); server.printP(html_e_td_td);
        server.printP(html_radio_s);
        server.radioButton("5", text_1, text_Yes, conf.tel[webTel] >> 5 & B1);
        server.radioButton("5", text_0, text_No, !(conf.tel[webTel] >> 5 & B1));
        server.printP(html_div_e);
        server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Group); server.printP(html_e_td_td);
        server.printP(html_select); server << "g"; server.printP(html_e_tag);
        for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
          server.printP(html_option); server << ii; 
          if ((conf.tel[webTel] >> 1 & B1111) == ii) { server.printP(html_selected); }
          else                                       { server.printP(html_e_tag); }
          server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option);
        }
        server.printP(html_e_select);
        server.printP(html_e_td_e_tr);
        server.printP(html_e_table);
            
      server.printP(html_F_A); // submit Apply
      server.printP(html_F_SA); // submit Save all
      server.printP(html_e_form);
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
}

uint8_t webSens = 0;
void webSetSens(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (server.checkCredentials(conf.user_pass)) {
    char name[2], value[17];
    uint8_t  _found, _update_node;
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
            value[1] = node[webSens].function;
            value[2] = node[webSens].type;
            value[3] = node[webSens].number;
            value[4] = (char)node[webSens].setting >> 8;
            value[5] = (char)node[webSens].setting & B11111111;
            sendData(node[webSens].address,value, 6);
          break;
          /*
          case 'n': // name
            _found = 0;
            for (_update_node = 0; _update_node < node_names; _update_node++) {
              if (node_name[_update_node].address == node[webSens].address) { 
                _found = 1;
                break;
              }
            }
            if (_found) {
              // copy to node_name
              strncpy (node_name[_update_node].name, value, 16);
              // prepare packet
              value[0] = 'R'; value[1] = 0;
              strcat (value, node_name[_update_node].name);
              sendData(node_name[_update_node].address,value, 17);
            }
          break;
          */
          case '0' ... '7': // Handle all single radio buttons for settings
            if (value[0] == '0') node[webSens].setting &= ~(1 << (name[0]-48));
            else                 node[webSens].setting |=  (1 << (name[0]-48));
          break;
          case 'g': // group
            n = strtol(value, NULL, 10);
            if ((n >> 0) & B1) node[webSens].setting |= (1 << 1);
            else node[webSens].setting &= ~(1 << 1);
            if ((n >> 1) & B1) node[webSens].setting |= (1 << 2);
            else node[webSens].setting &= ~(1 << 2);
            if ((n >> 2) & B1) node[webSens].setting |= (1 << 3);
            else node[webSens].setting &= ~(1 << 3);
            if ((n >> 3) & B1) node[webSens].setting |= (1 << 4);
            else node[webSens].setting &= ~(1 << 4);
          break;
          case 'e': saveConf(); break; // save to EEPROM          
          }
        } while (repeat);
      server.httpSeeOther(PREFIX "/s");
    } else {
      server.httpSuccess();
      webMenu(server, menu_Sens);
      server.printP(html_h1); server.printP(text_Node); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  
      server.printP(html_form_s); server << PREFIX "/s"; server.printP(html_form_e);    
      server.printP(html_table_tr_th); server.printP(text_hash);
      server.printP(html_e_th_th); server.printP(text_On);
      server.printP(html_e_th_th); server.printP(text_Address);
      server.printP(html_e_th_th); server.printP(text_MQTT);
      server.printP(html_e_th_th); server.printP(text_Last); server.printP(text_space); server.printP(text_Message);
      server.printP(html_e_th_th); server.printP(text_Function);
      server.printP(html_e_th_th); server.printP(text_Type);
      server.printP(html_e_th_th); server.printP(text_Value);
      server.printP(html_e_th_th); server.printP(text_Group);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < nodes; ++i) {
        server.printP(html_tr_td); 
        server << i+1; server.printP(text_dot); server.printP(html_e_td_td);
        (node[i].setting & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        for (uint8_t ii = 0; ii < node_names; ++ii) {
          if (node_name[ii].address == node[i].address) {
            server << node_name[ii].name << ":";
            break;
          }
        }
        if (node[i].address < RADIO_UNIT_OFFSET) { server << "W:" << node[i].address; }
        else                                 { server << "R:" << node[i].address-RADIO_UNIT_OFFSET; }
        server.printP(text_semic); server << node[i].number; server.printP(html_e_td_td);
        ((node[i].setting >> 7) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        time_temp = timestamp.get() - node[i].last_OK; 
        server.print((char*)time_temp.formatedUpTime()); server.printP(html_e_td_td);
        switch(node[i].function){
          case 'K': server.printP(text_Authentication); break;
          case 'S': server.printP(text_Sensor); break;
          case 'I': server.printP(text_Input); break;
          default: server.printP(text_i_question); break;
        }
        server.printP(html_e_td_td);
        switch(node[i].type){
          case 'T': server.printP(text_Temperature); break;
          case 'H': server.printP(text_Humidity); break;
          case 'P': server.printP(text_Pressure); break;
          case 'D': server.printP(text_Digital); break;
          case 'A': server.printP(text_Analog); break;
          case 'F': server.printP(text_Float); break;
          case 'V': server.printP(text_Voltage); break;
          case 'i': server.printP(text_iButton); break;
          default: server.printP(text_i_question); break;
        }
        server.printP(html_e_td_td);
        dtostrf(node[i].value, 6, 2, value);
        server << value;
        switch(node[i].type){
          case 'T': server.printP(text_degC); break;
          case 'H': server.printP(text_space); server.printP(text_percent); break;
          case 'P': server.printP(text_mBar); break;
          case 'V': server.printP(text_Volt); break;
          default: break;
        }
        server.printP(html_e_td_td);
        server << ((node[i].setting >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((node[i].setting >> 1) & B1111)];
        server.printP(html_e_td_e_tr);
      }
      server.printP(html_e_table); 
      server.printP(html_table_tr_td);
      server.printP(text_Node); server.printP(html_e_td_td);
      server.printP(html_select_submit); server << "P"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < nodes; ++ii) {
        server.printP(html_option); server << ii;
        if (webSens == ii) { server.printP(html_selected); }
        else               { server.printP(html_e_tag); }
        server << ii + 1; server.printP(html_e_option);
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Address); server.printP(html_e_td_td);
      if (node[webSens].address < RADIO_UNIT_OFFSET) { server << "W:" << node[webSens].address; }
      else                                             { server << "R:" << node[webSens].address-RADIO_UNIT_OFFSET; }
      server.printP(text_semic); server << node[webSens].number; server.printP(html_e_td_e_tr_tr_td);
      /*
      server.printP(text_Name); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << ; server.printP(html_e_tag);
      server.printP(html_e_td_e_tr_tr_td);
      */
      server.printP(text_Type); server.printP(html_e_td_td);
      switch(node[webSens].type){
        case 'T': server.printP(text_Temperature); break;
        case 'H': server.printP(text_Humidity); break;
        case 'P': server.printP(text_Pressure); break;
        case 'I': server.printP(text_Input); break;
        case 'V': server.printP(text_Voltage); break;
        default: server.printP(text_undefined); break;
      }
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Node); server.printP(text_space); server.printP(text_is); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0", text_1, text_On, node[webSens].setting & B1);
      server.radioButton("0", text_0, text_Off, !(node[webSens].setting & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_MQTT); server.printP(text_space); server.printP(text_publish); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("7", text_1, text_On, (node[webSens].setting >> 7) & B1);
      server.radioButton("7", text_0, text_Off, !((node[webSens].setting >> 7) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Group); server.printP(html_e_td_td);
      server.printP(html_select); server << "g"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
        server.printP(html_option); server << ii;
        if ((node[webSens].setting >> 1 & B1111) == ii) { server.printP(html_selected); }
        else                                              { server.printP(html_e_tag); }
        server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp); (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option);
      }
      server.printP(html_e_select);
      server.printP(html_e_td_e_tr);
      server.printP(html_e_table);
          
      server.printP(html_F_A); // submit Apply
      server.printP(html_F_SA); // submit Save all
      server.printP(html_F_RR); // submit Reregister
      server.printP(html_e_form);
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
}

void webSetMQTT(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (server.checkCredentials(conf.user_pass)) {
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
      server.httpSeeOther(PREFIX "/m");
    } else {
      server.httpSuccess();
      webMenu(server, menu_MQTT);
      server.printP(html_h1); server.printP(text_MQTT); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  
      server.printP(html_table_tr_td);

      server.printP(text_MQTT); server.printP(text_space); server.printP(text_is); server.printP(text_space); server.printP(text_connected); server.printP(html_e_td_td);
      client.connected() ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_e_tr);
      /*
      server.printP(html_tr_td); server.printP(text_Radio); server.printP(text_space); server.printP(text_key); server.printP(html_e_td_td);
      server.printP(text_sesp);
      server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << conf.tel_name[webTel]; server.printP(html_e_tag);
      */
      server.printP(html_e_table);
      
      server.printP(html_F_A); // submit Apply
      server.printP(html_F_SA); // submit Save all
      server.printP(html_e_form);
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
}

uint8_t webTrig = 0;
void webSetTriggers(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (server.checkCredentials(conf.user_pass)) {
    char name[2], value[17];
    if (type == WebServer::POST) {
      bool repeat;
      //char name[2], value[17];
      do {
        repeat = server.readPOSTparam(name, 2, value, 17);
        switch(name[0]){
          case 'P':
            n = strtol(value, NULL, 10);
            if (n != webTrig) {
              webTrig = n;
              repeat = 0;
            }
            break;
          case 'n': // name
            strncpy (trigger[webTrig].name, value, sizeof(value));
            trigger[webTrig].name[15] = 0;
            break;
          case 'v': trigger[webTrig].value = atof(value); break;
          case 'c': trigger[webTrig].constant_on = atof(value); break;
          case 'f': trigger[webTrig].constant_off = atof(value); break;
          case 's': trigger[webTrig].symbol = strtol(value, NULL, 10); 
            // for Zones
            if ((trigger[webTrig].type == 'Z') && (trigger[webTrig].symbol == 0)) {
              trigger[webTrig].symbol = 1; // Do NOT allow pass always
            }
            break;
          case 'a': n = strtol(value, NULL, 10);
            if (n < ALR_ZONES) {
              trigger[webTrig].address = 0;
              trigger[webTrig].type = 'Z';
              trigger[webTrig].number = n;
            } else {
              n = n - ALR_ZONES;
              trigger[webTrig].address = node[n].address;
              trigger[webTrig].type = node[n].type;
              trigger[webTrig].number = node[n].number;
            }            
          break;
          case 't': n = strtol(value, NULL, 10);
              trigger[webTrig].to_address = node[n].address;
              trigger[webTrig].to_type = node[n].type;
              trigger[webTrig].to_number = node[n].number;
          break;
          case '0' ... '7': // Handle all single radio buttons for settings
            if (value[0] == '0') trigger[webTrig].setting &= ~(1 << (name[0]-48));
            else                 trigger[webTrig].setting |=  (1 << (name[0]-48));
            // Pass once (bit 4) to clear Passed (bit 3)
            if (((name[0]-48) == 4) && (value[0] == '0')) trigger[webTrig].setting &= ~(1 << 3); 
            // for Zones
            if (trigger[webTrig].type == 'Z') {
              trigger[webTrig].setting &= ~(1 << 6); // turn OFF Logging 
              trigger[webTrig].setting |=  (1 << 4); // turn ON Pass Once
            }
          break;
          case 'e': saveConf(); break; // save to EEPROM
        }
      } while (repeat);
      server.httpSeeOther(PREFIX "/t");
    } else {
      server.httpSuccess();
      webMenu(server, menu_Triggers);
      server.printP(html_h1); server.printP(text_Trigger); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  
      server.printP(html_form_s); server << PREFIX "/t"; server.printP(html_form_e);    
      server.printP(html_table_tr_th); server.printP(text_hash);
      server.printP(html_e_th_th); server.printP(text_Name);
      server.printP(html_e_th_th); server.printP(text_On);
      server.printP(html_e_th_th); server.printP(text_Address);
      server.printP(html_e_th_th); server.printP(text_Symbol);
      server.printP(html_e_th_th); server.printP(text_Value);
      server.printP(html_e_th_th); server.printP(text_Logging);
      server.printP(html_e_th_th); server.printP(text_Pass);
      server.printP(html_e_th_th); server.printP(text_Pass); server.printP(text_space); server.printP(text_once);
      server.printP(html_e_th_th); server.printP(text_Pass); server.printP(text_space); server.printP(text_Off);
      server.printP(html_e_th_th); server.printP(text_Pass);
      server.printP(html_e_th_th); server.printP(text_To); server.printP(text_space); server.printP(text_address);
      server.printP(html_e_th_th); server.printP(text_On);
      server.printP(html_e_th_th); server.printP(text_Off);
      server.printP(html_e_th_th); server.printP(text_Trigger); server.printP(text_ed);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < TRIGGERS; ++i) {
        server.printP(html_tr_td); 
        server << i+1; server.printP(text_dot); server.printP(html_e_td_td);
        server << timer[i].name; server.printP(html_e_td_td);
        (trigger[i].setting & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        if (trigger[i].type == 'Z') {
          server.printP(text_Zone); server.printP(text_semic); server.printP(text_space); server << conf.zone_name[trigger[i].number];
          server.printP(text_spdashsp); server << trigger[i].number + 1;
        } else {
          if (trigger[i].address < RADIO_UNIT_OFFSET) { server << "W:" << trigger[i].address; }
          else                                        { server << "R:" << trigger[i].address-RADIO_UNIT_OFFSET; }
          server.printP(text_semic); server << trigger[i].number; server.printP(text_semic);
          switch(trigger[i].type){
            case 'T': server.printP(text_Temperature); break;
            case 'H': server.printP(text_Humidity); break;
            case 'P': server.printP(text_Pressure); break;
            case 'V': server.printP(text_Voltage); break;
            default: server.printP(text_i_question); break;
          }
        }
        server.printP(html_e_td_td);
        server << trigger_symbol[trigger[i].symbol]; server.printP(html_e_td_td);
        server << trigger[i].value; server.printP(html_e_td_td);
        ((trigger[i].setting >> 6) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((trigger[i].setting >> 2) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((trigger[i].setting >> 4) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((trigger[i].setting >> 7) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);        
        ((trigger[i].setting >> 1) & B1) ? server.printP(text_value) : server.printP(text_constant); server.printP(html_e_td_td);
        if (trigger[i].to_address < RADIO_UNIT_OFFSET) { server << "W:" << trigger[i].to_address; }
        else                                           { server << "R:" << trigger[i].to_address-RADIO_UNIT_OFFSET; }
        server.printP(text_semic); server << trigger[i].to_number; server.printP(text_semic);
        switch(trigger[i].to_type){
          case 'D': server.printP(text_Digital); break;
          case 'A': server.printP(text_Analog); break;
          case 'F': server.printP(text_Float); break;
          default: server.printP(text_i_question); break;
        }
        server.printP(html_e_td_td);      
        server << trigger[i].constant_on; server.printP(html_e_td_td);
        server << trigger[i].constant_off; server.printP(html_e_td_td);
        ((trigger[i].setting >> 5) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_e_tr);
      }
      server.printP(html_e_table); 
      server.printP(html_table_tr_td);
      server.printP(text_Trigger); server.printP(html_e_td_td);
      server.printP(html_select_submit); server << "P"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < TRIGGERS; ++ii) {
        server.printP(html_option); server << ii;
        if (webTrig == ii) { server.printP(html_selected); }
        else               { server.printP(html_e_tag); }
        server << ii + 1; server.printP(html_e_option);
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Name); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << trigger[webTrig].name; server.printP(html_e_tag);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Trigger); server.printP(text_space); server.printP(text_is); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0", text_1, text_On, trigger[webTrig].setting & B1);
      server.radioButton("0", text_0, text_Off, !(trigger[webTrig].setting & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Address); server.printP(html_e_td_td);
      server.printP(html_select); server << "a"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < ALR_ZONES; ++ii) {
        // Zone is enabled
        if (conf.zone[ii] & B1) {
          server.printP(html_option); server << ii;
          if (0   == trigger[webTrig].address &&
              ii  == trigger[webTrig].number &&
              'Z' == trigger[webTrig].type) { server.printP(html_selected); }
          else                              { server.printP(html_e_tag); }
          server.printP(text_Zone); server.printP(text_semic); server.printP(text_space); server << conf.zone_name[ii];
          server.printP(text_spdashsp); server << ii + 1;
          server.printP(html_e_option);
        }
      }
      for (uint8_t ii = ALR_ZONES; ii < (ALR_ZONES + nodes); ++ii) {
        if (node[ii-ALR_ZONES].function == 'S') {
          server.printP(html_option); server << ii;
          if (node[ii-ALR_ZONES].address == trigger[webTrig].address &&
              node[ii-ALR_ZONES].number  == trigger[webTrig].number &&
              node[ii-ALR_ZONES].type    == trigger[webTrig].type) { server.printP(html_selected); }
          else                                             { server.printP(html_e_tag); }
          if (node[ii-ALR_ZONES].address < RADIO_UNIT_OFFSET) { server << "W:" << node[ii-ALR_ZONES].address; }
          else                                                  { server << "R:" << node[ii-ALR_ZONES].address-RADIO_UNIT_OFFSET; }
          server.printP(text_semic); server << node[ii-ALR_ZONES].number; server.printP(text_semic);
          switch(node[ii-ALR_ZONES].type){
            case 'T': server.printP(text_Temperature); break;
            case 'H': server.printP(text_Humidity); break;
            case 'P': server.printP(text_Pressure); break;
            case 'V': server.printP(text_Voltage); break;
            default: server.printP(text_undefined); break;
          }
          server.printP(html_e_option);
        }
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Symbol); server.printP(html_e_td_td);      
      server.printP(html_select); server << "s"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < TRIGGER_SYMBOLS; ++ii) {
        server.printP(html_option); server << ii;
        if (trigger[webTrig].symbol == ii) { server.printP(html_selected); }
        else                               { server.printP(html_e_tag); }
        server << trigger_symbol[ii]; server.printP(html_e_option);
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Value); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "v"; server.printP(html_m_tag); 
      server << trigger[webTrig].value; server.printP(html_e_tag);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Logging); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("6", text_1, text_On, (trigger[webTrig].setting >> 6) & B1);
      server.radioButton("6", text_0, text_Off, !((trigger[webTrig].setting >> 6) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Pass); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("2", text_1, text_On, (trigger[webTrig].setting >> 2) & B1);
      server.radioButton("2", text_0, text_Off, !((trigger[webTrig].setting >> 2) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Pass); server.printP(text_space); server.printP(text_once); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("4", text_1, text_On, (trigger[webTrig].setting >> 4) & B1);
      server.radioButton("4", text_0, text_Off, !((trigger[webTrig].setting >> 4) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Pass); server.printP(text_space); server.printP(text_Off); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("7", text_1, text_On, (trigger[webTrig].setting >> 7) & B1);
      server.radioButton("7", text_0, text_Off, !((trigger[webTrig].setting >> 7) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Pass); server.printP(html_e_td_td);      
      server.printP(html_radio_sl);
      server.radioButton("1", text_1, text_value, (trigger[webTrig].setting >> 1) & B1);
      server.radioButton("1", text_0, text_constant, !((trigger[webTrig].setting >> 1) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_To); server.printP(text_space); server.printP(text_address); server.printP(html_e_td_td);
      server.printP(html_select); server << "t"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < nodes; ++ii) {
        if (node[ii].function == 'I') {
          server.printP(html_option); server << ii;
          if (node[ii].address == trigger[webTrig].to_address &&
              node[ii].number  == trigger[webTrig].to_number &&
              node[ii].type    == trigger[webTrig].to_type) { server.printP(html_selected); }
          else               { server.printP(html_e_tag); }
          if (node[ii].address < RADIO_UNIT_OFFSET) { server << "W:" << node[ii].address; }
          else                                      { server << "R:" << node[ii].address-RADIO_UNIT_OFFSET; }
          server.printP(text_semic); server << node[ii].number; server.printP(text_semic);
          switch(node[ii].type){
            case 'D': server.printP(text_Digital); break;
            case 'A': server.printP(text_Analog); break;
            case 'F': server.printP(text_Float); break;
            default: server.printP(text_undefined); break;
          }
          server.printP(html_e_option);
        }
        
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Constant); server.printP(text_space); server.printP(text_On); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "c"; server.printP(html_m_tag); 
      server << trigger[webTrig].constant_on; server.printP(html_e_tag); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Constant); server.printP(text_space); server.printP(text_Off); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "f"; server.printP(html_m_tag); 
      server << trigger[webTrig].constant_off; server.printP(html_e_tag); server.printP(html_e_td_e_tr);     
      server.printP(html_e_table);
          
      server.printP(html_F_A); // submit Apply
      server.printP(html_F_SA); // submit Save all
      server.printP(html_e_form);
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
}

uint8_t webTimer = 0;
void webSetTimers(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (server.checkCredentials(conf.user_pass)) {
    char name[2], value[17];
    if (type == WebServer::POST) {
      bool repeat;
      //char name[2], value[17];
      do {
        repeat = server.readPOSTparam(name, 2, value, 17);
        switch(name[0]){
          case 'P':
            n = strtol(value, NULL, 10);
            if (n != webTimer) {
              webTimer = n;
              repeat = 0;
            }
          break;
          case 'n': // name
            strncpy (timer[webTimer].name, value, sizeof(value));
            timer[webTimer].name[15] = 0;
            break;
          case 'r': timer[webTimer].run_time = strtol(value, NULL, 10); break;
          case 'm': timer[webTimer].start_time = strtol(value, NULL, 10) + ((timer[webTimer].start_time / 60) * 60); break;
          case 'h': timer[webTimer].start_time = (strtol(value, NULL, 10) * 60) + (timer[webTimer].start_time % 60); break;
          case 'i': // Run interval
            n = value[0] - 48;
            if ((n >> 0) & 1) timer[webTimer].setting |= (1 << 14);
            else timer[webTimer].setting &= ~(1 << 14);
            if ((n >> 1) & 1) timer[webTimer].setting |= (1 << 15);
            else timer[webTimer].setting &= ~(1 << 15);
          break;
          case 'p': timer[webTimer].period = strtol(value, NULL, 10); break;
          case 'l': // Period interval
            n = value[0] - 48;
            if ((n >> 0) & 1) timer[webTimer].setting |= (1 << 12);
            else timer[webTimer].setting &= ~(1 << 12);
            if ((n >> 1) & 1) timer[webTimer].setting |= (1 << 13);
            else timer[webTimer].setting &= ~(1 << 13);
          break;

          case 'o': timer[webTimer].constant_on  = atof(value); break;
          case 'f': timer[webTimer].constant_off = atof(value); break;
          case 't': n = strtol(value, NULL, 10);
              timer[webTimer].to_address = node[n].address;
              timer[webTimer].to_type = node[n].type;
              timer[webTimer].to_number = node[n].number;
          break;
          case '0' ... '8': // Handle all single radio buttons for settings
            if (value[0] == '0') timer[webTimer].setting &= ~(1 << (name[0]-48));
            else                 timer[webTimer].setting |=  (1 << (name[0]-48));
          break;
          case 'e': saveConf(); break; // save to EEPROM
        }
      } while (repeat);
      set_timer(webTimer); // At least set timer
      server.httpSeeOther(PREFIX "/i");
    } else {
      server.httpSuccess();
      webMenu(server, menu_Timers);
      server.printP(html_h1); server.printP(text_Timer); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  
      server.printP(html_form_s); server << PREFIX "/i"; server.printP(html_form_e);    
      server.printP(html_table_tr_th); server.printP(text_hash);   
      server.printP(html_e_th_th); server.printP(text_Name);
      server.printP(html_e_th_th); server.printP(text_On);
      server.printP(html_e_th_th); server.printP(text_Type);
      server.printP(html_e_th_th); server.printP(text_Mo);
      server.printP(html_e_th_th); server.printP(text_Tu);
      server.printP(html_e_th_th); server.printP(text_We);
      server.printP(html_e_th_th); server.printP(text_Th);
      server.printP(html_e_th_th); server.printP(text_Fr);
      server.printP(html_e_th_th); server.printP(text_Sa);
      server.printP(html_e_th_th); server.printP(text_Su);
      server.printP(html_e_th_th); server.printP(text_Time); 
      server.printP(html_e_th_th); server.printP(text_Period); 
      server.printP(html_e_th_th); server.printP(text_Run); server.printP(text_space); server.printP(text_time);
      server.printP(html_e_th_th); server.printP(text_On); server.printP(text_space); server.printP(text_time);
      server.printP(html_e_th_th); server.printP(text_Off); server.printP(text_space); server.printP(text_time);      
      server.printP(html_e_th_th); server.printP(text_To); server.printP(text_space); server.printP(text_address);
      server.printP(html_e_th_th); server.printP(text_On);
      server.printP(html_e_th_th); server.printP(text_Off);
      server.printP(html_e_th_th); server.printP(text_Trigger); server.printP(text_ed);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < TIMERS; ++i) {
        server.printP(html_tr_td); 
        server << i+1; server.printP(text_dot); server.printP(html_e_td_td);
        server << timer[i].name; server.printP(html_e_td_td);
        (timer[i].setting & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((timer[i].setting >> 1) & B1) ? server.printP(text_Calendar) : server.printP(text_Period); server.printP(html_e_td_td);
        ((timer[i].setting >> 8) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((timer[i].setting >> 7) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((timer[i].setting >> 6) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((timer[i].setting >> 5) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((timer[i].setting >> 4) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((timer[i].setting >> 3) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((timer[i].setting >> 2) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        server << (timer[i].start_time / 60); server.printP(text_sesp);
        server << (timer[i].start_time % 60); server.printP(html_e_td_td);
        server << timer[i].period; server.printP(text_space); 
        switch(timer[i].setting >> 12 & B11){
          case 0:  server.printP(text_seconds); break;
          case 1:  server.printP(text_minutes); break;
          case 2:  server.printP(text_hours); break;
          default: server.printP(text_days); break;
        }
        server.printP(html_e_td_td);
        server << timer[i].run_time ; server.printP(text_space);
        switch(timer[i].setting >> 14 & B11){
          case 0:  server.printP(text_seconds); break;
          case 1:  server.printP(text_minutes); break;
          case 2:  server.printP(text_hours); break;
          default: server.printP(text_days); break;
        }
        server.printP(html_e_td_td);
        time_temp = timer[i].next_on; server.print((char*)time_temp.formatedDateTime()); server.printP(html_e_td_td);
        time_temp = timer[i].next_off; server.print((char*)time_temp.formatedDateTime()); server.printP(html_e_td_td);
        if (timer[i].to_address < RADIO_UNIT_OFFSET) { server << "W:" << timer[i].to_address; }
        else                                         { server << "R:" << timer[i].to_address-RADIO_UNIT_OFFSET; }
        server.printP(text_semic); server << timer[i].to_number; server.printP(text_semic);
        switch(timer[i].to_type){
          case 'D': server.printP(text_Digital); break;
          case 'A': server.printP(text_Analog); break;
          case 'F': server.printP(text_Float); break;
          default: server.printP(text_i_question); break;
        }
        server.printP(html_e_td_td);      
        server << timer[i].constant_on; server.printP(html_e_td_td);
        server << timer[i].constant_off; server.printP(html_e_td_td);
        ((timer[i].setting >> 9) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_e_tr);
      }
      server.printP(html_e_table); 
      server.printP(html_table_tr_td);
      server.printP(text_Timer); server.printP(html_e_td_td);
      server.printP(html_select_submit); server << "P"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < TRIGGERS; ++ii) {
        server.printP(html_option); server << ii;
        if (webTimer == ii) { server.printP(html_selected); }
        else               { server.printP(html_e_tag); }
        server << ii + 1; server.printP(html_e_option);
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Name); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << timer[webTimer].name; server.printP(html_e_tag);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Timer); server.printP(text_space); server.printP(text_is); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0", text_1, text_On, timer[webTimer].setting & B1);
      server.radioButton("0", text_0, text_Off, !(timer[webTimer].setting & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Type); server.printP(html_e_td_td);      
      server.printP(html_radio_sl);
      server.radioButton("1", text_1, text_Calendar, (timer[webTimer].setting >> 1) & B1);
      server.radioButton("1", text_0, text_Period, !((timer[webTimer].setting >> 1) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Monday); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("8", text_1, text_On, (timer[webTimer].setting >> 8) & B1);
      server.radioButton("8", text_0, text_Off, !((timer[webTimer].setting >> 8) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Tuesday); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("7", text_1, text_On, (timer[webTimer].setting >> 7) & B1);
      server.radioButton("7", text_0, text_Off, !((timer[webTimer].setting >> 7) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Wednesday); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("6", text_1, text_On, (timer[webTimer].setting >> 6) & B1);
      server.radioButton("6", text_0, text_Off, !((timer[webTimer].setting >> 6) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Thursday); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("5", text_1, text_On, (timer[webTimer].setting >> 5) & B1);
      server.radioButton("5", text_0, text_Off, !((timer[webTimer].setting >> 5) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Friday); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("4", text_1, text_On, (timer[webTimer].setting >> 4) & B1);
      server.radioButton("4", text_0, text_Off, !((timer[webTimer].setting >> 4) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Saturday); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("3", text_1, text_On, (timer[webTimer].setting >> 3) & B1);
      server.radioButton("3", text_0, text_Off, !((timer[webTimer].setting >> 3) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Sunday); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("2", text_1, text_On, (timer[webTimer].setting >> 2) & B1);
      server.radioButton("2", text_0, text_Off, !((timer[webTimer].setting >> 2) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 

      server.printP(text_Start); server.printP(text_space); server.printP(text_time); server.printP(html_e_td_td);
      server.printP(html_select); server << "h"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < 24; ++ii) {
        if ((timer[webTimer].start_time / 60) == ii)  
          { server.printP(html_option); server << ii; server.printP(html_selected); server << ii; server.printP(html_e_option); }
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii; server.printP(html_e_option); }
      }
      server.printP(html_e_select); server.printP(text_sesp);
      server.printP(html_select); server << "m"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < 60; ++ii) {
        if ((timer[webTimer].start_time % 60) == ii)  
          { server.printP(html_option); server << ii; server.printP(html_selected); server << ii; server.printP(html_e_option); }
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii; server.printP(html_e_option); }
      }
      server.printP(html_e_select);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Period); server.printP(text_space); server.printP(text_time); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "p"; server.printP(html_m_tag); 
      server << timer[webTimer].period; server.printP(html_e_tag); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Period); server.printP(text_space); server.printP(text_interval); server.printP(html_e_td_td);
      server.printP(html_radio_sb);
      server.radioButton("l", text_0, text_seconds, !((timer[webTimer].setting >> 13 & B1) & (timer[webTimer].setting >> 12 & B1)));
      server.radioButton("l", text_1, text_minutes, (!(timer[webTimer].setting >> 13 & B1) & (timer[webTimer].setting >> 12 & B1)));
      server.radioButton("l", text_2, text_hours, ((timer[webTimer].setting >> 13 & B1) & !(timer[webTimer].setting >> 12 & B1)));
      server.radioButton("l", text_3, text_days, ((timer[webTimer].setting >> 13 & B1) & (timer[webTimer].setting >> 12 & B1)));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Run); server.printP(text_space); server.printP(text_time); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "r"; server.printP(html_m_tag); 
      server << timer[webTimer].run_time; server.printP(html_e_tag); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Run); server.printP(text_space); server.printP(text_interval); server.printP(html_e_td_td);
      server.printP(html_radio_sb);
      server.radioButton("i", text_0, text_seconds, !((timer[webTimer].setting >> 15 & B1) & (timer[webTimer].setting >> 14 & B1)));
      server.radioButton("i", text_1, text_minutes, (!(timer[webTimer].setting >> 15 & B1) & (timer[webTimer].setting >> 14 & B1)));
      server.radioButton("i", text_2, text_hours, ((timer[webTimer].setting >> 15 & B1) & !(timer[webTimer].setting >> 14 & B1)));
      server.radioButton("i", text_3, text_days, ((timer[webTimer].setting >> 15 & B1) & (timer[webTimer].setting >> 14 & B1)));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);

      server.printP(text_To); server.printP(text_space); server.printP(text_address); server.printP(html_e_td_td);
      server.printP(html_select); server << "t"; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < nodes; ++ii) {
        if (node[ii].function == 'I') {
          server.printP(html_option); server << ii;
          if (node[ii].address == timer[webTimer].to_address &&
              node[ii].number  == timer[webTimer].to_number &&
              node[ii].type    == timer[webTimer].to_type) { server.printP(html_selected); }
          else               { server.printP(html_e_tag); }
          if (node[ii].address < RADIO_UNIT_OFFSET) { server << "W:" << node[ii].address; }
          else                                      { server << "R:" << node[ii].address-RADIO_UNIT_OFFSET; }
          server.printP(text_semic); server << node[ii].number; server.printP(text_semic);
          switch(node[ii].type){
            case 'D': server.printP(text_Digital); break;
            case 'A': server.printP(text_Analog); break;
            case 'F': server.printP(text_Float); break;
            default: server.printP(text_undefined); break;
          }
          server.printP(html_e_option);
        }
        
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Constant); server.printP(text_space); server.printP(text_On); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "o"; server.printP(html_m_tag); 
      server << timer[webTimer].constant_on; server.printP(html_e_tag); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Constant); server.printP(text_space); server.printP(text_Off); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "f"; server.printP(html_m_tag); 
      server << timer[webTimer].constant_off; server.printP(html_e_tag); server.printP(html_e_td_e_tr);     
      server.printP(html_e_table);
          
      server.printP(html_F_A); // submit Apply
      server.printP(html_F_SA); // submit Save all
      server.printP(html_e_form);
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
}

void webSetGlobal(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (server.checkCredentials(conf.user_pass)) {
    char name[3], value[31];
    if (type == WebServer::POST) {
      bool repeat;
      do {
        repeat = server.readPOSTparam(name, 3, value, 31);
        switch(name[0]){
          case 'a': conf.alr_time = strtol(value, NULL, 10); break;
          case 'b': conf.auto_arm = strtol(value, NULL, 10); break;
          case 'c': conf.open_alarm = strtol(value, NULL, 10);  break;
          case 'd': conf.arm_delay = strtol(value, NULL, 10) * 4;  break;
          case 'e': saveConf(); break; // save to EEPROM
          case 'k': strncpy ((char*)conf.radioKey, value, 16); // key
                    conf.radioKey[16] = 0; // extra byte for null
                    break;
          case 'u': strncpy (conf.user, value, 10);// user
                    conf.user[11] = 0;
                    break;
          case 'p': // password
            strncpy (conf.password, value, 10);
            conf.password[11] = 0;
            b64_text[0] = 0;
            strcat (b64_text, conf.user);
            strcat (b64_text, ":");
            strcat (b64_text, conf.password);
            base64_encode(conf.user_pass, b64_text, strlen(b64_text));
            //WS.print("p>"); WS.println(_text);
            //WS.println(strlen(conf.user_pass));
            //WS.println(conf.user_pass);
          break;
          case 'i': // user
            strncpy (conf.SMTP_user, value, 30);
            conf.SMTP_user[31] = 0;
          break;
          case 'o': // password
            strncpy (conf.SMTP_password, value, 15);
            conf.SMTP_password[16] = 0;
          break;
          case '0' ... '2': // Handle all radio buttons in groups 0 .. 2, A .. P
            if (value[0] == '0') conf.alerts[name[0]-48] &= ~(1 << (name[1]-65));
            else conf.alerts[name[0]-48] |= (1 << (name[1]-65));
          break;
        }
      } while (repeat);
      server.httpSeeOther(PREFIX "/r");
    } else {
      server.httpSuccess();
      webMenu(server, menu_Global);
      server.printP(html_h1); server.printP(text_Global); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  
      server.printP(html_form_s); server << PREFIX "/r"; server.printP(html_form_e);    
      server.printP(html_table_tr_td);
      server.printP(text_User); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "u"; server.printP(html_m_tag); server << conf.user; server.printP(html_e_tag);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Password); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "p"; server.printP(html_m_tag); server << conf.password; server.printP(html_e_tag);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_User); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "i"; server.printP(html_m_tag); server << conf.SMTP_user; server.printP(html_e_tag);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Password); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "o"; server.printP(html_m_tag); server << conf.SMTP_password; server.printP(html_e_tag);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Authentication); server.printP(text_space); server.printP(text_time);server.printP(html_e_td_td);
      server.printP(html_select); server << "a"; server.printP(html_e_tag);
      for (uint8_t ii = 5; ii < 26; ++ii) {
        if ((conf.alr_time) == ii)  
          { server.printP(html_option); server << ii; server.printP(html_selected); server << ii; server.printP(html_e_option); }
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii; server.printP(html_e_option); }
      }
      server.printP(html_e_select);
      server.printP(text_space); server.printP(text_seconds); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Arm); server.printP(text_space); server.printP(text_delay); server.printP(html_e_td_td);
      server.printP(html_select); server << "d"; server.printP(html_e_tag);
      for (uint8_t ii = 10; ii < 41; ++ii) {
        if ((conf.arm_delay/4) == ii) // 250*4 = 1 sec.
          { server.printP(html_option); server << ii; server.printP(html_selected); server << ii; server.printP(html_e_option); }
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii; server.printP(html_e_option); }
      }
      server.printP(html_e_select);
      server.printP(text_space); server.printP(text_seconds); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Auto); server.printP(text_space); server.printP(text_arm); server.printP(text_space); server.printP(text_delay); server.printP(html_e_td_td);
      server.printP(html_select); server << "b"; server.printP(html_e_tag);
      for (uint8_t ii = 10; ii < 241; ii+=10) {
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
      server.printP(text_space); server.printP(text_minutes); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Open); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td_td);
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

      server.printP(html_h1); server.printP(text_Radio); server.printP(text_space); server.printP(text_setup); server.printP(html_e_h1);  
      server.printP(html_table_tr_td);
      server.printP(text_Radio); server.printP(text_space); server.printP(text_key); server.printP(html_e_td_td);
      server.printP(html_s_tag); server << "k"; server.printP(html_m_tag); server << (char*)conf.radioKey; server.printP(html_e_tag);
      server.printP(html_e_table);

      server.printP(html_h1); server.printP(text_Logging); server.printP(html_e_h1);  
      server.printP(html_table_tr_th);
      server.printP(text_Name); server.printP(html_e_th_th);
      server.printP(text_SMS); server.printP(html_e_th_th);
      server.printP(text_Email); server.printP(html_e_th_th);
      server.printP(text_Last); server.printP(html_e_th_e_tr);
      server.printP(html_tr_td);
      server.printP(text_Undefined); server.printP(text_space); server.printP(text_key); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0A", text_1, text_On, conf.alerts[alert_SMS] & B1);
      server.radioButton("0A", text_0, text_Off, !(conf.alerts[alert_SMS] & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("1A", text_1, text_On, conf.alerts[1] & B1);
      server.radioButton("1A", text_0, text_Off, !(conf.alerts[1] & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("2A", text_1, text_On, conf.alerts[2] & B1);
      server.radioButton("2A", text_0, text_Off, !(conf.alerts[2] & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_System); server.printP(text_space); server.printP(text_disarmed); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0B", text_1, text_On, conf.alerts[alert_SMS] >> 1 & B1);
      server.radioButton("0B", text_0, text_Off, !(conf.alerts[alert_SMS] >> 1 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("1B", text_1, text_On, conf.alerts[1] >> 1 & B1);
      server.radioButton("1B", text_0, text_Off, !(conf.alerts[1] >> 1 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("2B", text_1, text_On, conf.alerts[2] >> 1 & B1);
      server.radioButton("2B", text_0, text_Off, !(conf.alerts[2] >> 1 & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_System); server.printP(text_space); server.printP(text_armed); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0C", text_1, text_On, conf.alerts[alert_SMS] >> 2 & B1);
      server.radioButton("0C", text_0, text_Off, !(conf.alerts[alert_SMS] >> 2 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("1C", text_1, text_On, conf.alerts[1] >> 2 & B1);
      server.radioButton("1C", text_0, text_Off, !(conf.alerts[1] >> 2 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("2C", text_1, text_On, conf.alerts[2] >> 2 & B1);
      server.radioButton("2C", text_0, text_Off, !(conf.alerts[2] >> 2 & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Open); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0M", text_1, text_On, conf.alerts[alert_SMS] >> 12 & B1);
      server.radioButton("0M", text_0, text_Off, !(conf.alerts[alert_SMS] >> 12 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("1M", text_1, text_On, conf.alerts[1] >> 12 & B1);
      server.radioButton("1M", text_0, text_Off, !(conf.alerts[1] >> 12 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("2M", text_1, text_On, conf.alerts[2] >> 12 & B1);
      server.radioButton("2M", text_0, text_Off, !(conf.alerts[2] >> 12 & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Tamper); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0E", text_1, text_On, conf.alerts[alert_SMS] >> 4 & B1);
      server.radioButton("0E", text_0, text_Off, !(conf.alerts[alert_SMS] >> 4 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("1E", text_1, text_On, conf.alerts[1] >> 4 & B1);
      server.radioButton("1E", text_0, text_Off, !(conf.alerts[1] >> 4 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("2E", text_1, text_On, conf.alerts[2] >> 4 & B1);
      server.radioButton("2E", text_0, text_Off, !(conf.alerts[2] >> 4 & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_node); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0F", text_1, text_On, conf.alerts[alert_SMS] >> 5 & B1);
      server.radioButton("0F", text_0, text_Off, !(conf.alerts[alert_SMS] >> 5 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("1F", text_1, text_On, conf.alerts[1] >> 5 & B1);
      server.radioButton("1F", text_0, text_Off, !(conf.alerts[1] >> 5 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("2F", text_1, text_On, conf.alerts[2] >> 5 & B1);
      server.radioButton("2F", text_0, text_Off, !(conf.alerts[2] >> 5 & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_System); server.printP(text_space); server.printP(text_alarm); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0G", text_1, text_On, conf.alerts[alert_SMS] >> 6 & B1);
      server.radioButton("0G", text_0, text_Off, !(conf.alerts[alert_SMS] >> 6 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("1G", text_1, text_On, conf.alerts[1] >> 6 & B1);
      server.radioButton("1G", text_0, text_Off, !(conf.alerts[1] >> 6 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("2G", text_1, text_On, conf.alerts[2] >> 6 & B1);
      server.radioButton("2G", text_0, text_Off, !(conf.alerts[2] >> 6 & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Monitoring); server.printP(text_space); server.printP(text_started); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0H", text_1, text_On, conf.alerts[alert_SMS] >> 7 & B1);
      server.radioButton("0H", text_0, text_Off, !(conf.alerts[alert_SMS] >> 7 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("1H", text_1, text_On, conf.alerts[1] >> 7 & B1);
      server.radioButton("1H", text_0, text_Off, !(conf.alerts[1] >> 7 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("2H", text_1, text_On, conf.alerts[2] >> 7 & B1);
      server.radioButton("2H", text_0, text_Off, !(conf.alerts[2] >> 7 & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_System); server.printP(text_space); server.printP(text_group); server.printP(text_space); server.printP(text_armed); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0I", text_1, text_On, conf.alerts[alert_SMS] >> 8 & B1);
      server.radioButton("0I", text_0, text_Off, !(conf.alerts[alert_SMS] >> 8 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("1I", text_1, text_On, conf.alerts[1] >> 8 & B1);
      server.radioButton("1I", text_0, text_Off, !(conf.alerts[1] >> 8 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("2I", text_1, text_On, conf.alerts[2] >> 8 & B1);
      server.radioButton("2I", text_0, text_Off, !(conf.alerts[2] >> 8 & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Configuration); server.printP(text_space); server.printP(text_saved); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0J", text_1, text_On, conf.alerts[alert_SMS] >> 9 & B1);
      server.radioButton("0J", text_0, text_Off, !(conf.alerts[alert_SMS] >> 9 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("1J", text_1, text_On, conf.alerts[1] >> 9 & B1);
      server.radioButton("1J", text_0, text_Off, !(conf.alerts[1] >> 9 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("2J", text_1, text_On, conf.alerts[2] >> 9 & B1);
      server.radioButton("2J", text_0, text_Off, !(conf.alerts[2] >> 9 & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Power); server.printP(text_space); server.printP(text_state); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0K", text_1, text_On, conf.alerts[alert_SMS] >> 10 & B1);
      server.radioButton("0K", text_0, text_Off, !(conf.alerts[alert_SMS] >> 10 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("1K", text_1, text_On, conf.alerts[1] >> 10 & B1);
      server.radioButton("1K", text_0, text_Off, !(conf.alerts[1] >> 10 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("2K", text_1, text_On, conf.alerts[2] >> 10 & B1);
      server.radioButton("2K", text_0, text_Off, !(conf.alerts[2] >> 10 & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Battery); server.printP(text_space); server.printP(text_state); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton("0L", text_1, text_On, conf.alerts[alert_SMS] >> 11 & B1);
      server.radioButton("0L", text_0, text_Off, !(conf.alerts[alert_SMS] >> 11 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("1L", text_1, text_On, conf.alerts[1] >> 11 & B1);
      server.radioButton("1L", text_0, text_Off, !(conf.alerts[1] >> 11 & B1));
      server.printP(html_div_e_e_td_td_radio_s);
      server.radioButton("2L", text_1, text_On, conf.alerts[2] >> 11 & B1);
      server.radioButton("2L", text_0, text_Off, !(conf.alerts[2] >> 11 & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr);
      server.printP(html_e_table);
      
      server.printP(html_F_A); // submit Apply
      server.printP(html_F_SA); // submit Save all
      server.printP(html_e_form);
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
}

#endif