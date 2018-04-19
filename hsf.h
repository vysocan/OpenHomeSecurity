#ifndef HSF_h
#define HSF_h

// *********************************************************************************
// W E B   P A G E S              W E B   P A G E S                W E B   P A G E S  
// *********************************************************************************

const char PGM_Last[]   PROGMEM = "Last";
const char PGM_First[]  PROGMEM = "First";
const char PGM_Second[] PROGMEM = "Second";
const char PGM_Third[]  PROGMEM = "Third";
const char PGM_Four[]   PROGMEM = "Fourth";
const char* const PGM_week[] PROGMEM = {
  PGM_Last,
  PGM_First,
  PGM_Second,
  PGM_Third,
  PGM_Four
  };

const char PGM_Sunday[]    PROGMEM = "Sunday";
const char PGM_Monday[]    PROGMEM = "Monday";
const char PGM_Tuesday[]   PROGMEM = "Tuesday";
const char PGM_Wednesday[] PROGMEM = "Wednesday";
const char PGM_Thursday[]  PROGMEM = "Thursday";
const char PGM_Friday[]    PROGMEM = "Friday";
const char PGM_Saturday[]  PROGMEM = "Saturday";
const char* const PGM_dow[] PROGMEM = {
  PGM_Sunday,
  PGM_Monday,
  PGM_Tuesday,
  PGM_Wednesday,
  PGM_Thursday,
  PGM_Friday,
  PGM_Saturday  
  };

const char PGM_Jan[] PROGMEM = "Jan";
const char PGM_Feb[] PROGMEM = "Feb";
const char PGM_Mar[] PROGMEM = "Mar";
const char PGM_Apr[] PROGMEM = "Apr";
const char PGM_May[] PROGMEM = "May";
const char PGM_Jun[] PROGMEM = "Jun";
const char PGM_Jul[] PROGMEM = "Jul";
const char PGM_Aug[] PROGMEM = "Aug";
const char PGM_Sep[] PROGMEM = "Sep";
const char PGM_Oct[] PROGMEM = "Oct";
const char PGM_Nov[] PROGMEM = "Nov";
const char PGM_Dec[] PROGMEM = "Dec";
const char* const PGM_month[] PROGMEM = {
  PGM_Jan, PGM_Feb, PGM_Mar, PGM_Apr, PGM_May, PGM_Jun,
  PGM_Jul, PGM_Aug, PGM_Sep, PGM_Oct, PGM_Nov, PGM_Dec };

void flashPrintln(WebServer &server, const char* const* adress)
{
 server.print(reinterpret_cast<const __FlashStringHelper *> pgm_read_word(adress));
}

void printPeriodType(WebServer &server, uint8_t _type) {
  switch(_type){
    case 0:  server.printP(text_seconds); break;
    case 1:  server.printP(text_minutes); break;
    case 2:  server.printP(text_hours); break;
    default: server.printP(text_days); break;
  }
}

void printNodeType(WebServer &server, char _type) {
  switch(_type){
    case 'i': server.printP(text_iButton); break;
    case 'T': server.printP(text_Temperature); break;
    case 'H': server.printP(text_Humidity); break;
    case 'P': server.printP(text_Pressure); break;
    case 'V': server.printP(text_Voltage); break;
    case 'B': server.printP(text_Battery); break;
    case 'D': server.printP(text_Digital); break;
    case 'A': server.printP(text_Analog); break;
    case 'F': server.printP(text_Float); break;
    case 'X': server.printP(text_TX_Power); break;
    case 'G': server.printP(text_Gas); break;
    default : server.printP(text_Undefined); break;
  }
}

void printNodeFunction(WebServer &server, char _function) {
  switch(_function){
    case 'K': server.printP(text_Authentication); break;
    case 'S': server.printP(text_Sensor); break;
    case 'I': server.printP(text_Input); break;
    default: server.printP(text_Undefined); break;
  }
}

void selectGroup(WebServer &server, uint8_t _setting, char _name = 'g') {
  server.printP(html_select); server.print(_name); server.printP(html_e_tag);
  for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
    server.printP(html_option); server << ii;
    if ((_setting & B1111) == ii) { server.printP(html_selected); }
    else                              { server.printP(html_e_tag); }
    server << ii + 1; server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(text_spdashsp);
    (conf.group[ii] & B1) ? server.printP(text_On) : server.printP(text_Off); server.printP(html_e_option);
  }
  server.printP(html_e_select);
}

void printNodeAddress(WebServer &server, uint8_t node_num){
  server << conf.group_name[((node[node_num].setting >> 1) & B1111)]; server.printP(text_spdashsp);
  server << node[node_num].name; server.printP(text_spdashsp);
  if (node[node_num].address < RADIO_UNIT_OFFSET) { server << "W:" << node[node_num].address; }
  else                                            { server << "R:" << node[node_num].address-RADIO_UNIT_OFFSET; }
  server.print(':'); server << node[node_num].number;
  
}

void printNodeAddress(WebServer &server, char function, uint8_t address, uint8_t number, char type){
  uint8_t _found = 0, _node;
  for (_node = 0; _node < nodes; _node++) {
    if (node[_node].address  == address &&
        node[_node].function == function &&
        node[_node].number   == number &&
        node[_node].setting & B1) {
      _found = 1;
      break;
    }
  }
  if (_found) {
    printNodeAddress(server, _node);
    server.print(':'); printNodeType(server, type);
  }
}

void printOnOffButton(WebServer &server, char *name, uint8_t state) {
  server.printP(html_radio_s);
  server.radioButton2(name, 1, text_On, state);
  server.radioButton2(name, 0, text_Off, !state);
  server.printP(html_div_e);
}

void printKey(WebServer &server, char* in) { // Format the key value to nice HEX string
  uint8_t _in1, _in2;
  for (uint8_t ii = 0; ii < KEY_LEN; ++ii) { 
    _in1 = ((in[ii] >> 4) & B1111);
    _in2 =  (in[ii] & B1111);
    if (_in1 > 9) server.print(char(55+_in1));  // A = 65 - 10
    else          server.print(_in1);           // rest
    if (_in2 > 9) server.print(char(55+_in2));  // A = 65 - 10
    else          server.print(_in2);           // rest
  }
}
/*
void printKey(char* in, char* out) { // Format the key value to nice HEX string
  for (uint8_t ii = 0; ii < KEY_LEN; ++ii) { 
    out[ii*2]   = in[ii] >> 4 & B1111;
    out[ii*2+1] = in[ii] & B1111;
    if (out[ii*2] > 9) out[ii*2] = out[ii*2] + 'A' - 10; 
    else out[ii*2] = out[ii*2] + '0';
    if (out[ii*2+1] > 9) out[ii*2+1] = out[ii*2+1] + 'A' - 10;
    else out[ii*2+1] = out[ii*2+1] + '0';
  }
  out[16] = 0;
}*/

template<typename T>
void printInput(WebServer &server, char name, T value, uint8_t type = 1){
  if (type) server.printP(html_s_tag);  
  else      server.printP(html_s_tag_s);
  server << name;
  server.printP(html_m_tag); server << value;
  server.printP(html_id_tag); server << name;
  server.printP(html_e_tag);
}

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

void webMenu(WebServer &server, uint8_t selected, bool enableJS = false) {
  // Enable JavaScript
  server.printP(htmlHead_s);
  switch(selected){
    case menu_Timers:
      if (enableJS) server.printP(JSen);
      else          server.printP(JSdis);
    break;
    default : break;
  }
  server.printP(htmlHead_m);

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


#ifdef WEB_DEBUGGING 
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
      char name[2], value[17];
      uint16_t _ee_pos;
      do {
        repeat = server.readPOSTparam(name, 2, value, 17);
        switch(name[0]){
          /*
          case 't': // Browser Time Sync
            time_temp.set(strtol(value, NULL, 10));
            if (time_temp.get() > 0) {
              //WS.println(time_temp.get());
              time_temp.set(time_temp.get()-NTP_SECS_YR_1970_2000+(conf.time_std_offset * SECS_PER_MIN));
              //WS.println(time_temp.get());
              //WS.println(timestamp.get());
              time_dst = CalculateDST(time_temp.year(), conf.time_dst_week, conf.time_dst_dow,
                                      conf.time_dst_month, conf.time_dst_hour);
              time_std = CalculateDST(time_temp.year(), conf.time_std_week, conf.time_std_dow,
                                      conf.time_std_month, conf.time_std_hour);
              if ((time_temp.get() >= time_dst.get()) && (time_temp.get() <= time_std.get())) {
                conf.setting |= (1 << 1); // switch ON DST flag
                time_temp.set(time_temp.get() + ((conf.time_dst_offset - conf.time_std_offset) * SECS_PER_MIN));
              } else {
                conf.setting &= ~(1 << 1); // switch OFF DST flag
              }

              RTC.adjust(time_temp.get());
              timestamp = time_temp;
              // Re-set timers 
              for (int8_t i=0; i < TIMERS ; i++){
                //  timer enabled
                if (timer[i].setting & B1) set_timer(i);
              }
            }
          break;
          */
          case 'T': // NTP Sync
            time_temp = getNTPTime(udp);  
            if (time_temp.get() > 0) {
              RTC.adjust(time_temp.get());
              timestamp = time_temp;
              // Re-set timers 
              for (int8_t i=0; i < TIMERS ; i++){
                //  timer enabled
                if (timer[i].setting & B1) setTimer(i);
              }
            }
          break;
          case 'k': strncpy((char*)conf.radioKey, value, 16); // key
          conf.radioKey[16] = 0; // extra byte for null
          break;
          case 'e': saveConf(); break; // save to EEPROM
          case 'l': // load from EEPROM
            pushToLog("SCL"); // System configuration loaded
            _ee_pos = conf.ee_pos; // Save log position
            eeprom_read_block((void*)&conf, (void*)0, sizeof(conf)); // Read current configuration
            conf.ee_pos = _ee_pos;
          break; 
          case 'r': // reset ro default
            pushToLog("SCR"); // System configuration reset
            setDefault();
          break;
          case '0' ... '7': // Handle all single radio buttons for setting
            if (value[0] == '0') conf.setting &= ~(1 << (name[0]-48));
            else                 conf.setting |=  (1 << (name[0]-48));
          break;
          case 'z': conf.time_dst_week   = strtol(value, NULL, 10); break;
          case 'x': conf.time_dst_dow    = strtol(value, NULL, 10); break;
          case 'c': conf.time_dst_month  = strtol(value, NULL, 10); break;
          case 'v': conf.time_dst_hour   = strtol(value, NULL, 10); break;
          case 'b': conf.time_dst_offset = strtol(value, NULL, 10); break;
          case 'a': conf.time_std_week   = strtol(value, NULL, 10); break;
          case 's': conf.time_std_dow    = strtol(value, NULL, 10); break;
          case 'd': conf.time_std_month  = strtol(value, NULL, 10); break;
          case 'f': conf.time_std_hour   = strtol(value, NULL, 10); break;
          case 'g': conf.time_std_offset = strtol(value, NULL, 10); break;
          case 'H' ... 'K': conf.ntp_ip[name[0]-72] = strtol(value, NULL, 10); break;
          /*
          case 'H': conf.ntp_ip[0] = strtol(value, NULL, 10); break;
          case 'I': conf.ntp_ip[1] = strtol(value, NULL, 10); break;
          case 'J': conf.ntp_ip[2] = strtol(value, NULL, 10); break;
          case 'K': conf.ntp_ip[3] = strtol(value, NULL, 10); break;    
          */
          }
      } while (repeat);
      server.httpSeeOther(PREFIX "/");
    } else {   
      server.httpSuccess();
      webMenu(server, menu_Home);
      server.printP(html_form_s); server << PREFIX "/"; server.printP(html_form_e);
      server.printP(html_h1); server.printP(text_System); server.printP(html_e_h1);  
      server.printP(html_table_tr_td); 
      server.printP(text_Time); server.printP(html_e_td_td);
      server.print((char*)timestamp.formatedDateTime());
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Start); server.printP(text_ed); server.printP(html_e_td_td);
      server.print((char*)time_started.formatedDateTime());        
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Uptime); server.printP(html_e_td_td);
      time_temp.set(timestamp.get()-time_started.get());
      server.print((char*)time_temp.formatedUpTime());
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_PwrSp); server.printP(html_e_td_td);
      pinAC_OFF.read() ? server.printP(text_i_disabled) : server.printP(text_i_OK); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Battery); server.printP(html_e_td_td);
      pinBAT_OK.read() ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_e_tr_tr_td);
  
      server.printP(text_DS); server.print(' '); server.printP(text_flag); server.printP(html_e_td_td);
      ((conf.setting >> 1) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_e_tr_tr_td);      
      server.printP(text_DS); server.print(' '); server.printP(text_start); server.printP(html_e_td_td);
      server.printP(html_select); server << 'z'; server.printP(html_e_tag);
      for (int8_t ii = 0; ii < 5; ++ii) {
        if ((conf.time_dst_week) == ii)  
          { server.printP(html_option); server << ii; server.printP(html_selected);}
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag);}
        flashPrintln(server, &PGM_week[ii]); server.printP(html_e_option);
      }
      server.printP(html_e_select);
      server.print(' ');
      server.printP(html_select); server << 'x'; server.printP(html_e_tag);
      for (int8_t ii = 0; ii < 7; ++ii) {
        if ((conf.time_dst_dow) == ii)  
          { server.printP(html_option); server << ii; server.printP(html_selected);}
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag);}
        flashPrintln(server, &PGM_dow[ii]); server.printP(html_e_option);
      }
      server.printP(html_e_select);
      server.print(' '); server.printP(text_of); server.print(' ');
      server.printP(html_select); server << 'c'; server.printP(html_e_tag);
      for (int8_t ii = 0; ii < 12; ++ii) {
        if ((conf.time_dst_month-1) == ii)  
          { server.printP(html_option); server << ii+1; server.printP(html_selected);}
        else 
          { server.printP(html_option); server << ii+1; server.printP(html_e_tag);}
        flashPrintln(server, &PGM_month[ii]); server.printP(html_e_option);
      }
      server.printP(html_e_select);
      server.print(' '); server.printP(text_at); server.print(' ');
      server.printP(html_select); server << 'v'; server.printP(html_e_tag);
      for (int8_t ii = 0; ii < 24; ++ii) {
        if ((conf.time_dst_hour) == ii)  
          { server.printP(html_option); server << ii; server.printP(html_selected);}
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag);}
        server << ii; server.printP(html_e_option);
      }
      server.printP(html_e_select);
      server.printP(text_cosp); server.printP(text_offset); server.print(' ');
      //server.printP(html_s_tag_s); server << 'b'; server.printP(html_m_tag); 
      //server << conf.time_dst_offset; server.printP(html_e_tag);
      printInput(server, 'b', conf.time_dst_offset, 0);
      server.print(' '); server.printP(text_minutes); 
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(html_e_td_td); 
      server.print(time_dst.formatedDateTime());
      server.printP(html_e_td_e_tr_tr_td);
      
      server.printP(text_DS); server.print(' '); server.printP(text_end); server.printP(html_e_td_td);
      server.printP(html_select); server << 'a'; server.printP(html_e_tag);
      for (int8_t ii = 0; ii < 5; ++ii) {
        if ((conf.time_std_week) == ii)  
          { server.printP(html_option); server << ii; server.printP(html_selected);}
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag);}
        flashPrintln(server, &PGM_week[ii]); server.printP(html_e_option);
      }
      server.printP(html_e_select);
      server.print(' ');
      server.printP(html_select); server << 's'; server.printP(html_e_tag);
      for (int8_t ii = 0; ii < 7; ++ii) {
        if ((conf.time_std_dow) == ii)  
          { server.printP(html_option); server << ii; server.printP(html_selected);}
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag);}
        flashPrintln(server, &PGM_dow[ii]); server.printP(html_e_option);
      }
      server.printP(html_e_select);
      server.print(' '); server.printP(text_of); server.print(' ');
      server.printP(html_select); server << 'd'; server.printP(html_e_tag);
      for (int8_t ii = 0; ii < 12; ++ii) {
        if ((conf.time_std_month-1) == ii)  
          { server.printP(html_option); server << ii+1; server.printP(html_selected);}
        else 
          { server.printP(html_option); server << ii+1; server.printP(html_e_tag);}
        flashPrintln(server, &PGM_month[ii]); server.printP(html_e_option);
      }
      server.printP(html_e_select);
      server.print(' '); server.printP(text_at); server.print(' ');
      server.printP(html_select); server << 'f'; server.printP(html_e_tag);
      for (int8_t ii = 0; ii < 24; ++ii) {
        if ((conf.time_std_hour) == ii)  
          { server.printP(html_option); server << ii; server.printP(html_selected);}
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag);}
        server << ii; server.printP(html_e_option);
      }
      server.printP(html_e_select);
      server.printP(text_cosp); server.printP(text_offset); server.print(' ');
      //server.printP(html_s_tag_s); server << 'g'; server.printP(html_m_tag); 
      //server << conf.time_std_offset; server.printP(html_e_tag);
      printInput(server, 'g', conf.time_std_offset, 0);
      server.print(' '); server.printP(text_minutes); server.printP(html_e_td_e_tr_tr_td);
      server.printP(html_e_td_td); 
      server.print(time_std.formatedDateTime());
      server.printP(html_e_td_e_tr_tr_td);

      server.printP(text_NTP); server.print(' '); server.printP(text_address); server.printP(html_e_td_td);
      //server.printP(html_s_tag_s); server << 'H'; server.printP(html_m_tag); 
      //server << conf.ntp_ip[0]; server.printP(html_e_tag);
      printInput(server, 'H', conf.ntp_ip[0], 0);
      server.print('.');
      //server.printP(html_s_tag_s); server << 'I'; server.printP(html_m_tag); 
      //server << conf.ntp_ip[1]; server.printP(html_e_tag); 
      printInput(server, 'I', conf.ntp_ip[1], 0);
      server.print('.');
      //server.printP(html_s_tag_s); server << 'J'; server.printP(html_m_tag); 
      //server << conf.ntp_ip[2]; server.printP(html_e_tag); 
      printInput(server, 'J', conf.ntp_ip[2], 0);
      server.print('.');
      //server.printP(html_s_tag_s); server << 'K'; server.printP(html_m_tag); 
      //server << conf.ntp_ip[3]; server.printP(html_e_tag); 
      printInput(server, 'K', conf.ntp_ip[3], 0);
      server.printP(html_e_td_e_tr);
      server.printP(html_e_table);
    
      server.printP(html_h1); server.printP(text_Radio); server.printP(html_e_h1);  
      server.printP(html_table_tr_td);
      server.printP(text_Key); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'k'; server.printP(html_m_tag);
      //server << (char*)conf.radioKey; server.printP(html_e_tag);
      printInput(server, 'k', (char*)conf.radioKey); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Frequency); server.print(' '); server.printP(html_e_td_td);
      server.printP(html_radio_s);
      server.radioButton2("0", 1, text_915, (conf.setting >> 7) & B1);
      server.radioButton2("0", 0, text_868, !(conf.setting >> 7) & B1);
      server.printP(html_div_e);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Received); server.print(' '); server.printP(text_packets); server.printP(html_e_td_td);
      server << radio_received; server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Sent); server.print(' '); server.printP(text_packets); server.printP(html_e_td_td);
      server << radio_sent; server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Not); server.print(' '); server.printP(text_acknowledged); server.print(' '); server.printP(text_packets); server.printP(html_e_td_td);
      server << radio_no_ack; server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Failed); server.print(' '); server.printP(text_packets); server.printP(html_e_td_td);
      server << radio_failed; server.printP(html_e_td_e_tr);
      server.printP(html_e_table);

      server.printP(html_h1); server.printP(text_GSM); server.print(' '); server.printP(text_modem); server.printP(html_e_h1);  
      server.printP(html_table_tr_td);
      server.printP(text_Type); server.printP(html_e_td_td); 
      server.print(modem_info); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_On); server.printP(html_e_td_td);
      pinGSM_ON.read() ? server.printP(text_i_disabled) : server.printP(text_i_OK); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Connected); server.printP(html_e_td_td);
      if (GSMisAlive == 1) server.printP(text_i_OK); 
      else                 server.printP(text_i_disabled);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Network);
      server.printP(html_e_td_td);
      switch(GSMreg){
        case 0 : server.printP(text_not); server.print(' '); server.printP(text_registered); break;
        case 1 : server.printP(text_i_home); break;
        case 2 : server.printP(text_i_starting); break;
        case 3 : server.printP(text_registration); server.print(' '); server.printP(text_denied); break;
        // case 4 : server.printP(text_unk); break;
        case 5 : server.printP(text_roaming); break;
        default : server.printP(text_i_question);; break;
      }
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Signal); server.print(' '); server.printP(text_strength);
      server.printP(html_e_td_td);
      server << GSMstrength; server.print('%'); 
      server.printP(html_e_td_e_tr);
      server.printP(html_e_table);
      /*
      server << "<input type='hidden' id='dt' name='t' value='0'/><script type='text/javascript'>"
                "function GT(){document.getElementById('dt').value=Math.floor(Date.now()/1000)}"
                "</script>";
      */
      server.printP(html_F_A); // submit Apply
      server.printP(html_F_SA); // submit Save all
      server.printP(html_F_GetNTP);
      //server << "<input type='submit' name='D' value='Get browser time' onclick='GT()'/>";
      server.printP(html_F_LA); // submit Load all
      server.printP(html_F_RD); // submit Reset default
      server.printP(html_e_form);
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
}

uint16_t ses_eeprom_add = 0;
void webListLog(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (server.checkCredentials(conf.user_pass)) {
    bool repeat;
    char name[2], value[16];
    if (type == WebServer::POST) {
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
  
      server.printP(html_table_tr_th_hash); server.printP(text_Date); 
      server.printP(html_e_th_th); server.printP(text_Message);
      server.printP(html_e_th_th); server.printP(text_Alert); server.print(' '); server.printP(text_requested); server.print(' '); server.printP(text_for);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < 21; ++i){
        server.printP(html_tr_td);
        // Get data from extrenal EEPROM
        nilSemWait(&TWISem);     // wait for slot
        eeprom.readBytes((uint16_t)(ses_eeprom_add + (i*EEPROM_MESSAGE)), EEPROM_MESSAGE, value);
        nilSemSignal(&TWISem);   // Exit region.
        //value[EEPROM_MESSAGE-1] = 0; // Terminate array

        if ((uint16_t)(ses_eeprom_add/EEPROM_MESSAGE + i) > (EEPROM_SIZE/EEPROM_MESSAGE) - 1) 
             server << (uint16_t)(ses_eeprom_add/EEPROM_MESSAGE + i - (EEPROM_SIZE/EEPROM_MESSAGE) + 1); 
        else server << (uint16_t)(ses_eeprom_add/EEPROM_MESSAGE + i + 1);
        server.print('.'); server.printP(html_e_td_td);
             
        // print date and time
        l.b[0] = value[0]; l.b[1] = value[1]; l.b[2] = value[2]; l.b[3] = value[3];
        if (l.lval >= NTP_SECS_YR_1900_2000) time_temp.set(0); // Needed for fresh EEPROM, will show nonsense in all log entries.
        else time_temp.set(l.lval);
        /*
        GSM.print((char*)time_temp.formatedDateTime()); GSM.print(':');
        for (uint8_t i = 5; i < EEPROM_MESSAGE; ++i) {GSM.print(value[i]);}          
        GSM.println();
        */
        server.print((char*)time_temp.formatedDateTime()); server.printP(html_e_td_td);
        //char _alert = value[4]; // Store alert state in case value[] is used as temp array, like key

        switch(value[5]){
          case 'S': // System
            server.printP(text_System); server.print(' ');
            switch(value[6]){
              case 'B': // Battery
                server.printP(text_battery); server.print(' ');
                if (value[7] == 'L') server.printP(text_low);
                else server.printP(text_OK);
              break;
              case 'A': // AC
                server.printP(text_power); server.print(' ');
                server.printP(text_is); server.print(' ');
                if (value[7] == 'L') server.printP(text_On);
                else server.printP(text_Off);
              break;
              case 'C': server.printP(text_configuration); server.print(' ');
                switch(value[7]){
                  case 'W': server.printP(text_saved); break; // conf. saved
                  case 'P': server.printP(text_saved); server.printP(text_cosp); server.printP(text_system);
                    server.print(' '); server.printP(text_disabled);break; // conf. saved
                  case 'L': server.printP(text_loaded); break; // conf. saved
                  case 'R': server.printP(text_reset); break; // conf. saved
                  default:  server.printP(text_undefined); break; // unknown  
                }
              break;
              case 'S': server.printP(text_monitoring); server.print(' '); server.printP(text_started); break; // monitoring strted
              case 's': server.printP(text_started); break;   // boot
              case 'X': server.printP(text_ALARM); break;     // alarm
              case 'M': 
                server.printP(text_MQTT); server.print(' '); server.printP(text_network); server.print(' ');
                switch(value[7]){
                  case 'F': server.printP(text_failed); break;
                  case 'O': server.printP(text_OK); break;
                  default:  server.printP(text_undefined); break; // unknown  
                }
              break;
              case 'L': server.printP(text_Log); server.print(' '); server.printP(text_erased);break;     // Log cleared
              default:  server.printP(text_undefined); break; // unknown
            }
          break;
          case 'Z': // Zone
            server.printP(text_Zone); server.print(' ');
            if ((uint8_t)value[7] < ALR_ZONES) {
              server << (uint8_t)value[7]+1; server.printP(text_spdashsp); server << conf.zone_name[value[7]]; server.print(' ');
            }
            switch(value[6]){
              case 'P': server.printP(text_triggered); server.print(' '); server.printP(text_alarm); break;
              case 'T': server.printP(text_tamper); server.printP(text_ed); break;
              case 'O': server.printP(text_is); server.print(' '); server.printP(text_open); break;
              case 'R': server.printP(text_registered); break;
              case 'r': server.printP(text_re); server.printP(text_registered); break;
              case 'E': server.printP(text_registration); server.print(' '); server.printP(text_error); break;
              case 'e': server.printP(text_error); server.printP(text_cosp); server.printP(text_not); server.print(' '); 
                switch(value[8]){
                  case 'M': server.printP(text_matched);break;
                  default : server.printP(text_allowed); break;
                }
              break;
              default : server.printP(text_unk); break;
            }
          break;
          case 'A': // Authentication
            server.printP(text_Authentication); server.print(' '); server.printP(text_key); server.print(' '); 
            if (value[6] != 'U') { server << conf.key_name[value[7]]; server.print(' '); }
            switch(value[6]){
              case 'D': server.printP(text_disarmed); break;
              case 'A': server.printP(text_armed); break;
              case 'U': server.printP(text_unk); 
                server.printP(text_cosp); server.printP(text_value); server.print(' ');
                char _key[KEY_LEN]; for (uint8_t ii = 0; ii < KEY_LEN; ++ii) {_key[ii] = value[7+ii];}
                printKey(server, _key);
                break;
              case 'F': server.printP(text_is); server.print(' '); server.printP(text_disabled); break;              
              default : server.printP(text_unk); break;
            }
          break;    
          case 'M': // GSM modem
            server.printP(text_GSM); server.print(' '); server.printP(text_network); server.print(' ');
            switch(value[6]){
              case '0' : server.printP(text_not); server.print(' '); server.printP(text_registered); break;
              case '1' : server.printP(text_registered); break;
              case '2' : server.printP(text_searching); break;
              case '3' : server.printP(text_registration); server.print(' '); server.printP(text_denied); break;
              // case 4 : server.printP(text_unk); break;
              case '5' : server.printP(text_roaming); break;
              default : server.printP(text_unk); break;
            }
            server.printP(text_cosp); server.printP(text_strength); server.print(' ');
            server << (uint8_t)value[7] << '%';
          break;
          case 'N': // remote nodes
            printNodeFunction(server, value[9]); server.print(':');
            printNodeType(server, value[10]); 
            server.print(' '); server.printP(text_address); server.print(' ');
            if ((uint8_t)value[7] < RADIO_UNIT_OFFSET) { server << "W:" << (uint8_t)value[7]; }
            else                                       { server << "R:" << (uint8_t)value[7]-RADIO_UNIT_OFFSET; }
            server.print(':'); server << (uint8_t)value[8]; server.print(' ');
            if (value[6] != 'E') {server.printP(text_is); server.print(' ');}
            switch(value[6]){
              case 'F' : server.printP(text_disabled); break;
              case 'R' : server.printP(text_registered); break;
              case 'r' : server.printP(text_re); server.printP(text_registered); break;
              default : server.printP(text_registration); server.print(' '); server.printP(text_error); break;
            }
          break;
          case 'G': // Groups
            server.printP(text_Group); server.print(' ');
            server << (uint8_t)value[7]+1; server.printP(text_spdashsp); server << conf.group_name[value[7]]; server.print(' ');
            switch(value[6]){
              case 'F': server.printP(text_is); server.print(' '); server.printP(text_disabled); break;
              case 'S': server.printP(text_is); server.print(' '); server.printP(text_armed); break;
              case 'D': server.printP(text_is); server.print(' '); server.printP(text_disarmed); break;
              case 'A': server.printP(text_is); server.print(' '); server.printP(text_auto); server.print(' '); server.printP(text_armed); break;
              default : server.printP(text_unk); break;
            }
          break;
          case 'F': // Fifo      
            switch(value[6]){
              case 'A' : server.printP(text_Alarm); break;
              case 'L' : server.printP(text_Alert); break;
              case 'S' : server.printP(text_Node); break;
              case 'R' : server.printP(text_Registration); break;
              case 'M' : server.printP(text_Message); break;
              default : server.printP(text_unk); break;
            }
            server.print(' '); server.printP(text_queue); server.print(' '); server.printP(text_is); server.print(' '); server.printP(text_full);
          break;
          case 'R': // Trigger
            server.printP(text_Trigger); server.print(' ');
            // Special global battery low trigger
            if ((uint8_t)value[7] == 255) {
              server.printP(text_battery); server.print(' '); server.printP(text_low);
            } else {
              server << (uint8_t)value[7]+1; server.printP(text_spdashsp); server << trigger[value[7]].name;  
            } 
            server.print(' ');
            switch(value[6]){
              case 'D' : server.printP(text_de); server.printP(text_activated); break;
              default : server.printP(text_activated); break;
            }
          break;
          case 'T': // Alert
            switch(value[6]){
              case 'E' : server.printP(text_Email); break;
              default : server.printP(text_unk); break;
            }
            server.print(' '); server.printP(text_failed); server.printP(text_sesp);
            switch(value[7]){
              case 1 ... 2 : server.printP(text_connection); break;
              case 3 ... 5 : server.printP(text_login); break;
              case 6 : server.printP(text_From); server.print(' '); server.printP(text_address);break;
              case 7 : server.printP(text_To); server.print(' '); server.printP(text_address);break;
              default : server.printP(text_body); break;
            }
            server.print(' '); server.printP(text_rejected);
          break;
          case 'C': // SMS Commands
            server.printP(text_SMS); server.print(' '); server.printP(text_From); server.print(' ');
            server << conf.tel_name[value[6]]; server.printP(text_sesp);
            switch(value[7]){
              case 'G' : server.printP(text_Group); 
                server.print(' '); server << (uint8_t)value[8]+1; server.printP(text_spdashsp); server << conf.group_name[value[8]]; server.print(' ');
                switch(value[9]){
                  case 'S' : server.printP(text_state);
                  case 'A' : server.printP(text_Arm);
                  case 'D' : server.printP(text_Disarm);
                  case 'O' : server.printP(text_On);
                  case 'F' : server.printP(text_Off);
                default : server.printP(text_unk); break;
                } 
              break;
              case 'I' : server.printP(text_Input); break;
              default : server.printP(text_unk); break;
            }
          break;
          default:
            server.printP(text_Undefined); server.printP(text_sesp);
            for (uint8_t i = 5; i < EEPROM_MESSAGE; ++i) {server.print(value[i]);}
          break;
        }
        server.print('.');
        server.printP(html_e_td_td);
        n = 0;
        if ((value[4] >> alert_SMS) & B1)   { server.printP(text_SMS); n = 1; }
        if ((value[4]  >> alert_email) & B1) { 
          if (n) server.printP(text_cosp);
          server.printP(text_Email); n = 1; }
        if ((value[4]  >> alert_page) & B1)  { 
          if (n) server.printP(text_cosp);
          server.printP(text_Page); }
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
      char name[2], value[16], message[REG_LEN];
      uint8_t  _found;
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
          case 'A': // Apply
            //   For remote node send reg. to node
            if ((conf.zone[webZone] >> 12) & B1) {
              message[0] = 'R';
              message[1] = 'Z';
              if ((conf.zone[webZone] >> 15) & B1) message[2] = 'A';
              else                                 message[2] = 'D';
              message[3] = webZone;
              message[4] = (char)((conf.zone[webZone] >> 8) & B11111111);
              message[5] = (char)(conf.zone[webZone] & B11111111);
              for (uint8_t _i = 0; _i < NAME_LEN; _i++) { message[6+_i] = conf.zone_name[webZone][_i];}
              if (!sendData(conf.zone_address[webZone-HW_ZONES], message, REG_LEN)) {
                // look queue slot
                _found = 255;
                // Look for empty queue slot or last this zone used slot
                for (uint8_t i = 0; i < NODE_QUEUE; ++i) {
                  //   queue flag 
                  if ((zone[webZone].setting >> 6) & B1) {
                    if(node_queue[i].index  == NODES+webZone) { _found = i; break; }
                  } else {
                    if(node_queue[i].expire == 0)             { _found = i; break; }
                  }
                }
                //WS.print("_found: "); WS.println(_found);
                if (_found != 255) {
                  // Put message into queue
                  node_queue[_found].address  = conf.zone_address[webZone-HW_ZONES];
                  node_queue[_found].index    = NODES+webZone;
                  node_queue[_found].expire   = timestamp.get() + SECS_PER_HOUR; // Message expires in 1 hour
                  node_queue[_found].length   = REG_LEN; 
                  memcpy(node_queue[_found].msg, message, REG_LEN);
                  zone[webZone].setting      |=  (1 << 6); // flag message is in queue
                } else {
                  pushToLog("FM"); // Message queue is full
                }
              }
            }
          break;
          case 'n': // zone name
            strncpy(conf.zone_name[webZone], value, NAME_LEN);
            conf.zone_name[webZone][NAME_LEN-1] = 0;
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
        server.printP(html_h1); server.printP(text_Zone); server.print(' '); server.printP(text_setup); server.printP(html_e_h1); 
        // *** server.printP(html_form_s); server << PREFIX "/z"; server.printP(html_form_e);
        server.printP(html_table_tr_th_hash); server.printP(text_Name); 
        server.printP(html_e_th_th); server.printP(text_On);        
        server.printP(html_e_th_th); server.printP(text_Type);
        //server.printP(html_e_th_th); server.printP(text_Auto); server.print(' '); server.printP(text_arm);
        server.printP(html_e_th_th); server.printP(text_Open); server.print(' '); server.printP(text_alarm);
        server.printP(html_e_th_th); server.printP(text_Alarm); server.print(' '); server.printP(text_as); server.print(' '); server.printP(text_tamper);
        server.printP(html_e_th_th); server.printP(text_Delay);
        server.printP(html_e_th_th); server.printP(text_Group);
        server.printP(html_e_th_th); server.printP(text_Last); server.print(' '); server.printP(text_alarm);
        server.printP(html_e_th_th); server.printP(text_Last); server.print(' '); server.printP(text_OK);
        server.printP(html_e_th_th); server.printP(text_Status);
        server.printP(html_e_th_e_tr);
        for (uint8_t i = 0; i < ALR_ZONES; ++i) {
          //   Zone is connected
          if ((conf.zone[i] >> 14) & B1) {
            server.printP(html_tr_td); 
            server << i+1; server.print('.'); server.printP(html_e_td_td);
            server << conf.zone_name[i]; server.printP(html_e_td_td);
            (conf.zone[i] & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
            ((conf.zone[i] >> 12) & B1) ? server.printP(text_remote) : server.printP(text_local); server.print(' ');
            if ((conf.zone[i] >> 11) & B1) { server.printP(text_battery); server.print(' '); }
            ((conf.zone[i] >> 15) & B1) ? server.printP(text_analog) : server.printP(text_digital); server.printP(html_e_td_td);
            //((conf.zone[i] >> 7) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
            ((conf.zone[i] >> 8) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
            ((conf.zone[i] >> 9) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
            server << (((conf.zone[i] >> 5) & B11)*conf.alr_time); server.print(' '); server.printP(text_seconds); server.printP(html_e_td_td);
            if ((conf.zone[i] & B1)) { server << ((conf.zone[i] >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((conf.zone[i] >> 1) & B1111)]; }
            else                     { server.printP(text_spdashsp); }
            server.printP(html_e_td_td);
            time_temp.set(timestamp.get() - zone[i].last_PIR);
            if ((conf.zone[i] & B1)) { server.print((char*)time_temp.formatedUpTime()); }
            else                     { server.printP(text_spdashsp); }
            time_temp.set(timestamp.get() - zone[i].last_OK);
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

        server.printP(html_form_s); server << PREFIX "/z"; server.printP(html_form_e);
        server.printP(html_table_tr_td);
        server.printP(text_Zone);server.printP(html_e_td_td);
        server.printP(html_select_submit); server << 'Z'; server.printP(html_e_tag);
        for (uint8_t ii = 0; ii < ALR_ZONES; ++ii) {
            if ((conf.zone[ii] >> 14) & B1) {
            server.printP(html_option); server << ii;
            if (webZone == ii) { server.printP(html_selected); }
            else               { server.printP(html_e_tag); }
            server << ii + 1; server.printP(text_spdashsp); server << conf.zone_name[ii]; server.printP(html_e_option);
          }
        }
        server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Name); server.printP(html_e_td_td);
        //server.printP(html_s_tag); server << 'n'; server.printP(html_m_tag); server << conf.zone_name[webZone]; server.printP(html_e_tag);
        printInput(server, 'n', conf.zone_name[webZone]); server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Zone); server.print(' '); server.printP(text_is); server.printP(html_e_td_td);
        printOnOffButton(server, "0", conf.zone[webZone] & B1);
        server.printP(html_e_td_e_tr_tr_td);
        //server.printP(text_Type); server.printP(html_e_td_td);
        //((conf.zone[webZone] >> 15) & B1) ? server.printP(text_analog) : server.printP(text_digital); server.printP(html_e_td_e_tr_tr_td);        
        //server.printP(text_Auto); server.print(' '); server.printP(text_arm); server.printP(html_e_td_td);
        //printOnOffButton(server, "7", conf.zone[webZone] >> 7 & B1);
        //server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Open); server.print(' '); server.printP(text_alarm); server.printP(html_e_td_td);
        printOnOffButton(server, "8", conf.zone[webZone] >> 8 & B1);
        server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Alarm); server.print(' '); server.printP(text_as); server.print(' '); server.printP(text_tamper); server.printP(html_e_td_td);
        printOnOffButton(server, "9", conf.zone[webZone] >> 9 & B1);
        server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Authentication); server.print(' '); server.printP(text_delay); 
        server.printP(html_e_td_td);
        server.printP(html_radio_sl);
        server.radioButton2("d", 0, text_0, !((conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
        server.radioButton2("d", 1, text_1, (!(conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
        server.radioButton2("d", 2, text_2, ((conf.zone[webZone] >> 6 & B1) & !(conf.zone[webZone] >> 5 & B1)));
        server.radioButton2("d", 3, text_3, ((conf.zone[webZone] >> 6 & B1) & (conf.zone[webZone] >> 5 & B1)));
        server.printP(html_div_e); server.print(' '); server.printP(text_x); server << conf.alr_time; server.print(' '); server.printP(text_seconds); server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Group); server.printP(html_e_td_td);
        selectGroup(server, conf.zone[webZone] >> 1);
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
          case 'D':
            disarmGroup(webGroup, 255, 0); // Disarm only this group and do not publish
          break;
          case 'n': // zone name
            strncpy(conf.group_name[webGroup], value, NAME_LEN);
            conf.group_name[webGroup][NAME_LEN-1] = 0;
          break;
          case '0' ... '7': // Handle all single radio buttons for settings
            if (value[0] == '0') conf.group[webGroup] &= ~(1 << (name[0]-48));
            else                 conf.group[webGroup] |=  (1 << (name[0]-48));
            group[webGroup].setting &= ~(1 << 7);  // Clear disable group log bit
          break;
          case 'a': // group
            n = strtol(value, NULL, 10);
            if ((n >> 0) & 1) conf.group[webGroup] |= (1 << 8);
            else conf.group[webGroup] &= ~(1 << 8);
            if ((n >> 1) & 1) conf.group[webGroup] |= (1 << 9);
            else conf.group[webGroup] &= ~(1 << 9);
            if ((n >> 2) & 1) conf.group[webGroup] |= (1 << 10);
            else conf.group[webGroup] &= ~(1 << 20);
            if ((n >> 3) & 1) conf.group[webGroup] |= (1 << 11);
            else conf.group[webGroup] &= ~(1 << 11);
          break;
          case 'd': // group
            n = strtol(value, NULL, 10);
            if ((n >> 0) & 1) conf.group[webGroup] |= (1 << 12);
            else conf.group[webGroup] &= ~(1 << 12);
            if ((n >> 1) & 1) conf.group[webGroup] |= (1 << 13);
            else conf.group[webGroup] &= ~(1 << 13);
            if ((n >> 2) & 1) conf.group[webGroup] |= (1 << 14);
            else conf.group[webGroup] &= ~(1 << 14);
            if ((n >> 3) & 1) conf.group[webGroup] |= (1 << 15);
            else conf.group[webGroup] &= ~(1 << 15);
          break;
          case 'e': saveConf(); break; // save to EEPROM         
          }
        } while (repeat);
        server.httpSeeOther(PREFIX "/g");
    } else {
      server.httpSuccess();
      webMenu(server, menu_Groups);
      server.printP(html_h1); server.printP(text_Group); server.print(' '); server.printP(text_setup); server.printP(html_e_h1);  
      server.printP(html_table_tr_th_hash); server.printP(text_Name); 
      server.printP(html_e_th_th); server.printP(text_On);
      server.printP(html_e_th_th); server.printP(text_Armed);
      //server.printP(html_e_th_th); server.printP(text_Authentication);      
      //server.printP(html_e_th_th); server.printP(text_Delay);
      server.printP(html_e_th_th); server.printP(text_Auto); server.print(' '); server.printP(text_arm);
      server.printP(html_e_th_th); server.printP(text_Arm);
      server.printP(html_e_th_th); server.printP(text_Disarm);
      server.printP(html_e_th_th); server.printP(text_i_zone); server.print(' '); server.printP(text_Zone); server.print('s');
      server.printP(html_e_th_th); server.printP(text_i_auth); server.print(' ');server.printP(text_Authentication); //server.print(' '); server.printP(text_node); server.print('s');
      server.printP(html_e_th_th); server.printP(text_i_sens); server.print(' ');server.printP(text_Sensor); server.print('s');
      server.printP(html_e_th_th); server.printP(text_i_contact); server.print(' ');server.printP(text_Contact); server.print('s');
      server.printP(html_e_th_th); server.printP(text_Alarm);
      server.printP(html_e_th_th); server.printP(text_Tamper);
      server.printP(html_e_th_th); server.printP(text_Status);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < ALR_GROUPS; ++i) {
        server.printP(html_tr_td); 
        server << i+1; server.print('.'); server.printP(html_e_td_td);
        server << conf.group_name[i]; server.printP(html_e_td_td);
        (conf.group[i] & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        (group[i].setting & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        //((group[i].setting >> 2) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        //server << group[i].arm_delay/4; server.print(' '); server.printP(text_seconds); server.printP(html_e_td_td);
        ((conf.group[i] >> 5) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        if (((conf.group[i] >> 8) & B1111) != i) {
          server << ((conf.group[i] >> 8) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((conf.group[i] >> 8) & B1111)];
        } else { server.printP(text_spdashsp); }
        server.printP(html_e_td_td);
        if (((conf.group[i] >> 12) & B1111) != i) {
          server << ((conf.group[i] >> 12) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((conf.group[i] >> 12) & B1111)];
        } else { server.printP(text_spdashsp); }
        server.printP(html_e_td_td);
        n = 0;
        for (uint8_t ii = 0; ii < ALR_ZONES; ++ii) {
          if ((((conf.zone[ii] >> 1) & B1111) == i) && (conf.zone[ii] & B1)) {
            if (n) { server.printP(html_br); }
            server << ii+1; server.printP(text_spdashsp); server << conf.zone_name[ii];
            n = 1;
          }
        }
        server.printP(html_e_td_td);
        n = 0;
        for (uint8_t ii = 0; ii < nodes; ++ii) {      
          //     group number                              Node enabled               Auth. node
          if ((((node[ii].setting >> 1) & B1111) == i) && (node[ii].setting & B1) && (node[ii].function == 'K')) { 
            if (n) { server.printP(html_br); }
            printNodeAddress(server, ii);
            n = 1;
          }
        }
        server.printP(html_e_td_td);
        n = 0;
        for (uint8_t ii = 0; ii < nodes; ++ii) {
          //     group number                              Node enabled               Sensor   
          if ((((node[ii].setting >> 1) & B1111) == i) && (node[ii].setting & B1) && (node[ii].function == 'S')) {
            if (n) { server.printP(html_br); }
            printNodeAddress(server, ii);
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
        if ((conf.group[i] >> 4) & B1) { server.printP(text_OUT1); server.print(' '); }
        if ((conf.group[i] >> 3) & B1) server.printP(text_OUT2);
        server.printP(html_e_td_td);
        if ((conf.group[i] >> 2) & B1) { server.printP(text_OUT1); server.print(' '); }
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
      server.printP(html_select_submit); server << 'G'; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < ALR_GROUPS; ++ii) {
        if (webGroup == ii) 
          { server.printP(html_option); server << ii; server.printP(html_selected); server << ii + 1;}
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii + 1;}
        server.printP(text_spdashsp); server << conf.group_name[ii]; server.printP(html_e_option);
      }
      server.printP(html_e_select);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Name); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'n'; server.printP(html_m_tag); server << conf.group_name[webGroup]; server.printP(html_e_tag);
      printInput(server, 'n', conf.group_name[webGroup]); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Group); server.print(' '); server.printP(text_is); server.printP(html_e_td_td);
      printOnOffButton(server, "0", conf.group[webGroup] & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Auto); server.print(' '); server.printP(text_arm); server.printP(html_e_td_td);
      printOnOffButton(server, "5", conf.group[webGroup] >> 5 & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Alarm); server.print(' '); server.printP(text_trigger); server << "s "; server.printP(text_OUT1); server.printP(html_e_td_td);
      printOnOffButton(server, "4", conf.group[webGroup] >> 4 & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Alarm); server.print(' '); server.printP(text_trigger); server << "s "; server.printP(text_OUT2); server.printP(html_e_td_td);
      printOnOffButton(server, "3", conf.group[webGroup] >> 3 & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Tamper); server.print(' '); server.printP(text_trigger); server << "s "; server.printP(text_OUT1); server.printP(html_e_td_td);
      printOnOffButton(server, "2", conf.group[webGroup] >> 2 & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Tamper); server.print(' '); server.printP(text_trigger); server << "s "; server.printP(text_OUT2); server.printP(html_e_td_td);
      printOnOffButton(server, "1", conf.group[webGroup] >> 1 & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Arm); server.print(' '); server.printP(text_Group); server.printP(html_e_td_td);
      selectGroup(server, conf.group[webGroup] >> 8, 'a');
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Disarm); server.print(' '); server.printP(text_Group); server.printP(html_e_td_td);
      selectGroup(server, conf.group[webGroup] >> 12, 'd');
      server.printP(html_e_td_e_tr);
      server.printP(html_e_table);
          
      server.printP(html_F_A); // submit Apply
      server.printP(html_F_Disarm); 
      server.printP(html_F_SA); // submit Save all
      server.printP(html_e_form);
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
}

uint8_t webKey = 0;
void webSetKey(WebServer &server, WebServer::ConnectionType type, char *url_tail, bool tail_complete) {
  if (server.checkCredentials(conf.user_pass)) {
    bool repeat;
    char name[2], value[17];
    if (type == WebServer::POST) {
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
            strncpy(conf.key_name[webKey], value, NAME_LEN);
            conf.key_name[webKey][NAME_LEN-1] = 0;
          break;
          case 'g': // group
            n = strtol(value, NULL, 10);
            if ((n >> 0) & 1) conf.key_setting[webKey] |= (1 << 1);
            else conf.key_setting[webKey] &= ~(1 << 1);
            if ((n >> 1) & 1) conf.key_setting[webKey] |= (1 << 2);
            else conf.key_setting[webKey] &= ~(1 << 2);
            if ((n >> 2) & 1) conf.key_setting[webKey] |= (1 << 3);
            else conf.key_setting[webKey] &= ~(1 << 3);
            if ((n >> 3) & 1) conf.key_setting[webKey] |= (1 << 4);
            else conf.key_setting[webKey] &= ~(1 << 4);
          break;
          case '0' ... '7': // Handle all single radio buttons for settings
            if (value[0] == '0') conf.key_setting[webKey] &= ~(1 << (name[0]-48));
            else                 conf.key_setting[webKey] |=  (1 << (name[0]-48));
          break;
          case 'e': saveConf(); break; // save to EEPROM          
          }
        } while (repeat);
        server.httpSeeOther(PREFIX "/k");
    } else {
      server.httpSuccess();
      webMenu(server, menu_Keys);
      server.printP(html_h1); server.printP(text_Key); server.print(' '); server.printP(text_setup); server.printP(html_e_h1); 

      server.printP(html_table_tr_th_hash); server.printP(text_Name);
      server.printP(html_e_th_th); server.printP(text_On);
      server.printP(html_e_th_th); server.printP(text_Value);
      server.printP(html_e_th_th); server.printP(text_Global);
      server.printP(html_e_th_th); server.printP(text_Group);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < NUM_OF_KEYS; ++i) {
        server.printP(html_tr_td); 
        server << i+1; server.print('.'); server.printP(html_e_td_td);
        server << conf.key_name[i]; server.printP(html_e_td_td);
        (conf.key_setting[i] & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        //printKey(conf.key[i], value); server << value; 
        printKey(server, conf.key[i]);
        server.printP(html_e_td_td);
        (conf.key_setting[i] >> 5 & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        if (conf.key_setting[i] & B1) {
          if (conf.key_setting[i] >> 5 & B1) { server.printP(text_all); }
          else { server << ((conf.key_setting[i] >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((conf.key_setting[i] >> 1) & B1111)]; }
        } 
        else { server.printP(text_i_disabled); }
        server.printP(html_e_td_e_tr);
      }
      server.printP(html_e_table); 
      server.printP(html_form_s); server << PREFIX "/k"; server.printP(html_form_e);      
      server.printP(html_table_tr_td);
      server.printP(text_Key); server.printP(html_e_td_td);
      server.printP(html_select_submit); server << 'K'; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < NUM_OF_KEYS; ++ii) {
        server.printP(html_option); server << ii;
        if (webKey == ii) { server.printP(html_selected); }
        else              { server.printP(html_e_tag); }
        server << ii + 1; server.printP(text_spdashsp); server << conf.key_name[ii]; server.printP(html_e_option);
      }
      server.printP(html_e_select);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Name); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'n'; server.printP(html_m_tag); server << conf.key_name[webKey]; server.printP(html_e_tag);
      printInput(server, 'n', conf.key_name[webKey]); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Key); server.print(' '); server.printP(text_is); server.printP(html_e_td_td);
      
      printOnOffButton(server, "0", conf.key_setting[webKey] & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Value); server.printP(html_e_td_td);
      // server.printP(html_s_tag); server << 'k'; server.printP(html_m_tag); server << value; server.printP(html_e_tag);
      server.printP(html_s_tag); server << 'k'; server.printP(html_m_tag); printKey(server, conf.key[webKey]); server.printP(html_e_tag);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Global); server.print(' '); server.printP(text_key); server.printP(html_e_td_td);
      
      printOnOffButton(server, "5", conf.key_setting[webKey] >> 5 & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Group); server.printP(html_e_td_td);
      selectGroup(server, conf.key_setting[webKey] >> 1); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Last); server.print(' '); server.printP(text_key); server.printP(html_e_td_td);
      //printKey(last_key, value); server << value;
      printKey(server, last_key);
      server.printP(html_e_td_e_tr);
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
            strncpy(conf.tel_num[webTel], value, PHONE_LEN);
            conf.tel_num[webTel][PHONE_LEN-1] = 0;
          break;
          case 'n': // name
            strncpy(conf.tel_name[webTel], value, NAME_LEN);
            conf.tel_name[webTel][NAME_LEN-1] = 0;
          break;
          case 'm': // email
            strncpy(conf.email[webTel], value, EMAIL_LEN);
            conf.email[webTel][EMAIL_LEN-1] = 0;
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
        server.printP(html_h1); server.printP(text_Contact); server << "s "; server.printP(text_setup); server.printP(html_e_h1); 

        server.printP(html_table_tr_th_hash);server.printP(text_Name);
        server.printP(html_e_th_th); server.printP(text_On);
        server.printP(html_e_th_th); server.printP(text_Number);
        server.printP(html_e_th_th); server.printP(text_Email);
        server.printP(html_e_th_th); server.printP(text_Global);
        server.printP(html_e_th_th); server.printP(text_Group);
        server.printP(html_e_th_e_tr);
        for (uint8_t i = 0; i < NUM_OF_PHONES; ++i) {
          server.printP(html_tr_td); 
          server << i+1; server.print('.'); server.printP(html_e_td_td);
          server << conf.tel_name[i]; server.printP(html_e_td_td);
          (conf.tel[i] & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
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
        server.printP(html_select_submit); server << 'P'; server.printP(html_e_tag);
        for (uint8_t ii = 0; ii < NUM_OF_PHONES; ++ii) {
          server.printP(html_option); server << ii;
          if (webTel == ii) { server.printP(html_selected); }
          else              { server.printP(html_e_tag); }
          server << ii + 1; server.printP(text_spdashsp); server << conf.tel_name[ii]; server.printP(html_e_option);
        }
        server.printP(html_e_select);
        server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Name); server.printP(html_e_td_td);
        //server.printP(html_s_tag); server << 'n'; server.printP(html_m_tag); server << conf.tel_name[webTel]; server.printP(html_e_tag);
        printInput(server, 'n', conf.tel_name[webTel]); server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Contact); server.print(' '); server.printP(text_is); server.printP(html_e_td_td);
        printOnOffButton(server, "0", conf.tel[webTel] & B1);
        server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Phone); server.print(' '); server.printP(text_number); server.printP(html_e_td_td);
        //server.printP(html_s_tag); server << 'p'; server.printP(html_m_tag); server << conf.tel_num[webTel]; server.printP(html_e_tag);
        printInput(server, 'p', conf.tel_num[webTel]); server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Email); server.printP(html_e_td_td);
        //server.printP(html_s_tag); server << 'm'; server.printP(html_m_tag); server << conf.email[webTel]; server.printP(html_e_tag);
        printInput(server, 'm', conf.email[webTel]); server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Global); server.print(' '); server.printP(text_contact); server.printP(html_e_td_td);
        printOnOffButton(server, "5", conf.tel[webTel] >> 5 & B1);
        server.printP(html_e_td_e_tr_tr_td);
        server.printP(text_Group); server.printP(html_e_td_td);
        selectGroup(server, conf.tel[webTel] >> 1);
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
    char name[2], value[17], message[REG_LEN];
    uint8_t  _found;
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
            message[0] = 'R';
            message[1] = node[webSens].function;
            message[2] = node[webSens].type;
            message[3] = node[webSens].number;
            message[4] = (char)((node[webSens].setting >> 8) & B11111111);;
            message[5] = (char)(node[webSens].setting & B11111111);
            for (uint8_t _i = 0; _i < NAME_LEN; _i++) { message[6+_i] = node[webSens].name[_i];}
            if (!sendData(node[webSens].address, message, REG_LEN)) {
              // look queue slot
              _found = 255;
              if (node[webSens].queue != 255) {
                _found = node[webSens].queue; // Replace last message in queue
              } else {
                // Look for empty queue slot
                for (uint8_t i = 0; i < NODE_QUEUE; ++i) {
                  if(node_queue[i].expire == 0) { _found = i; break; }
                }
              }
              if (_found != 255) {
                // Put message into queue
                node_queue[_found].address  = node[webSens].address;
                node_queue[_found].index    = webSens;
                node_queue[_found].expire   = timestamp.get() + SECS_PER_HOUR; // Message expires in 1 hour
                node_queue[_found].length   = REG_LEN; 
                memcpy(node_queue[_found].msg, message, REG_LEN);
                node[webSens].queue         = _found; // Pointer to message queue
              } else {
                pushToLog("FM"); // Message queue is full
              }
            }
          break;
          case 'n': // name
            strncpy (node[webSens].name, value, NAME_LEN);
          break;
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
      server.printP(html_h1); server.printP(text_Node); server.print(' '); server.printP(text_setup); server.printP(html_e_h1);  
      server.printP(html_form_s); server << PREFIX "/s"; server.printP(html_form_e);    
      server.printP(html_table_tr_th_hash); server.printP(text_Address);
      server.printP(html_e_th_th); server.printP(text_On);
      server.printP(html_e_th_th); server.printP(text_MQTT);
      server.printP(html_e_th_th); server.printP(text_Last); server.print(' '); server.printP(text_Message);
      server.printP(html_e_th_th); server.printP(text_Function);
      server.printP(html_e_th_th); server.printP(text_Type);
      server.printP(html_e_th_th); server.printP(text_Value);
      server.printP(html_e_th_th); server.printP(text_Group);
      server.printP(html_e_th_th); server.printP(text_Queue); server.print('d');
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < nodes; ++i) {
        server.printP(html_tr_td); 
        server << i+1; server.print('.'); server.printP(html_e_td_td);        
        printNodeAddress(server, i);
        server.printP(html_e_td_td);
        (node[i].setting & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((node[i].setting >> 7) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        time_temp.set(timestamp.get() - node[i].last_OK); 
        server.print((char*)time_temp.formatedUpTime()); server.printP(html_e_td_td);
        printNodeFunction(server, node[i].function);      
        server.printP(html_e_td_td);
        printNodeType(server, node[i].type);
        if ((node[i].type == 'B') && ((node[i].setting >> 5) & B1)) { server.printP(text_spdashsp); server.printP(text_low); }
        server.printP(html_e_td_td);
        //  Do not show value for authentication units
        if (node[i].function != 'K') {
          dtostrf(node[i].value, 6, 2, value); server << value;
          switch(node[i].type){
            case 'T': server.printP(text_degC); break;
            case 'H': 
            case 'X': server.printP(text_percent); break;
            case 'P': server.printP(text_mBar); break;
            case 'V':
            case 'B': server.printP(text_Volt); break;
            case 'G': server.printP(text_ppm); break;
            default: break;
          }
        }
        server.printP(html_e_td_td);
        server << ((node[i].setting >> 1) & B1111) + 1; server.printP(text_spdashsp); server << conf.group_name[((node[i].setting >> 1) & B1111)];
        server.printP(html_e_td_td);
        (node[i].queue != 255) ? server.printP(text_i_OK) : server.printP(text_i_disabled);
        server.printP(html_e_td_e_tr);
      }
      server.printP(html_e_table); 
      server.printP(html_table_tr_td);
      server.printP(text_Node); server.printP(html_e_td_td);
      server.printP(html_select_submit); server << 'P'; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < nodes; ++ii) {
        server.printP(html_option); server << ii;
        if (webSens == ii) { server.printP(html_selected); }
        else               { server.printP(html_e_tag); }
        server << ii + 1; server.printP(text_spdashsp); server << node[ii].name; server.printP(html_e_option);
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Name); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << "n"; server.printP(html_m_tag); server << node[webSens].name; server.printP(html_e_tag);
      printInput(server, 'n', node[webSens].name); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Address); server.printP(html_e_td_td);
      printNodeAddress(server, webSens);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Node); server.print(' '); server.printP(text_is); server.printP(html_e_td_td);
      printOnOffButton(server, "0", node[webSens].setting & B1);
      server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Type); server.printP(html_e_td_td);
      printNodeFunction(server, node[webSens].function);
      server.print(':');
      printNodeType(server, node[webSens].type);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_MQTT); server.print(' '); server.printP(text_publish); server.printP(html_e_td_td);
      printOnOffButton(server, "7", (node[webSens].setting >> 7) & B1);
      server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Group); server.printP(html_e_td_td);
      selectGroup(server, node[webSens].setting >> 1);
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
          case '0' ... '7': // Handle all single radio buttons for settings
            if (value[0] == '0') conf.mqtt &= ~(1 << (name[0]-48));
            else                 conf.mqtt |=  (1 << (name[0]-48));
          break;
          case 'n' ... 'q':conf.mqtt_ip[name[0]-110] = strtol(value, NULL, 10); break;
          case 'P': conf.mqtt_port = strtol(value, NULL, 10); break;
          case 'a' ... 'd': conf.ip[name[0]-97] = strtol(value, NULL, 10); break;
          case 'f' ... 'i': conf.gw[name[0]-102] = strtol(value, NULL, 10); break;
          case 'j' ... 'm': conf.mask[name[0]-106] = strtol(value, NULL, 10); break;
          case 'A': // Apply
            // client.disconnect(); // This have problems with Teensy Eth.
            client.setServer(conf.mqtt_ip, conf.mqtt_port);
            if (client.connect(str_MQTT_clientID)) {
              client.subscribe(str_MQTT_Subscribe);
            }
          break;
          case 'e': saveConf(); break; // save to EEPROM
        }
      } while (repeat);
      server.httpSeeOther(PREFIX "/m");
    } else {
      server.httpSuccess();
      webMenu(server, menu_MQTT);
      server.printP(html_h1); server.printP(text_Network); server.printP(html_e_h1);
      server.printP(html_form_s); server << PREFIX "/m"; server.printP(html_form_e);    
      server.printP(html_table_tr_td);
      server.printP(text_IP); server.print(' '); server.printP(text_address); server.printP(html_e_td_td);
      //server.printP(html_s_tag_s); server << 'a'; server.printP(html_m_tag); 
      //server << conf.ip[0]; server.printP(html_e_tag); 
      printInput(server, 'a', conf.ip[0], 0); server.print('.');
      //server.printP(html_s_tag_s); server << 'b'; server.printP(html_m_tag); 
      //server << conf.ip[1]; server.printP(html_e_tag); 
      printInput(server, 'b', conf.ip[1], 0); server.print('.');
      //server.printP(html_s_tag_s); server << 'c'; server.printP(html_m_tag); 
      //server << conf.ip[2]; server.printP(html_e_tag);
      printInput(server, 'c', conf.ip[2], 0); server.print('.');
      //server.printP(html_s_tag_s); server << 'd'; server.printP(html_m_tag); 
      //server << conf.ip[3]; server.printP(html_e_tag); 
      printInput(server, 'd', conf.ip[3], 0); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Gateway); server.print(' '); server.printP(text_address); server.printP(html_e_td_td);
      //server.printP(html_s_tag_s); server << 'f'; server.printP(html_m_tag); 
      //server << conf.gw[0]; server.printP(html_e_tag);
      printInput(server, 'f', conf.gw[0], 0); server.print('.');
      //server.printP(html_s_tag_s); server << 'g'; server.printP(html_m_tag); 
      //server << conf.gw[1]; server.printP(html_e_tag);
      printInput(server, 'g', conf.gw[1], 0); server.print('.');
      //server.printP(html_s_tag_s); server << 'h'; server.printP(html_m_tag); 
      //server << conf.gw[2]; server.printP(html_e_tag);
      printInput(server, 'h', conf.gw[2], 0); server.print('.');
      //server.printP(html_s_tag_s); server << 'i'; server.printP(html_m_tag); 
      //server << conf.gw[3]; server.printP(html_e_tag);
      printInput(server, 'i', conf.gw[3], 0); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Mask); server.printP(html_e_td_td);
      //server.printP(html_s_tag_s); server << 'j'; server.printP(html_m_tag); 
      //server << conf.mask[0]; server.printP(html_e_tag);
      printInput(server, 'j', conf.mask[0], 0); server.print('.');
      //server.printP(html_s_tag_s); server << 'k'; server.printP(html_m_tag); 
      //server << conf.mask[1]; server.printP(html_e_tag);
      printInput(server, 'k', conf.mask[1], 0); server.print('.');
      //server.printP(html_s_tag_s); server << 'l'; server.printP(html_m_tag); 
      //server << conf.mask[2]; server.printP(html_e_tag);
      printInput(server, 'l', conf.mask[2], 0); server.print('.');
      //server.printP(html_s_tag_s); server << 'm'; server.printP(html_m_tag); 
      //server << conf.mask[3]; server.printP(html_e_tag);
      printInput(server, 'm', conf.mask[3], 0); server.printP(html_e_td_e_tr); 
      server.printP(html_e_table);   

      server.printP(html_h1); server.printP(text_MQTT); server.printP(html_e_h1);
      server.printP(html_table_tr_td);
      server.printP(text_MQTT); server.print(' '); server.printP(text_state); server.printP(html_e_td_td);
      switch(client.state()){
        case -4: server.printP(text_connection); server.print(' '); server.printP(text_time); server.printP(text_out); break;
        case -3: server.printP(text_connection); server.print(' '); server.printP(text_lost); break;
        case -2: server.printP(text_connection); server.print(' '); server.printP(text_failed); break;
        case -1: server.printP(text_i_disabled); break;
        case  0: server.printP(text_i_OK); break;
        case  1: server.printP(text_bad); server.print(' '); server.printP(text_protocol); break;
        case  2: server.printP(text_bad); server.print(' '); server.printP(text_client_id); break;
        case  3: server.printP(text_unavailable); break;
        case  4: server.printP(text_bad); server.print(' '); server.printP(text_credentials); break;
        case  5: server.printP(text_unauthorized); break;
        default: server.printP(text_i_question); break;
      }
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Sensor); server.print(' '); server.printP(text_publish); server.printP(html_e_td_td);
      printOnOffButton(server, "0", (conf.mqtt >> 0) & B1);
      server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Group); server.print(' '); server.printP(text_publish); server.printP(html_e_td_td);
      printOnOffButton(server, "3", (conf.mqtt >> 3) & B1);
      server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Sensor); server.print(' '); server.printP(text_subscribe); server.printP(html_e_td_td);
      printOnOffButton(server, "1", (conf.mqtt >> 1) & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_SMS); server.print(' '); server.printP(text_Gateway); server.printP(html_e_td_td);
      printOnOffButton(server, "2", (conf.mqtt >> 2) & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_MQTT); server.print(' '); server.printP(text_address); server.printP(html_e_td_td);
      //server.printP(html_s_tag_s); server << 'n'; server.printP(html_m_tag); 
      //server << conf.mqtt_ip[0]; server.printP(html_e_tag);
      printInput(server, 'n', conf.mqtt_ip[0], 0); server.print('.');
      //server.printP(html_s_tag_s); server << 'o'; server.printP(html_m_tag); 
      //server << conf.mqtt_ip[1]; server.printP(html_e_tag);
      printInput(server, 'o', conf.mqtt_ip[1], 0); server.print('.');
      //server.printP(html_s_tag_s); server << 'p'; server.printP(html_m_tag); 
      //server << conf.mqtt_ip[2]; server.printP(html_e_tag);
      printInput(server, 'p', conf.mqtt_ip[2], 0); server.print('.');
      //server.printP(html_s_tag_s); server << 'q'; server.printP(html_m_tag); 
      //server << conf.mqtt_ip[3]; server.printP(html_e_tag);
      printInput(server, 'q', conf.mqtt_ip[3], 0); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_MQTT); server.print(' '); server.printP(text_port); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'P'; server.printP(html_m_tag); 
      //server << conf.mqtt_port; server.printP(html_e_tag);
      printInput(server, 'P', conf.mqtt_port); server.printP(html_e_td_e_tr); 
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
            strncpy(trigger[webTrig].name, value, NAME_LEN);
            trigger[webTrig].name[NAME_LEN-1] = 0;
            break;
          case 'v': trigger[webTrig].value = atof(value); break;
          case 'h': trigger[webTrig].hysteresis = atof(value); break;
          case 'c': trigger[webTrig].constant_on = atof(value); break;
          case 'f': trigger[webTrig].constant_off = atof(value); break;
          case 'r': trigger[webTrig].off_time = strtol(value, NULL, 10); 
            if (trigger[webTrig].off_time == 0) trigger[webTrig].off_time = 1;
            break;
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
          case 'B' ... 'G': // Handle all single radio buttons for settings B(66)=0
            if (value[0] == '0') trigger[webTrig].setting &= ~(1 << (name[0]-66));
            else                 trigger[webTrig].setting |=  (1 << (name[0]-66));
            // Pass once (bit 4) to clear Passed (bit 3)
            if (((name[0]-66) == 4) && (value[0] == '0')) trigger[webTrig].setting &= ~(1 << 3); 
            // for Zones
            if (trigger[webTrig].type == 'Z') {
              trigger[webTrig].setting &= ~(1 << 6); // turn OFF Logging 
              trigger[webTrig].setting |=  (1 << 4); // turn ON Pass Once
            }
          break;
          case 'm': // Off interval
            n = value[0] - 48;
            if ((n >> 0) & 1) trigger[webTrig].setting |= (1 << 7);
            else              trigger[webTrig].setting &= ~(1 << 7);
            if ((n >> 1) & 1) { trigger[webTrig].setting |= (1 << 8);  // For off timer set:
                                trigger[webTrig].setting &= ~(1 << 7); 
                                trigger[webTrig].setting |=  (1 << 2); // turn ON Pass 
                                trigger[webTrig].setting |=  (1 << 4); // turn ON Pass Once 
                                trigger[webTrig].setting &= ~(1 << 1); // pass CONSTANT
                              } 
            else                trigger[webTrig].setting &= ~(1 << 8);
          break;
          case 'l': // Off interval
            n = value[0] - 48;
            if ((n >> 0) & 1) trigger[webTrig].setting |= (1 << 14);
            else              trigger[webTrig].setting &= ~(1 << 14);
            if ((n >> 1) & 1) trigger[webTrig].setting |= (1 << 15);
            else              trigger[webTrig].setting &= ~(1 << 15);
          break;
          case 'e': saveConf(); break; // save to EEPROM
        }
      } while (repeat);
      server.httpSeeOther(PREFIX "/t");
    } else {
      server.httpSuccess();
      //if (trigger[webTrig].type == 'Z') webMenu(server, menu_Triggers, false);
      //else                              webMenu(server, menu_Triggers, true);
      webMenu(server, menu_Triggers);
      server.printP(html_h1); server.printP(text_Trigger); server.print(' '); server.printP(text_setup); server.printP(html_e_h1);  
      server.printP(html_form_s); server << PREFIX "/t"; server.printP(html_form_e);    
      server.printP(html_table_tr_th_hash); server.printP(text_Name);
      server.printP(html_e_th_th); server.printP(text_On);
      server.printP(html_e_th_th); server.printP(text_Address);
      server.printP(html_e_th_th); server.printP(text_Symbol);
      server.printP(html_e_th_th); server.printP(text_Value);
      server.printP(html_e_th_th); server.printP(text_Hysteresis);
      server.printP(html_e_th_th); server.printP(text_Logging);
      server.printP(html_e_th_th); server.printP(text_Pass);
      server.printP(html_e_th_th); server.printP(text_Pass); server.print(' '); server.printP(text_once);
      server.printP(html_e_th_th); server.printP(text_Pass); server.print(' '); server.printP(text_Off);
      server.printP(html_e_th_th); server.printP(text_Off); server.print(' '); server.printP(text_interval);
      server.printP(html_e_th_th); server.printP(text_Pass);
      server.printP(html_e_th_th); server.printP(text_To); server.print(' '); server.printP(text_address);
      server.printP(html_e_th_th); server.printP(text_On);
      server.printP(html_e_th_th); server.printP(text_Off);
      server.printP(html_e_th_th); server.printP(text_Trigger); server.printP(text_ed);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < TRIGGERS; ++i) {
        server.printP(html_tr_td); 
        server << i+1; server.print('.'); server.printP(html_e_td_td);
        server << trigger[i].name; server.printP(html_e_td_td);
        (trigger[i].setting & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        if (trigger[i].type == 'Z') {
          server.printP(text_Zone); server.printP(text_cosp); server << conf.zone_name[trigger[i].number];
          server.printP(text_spdashsp); server << trigger[i].number + 1;
        } else {
          printNodeAddress(server, 'S', trigger[i].address, trigger[i].number, trigger[i].type);
        }
        server.printP(html_e_td_td);
        server << trigger_symbol[trigger[i].symbol]; server.printP(html_e_td_td);
        server << trigger[i].value; server.printP(html_e_td_td);
        server << trigger[i].hysteresis; server.printP(html_e_td_td);
        ((trigger[i].setting >> 6) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((trigger[i].setting >> 2) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((trigger[i].setting >> 4) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((trigger[i].setting >> 8) & B1) ? server.printP(text_Timer) : ((trigger[i].setting >> 7) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        server << trigger[i].off_time ; server.print(' ');
        printPeriodType(server, trigger[i].setting >> 14 & B11);
        
        server.printP(html_e_td_td);
        ((trigger[i].setting >> 1) & B1) ? server.printP(text_value) : server.printP(text_constant); server.printP(html_e_td_td);
        printNodeAddress(server, 'I', trigger[i].to_address, trigger[i].to_number, trigger[i].to_type);
        server.printP(html_e_td_td);      
        server << trigger[i].constant_on; server.printP(html_e_td_td);
        server << trigger[i].constant_off; server.printP(html_e_td_td);
        ((trigger[i].setting >> 5) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_e_tr);
      }
      server.printP(html_e_table); 
      server.printP(html_table_tr_td);
      server.printP(text_Trigger); server.printP(html_e_td_td);
      server.printP(html_select_submit); server << 'P'; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < TRIGGERS; ++ii) {
        server.printP(html_option); server << ii;
        if (webTrig == ii) { server.printP(html_selected); }
        else               { server.printP(html_e_tag); }
        server << ii + 1; server.printP(text_spdashsp); server << trigger[ii].name; server.printP(html_e_option);
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Name); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'n'; server.printP(html_m_tag); server << trigger[webTrig].name; server.printP(html_e_tag);
      printInput(server, 'n', trigger[webTrig].name); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Trigger); server.print(' '); server.printP(text_is); server.printP(html_e_td_td);
      printOnOffButton(server, "B", trigger[webTrig].setting & B1);
      server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Address); server.printP(html_e_td_td);
      server.printP(html_select); server << 'a'; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < ALR_ZONES; ++ii) {
        // Zone is enabled
        if (conf.zone[ii] & B1) {
          server.printP(html_option); server << ii;
          if (0   == trigger[webTrig].address &&
              ii  == trigger[webTrig].number &&
              'Z' == trigger[webTrig].type) { server.printP(html_selected); }
          else                              { server.printP(html_e_tag); }
          server.printP(text_Zone); server.print(':'); server.print(' '); server << ii + 1;
          server.printP(text_spdashsp); server << conf.zone_name[ii];
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
          printNodeAddress(server, ii-ALR_ZONES);
          server.print(':'); printNodeType(server, node[ii-ALR_ZONES].type);
          server.printP(html_e_option);
        }
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Symbol); server.printP(html_e_td_td);      
      server.printP(html_select); server << 's'; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < TRIGGER_SYMBOLS; ++ii) {
        server.printP(html_option); server << ii;
        if (trigger[webTrig].symbol == ii) { server.printP(html_selected); }
        else                               { server.printP(html_e_tag); }
        server << trigger_symbol[ii]; server.printP(html_e_option);
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Value); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'v'; server.printP(html_m_tag); 
      //server << trigger[webTrig].value; server.printP(html_e_tag);
      printInput(server, 'v', trigger[webTrig].value); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Hysteresis); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'h'; server.printP(html_m_tag); 
      //server << trigger[webTrig].hysteresis; server.printP(html_e_tag);
      printInput(server, 'h', trigger[webTrig].hysteresis); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Logging); server.printP(html_e_td_td);
      printOnOffButton(server, "H", (trigger[webTrig].setting >> 6) & B1);
      server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Pass); server.printP(html_e_td_td);
      printOnOffButton(server, "D", (trigger[webTrig].setting >> 2) & B1);
      server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Pass); server.print(' '); server.printP(text_once); server.printP(html_e_td_td);
      printOnOffButton(server, "F", (trigger[webTrig].setting >> 4) & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Pass); server.printP(html_e_td_td);      
      server.printP(html_radio_sl);
      server.radioButton2("C", 1, text_value, (trigger[webTrig].setting >> 1) & B1);
      server.radioButton2("C", 0, text_constant, !((trigger[webTrig].setting >> 1) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);  
      server.printP(text_Pass); server.print(' '); server.printP(text_Off); server.printP(html_e_td_td);
      server.printP(html_radio_sl);
      server.radioButton2("m", 1, text_Yes,  ((trigger[webTrig].setting >> 7) & B1) & !((trigger[webTrig].setting >> 8) & B1));
      server.radioButton2("m", 0, text_No, !((trigger[webTrig].setting >> 7) & B1) & !((trigger[webTrig].setting >> 8) & B1));
      server.radioButton2("m", 2, text_Timer, !((trigger[webTrig].setting >> 7) & B1) & ((trigger[webTrig].setting >> 8) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Pass); server.print(' '); server.printP(text_Off); server.print(' '); server.printP(text_time); server.printP(html_e_td_td);
      //server.printP(html_s_tag_s); server << 'r'; server.printP(html_m_tag); 
      //server << trigger[webTrig].off_time; server.printP(html_e_tag);
      printInput(server, 'r', trigger[webTrig].off_time, 0); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Pass); server.print(' '); server.printP(text_Off); server.print(' '); server.printP(text_interval); server.printP(html_e_td_td);
      server.printP(html_radio_sb);
      server.radioButton2("l", 0, text_seconds, !((trigger[webTrig].setting >> 15) & B1) & !((trigger[webTrig].setting >> 14) & B1));
      server.radioButton2("l", 1, text_minutes, !((trigger[webTrig].setting >> 15) & B1) & ((trigger[webTrig].setting >> 14) & B1));
      server.radioButton2("l", 2, text_hours, ((trigger[webTrig].setting >> 15) & B1) & !((trigger[webTrig].setting >> 14) & B1));
      server.radioButton2("l", 3, text_days, ((trigger[webTrig].setting >> 15) & B1) & ((trigger[webTrig].setting >> 14) & B1));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_To); server.print(' '); server.printP(text_address); server.printP(html_e_td_td);
      server.printP(html_select); server << 't'; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < nodes; ++ii) {
        if (node[ii].function == 'I') {
          server.printP(html_option); server << ii;
          if (node[ii].address == trigger[webTrig].to_address &&
              node[ii].number  == trigger[webTrig].to_number &&
              node[ii].type    == trigger[webTrig].to_type) { server.printP(html_selected); }
          else                                              { server.printP(html_e_tag); }
          printNodeAddress(server, ii);
          server.print(':'); printNodeType(server, node[ii].type);
          server.printP(html_e_option);
        }
        
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Constant); server.print(' '); server.printP(text_On); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'c'; server.printP(html_m_tag); 
      //server << trigger[webTrig].constant_on; server.printP(html_e_tag);
      printInput(server, 'c', trigger[webTrig].constant_on); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Constant); server.print(' '); server.printP(text_Off); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'f'; server.printP(html_m_tag); 
      //server << trigger[webTrig].constant_off; server.printP(html_e_tag);
      printInput(server, 'f', trigger[webTrig].constant_off); server.printP(html_e_td_e_tr);     
      server.printP(html_e_table);
      //server.printP(JSTrigger); // JavaScript         
      //server.printP(JSEnDis); // JavaScript
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
            strncpy(timer[webTimer].name, value, NAME_LEN);
            timer[webTimer].name[NAME_LEN-1] = 0;
            break;
          case 'r': timer[webTimer].run_time = strtol(value, NULL, 10);
            if (timer[webTimer].run_time == 0) timer[webTimer].run_time = 1;
          break;
          case 'a': timer[webTimer].asc_trigger = strtol(value, NULL, 10); break;
          case 'm': timer[webTimer].start_time = strtol(value, NULL, 10) + ((timer[webTimer].start_time / 60) * 60); break;
          case 'h': timer[webTimer].start_time = (strtol(value, NULL, 10) * 60) + (timer[webTimer].start_time % 60); break;
          case 'i': // Run interval
            n = value[0] - 48;
            if ((n >> 0) & 1) timer[webTimer].setting |= (1 << 14);
            else timer[webTimer].setting &= ~(1 << 14);
            if ((n >> 1) & 1) timer[webTimer].setting |= (1 << 15);
            else timer[webTimer].setting &= ~(1 << 15);
          break;
          case 'p': timer[webTimer].period = strtol(value, NULL, 10); 
            if (timer[webTimer].period == 0) timer[webTimer].period = 1;
            break;
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
          case 'B' ... 'J': // Handle all single radio buttons for settings B(66)=0
            if (value[0] == '0') timer[webTimer].setting &= ~(1 << (name[0]-66));
            else                 timer[webTimer].setting |=  (1 << (name[0]-66));
          break;
          case 'e': saveConf(); break; // save to EEPROM
        }
      } while (repeat);
      setTimer(webTimer); // At least set timer
      server.httpSeeOther(PREFIX "/i");
    } else {
      server.httpSuccess();
      webMenu(server, menu_Timers, ((timer[webTimer].setting >> 1) & B1));
      server.printP(html_h1); server.printP(text_Timer); server.print(' '); server.printP(text_setup); server.printP(html_e_h1);  
      server.printP(html_form_s); server << PREFIX "/i"; server.printP(html_form_e);    
      server.printP(html_table_tr_th_hash); server.printP(text_Name);
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
      server.printP(html_e_th_th); server.printP(text_Run); server.print(' '); server.printP(text_time);
      server.printP(html_e_th_th); server.printP(text_Assoc); server.print(' '); server.printP(text_trigger);
      server.printP(html_e_th_th); server.printP(text_On); server.print(' '); server.printP(text_time);
      server.printP(html_e_th_th); server.printP(text_Off); server.print(' '); server.printP(text_time);      
      server.printP(html_e_th_th); server.printP(text_To); server.print(' '); server.printP(text_address);
      server.printP(html_e_th_th); server.printP(text_On);
      server.printP(html_e_th_th); server.printP(text_Off);
      server.printP(html_e_th_th); server.printP(text_Trigger); server.printP(text_ed);
      server.printP(html_e_th_e_tr);
      for (uint8_t i = 0; i < TIMERS; ++i) {
        server.printP(html_tr_td); 
        server << i+1; server.print('.'); server.printP(html_e_td_td);
        server << timer[i].name; server.printP(html_e_td_td);
        (timer[i].setting & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((timer[i].setting >> 1) & B1) ? server.printP(text_Calendar) : server.printP(text_Period); server.printP(html_e_td_td);
        ((timer[i].setting >> 7) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((timer[i].setting >> 6) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((timer[i].setting >> 5) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((timer[i].setting >> 4) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((timer[i].setting >> 3) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((timer[i].setting >> 2) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        ((timer[i].setting >> 8) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_td);
        n = (timer[i].start_time / 60); 
        if (n < 10) server << '0'; server << n;
        server.print(':');
        n = (timer[i].start_time % 60);
        if (n < 10) server << '0'; server << n;
        server.printP(html_e_td_td);
        server << timer[i].period; server.print(' '); 
        printPeriodType(server, timer[i].setting >> 12 & B11);
        
        server.printP(html_e_td_td);
        server << timer[i].run_time ; server.print(' ');
        printPeriodType(server, timer[i].setting >> 14 & B11);
        
        server.printP(html_e_td_td);
        if (timer[i].asc_trigger == 0) { server.printP(text_none); }
        else                           { 
          server << timer[i].asc_trigger; server.printP(text_spdashsp);
          server << trigger[timer[i].asc_trigger-1].name; server.printP(text_spdashsp);
          ((trigger[timer[i].asc_trigger-1].setting >> 5) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled);
         }
        server.printP(html_e_td_td);
        time_temp.set(timer[i].next_on); server.print((char*)time_temp.formatedDateTime()); server.printP(html_e_td_td);
        time_temp.set(timer[i].next_off); server.print((char*)time_temp.formatedDateTime()); server.printP(html_e_td_td);
        printNodeAddress(server, 'I', timer[i].to_address, timer[i].to_number, timer[i].to_type);
        server.printP(html_e_td_td);      
        server << timer[i].constant_on; server.printP(html_e_td_td);
        server << timer[i].constant_off; server.printP(html_e_td_td);
        ((timer[i].setting >> 9) & B1) ? server.printP(text_i_OK) : server.printP(text_i_disabled); server.printP(html_e_td_e_tr);
      }
      server.printP(html_e_table); 
      server.printP(html_table_tr_td);
      server.printP(text_Timer); server.printP(html_e_td_td);
      server.printP(html_select_submit); server << 'P'; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < TRIGGERS; ++ii) {
        server.printP(html_option); server << ii;
        if (webTimer == ii) { server.printP(html_selected); }
        else                { server.printP(html_e_tag); }
        server << ii + 1; server.printP(text_spdashsp); server << timer[ii].name; server.printP(html_e_option);
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Name); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'n'; server.printP(html_m_tag); server << timer[webTimer].name; server.printP(html_e_tag);
      printInput(server, 'n', timer[webTimer].name); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Timer); server.print(' '); server.printP(text_is); server.printP(html_e_td_td);
      printOnOffButton(server, "B", timer[webTimer].setting & B1);
      server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Type); server.printP(html_e_td_td);      
      server.printP(html_radio_sl);
      server.radioButton2("C", 1, text_Calendar, (timer[webTimer].setting >> 1) & B1, true);
      server.radioButton2("C", 0, text_Period, !((timer[webTimer].setting >> 1) & B1), true);
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      flashPrintln(server, &PGM_dow[1]); server.printP(html_e_td_td);
      printOnOffButton(server, "I", (timer[webTimer].setting >> 7) & B1);
      server.printP(html_e_td_e_tr_tr_td); 
      flashPrintln(server, &PGM_dow[2]); server.printP(html_e_td_td);
      printOnOffButton(server, "H", (timer[webTimer].setting >> 6) & B1);
      server.printP(html_e_td_e_tr_tr_td); 
      flashPrintln(server, &PGM_dow[3]); server.printP(html_e_td_td);
      printOnOffButton(server, "G", (timer[webTimer].setting >> 5) & B1);
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td); 
      flashPrintln(server, &PGM_dow[4]); server.printP(html_e_td_td);
      printOnOffButton(server, "F", (timer[webTimer].setting >> 4) & B1);
      server.printP(html_e_td_e_tr_tr_td); 
      flashPrintln(server, &PGM_dow[5]); server.printP(html_e_td_td);
      printOnOffButton(server, "E", (timer[webTimer].setting >> 3) & B1);
      server.printP(html_e_td_e_tr_tr_td); 
      flashPrintln(server, &PGM_dow[6]); server.printP(html_e_td_td);
      printOnOffButton(server, "D", (timer[webTimer].setting >> 2) & B1);
      server.printP(html_e_td_e_tr_tr_td); 
      flashPrintln(server, &PGM_dow[0]); server.printP(html_e_td_td);
      printOnOffButton(server, "J", (timer[webTimer].setting >> 8) & B1);
      server.printP(html_e_td_e_tr_tr_td); 

      server.printP(text_Start); server.print(' '); server.printP(text_time); server.printP(html_e_td_td);
      server.printP(html_select); server << 'h'; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < 24; ++ii) {
        if ((timer[webTimer].start_time / 60) == ii)  
          { server.printP(html_option); server << ii; server.printP(html_selected); server << ii;}
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii;}
         server.printP(html_e_option);         
      }
      server.printP(html_e_select); server.printP(text_sesp);
      server.printP(html_select); server << 'm'; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < 60; ++ii) {
        if ((timer[webTimer].start_time % 60) == ii)  
          { server.printP(html_option); server << ii; server.printP(html_selected); server << ii;}
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii;}
        server.printP(html_e_option); 
      }
      server.printP(html_e_select);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Period); server.print(' '); server.printP(text_time); server.printP(html_e_td_td);
      //server.printP(html_s_tag_s); server << 'p'; server.printP(html_m_tag); 
      //server << timer[webTimer].period; server.printP(html_id_tag); server << 'p'; 
      //server.printP(html_e_tag);
      printInput(server, 'p', timer[webTimer].period, 0); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Period); server.print(' '); server.printP(text_interval); server.printP(html_e_td_td);
      server.printP(html_radio_sb);
      server.radioButton2("l", 0, text_seconds, !((timer[webTimer].setting >> 13 & B1) & (timer[webTimer].setting >> 12 & B1)));
      server.radioButton2("l", 1, text_minutes, (!(timer[webTimer].setting >> 13 & B1) & (timer[webTimer].setting >> 12 & B1)));
      server.radioButton2("l", 2, text_hours, ((timer[webTimer].setting >> 13 & B1) & !(timer[webTimer].setting >> 12 & B1)));
      server.radioButton2("l", 3, text_days, ((timer[webTimer].setting >> 13 & B1) & (timer[webTimer].setting >> 12 & B1)));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Run); server.print(' '); server.printP(text_time); server.printP(html_e_td_td);
      //server.printP(html_s_tag_s); server << 'r'; server.printP(html_m_tag); 
      //server << timer[webTimer].run_time; server.printP(html_e_tag);
      printInput(server, 'r', timer[webTimer].run_time, 0); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Run); server.print(' '); server.printP(text_interval); server.printP(html_e_td_td);
      server.printP(html_radio_sb);
      server.radioButton2("i", 0, text_seconds, !((timer[webTimer].setting >> 15 & B1) & (timer[webTimer].setting >> 14 & B1)));
      server.radioButton2("i", 1, text_minutes, (!(timer[webTimer].setting >> 15 & B1) & (timer[webTimer].setting >> 14 & B1)));
      server.radioButton2("i", 2, text_hours, ((timer[webTimer].setting >> 15 & B1) & !(timer[webTimer].setting >> 14 & B1)));
      server.radioButton2("i", 3, text_days, ((timer[webTimer].setting >> 15 & B1) & (timer[webTimer].setting >> 14 & B1)));
      server.printP(html_div_e); server.printP(html_e_td_e_tr_tr_td);

      server.printP(text_To); server.print(' '); server.printP(text_address); server.printP(html_e_td_td);
      server.printP(html_select); server << 't'; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < nodes; ++ii) {
        if (node[ii].function == 'I') {
          server.printP(html_option); server << ii;
          if (node[ii].address == timer[webTimer].to_address &&
              node[ii].number  == timer[webTimer].to_number &&
              node[ii].type    == timer[webTimer].to_type) { server.printP(html_selected); }
          else               { server.printP(html_e_tag); }
          printNodeAddress(server, ii);
          server.print(':'); printNodeType(server, node[ii].type);
          server.printP(html_e_option);
        }
        
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);

      server.printP(text_Assoc); server.print(' '); server.printP(text_trigger); server.printP(html_e_td_td);
      server.printP(html_select); server << 'a'; server.printP(html_e_tag);
      for (uint8_t ii = 0; ii < TRIGGERS + 1; ++ii) {
        server.printP(html_option); server << ii;
        if ( timer[webTimer].asc_trigger == ii) { server.printP(html_selected); }
        else                                    { server.printP(html_e_tag); }
        if (ii == 0) { server.printP(text_none); }
        else         { server << ii; server.printP(text_spdashsp); server << trigger[ii-1].name; }
        server.printP(html_e_option);
      }
      server.printP(html_e_select); server.printP(html_e_td_e_tr_tr_td);

      server.printP(text_Constant); server.print(' '); server.printP(text_On); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'o'; server.printP(html_m_tag); 
      //server << timer[webTimer].constant_on; server.printP(html_e_tag);
      printInput(server, 'o', timer[webTimer].constant_on); server.printP(html_e_td_e_tr_tr_td); 
      server.printP(text_Constant); server.print(' '); server.printP(text_Off); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'f'; server.printP(html_m_tag); 
      //server << timer[webTimer].constant_off; server.printP(html_e_tag);
      printInput(server, 'f', timer[webTimer].constant_off); server.printP(html_e_td_e_tr);     
      server.printP(html_e_table);
      server.printP(JSTimer); // JavaScript to toggle Clendar/Period related items          
      server.printP(JSEnDis); // JavaScript
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
          case 'u': strncpy(conf.user, value, USER_LEN);// user
                    conf.user[USER_LEN-1] = 0;
                    break;
          case 'p': // password
            strncpy(conf.password, value, USER_LEN);
            conf.password[USER_LEN-1] = 0;
            b64_text[0] = 0;
            strcat (b64_text, conf.user);
            strcat (b64_text, ":");
            strcat (b64_text, conf.password);
            base64Encode(conf.user_pass, b64_text, strlen(b64_text));
            //WS.print("p>"); WS.println(_text);
            //WS.println(strlen(conf.user_pass));
            //WS.println(conf.user_pass);
          break;
          case 'i': // user
            strncpy(conf.SMTP_user, value, EMAIL_LEN);
            conf.SMTP_user[EMAIL_LEN-1] = 0;
          break;
          case 'o': // password
            strncpy(conf.SMTP_password, value, USER_LEN);
            conf.SMTP_password[USER_LEN-1] = 0;
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
      server.printP(html_h1); server.printP(text_Global); server.print(' '); server.printP(text_setup); server.printP(html_e_h1);  
      server.printP(html_form_s); server << PREFIX "/r"; server.printP(html_form_e);    
      server.printP(html_table_tr_td);
      server.printP(text_System); server.print(' '); server.printP(text_user); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'u'; server.printP(html_m_tag); server << conf.user; server.printP(html_e_tag);
      printInput(server, 'u', conf.user); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_System); server.print(' '); server.printP(text_password); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'p'; server.printP(html_m_tag); server << conf.password; server.printP(html_e_tag);
      printInput(server, 'p', conf.password); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Email); server.print(' '); server.printP(text_user); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'i'; server.printP(html_m_tag); server << conf.SMTP_user; server.printP(html_e_tag);
      printInput(server, 'i', conf.SMTP_user); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Email); server.print(' '); server.printP(text_password); server.printP(html_e_td_td);
      //server.printP(html_s_tag); server << 'o'; server.printP(html_m_tag); server << conf.SMTP_password; server.printP(html_e_tag);
      printInput(server, 'o', conf.SMTP_password); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Authentication); server.print(' '); server.printP(text_time);server.printP(html_e_td_td);
      server.printP(html_select); server << 'a'; server.printP(html_e_tag);
      for (uint8_t ii = 5; ii < 26; ++ii) {
        if ((conf.alr_time) == ii)  
          { server.printP(html_option); server << ii; server.printP(html_selected); server << ii;}
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii;}
        server.printP(html_e_option); 
      }
      server.printP(html_e_select);
      server.print(' '); server.printP(text_seconds); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Arm); server.print(' '); server.printP(text_delay); server.printP(html_e_td_td);
      
      server.printP(html_select); server << 'd'; server.printP(html_e_tag);
      for (uint8_t ii = 10; ii < 41; ++ii) {
        if ((conf.arm_delay/4) == ii) // 250*4 = 1 sec.
          { server.printP(html_option); server << ii; server.printP(html_selected); server << ii;}
        else 
          { server.printP(html_option); server << ii; server.printP(html_e_tag); server << ii;}
        server.printP(html_e_option); 
      }
      server.printP(html_e_select);
      server.print(' '); server.printP(text_seconds); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Auto); server.print(' '); server.printP(text_arm); server.print(' '); server.printP(text_delay); server.printP(html_e_td_td);
      server.printP(html_select); server << 'b'; server.printP(html_e_tag);
      for (uint8_t ii = 10; ii < 241; ii+=10) {
        if ((conf.auto_arm) == ii) {
          server.printP(html_option); server << ii; server.printP(html_selected); 
          //if (ii>59) { server << '0' << ii/60 << ':' << ii%60;}
          //else       { server << ii;}
          server << ii;
          server.printP(html_e_option);
        }
        else { 
          server.printP(html_option); server << ii; server.printP(html_e_tag);
          //if (ii>59) { server << '0' << ii/60 << ':'' << ii%60;}
          //else       { server << ii;}
          server << ii;
          server.printP(html_e_option);
        }
      }
      server.printP(html_e_select);
      server.print(' '); server.printP(text_minutes); server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Open); server.print(' '); server.printP(text_alarm); server.printP(html_e_td_td);
      server.printP(html_select); server << 'c'; server.printP(html_e_tag);
      for (uint8_t ii = 10; ii < 241; ii+=5) {
        if ((conf.open_alarm) == ii) {
          server.printP(html_option); server << ii; server.printP(html_selected); 
      //if (ii>59) { server << '0' << ii/60 << ':' << ii%60;}
      //else       { server << ii;}
          server << ii;
          server.printP(html_e_option); }
          else { 
            server.printP(html_option); server << ii; server.printP(html_e_tag);
      //if (ii>59) { server << '0' << ii/60 << ':' << ii%60;}
      //else       { server << ii;}
            server << ii;
            server.printP(html_e_option);
          }
      }
      server.printP(html_e_select);
      server.print(' '); server.printP(text_minutes); server.printP(html_e_td_e_tr);
      server.printP(html_e_table);
      server.printP(html_h1); server.printP(text_Alert); server.print('s'); server.printP(html_e_h1);  
      server.printP(html_table_tr_th);
      server.printP(text_Name); server.printP(html_e_th_th);
      server.printP(text_SMS); server.printP(html_e_th_th);
      server.printP(text_Email); server.printP(html_e_th_th);
      server.printP(text_Page); server.printP(html_e_th_e_tr);
      server.printP(html_tr_td);
      server.printP(text_Undefined); server.print(' '); server.printP(text_key); server.printP(html_e_td_td);
      printOnOffButton(server, "0A", conf.alerts[alert_SMS] >> ALERT_FALSE_KEY & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "1A", conf.alerts[alert_email] >> ALERT_FALSE_KEY & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "2A", conf.alerts[alert_page] >> ALERT_FALSE_KEY & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Group); server.print(' '); server.printP(text_disarmed); server.printP(html_e_td_td);
      printOnOffButton(server, "0B", conf.alerts[alert_SMS] >> ALERT_DISARMED & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "1B", conf.alerts[alert_email] >> ALERT_DISARMED & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "2B", conf.alerts[alert_page] >> ALERT_DISARMED & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Group); server.print(' '); server.printP(text_armed); server.printP(html_e_td_td);
      printOnOffButton(server, "0C", conf.alerts[alert_SMS] >> ALERT_ARMED & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "1C", conf.alerts[alert_email] >> ALERT_ARMED & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "2C", conf.alerts[alert_page] >> ALERT_ARMED & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Zone); server.print(' '); server.printP(text_open); server.print(' '); server.printP(text_alarm); server.printP(html_e_td_td);
      printOnOffButton(server, "0D", conf.alerts[alert_SMS] >> ALERT_OPEN & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "1D", conf.alerts[alert_email] >> ALERT_OPEN & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "2D", conf.alerts[alert_page] >> ALERT_OPEN & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Zone); server.print(' '); server.printP(text_tamper); server.print(' '); server.printP(text_alarm); server.printP(html_e_td_td);
      printOnOffButton(server, "0E", conf.alerts[alert_SMS] >> ALERT_TAMPER & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "1E", conf.alerts[alert_email] >> ALERT_TAMPER & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "2E", conf.alerts[alert_page] >> ALERT_TAMPER & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Zone); server.print(' '); server.printP(text_alarm); server.printP(html_e_td_td);
      printOnOffButton(server, "0F", conf.alerts[alert_SMS] >> ALERT_PIR & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "1F", conf.alerts[alert_email] >> ALERT_PIR & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "2F", conf.alerts[alert_page] >> ALERT_PIR & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_System); server.print(' '); server.printP(text_alarm); server.printP(html_e_td_td);
      printOnOffButton(server, "0G", conf.alerts[alert_SMS] >> ALERT_ALARM & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "1G", conf.alerts[alert_email] >> ALERT_ALARM & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "2G", conf.alerts[alert_page] >> ALERT_ALARM & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Monitoring); server.print(' '); server.printP(text_started); server.printP(html_e_td_td);
      printOnOffButton(server, "0H", conf.alerts[alert_SMS] >> ALERT_MONITORING_STARTED & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "1H", conf.alerts[alert_email] >> ALERT_MONITORING_STARTED & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "2H", conf.alerts[alert_page] >> ALERT_MONITORING_STARTED & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Queue); server.print(' '); server.printP(text_full); server.printP(html_e_td_td);
      printOnOffButton(server, "0I", conf.alerts[alert_SMS] >> ALERT_QUEUE & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "1I", conf.alerts[alert_email] >> ALERT_QUEUE & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "2I", conf.alerts[alert_page] >> ALERT_QUEUE & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Configuration); server.print(' '); server.printP(text_saved); server.printP(html_e_td_td);
      printOnOffButton(server, "0J", conf.alerts[alert_SMS] >> ALERT_CONF_SAVED & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "1J", conf.alerts[alert_email] >> ALERT_CONF_SAVED & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "2J", conf.alerts[alert_page] >> ALERT_CONF_SAVED & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Power); server.print(' '); server.printP(text_state); server.printP(html_e_td_td);
      printOnOffButton(server, "0K", conf.alerts[alert_SMS] >> ALERT_AC_STATE & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "1K", conf.alerts[alert_email] >> ALERT_AC_STATE & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "2K", conf.alerts[alert_page] >> ALERT_AC_STATE & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Battery); server.print(' '); server.printP(text_state); server.printP(html_e_td_td);
      printOnOffButton(server, "0L", conf.alerts[alert_SMS] >> ALERT_BATERY_STATE & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "1L", conf.alerts[alert_email] >> ALERT_BATERY_STATE & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "2L", conf.alerts[alert_page] >> ALERT_BATERY_STATE & B1);
      server.printP(html_e_td_e_tr_tr_td);
      server.printP(text_Trigger); server.print(' '); server.printP(text_alarm); server.printP(html_e_td_td);
      printOnOffButton(server, "0M", conf.alerts[alert_SMS] >> ALERT_TRIGGER & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "1M", conf.alerts[alert_email] >> ALERT_TRIGGER & B1);
      server.printP(html_e_td_td);
      printOnOffButton(server, "2M", conf.alerts[alert_page] >> ALERT_TRIGGER & B1);
      server.printP(html_e_td_e_tr);
      server.printP(html_e_table);
      server.printP(html_F_A); // submit Apply
      server.printP(html_F_SA); // submit Save all
      server.printP(html_e_form);
      server.printP(htmlFoot);
    }
  } else { server.httpUnauthorized(); }
}

#endif