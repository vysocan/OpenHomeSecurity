#ifndef HTML_h
#define HTML_h

// no-cost stream operator as described at // http://sundial.org/arduino/?page_id=119
template<class T>
inline Print &operator <<(Print &obj, T arg)
{ obj.print(arg); return obj; }

	
P(text_System)         = "System";
P(text_system)         = "system";
P(text_battery)        = "battery";
P(text_low)            = "low";
P(text_power)          = "power";
P(text_Power)          = "Power";
P(text_state)          = "state";
P(text_On)             = "On";
P(text_Off)            = "Off";
P(text_0)              = "0";
P(text_1)              = "1";
P(text_2)              = "2";
P(text_3)              = "3";
P(text_x)              = " x ";
P(text_Yes)            = "Yes";
P(text_No)             = "No";	
P(text_Configuration)  = "Configuration";
P(text_configuration)  = "configuration";
P(text_saved)          = "saved";
P(text_loaded)         = "loaded";
P(text_reset)          = "reset";
P(text_armed)          = "armed";
P(text_Arm)            = "Arm";
P(text_arm)            = "arm";
P(text_Armed)          = "Armed";
P(text_ALARM)          = "ALARM !!!";
P(text_Monitoring)     = "Monitoring";
P(text_monitoring)     = "monitoring";
P(text_started)        = "started";
P(text_start)          = "start";
P(text_end)            = "end";
P(text_undefined)      = "undefined";
P(text_Undefined)      = "Undefined";
P(text_Alarm)          = "Alarm";
P(text_alarm)          = "alarm";
P(text_Last)           = "Last";
P(text_zone)           = "zone";
P(text_Tamper)         = "Tamper";
P(text_tamper)         = "tamper";
P(text_Damaged)        = "Damaged";
P(text_Authentication) = "Authentication";
P(text_trigger)        = "trigger";
P(text_Trigger)        = "Trigger";
P(text_timer)          = "timer";
P(text_Timer)          = "Timer";
P(text_ed)             = "ed";
P(text_disarmed)       = "disarmed";
P(text_OK)             = "OK";
P(text_not)            = "not";
P(text_disabled)       = "disabled";
P(text_enabled)        = "enabled";
P(text_Enabled)        = "Enabled";
P(text_all)            = "all";
P(text_activated)      = "activated";
P(text_de)             = "de";
P(text_none)           = "none";
P(text_Assoc)          = "Assoc.";
P(text_Hysteresis)     = "Hysteresis";

P(text_i_OK)             = "<i class='fa fa-check'></i>";
P(text_i_ALARM)          = "<i class='fa fa-bell'></i>";
P(text_i_disabled)       = "<i class='fa fa-ban'></i>";
P(text_i_starting)       = "<i class='fa fa-spinner fa-pulse'></i>";
P(text_i_home)           = "<i class='fa fa-home'></i>";
P(text_i_question)       = "<i class='fa fa-question'></i>";
P(text_i_zone)           = "<i class='fa fa-square-o'></i>";
P(text_i_qlobe)          = "<i class='fa fa-globe'></i>";
P(text_i_auth)           = "<i class='fa fa-lock'></i>";
P(text_i_contact)        = "<i class='fa fa-address-card-o'></i>";
P(text_i_key)            = "<i class='fa fa-key'></i>";
P(text_i_sens)           = "<i class='fa fa-share-alt'></i>";

P(text_nr)     = "not registered"; 
P(text_rh)     = "registered (home)";
P(text_nrs)    = "not registered (searching)"; 
P(text_rd)     = "registration denied"; 
P(text_rr)     = "registered (roaming)";
P(text_unk)    = "unknown";
P(text_Zone)   = "Zone";
P(text_Type)   = "Type";
P(text_Function) = "Function";
P(text_Group)  = "Group";
P(text_group)  = "group";
P(text_Phone)  = "Phone";
P(text_phone)  = "phone";
P(text_Contact)= "Contact";
P(text_contact)= "contact";
P(text_Remote) = "Remote";
P(text_node)   = "node";
P(text_Node)   = "Node";
P(text_unit)   = "unit";
P(text_Unit)   = "Unit";
P(text_Key)    = "Key";
P(text_key)    = "key";
P(text_name)   = "name";
P(text_Name)   = "Name";
P(text_Number) = "Number";
P(text_number) = "number";
P(text_Email)  = "Email";
P(text_Page)   = "Page";
P(text_for)    = "for";
P(text_Status) = "Status";
P(text_Log)    = "Log";
P(text_Global) = "Global";
P(text_setup)  = "setup";
P(text_digital)= "digital";
P(text_analog) = "analog";
P(text_Float)  = "Float";
P(text_Analog) = "Analog";
P(text_Digital)= "Digital";
P(text_auto)   = "auto";
P(text_Auto)   = "Auto";
P(text_To)     = "To";
P(text_GSM)    = "GSM";
P(text_SMS)    = "SMS";
P(text_Debug)  = "Debug";
P(text_Delay)  = "Delay";
P(text_delay)  = "delay";
P(text_Alert) = "Alert";
P(text_Logging) = "Logging";
P(text_modem)  = "modem";
P(text_is)     = "is";
P(text_network)= "network";
P(text_Network)= "Network";
P(text_Signal)   = "Signal";
P(text_strength) = "strength";
P(text_Time)   = "Time";
P(text_time)   = "time";
P(text_Uptime) = "Uptime";
P(text_Start) = "Start";
P(text_seconds)    = "second(s)";
P(text_minutes)    = "minute(s)";
P(text_hours)  = "hour(s)";
P(text_days)   = "day(s)";
/*
P(text_space)  = " ";
P(text_colon)  = ":";
P(text_semic)  = ";";
P(text_dot)    = ".";
P(text_percent)  = "%";
P(text_comma)    = ",";
*/
P(text_cosp)    = ", ";
P(text_sesp)   = ": ";
P(text_spdashsp) = " - ";
P(text_PwrSp)  = "Power supply";
P(text_Battery)= "Battery";
P(text_Date)   = "Date";
P(text_Message)= "Message";
P(text_Address)= "Address";
P(text_address)= "address";
P(text_IP)= "IP";
P(text_port)= "port";
P(text_Gateway)= "Gateway";
P(text_Mask)= "Mask";
P(text_NTP)= "NTP";
P(text_connection) = "connection";
P(text_out)        = "out";
P(text_lost)       = "lost";
P(text_bad)        = "bad";
P(text_protocol)   = "protocol";
P(text_client_id)  = "client ID";
P(text_unavailable)     = "unavailable";
P(text_credentials) = "credentials";
P(text_unauthorized)    = "unauthorized";
P(text_registered) = "registered";
P(text_registration) = "registration";
P(text_Registration) = "Registration";
P(text_queue) = "queue";
P(text_Queue) = "Queue";
P(text_error) = "error";
P(text_re)     = "re";
P(text_iButton)= "iButton";
P(text_Temperature) = "Temperature";
P(text_Humidity)    = "Humidity";
P(text_Pressure)    = "Pressure";
P(text_Input)       = "Input";
P(text_Voltage)     = "Voltage";
P(text_TX_Power)    = "TX Power";
P(text_sensor)      = "sensor";
P(text_Sensor)      = "Sensor";
P(text_Value)       = "Value";
P(text_value)       = "value";
P(text_Constant)       = "Constant";
P(text_constant)       = "constant";
P(text_Pass)       = "Pass";
P(text_once)       = "once";
P(text_Symbol)       = "Symbol";
P(text_degC)        = " °C";
P(text_mBar)        = " mBar";
P(text_Volt)        = " V";
P(text_Open)   = "Open";
P(text_open)   = "open";
P(text_OUT1)   = "OUT 1";
P(text_OUT2)   = "OUT 2";
P(text_MQTT)   = "MQTT";
P(text_failed)   = "failed";
P(text_Failed)   = "Failed";
P(text_868)   = "868";
P(text_915)   = "915";
P(text_Frequency)   = "Frequency";
P(text_OHS)    = "OHS";
P(text_slash)  = "/";
P(text_publish) = "publish";
P(text_subscribe) = "subscribe";
P(text_Radio)  = "Radio";
P(text_Received)  = "Received";
P(text_packets)  = "packets";
P(text_Not)  = "Not";
P(text_as)  = "as";
P(text_acknowledged)  = "acknowledged";
P(text_requested) = "requested";
P(text_Fifo)  = "Fifo";
P(text_full)  = "full";
P(text_sent)    = "sent";
P(text_Sent)    = "Sent";
P(text_connected) = "connected";
P(text_hash)   = "#";
P(text_User)   = "User";
P(text_Password)   = "Password";
P(text_user)   = "user";
P(text_password)   = "password";
P(text_erased) = "erased";
P(text_Period) = "Period";
P(text_Calendar) = "Calendar";
P(text_Run) = "Run";
P(text_interval) = "interval";
P(text_Mo) = "Mo";
P(text_Tu) = "Tu";
P(text_We) = "We";
P(text_Th) = "Th";
P(text_Fr) = "Fr";
P(text_Sa) = "Sa";
P(text_Su) = "Su";
P(text_Monday) = "Monday";
P(text_Tuesday) = "Tuesday";
P(text_Wednesday) = "Wednesday";
P(text_Thursday) = "Thursday";
P(text_Friday) = "Friday";
P(text_Saturday) = "Saturday";
P(text_Sunday) = "Sunday";
P(text_DS) = "Daylight saving";
P(text_flag) = "flag";
P(text_of) = "of";
P(text_at) = "at";
P(text_offset) = "offset";



P(html_F_SA)    = "<input type='submit' name='e' value='Save all'/>";
P(html_F_RD)    = "<input type='submit' name='r' value='Reset to default'/>";
P(html_F_LA)    = "<input type='submit' name='l' value='Load last'/>";
P(html_F_A)     = "<input type='submit' name='A' value='Apply'/>";
P(html_F_S)     = "<input type='submit' name='S' value='Select'/>";
P(html_F_LOG)   = "<input type='submit' name='p' value='<<'/><input type='submit' name='n' value='now'/><input type='submit' name='f' value='>>'/>";
//P(html_F_Clear) = "<input type='submit' name='C' value='Clear log' onsubmit='return cL()'/>";
P(html_F_Clear) = "<input type='submit' name='C' value='Clear log'/>";
//P(html_F_CL)    = "<script>function cL() {alert('Erasing entire log, it takes few seconds.');return true;}</script>";  
P(html_F_GetNTP)= "<input type='submit' name='T' value='Get NTP time'/>";
P(html_F_RR)    = "<input type='submit' name='R' value='Reregister'/>";
P(html_e_table) = "</table>";
P(html_table)   = "<table>";
P(html_table_tr_td)   = "<table><tr><td>";
P(html_table_tr_th)   = "<table><tr><th>";
P(html_table_tr_th_hash)   = "<table><tr><th>#</th><th>";
P(html_e_td)    = "</td>";
P(html_e_td_td)    = "</td><td>";
P(html_td)      = "<td>";
P(html_e_tr)    = "</tr>";
P(html_e_td_e_tr)    = "</td></tr>";
P(html_e_td_e_tr_tr_td)    = "</td></tr><tr><td>";
P(html_tr)      = "<tr>";
P(html_tr_td)      = "<tr><td>";
P(html_e_th)    = "</th>";
P(html_e_th_th)    = "</th><th>";
P(html_e_th_e_tr)    = "</th><tr>";
P(html_th)      = "<th>";
P(html_h1)      = "<h1>";
P(html_e_h1)    = "</h1>";
P(html_pre)     = "<pre>";
P(html_e_pre)     = "</pre>";
P(html_form_s)    = "<form action='";
P(html_form_e)    = "' method='post'>";
P(html_form_e_CL) = "' method='post'>";
P(html_e_form)  = "</form>";
P(html_select)  = "<select name='";
P(html_select_submit)  = "<select onchange='this.form.submit()' name='";
P(html_e_select)  = "</select>";
P(html_s_tag)   = "<input type='text' name='";
P(html_s_tag_s) = "<input type='text' maxlength='3' size='3' name='";
P(html_m_tag)   = "' value='";
P(html_e_tag)   = "'>";
P(html_option)  = "<option value='";
P(html_e_option)  = "</option>";
P(html_selected)  = "' selected>";
P(html_br)     = "</br>";

P(html_radio_s)  = "<div class='rm'>";
P(html_radio_sl) = "<div class='rml'>";
P(html_radio_sb) = "<div class='rmb'>";
P(html_div_e)    = "</div>";

// "@import url(\"http://fonts.googleapis.com/css?family=Montserrat:400,700\");\n"
// "body {font: 14px/1 'Montserrat', sans-serif;color: #333;background: #333;overflow-x: hidden}\n"

P(htmlHead_s) = "<!DOCTYPE html><html ><head><meta charset=\"UTF-8\"><title>Open home security</title>"
"<style type=\"text/css\">\n"
"@import url(\"http://netdna.bootstrapcdn.com/font-awesome/4.7.0/css/font-awesome.css\");\n"
"* {margin:0;padding:0}\n"
"*, *:before, *:after {box-sizing:border-box}\n"
"html, body {height:100%}\n"
"body {font: 12px/1 Georgia, sans-serif;color: #333;background: #333;overflow-x: hidden}\n"
".wrp {display:flex;min-height:100%}\n"
".sbar {position:absolute;width:160px}\n"
".content {flex:1;padding:10px 10px 1px 20px;;background:#eee;box-shadow:0 0 5px black;transform: translate3d(0,0,0);transition:transform .3s}\n"
".content.isOpen {transform: translate3d(160px,0,0)}\n"
".title {font-size:22px;line-height:50px;text-align:center;color:#eee;border-bottom:1px solid #222;background:#2a2a2a}\n"
".nav li a {position:relative;display:block;padding:15px 15px 15px 50px;color:#eee;border-bottom:1px solid #222}\n"
".nav li a:before {font:14px fontawesome;position:absolute;top:14px;left:20px}\n"
".nav li:nth-child(1) a:before {content:'\\f015'}"
".nav li:nth-child(2) a:before {content:'\\f085'}"
".nav li:nth-child(3) a:before {content:'\\f096'}"
".nav li:nth-child(4) a:before {content:'\\f24d'}"
".nav li:nth-child(5) a:before {content:'\\f084'}"
".nav li:nth-child(6) a:before {content:'\\f2bc'}"
".nav li:nth-child(7) a:before {content:'\\f1e0'}"
".nav li:nth-child(8) a:before {content:'\\f064'}"
".nav li:nth-child(9) a:before {content:'\\f017'}"
".nav li:nth-child(10) a:before {content:'\\f0e8'}"
".nav li:nth-child(11) a:before {content:'\\f15c'}"
".nav li:nth-child(12) a:before {content:'\\f188'}"
".nav li a:hover {background:#444}\n"
".nav li a.active {box-shadow:inset 5px 0 0 #5b5,inset 6px 0 0 #222;background:#444}\n"
"input, select {cursor:pointer;height: 25px;border: 0px none;outline: 0px none;box-shadow: 0px 1px 2px rgba(0, 0, 0, 0.2);padding: 0px 5px;background:#fff}\n"
"input[type='submit'] {margin: 0px 10px 10px 0px}\n"
"h1 {margin: 0px 0 5px;font-size: 28px;padding: 10px}\n"
"h2 {font-size: 18px;font-weight: 400;color: #999}\n"
"table{margin:0 0 10px;border-collapse:collapse;border:1px solid #bbb;background-color:#fff}"
"th, td {padding:8px;border-right:1px solid #bbb}"
"th {font-weight:600;background:#ddd}"
"tr:nth-child(even) {background: #eee}"
"a:link {text-decoration:none}\n"
".rm, .rml, .rmb {display:flex;height:25px;margin:0;background:#fff;box-shadow:0 1px 2px rgba(0,0,0,.2);overflow:hidden}\n"
".rm {width:80px;}\n"
".rml {width:160px;}\n"
".rmb {width:320px;}\n"
".rc {flex:1}\n"
".rc input {position:absolute;opacity:0}\n"
".rc label {display:block;height:25px;line-height:25px;text-align:center;cursor:pointer}\n"
".rc label:hover {background:#eee}\n"
".rc input:checked ~ label {color:#fff;background:#5b5}\n"
"</style></head>\n<body>"
"<div class='wrp'><div class='sbar'><div class='title'>OHS v.1.7.5</div><ul class='nav'>";

P(htmlHead_e) = "</ul></div><div class='content isOpen'>";

P(htmlFoot) = "</div></div></body></html>";	

P(html_li_a)          = "<li><a ";
P(html_li_e)          = "</a></li>";
P(html_li_e_li_a)     = "</a></li><li><a ";
P(html_c_active)      = "class='active' ";
P(html_href)          = "href='";
P(html_menu_Home)     = "/'>Home";
P(html_menu_Global)   = "r'>Global";
P(html_menu_Zones)    = "z'>Zones";
P(html_menu_Groups)   = "g'>Groups";
P(html_menu_Keys)     = "k'>Keys";
P(html_menu_Contacts) = "c'>Contacts";
P(html_menu_Timers)   = "i'>Timers";
P(html_menu_Sens)     = "s'>Nodes";
P(html_menu_Triggers) = "t'>Triggers";
P(html_menu_MQTT)     = "m'>Network";
P(html_menu_Log)      = "l'>Log";
P(html_menu_Debug)    = "d'>Debug";


#endif


	
