#ifndef HTML_h
#define HTML_h
	
	P(text_System)         = "System";
	P(text_BR)             = "battery reached ";
	P(text_low)            = "low";
	P(text_high)           = "high";
	P(text_level)          = "level";
	P(text_PWS)            = "power was switched ";
	P(text_On)             = "On";
	P(text_Off)            = "Off";
	P(text_Yes)            = "Yes";
	P(text_No)             = "No";	
	P(text_CS)             = "configuration saved";
	P(text_armed)          = "armed";
	P(text_Armed)          = "Armed";
	P(text_ALARM)          = "ALARM !!!";
	P(text_MS)             = "monitoring started";
	P(text_undefined)      = "undefined";
	P(text_Undefined)      = "Undefined";
	P(text_Alarm)          = "Alarm";
	P(text_zone)           = "zone";
	P(text_Tamper)         = "Tamper";
	P(text_Damaged)        = "Damaged";
	P(text_Authentication) = "Authentication";
	P(text_triggered)      = "triggered";
	P(text_disarmed)       = "disarmed";
	P(text_tamper)         = "tamper";
	P(text_OK)             = "OK";
	P(text_disabled)       = "disabled";
	P(text_enabled)        = "enabled";
	P(text_Enabled)        = "Enabled";


	P(text_nr)     = "not registered"; 
    P(text_rh)     = "registered (home)";
    P(text_nrs)    = "not registered (searching)"; 
    P(text_rd)     = "registration denied"; 
    P(text_rr)     = "registered (roaming)";
    P(text_unk)    = "unknown";
    P(text_Zone)   = "Zone";
    P(text_Type)   = "Type";
    P(text_Zones)  = "Zones";
    P(text_Group)  = "Group";
    P(text_Groups) = "Groups";
    P(text_group)  = "group";
    P(text_Phone)  = "Phone";
    P(text_Remote) = "Remote";
    P(text_unit)   = "unit";
    P(text_Key)    = "Key";
    P(text_key)    = "key";
    P(text_name)   = "name";
    P(text_Name)   = "Name";
    P(text_Number) = "Number";
    P(text_Status) = "Status";
    P(text_Log)    = "Log";
    P(text_Global) = "Global";
    P(text_setup)  = "setup";
    P(text_digital)= "digital";
    P(text_analog) = "analog";
    P(text_auto)   = "auto";
    P(text_GSM)    = "GSM";
    P(text_SMS)    = "SMS";
    P(text_Debug)  = "Debug";
    P(text_Delay)  = "Delay";
    P(text_alerting) = "alerting";
    P(text_modem)  = "modem";
    P(text_is)     = "is";
    P(text_network)= "network";
    P(text_SS)     = "Signal strength";
    P(text_seconds)= "seconds";
    P(text_Time)   = "Time";
    P(text_Uptime) = "Uptime";
    P(text_Started) = "Started";
    P(text_sec)    = "sec.";
    P(text_space)  = " ";
    P(text_semic)  = ":";
    P(text_dot)    = ".";
    P(text_comma)    = ",";
    P(text_sesp)   = ": ";
    P(text_spdashsp) = " - ";
	P(text_PwrSp)  = "Power supply";
	P(text_Battery)= "Battery";
	P(text_Date)   = "Date";
	P(text_Message)= "Message";
	P(text_hash)   = "#";

    P(text_MRF)    = "requested, failed";
    P(text_MNRF)   = "not requested, failed";
    P(text_MNRA)   = "not requested, acknowledged";
    P(text_MNC)    = "modem not connected";
    P(text_MOK)    = "sent";

	P(html_F_SA)     = "<input type='submit' name='e' value='Save all'/>";
	P(html_F_A)      = "<input type='submit' name='A' value='Apply'/>";
	P(html_F_S)      = "<input type='submit' name='S' value='Select'/>";
	P(html_F_LOG)   = "<input type='submit' name='p' value='<<'/><input type='submit' name='n' value='now'/><input type='submit' name='f' value='>>'/>";
	P(html_e_table) = "</table>";
	P(html_table)   = "<table>";
	P(html_e_td)    = "</td>";
	P(html_td)      = "<td>";
	P(html_e_tr)    = "</tr>";
	P(html_tr)      = "<tr>";
	P(html_tr_ev)   ="<tr class='even'>";
	P(html_e_th)    = "</th>";
	P(html_th)      = "<th>";
	P(html_h1)      = "<h1>";
	P(html_e_h1)    = "</h1>";
	P(html_p)       = "<p>";
	P(html_e_p)     = "</p>";
	P(html_pre)     = "<pre>";
	P(html_e_pre)     = "</pre>";
	P(html_form_s)    = "<form action='";
	P(html_form_e)    = "' method='post'>";
	P(html_e_form)  = "</form>";
	P(html_select)  = "<select name='";
	P(html_e_select)  = "</select>";
	P(html_s_tag)   = "<input type='text' name='";
	P(html_m_tag)   = "' value='";
	P(html_e_tag)   = "'>";
	P(html_option)  = "<option value='";
	P(html_e_option)  = "</option>";
	P(html_selected)  = "' selected>";

	

	P(html_br)     = "</br>";


	P(htmlHead) = "<html><head><title>Open home security</title>"
	"<style type=\"text/css\">\n"
	"BODY {font-family:sans-serif}\n"
	"TABLE {font-size:10pt;border:0px}\n"
	"TD {text-align:left;padding:3px}\n"
	"TH {text-align:left;background-color:#EAF2D3;padding:3px}\n"
	"H1 {font-size:14pt;text-decoration:underline}\n"
	"P {font-size:10pt}\n"
	"DIV {font-size:12pt}\n"
	"tr.even {color:#000;background-color:#EAF2D3}\n"
	"</style></head>\n<body><div>"
	"<a href='/'>Home</a> "
	"<a href='set'>Global</a> "
	"<a href='zone'>Zones</a> "
	"<a href='group'>Groups</a> "
	"<a href='key'>Keys</a> "
	"<a href='phone'>Phone</a> "
	"<a href='auth'>Authentication</a> "
	"<a href='log'>Log</a> "
	"<a href='debug'>Debug</a>"
	"<hr></div>";

      
	P(htmlFoot) = "</body></html>";	
	
#endif


