// A library for handling real-time clocks, dates, etc.
// 2010-02-04 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
// 2012-11-08 RAM methods - idreammicro.com
// 2012-11-14 SQW/OUT methods - idreammicro.com

// Progmem strings to save somy RAM
const char RTC_text_0[] PROGMEM       = "0";
const char RTC_text_semicol[] PROGMEM = ":";
const char RTC_text_space[] PROGMEM   = " ";
const char RTC_text_dot[] PROGMEM     = ".";
const char RTC_text_dot_tt[] PROGMEM  = ".20";
const char RTC_text_days[] PROGMEM    = "d, ";
// Simple general-purpose date/time class (no TZ / DST / leap second handling!)
class DateTime {
public:
    DateTime (long t =0);
    DateTime (uint16_t year, uint8_t month, uint8_t day,
                uint8_t hour =0, uint8_t min =0, uint8_t sec =0);
    DateTime (const char* date, const char* time);

    uint16_t year() const       { return 2000 + yOff; }
    uint8_t month() const       { return m; }
    uint8_t day() const         { return d; }
    uint8_t hour() const        { return hh; }
    uint8_t minute() const      { return mm; }
    uint8_t second() const      { return ss; }
    uint8_t dayOfWeek() const;
    // 
    char *timestamp();
    char *formatedDateTime();
    char *formatedUpTime();

    // 32-bit times as seconds since 1/1/2000
    long get() const;

protected:
    uint8_t yOff, m, d, hh, mm, ss;
    uint16_t days;
    char _tmp_ts[20];

};

// RTC based on the DS1307 chip connected via I2C and the Wire library
class RTC_DS1307 {
public:

    // SQW/OUT frequencies.
    enum Frequencies
    {
        Frequency_1Hz,
        Frequency_4096Hz,
        Frequency_8192Hz,
        Frequency_32768Hz
    };

    static void begin() {}
    static void adjust(const DateTime& dt);
    static DateTime now();
    static uint8_t isrunning();

    // SQW/OUT functions.
    void setSqwOutLevel(uint8_t level);
    void setSqwOutSignal(Frequencies frequency);

    // RAM registers read/write functions. Address locations 08h to 3Fh.
    // Max length = 56 bytes.
    static uint8_t readByteInRam(uint8_t address);
    static void readBytesInRam(uint8_t address, uint8_t length, uint8_t* p_data);
    static void writeByteInRam(uint8_t address, uint8_t data);
    static void writeBytesInRam(uint8_t address, uint8_t length, uint8_t* p_data);

    // utility functions
    static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
    static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }
};
