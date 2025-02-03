// Global variables

// Networking
AsyncWebServer OTAserver(80);
WiFiServer server(TELNET_PORT);
WiFiClient telnet;
HTTPClient http;
String http_resp;
String DDNS_address = "0.0.0.0";

// BMS
byte BMS_response[BMS_RESPONSE_SIZE];

// Errors
int error_counter[ERROR_TYPES];
unsigned long errortime[ERROR_TYPES];
int errors_consecutive = 0;
String error_msg;  // if error(s) occured during polling cycle, type of error is written in here
bool error_flag = false;  // errors occured and not yet read by user?

// Hoymiles/RF24
byte hm_radio_ID[5] = {0x01, 0xFF, 0xFF, 0xFF, 0xFF};  // 0xFF will be replaced by HM serial number
// in the following HM commands, 0xFF will be replaced by HM serial number, 0x00 will be replaced by calculation
byte hm_turnon[15] =  {0x51, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x17, 0x41, 0x72, 0x81, 0x00, 0x00, 0xB0, 0x01, 0x00};
byte hm_turnoff[15] = {0x51, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x17, 0x41, 0x72, 0x81, 0x01, 0x00, 0x20, 0x00, 0x00};
byte hm_power[19] =   {0x51, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x17, 0x41, 0x72, 0x81, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);
CRC8 crc8;
CRC16 crc16;

// Time variables
String current_time = "00:00", sunrise = "00:00", sunset = "00:00";  // will be read from Shelly 3EM or replaced by calculation
String datetime_string = "00/00/0000 00:00:00";  // temp string for formatting date and time info with sprintf()
String ct_string = "00:00:00 +";  // temp string for formatting current time with sprintf()
bool daytime;  // current time is day or night
int UTC_offset;  // time difference (in hours) between UTC and current time
Dusk2Dawn ESS_LOCATION(ESS_LATITUDE, ESS_LONGITUDE, ESS_TIMEZONE);
unsigned long unixtime, starttime, resettime_errors, resettime_energy; // epoch times (will be read from Shelly 3EM)
unsigned long minpower_time = 0;  // time of last lowest power consumption reading
unsigned long DDNS_time = 0;  // time of last DDNS update
unsigned long ts_cycle = 0, ts_power = 0, ts_HM_ON = 0, ts_MW_ON = 0, ts_DDNS = 0, ts_EM = 0, ts_EM_reset = 0;  // various timestamps

// Other global variables
String power_string = "0000", energy_string = "000.000";  // temp strings for formatting output with sprintf()
int power_grid = 0, power_grid_min = 10000, power_pv = 0, power_new = 0, power_old = 0, power_manual = 0;
int power_target = POWER_TARGET_DEFAULT;
int filter_cycles = POWER_FILTER_CYCLES;  // for filtering out power spikes
int vcell_min = 0, vcell_max = 10000, vbat = 28000;  // Cell min/max voltages and batt voltage in millivolts; assumption: batt is full
int mw_max_power;  // MW max charging power depends on vbat, will be calculated after first read of vbat from BMS
int hm_power_limit = HM_MAX_POWER, mw_power_limit = mw_max_power;
int uvp_countdown = UVP_WAKEUP_RESET;  // countdown before ESS falls asleep
float from_pv, from_ess, to_ess;  // energy counters (will be read from Shelly 1PM and Shelly Plugs)
float from_grid = 0, grid_to_ess = 0, pv_to_grid = 0, ess_to_grid = 0;  // energy counters (will be calculated)
bool manual_mode = false, auto_recharge = false, pm_eco_mode = true, rampdown = false;
String cycle_msg, ota_msg, cmd_resp, filter_symbol;
char repeat_command = ' ';
