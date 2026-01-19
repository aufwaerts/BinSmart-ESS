// Global variables

// Networking
AsyncWebServer OTAserver(80);
WiFiServer server(TELNET_PORT);
WiFiClient telnet;
HTTPClient http;
String http_command, http_resp;
int http_resp_code;
String public_IP = "000.000.000.000", DDNS_address = "000.000.000.000";

// BMS
int BMS_balancer_start;  // BMS balancer cell voltage threshold [mV] (read from BMS)
int BMS_balancer_trigger;  // BMS balancer cell diff threshold [mV] (read from BMS)
byte BMS_resp[300];  // buffer for BMS response
NimBLEAddress serverAddress(JKBMS_MAC_ADDR);  // JKBMS BLE server address
NimBLEClient* pClient = nullptr;  // pointer to BLE client object

// Errors
int error_counter[ERROR_TYPES];
unsigned long errortime[ERROR_TYPES];
int errors_consecutive = 0;
String error_msg, last_error_msg;  // if error(s) occured during polling cycle, type of error is written in here
bool error_flag = false;  // errors occured and not yet read by user?

// Hoymiles/RF24
RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);
CRC8 crc8;
CRC16 crc16;

// Time variables
Dusk2Dawn ess_location(ESS_LATITUDE, ESS_LONGITUDE, ESS_TIMEZONE);  // will be used for calculating sunrise, sunset etc.
String sunrise = "00:00", sunset = "00:00";  // will be replaced by calculation
bool dst, daytime;  // flags for daylight saving time and daytime/nighttime
unsigned long local_unixtime = 0, starttime = 0, resettime_errors = 0, resettime_energy = 0; // epoch times (will be read from Shelly 3EM)
unsigned long minpower_time = 0;  // unixtime of last lowest power consumption reading
unsigned long pubip_time = 0, DDNS_time = 0;  // unixtime of last public IP address check and last DDNS update
unsigned long ts_power = 0, ts_pubip = 0, ts_MW = 0, ts_HM = 0, ts_BMS = 0, ts_input = 0;  // various timestamps
float secs_cycle;  // duration of one polling cycle in secs

// Other global variables
float power_grid = 0, power_grid_min = 10000, power_pv = 0, power_ess = 0;  // power measured by Shellies
int power_new = 0, power_old = 0, power_manual = 0;  // power settings
int mw_limit = MW_MAX_POWER, mw_limit_old = MW_MAX_POWER;  // Meanwell power limit settings
int hm_limit = HM_MAX_POWER, hm_limit_old = HM_MAX_POWER;  // Hoymiles power limit settings
int power_target = POWER_TARGET_DEFAULT;  // Systems aims for this grid power target
int filter_cycles = POWER_FILTER_CYCLES;  // for filtering out power spikes
int vcell_min, vcell_max;  // Cell min/max voltages (in millivolts)
int bms_uvp;  // BMS Cell UVP value (read from BMS)
bool bms_bal_on;  // BMS balancer switch setting
int cbat;  // Batt DC current [cA]
int vbat, vbat_idle;  // Total batt voltage [mV] (vbat_idle for voltage at cbat=0)
int bat_level;  // Battery state of charge, as number between 0 and BAT_SOC_LEVELS-1
float pbat;  // Batt DC power [W]
unsigned long mw_counter = 0;  // counter for Meanwell relay operations
bool pm1_eco_mode, pm2_eco_mode;  // eco mode of Shelly PMs
bool mw_on = false;  // state of Meanwell relay
float en_from_pv = 0, en_pv_to_cons = 0, en_pv_to_ess = 0, en_pv_to_grid = 0, en_pv_consumed = 0, en_pv_wasted = 0;  // PV energy counters [Wh]
float en_from_grid = 0, en_to_grid = 0, en_grid_to_cons = 0, en_grid_to_ess = 0;  // Grid energy counters [Wh]
float en_from_ess = 0, en_to_ess = 0, en_ess_to_cons = 0, en_ess_to_grid = 0;  // ESS AC energy counters [Wh]
float en_from_batt = 0, en_to_batt = 0;  // ESS DC energy counters [Wh]
bool manual_mode = false, auto_recharge = false;
char buf[30];  // buffer for formatting output with sprintf()
String cycle_msg, ota_msg, cmd_resp;  // output message strings
String input_str;  // user input string
char command = 0;  // user command (read via telnet)
