// Global variables

// Networking
AsyncWebServer OTAserver(80);
WiFiServer server(TELNET_PORT);
WiFiClient telnet;
HTTPClient http;
String http_resp;
int http_resp_code;
String public_IP = "000.000.000.000", DDNS_address = "000.000.000.000";

// BMS
byte BMS_resp[300];
int BMS_balancer_start;  // BMS balancer cell voltage threshold [mV] (read from BMS)
int BMS_balancer_trigger;  // BMS balancer cell diff threshold [mV] (read from BMS)

// Errors
int error_counter[ERROR_TYPES];
unsigned long errortime[ERROR_TYPES];
int errors_consecutive = 0;
String error_msg = "", last_error_msg = "";  // if error(s) occured during polling cycle, type of error is written in here
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
unsigned long ts_power = 0, ts_pubip = 0, ts_EM_reset = 0, ts_MW = 0, ts_HM = 0, ts_BMS = 0;  // various timestamps
float secs_cycle;  // duration of one polling cycle in secs

// Other global variables
int power_grid = 0, power_grid_min = 10000, power_pv = 0, power_old = 0, power_new = 0, power_manual = 0;
int power_target = POWER_TARGET_DEFAULT;
int filter_cycles = POWER_FILTER_CYCLES;  // for filtering out power spikes
bool rampdown = false;  // indicating if rampdown is active
int vcell_min, vcell_max;  // Cell min/max voltages (in millivolts)
int bms_uvp;  // BMS Cell UVP value (read from BMS)
int vbat;  // Total batt voltage [mV]
int cbat;  // Batt charging/discharging current [cA]
float pbat;  // Batt DC power [W]
int mw_max_power;  // MW max charging power depends on vbat, will be calculated after first read of vbat from BMS
int hm_power_limit = HM_MAX_POWER, mw_power_limit = mw_max_power;
unsigned long mw_counter = 0;  // counter for Meanwell plug state changes (i.e. relay operations)
bool pm_eco_mode = true, mwplug_eco_mode = true;  // eco mode of Shelly 1PM and Meanwell Shelly plug
bool mwplug_on = false;  // state of Meanwell Shelly plug (allows counting of plug relay operations)
bool uvp_sleep_mode = false;  // true if system is in sleep mode (UVP active and low PV production)
float from_pv = 0, pv_to_cons = 0, pv_to_ess = 0, pv_to_grid = 0, pv_consumed = 0;  // PV energy counters [Wh]
float from_grid = 0, to_grid = 0, grid_to_cons = 0, grid_to_ess = 0;  // Grid energy counters [Wh]
float from_ess = 0, to_ess = 0, ess_to_cons = 0, ess_to_grid = 0;  // ESS energy counters [Wh]
bool manual_mode = false, auto_recharge = false;
char buf[30];  // buffer for formatting output with sprintf()
String cycle_msg, ota_msg, cmd_resp;
char repeat_command = 0;  // saves user commands that are to be repeated
