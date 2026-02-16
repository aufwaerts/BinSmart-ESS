// Global variables

// Networking
WebServer OTA_server(HTTP_PORT);
WiFiServer telnet_server(TELNET_PORT);
WiFiClient telnet, shelly_resp;
HTTPClient http;
char http_command[150];
IPAddress pubip_addr, ddns_addr;
NimBLEAddress serverAddress(JKBMS_MAC_ADDR);  // JKBMS BLE server address
NimBLEClient* pClient;  // pointer to BLE client object

// BMS
int vcell_min, vcell_max;  // batt cell min/max voltages [mV]
int cbat, vbat;  // batt DC current [cA], total batt voltage [mV]
int bat_level;  // batt state of charge, as number between 0 and BAT_SOC_LEVELS-1
int bms_balancer_start;  // BMS balancer cell voltage threshold [mV] (read from BMS)
int bms_balancer_trigger;  // BMS balancer cell diff threshold [mV] (read from BMS)
bool bms_bal_on, bms_bal_active;  // BMS balancer switch and balancing activity
int bms_uvp;  // BMS cell UVP value
byte bms_resp[300];  // buffer for BMS response
int bms_resp_wait_counter;  // how often did system have to wait for BMS response bytes (indicates RS485 transmission problems)

// Errors
int error_counter[ERROR_TYPES];
unsigned long errortime[ERROR_TYPES];
int errors_consecutive;
char error_str[200], last_error_str[200];  // if error occured during polling cycle, type of error is written in here
bool error_flag;  // errors occured and not yet read by user?

// Hoymiles/RF24
RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);
CRC8 crc8;
CRC16 crc16;
byte hm_power[19] = {0x51, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5], 0x80, 0x17, 0x41, 0x72, 0x81, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
bool hm_awake;  // does Hoymiles AC & DC side have power, i.e. is Hoymiles awake or asleep?

// Time variables
Dusk2Dawn ess_location(LATITUDE, LONGITUDE, TIMEZONE);  // latitude, longitude, timezone of ESS
int min_of_day;  // local time (minutes after midnight)
int utc_offset;  // UTC offset (timezone + dst) of local time
int sunrise, sunset;  // current day's sunrise and sunset (minutes after midnight)
unsigned long unixtime; // epoch time of local time (will be read from Shelly 3EM)
unsigned long minpower_time, starttime, resettime_errors, resettime_energy;  // unixtime of certain events
unsigned long pubip_time, ddns_time;  // unixtime of last public IP address check and last DDNS update
unsigned long ts_power, ts_pubip, ts_MW, ts_HM, ts_BMS, ts_userio;  // various millis() timestamps
unsigned long msecs_cycle;  // duration of one polling cycle in secs

// Power and energy variables ([W] and [Wh])
int power_grid_target = POWER_TARGET_DEFAULT;  // Systems aims for this grid power target
int power_new, power_old, power_manual;  // ESS power settings
int mw_limit = MW_MAX_POWER, mw_limit_old = MW_MAX_POWER;  // Meanwell power limit settings
int hm_limit = HM_MAX_POWER, hm_limit_old = HM_MAX_POWER;  // Hoymiles power limit settings
float power_grid, power_grid_min = 10000, power_pv, power_ess;  // actual AC power measured by Shellies
float pbat;  // actual DC power measured by BMS
float power_from_grid, power_from_ess, power_to_ess, power_cons;  // main power flows
float power_pv_to_ess, power_pv_to_cons, power_pv_to_grid, power_ess_to_cons, power_ess_to_grid, power_grid_to_ess, power_grid_to_cons; // detailed power flows
float en_from_pv, en_pv_to_cons, en_pv_to_ess, en_pv_to_grid, en_pv_wasted;  // PV energy counters
float en_from_grid, en_grid_to_cons, en_grid_to_ess;  // Grid energy counters
float en_from_ess, en_to_ess, en_ess_to_cons, en_ess_to_grid;  // ESS AC energy counters
float en_from_batt, en_to_batt;  // ESS DC energy counters

// Other global variables
int filter_cycles = POWER_FILTER_CYCLES;  // number of cycles where power spikes are filtered out
bool pm1_eco_mode, pm2_eco_mode;  // eco mode of Shellies
bool manual_mode, auto_recharge;  // special ESS operating modes
bool em_data_cleared;  // indicates if Shelly 3EM energy data has been cleared
bool mw_on, hm_on;  // state of Shelly 2PM Meanwell/Hoymiles relays
unsigned long mw_counter, hm_counter;  // counter for Meanwell and Hoymiles relay operations
char tn_str[1000];  // output (telnet) message string
char command;  // last user command (read via telnet)
char resp_str[100];  // last user command response
