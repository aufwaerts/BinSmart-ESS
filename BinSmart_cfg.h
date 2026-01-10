// ESS configuration

// ESP32 pin definitions
#define LED_PIN 12
#define PWM_OUTPUT_PIN 13
#define RF24_CE_PIN 32
#define RF24_CSN_PIN 33
#define UART_RX_PIN 16
#define UART_TX_PIN 17

// Networking
const String WIFI_SSID = "***";
const String WIFI_PWD = "***";
const int GOOD_WIFI_RSSI = -70;  // RSSI above this value is considered "good enough"
const int TELNET_PORT = ***;  // TCP port for communication with terminal
IPAddress ESP32_ADDR(*,*,*,*);  // Local IP address of ESS
IPAddress ROUTER_ADDR(*,*,*,*);   // Local WiFi router
IPAddress SUBNET(*,*,*,*);   // WiFi subnet
IPAddress DNS_SERVER1(8,8,8,8);  // Google DNS resolver
IPAddress DNS_SERVER2(9,9,9,9);  // Quad9 DNS server
const String EM_ADDR = "*.*.*.*";  // Shelly 3EM
const String PM1_ADDR = "*.*.*.*";  // Shelly Plus 1PM, connecting Maxeon solar panel to AC
const String PM2_ADDR = "*.*.*.*";  // Shelly 2PM Gen3, connecting Meanwell charger and Hoymiles inverter to AC

// Power settings
const int PV_MAX_POWER = 360;  // PV module/inverter max AC output
const int POWER_TARGET_DEFAULT = 5;  // System is aiming for this amount of watts to be drawn from grid
const int POWER_TARGET_TOLERANCE = 5;  // Max tolerated deviation (+/-) from target power
const int HM_LOW_POWER_TOLERANCE = 15;  // Max tolerated positive deviation when Hoymiles is below HM_LOW_POWER_THRESHOLD
const int POWER_RAMPDOWN_RATE = -40; // Max power decrease per cycle
const int POWER_FILTER_CYCLES = 12;  // Number of cycles during which power spikes are filtered out
const float POWER_LIMIT_RAMPDOWN = 0.67;  // Power rampdown rate when CELL_OVP or CELL_UVP is reached

// Time/Timer settings
const unsigned long PROCESSING_DELAY = 2000;  // minimum delay (in ms) for power changes to take effect
const unsigned long DDNS_UPDATE_INTERVAL = 60000;  // DDNS IP address check interval (in ms)
const unsigned long READINPUT_TIMEOUT = 4000;  // max waiting time (in ms) for terminal input
const unsigned int HTTP_TIMEOUT = 6000;  // max waiting time (in ms) for HTTP responses
const unsigned long RF24_TIMEOUT = 4000;  // max waiting time (in ms) for RF24 responses
const unsigned long RF24_KEEPALIVE = 30000;  // number of ms after which Hoymiles RF24 interface receives "keep alive" message
const unsigned long MW_TIMER = 60000;  // number of ms after which Meanwell is automatically turned off (unless keep-alive message is received)
const int ESS_TIMEZONE = +1;  // ESS is installed in this timezone (relative to UTC)
const float ESS_LATITUDE = 46.817;  // geo coordinates of ESS
const float ESS_LONGITUDE = 13.520;
const String GET_ASTRO_TIME = "03:30";  // time at which astro times (sunrise/sunset) will be calculated (after a possible DST change, before sunrise)

// Meanwell (charging) power parameters
const int MW_MAX_POWER = 300;  // max power output at minimum voltage (24V)
// max power output ranges from 300 to 350W, depending on vbat
#define MW_POWER_LIMIT_FORMULA vbat/77.057*(0.9636-1.0/PWM_DUTY_CYCLE_MAX)
const int MW_MIN_POWER = 15;  // Meanwell turned off below min_power (power output would be unstable and very inefficient)
const int MW_LOW_POWER_THRESHOLD = 25;  // power output below this threshold is non-linear
const int MW_RECHARGE_POWER = 200;  // power setting for automatic recharging (prevents BMS UVP)
// the following formulas are the results of Meanwell HLG-320 power output tests
// PWM signal controls Meanwell charging current; charging power also depends on vbat
// higher PWM value means less power
#define MW_POWER_FORMULA PWM_DUTY_CYCLE_MAX*(0.9636-77.057*current)
#define MW_LOW_POWER_FORMULA PWM_DUTY_CYCLE_MAX*(129119.635*current*current-321.337*current+1.08)

// PWM params for Meanwell power control
#define PWM_CHANNEL 0
#define PWM_FREQ 250
#define PWM_RESOLUTION 10
#define PWM_DUTY_CYCLE_MIN 1
const int PWM_DUTY_CYCLE_MAX = pow(2,PWM_RESOLUTION)-1;

// Hoymiles (discharging) power parameters
const int HM_MAX_POWER = -180;  // Limit of linear power output range
const int HM_LOW_POWER_THRESHOLD = -61;  // Hoymiles power output above this threshold is unstable
const int HM_MIN_POWER = -15;  // Hoymiles turned off above min_power (power would be too unstable)
#define HM_OFF false
#define HM_ON true

// Hoymiles/RF24 comms
const byte RF24_CHANNEL = 03; // Possible RF24 channles for Hoymiles comms are 03, 23, 40, 61, 75; frequency in MHz is 2400 + channel
const byte RF24_PALEVEL = RF24_PA_MIN; // Possible RF24 PA levels are RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
const byte HM_SN[] = {0x**, 0x**, 0x**, 0x**, 0x**, 0x**};  // serial number of Hoymiles inverter
const byte HM_RADIO_ID[5] = {0x01, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5]};
byte hm_turnon[15] =  {0x51, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5], 0x80, 0x17, 0x41, 0x72, 0x81, 0x00, 0x00, 0xB0, 0x01, 0x00};
byte hm_turnoff[15] = {0x51, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5], 0x80, 0x17, 0x41, 0x72, 0x81, 0x01, 0x00, 0x20, 0x00, 0x00};
byte hm_power[19] =   {0x51, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5], 0x80, 0x17, 0x41, 0x72, 0x81, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// URLs
const String EM_STATUS = "http://" + EM_ADDR + "/status";
const String EM_RESET = "http://" + EM_ADDR + "/reset_data";
const String PM1_STATUS = "http://" + PM1_ADDR + "/rpc/Switch.GetStatus?id=0";
const String PM1_ECO_ON = "http://" + PM1_ADDR + "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":true}}";
const String PM1_ECO_OFF = "http://" + PM1_ADDR + "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":false}}";
const String PM2_MW_STATUS = "http://" + PM2_ADDR + "/rpc/Switch.GetStatus?id=0";
const String PM2_HM_STATUS = "http://" + PM2_ADDR + "/rpc/Switch.GetStatus?id=1";
const String PM2_MW_ON = "http://" + PM2_ADDR + "/relay/0?turn=on&timer=" + String(MW_TIMER/1000);
const String PM2_MW_OFF = "http://" + PM2_ADDR + "/relay/0?turn=off";
const String PM2_HM_ON = "http://" + PM2_ADDR + "/relay/1?turn=on";
const String PM2_HM_OFF = "http://" + PM2_ADDR + "/relay/1?turn=off";
const String PM2_ECO_ON = "http://" + PM2_ADDR + "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":true}}";
const String PM2_ECO_OFF = "http://" + PM2_ADDR + "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":false}}";
const String PUBLIC_IP_SERVER = "http://api.ipify.org";  // public service for obtaining WiFi routers public IP address
// const String PUBLIC_IP_SERVER = "http://ifconfig.me/ip";  // alternative service
const String DDNS_SERVER_UPDATE = "http://***:***@dynupdate.no-ip.com/nic/update?hostname=***.ddns.net&myip=";  // public DDNS service

// BMS/batt voltage protection settings in millivolts
const int ESS_BMS_OVP_DIFF = 100;  // min difference between ESS and BMS OVP settings
const int ESS_BMS_UVP_DIFF = 100;  // min difference between ESS and BMS UVP settings
const int ESS_OVP = 3500;  // one cell above this voltage: ramp down charging power (BMS_OVP - ESS_OVP >= ESS_BMS_OVP_DIFF)
const int ESS_OVPR = 3450;  // all cells below this voltage: re-enable charging (should be the same as BMS Balancer Start Voltage)
const int ESS_UVP = 3200;  // one cell below this voltage: ramp down discharging power (ESS_UVP - BMS_UVP >= ESS_BMS_UVP_DIFF)
const int ESS_UVPR = 3250;  // all cells above this voltage: re-enable discharging
const int BAT_FULL = 27100;  // voltage at which battery is considered full
const int BAT_EMPTY = 25600;  // voltage at which battery is considered empty

// BMS definitions and commands
#define BMS_STX_1 0x4E
#define BMS_STX_2 0x57
#define BMS_COMMAND_POS 8
#define BMS_READ_ALL 0x06
#define BMS_READ_DATA 0x03
#define BMS_DATA_ID_POS 11
#define BMS_VCELLS_ID 0x79
#define BMS_CURRENT_ID 0x84
#define BMS_WARNINGS_POS 67
#define BMS_WARNINGS_ID 0x8B
#define BMS_OVP_POS 79
#define BMS_OVP_ID 0x90
#define BMS_UVP_POS 88
#define BMS_UVP_ID 0x93
#define BMS_SBAL_POS 112
#define BMS_SBAL_ID 0x9B
#define BMS_TBAL_POS 115
#define BMS_TBAL_ID 0x9C
const byte BMS_SETTINGS[] = {BMS_STX_1, BMS_STX_2, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, BMS_READ_ALL, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x29};
const byte BMS_VOLTAGES[] = {BMS_STX_1, BMS_STX_2, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, BMS_READ_DATA, 0x03, 0x00, BMS_VCELLS_ID, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x9F};
const byte BMS_CURRENT[] = {BMS_STX_1, BMS_STX_2, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, BMS_READ_DATA, 0x03, 0x00, BMS_CURRENT_ID, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0xAA};

// Errors
const String ERROR_TYPE[] = {"WIFI", "DDNS", "BMS", "RF24", "3EM", "1PM", "2PM"};  // error messages correspond with these types! changes here also need changed error messages
const int ERROR_TYPES = sizeof(ERROR_TYPE)/sizeof(ERROR_TYPE[0]);
const int ERROR_LIMIT = 20;  // number of consecutive erroneous cycles before error is considered persistent and system is halted
const int UNCRITICAL_ERROR_TYPES = 2;  // ERROR_LIMIT doesn't apply to first ... error types

// Symbols for a nice telnet frontend
const String ESS_FLOW_SYMBOL[] = {"───","┃\033[32m◀\033[0m╶","╴\033[32m◀\033[0m╶","┇\033[32m◀\033[0m╶","╴\033[32m\033[1m«\033[0m╶","╴\033[32m▶\033[0m╶"};  // green flow symbols
const String PV_FLOW_SYMBOL[] = {"───","╴\033[33m▶\033[0m╶","╴\033[33m▶\033[0m┃","╴\033[33m▶\033[0m┇","╴\033[33m\033[1m»\033[0m╶","╴\033[33m◀\033[0m╶"};  // yellow flow symbols
const String GRID_FLOW_SYMBOL[] = {"───","╴\033[31m▶\033[0m╶","╴\033[31m▶\033[0m┃","╴\033[31m▶\033[0m┇","╴\033[31m\033[1m»\033[0m╶"};  // red flow symbols
const String DIFF_SYMBOL[] = {" ▲"," ▼"," ▼🪜"};
const String BAT_LEVEL_SYMBOL[] = {"🔋\033[33m⡀\033[0m ","🔋\033[32m⡀\033[0m ","🔋\033[32m⣀\033[0m ","🔋\033[32m⣄\033[0m ","🔋\033[32m⣤\033[0m ","🔋\033[32m⣦\033[0m ","🔋\033[32m⣶\033[0m ","🔋\033[32m⣷\033[0m ","🔋\033[32m⣿\033[0m "};
const int BAT_LEVELS = sizeof(BAT_LEVEL_SYMBOL)/sizeof(BAT_LEVEL_SYMBOL[0]);
const String BAT_OVP_SYMBOL = "▁▁▁";
const String BAT_UVP_SYMBOL = "▔▔";
const String MOON_SYMBOL = " 🌙╶";
const String SUN_SYMBOL[] = {" 🌞╶"," 🌞╺"};
const String CABLE_SYMBOL = "─";
const String PV_CABLE_SYMBOL = "──┐";
const String ESS_CABLE_SYMBOL = "┌──";
const String ESS_SYMBOL = "─🔋 ";
const String HOUSE_SYMBOL = "             🏠";
const String GRID_SYMBOL = " 🏭╶";
const String GRID_CABLE_SYMBOL = "──┘";
const String CONS_CABLE_SYMBOL = "└──";
const String CONS_SYMBOL = "╴📺 ";
const String OPS_SYMBOL[] = {" 🏃"," 🧍"," 💤🛌"};
const String POWERFILTER_SYMBOL = " ⏳";
const String MANUAL_MODE_SYMBOL = " 👈";
const String AUTO_RECHARGE_SYMBOL = " 🔌";
const String WIFI_SYMBOL[] = {" ⚠️­"," 📶"};
const String ERROR_SYMBOL = "❌";
const String BALANCER_SYMBOL = " 🔄";
const String CLEAR_SCREEN = "\033[0H\033[0J";
const String SHOW_CURSOR = "\033[?25h";
const String HIDE_CURSOR = "\033[?25l";
