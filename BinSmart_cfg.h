// ESS configuration

// ESP32 pin definitions
#define LED_PIN 12
#define PWM_OUTPUT_PIN 13
#define RF24_CE_PIN 32
#define RF24_CSN_PIN 33
#define UART_RX_PIN 16
#define UART_TX_PIN 17

// Definitions for UserCommand()
#define REPEAT false
#define READ true

// Networking
const String WIFI_SSID = "***";
const String WIFI_PWD = "***";
const int GOOD_WIFI_RSSI = -70;  // RSSI above this value is considered "good enough"
const int TELNET_PORT = ***;  // TCP port for communication with terminal
IPAddress ESP32_ADDR(*,*,*,*);  // Local IP address of ESS
IPAddress ROUTER_ADDR(*,*,*,*);   // Local WiFi router
IPAddress WIN_CLIENT_ADDR(*,*,*,*);
IPAddress SUBNET(*,*,*,*);   // WiFi subnet
IPAddress DNS_SERVER1(8,8,8,8);  // Google DNS resolver
IPAddress DNS_SERVER2(9,9,9,9);  // Quad9 DNS server
const String EM_ADDR = "*.*.*.*";  // Shelly 3EM
const String PM_ADDR = "*.*.*.*";  // Shelly Plus 1PM
const String MWPLUG_ADDR = "*.*.*.*";  // Shelly Plus Plug, connecting Meanwell charger to AC
const String HMPLUG_ADDR = "*.*.*.*";  // Shelly Plus Plug, connecting Hoymiles inverter to AC

// PV module/inverter max AC output
const int PV_MAX_POWER = 360;

// Hoymiles params
const int HM_MIN_POWER = -10;  // Hoymiles turned off above min_power
const int HM_MAX_POWER = -300;  // Hoymiles discharging power limit
// tests have shown that Hoymiles power output is non-linear, the following formula corrects it
// the formula needs adapting for different inverters and/or different battery voltages
// this is an excellent site for determining the formula parameters: https://www.arndt-bruenner.de/mathe/scripts/regrnl.htm
// formulas include conversion to positive power values in deciwatts
#define HM_POWER_FORMULA -10*power
const int HM_LOW_POWER_THRESHOLD = -90;  // tests have shown that Hoymiles power below this threshold needs a different formula
// #define HM_LOW_POWER_FORMULA -0.001308*power*power*power-0.1891*power*power-17.1*power-19 (this was for HM-300)
#define HM_LOW_POWER_FORMULA -0.00072275*power*power*power-0.136224*power*power-16.484*power-9.45
const int HM_HIGH_POWER_THRESHOLD = -160;  // tests have also shown that Hoymiles power above this threshold needs a different formula
// #define HM_HIGH_POWER_FORMULA -0.01977*power*power-13.40136*power+24.2 (this was for HM-300)
#define HM_HIGH_POWER_FORMULA -0.00019255*power*power*power-0.1629286114853008*power*power-48.3149403751306*power-2761.25

// Meanwell params
const int MW_MIN_POWER = 12;  // Meanwell turned off below min_power
const int MW_RECHARGE_POWER = 200;  // Meanwell power setting for automatic recharging (to prevent BMS turnoff): MW operates at highest efficiency
// the following formulas are the result of Meanwell power output tests, they need adapting for different chargers
// translating MW charging power to PWM value: higher PWM value means less power
#define MW_PWM_FORMULA PWM_DUTY_CYCLE_MAX*(0.9636-77.057*power/vbat)  // PWM signal controls Meanwell charging current; for correct charging power, vbat needs to be included in PWM formula
const int MW_LOW_POWER_THRESHOLD = 25;  // Meanwell power output below this threshold is not linear
#define MW_LOW_POWER_FORMULA PWM_DUTY_CYCLE_MAX*(129119.635*power/vbat*power/vbat-321.337*power/vbat+1.08)
#define MW_POWER_LIMIT_FORMULA vbat/77.057*(0.9636-1.0/PWM_DUTY_CYCLE_MAX)  // MW max charging power also depends on vbat

// Hoymiles/RF24 comms
const byte HM_SERIAL[] = {0x**, 0x**, 0x**, 0x**, 0x**, 0x**};  // serial number of Hoymiles inverter
const byte RF24_CHANNELS[] = {03, 23, 40, 61, 75};  // Frequency is 2400 + RF24_CHANNELS [MHz]
const byte RF24_PALEVELS[] = {RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX};
const int RF24_TX_TIMEOUT = 2000;  // max time (in ms) for sending RF24 data and receiving an ACK packet
const int RF24_MIN_DELAY = 20;  // Minimum time gap (in ms) between two consecutive Hoymiles commands

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

// Time/Timer settings
const int PROCESSING_DELAY = 2000;  // minimum delay (in msecs) for power changes to take effect
const int UVP_SLEEP_DELAY = 20000;  // cycle duration (in msecs) during UVP sleep mode
const int UVP_WAKEUP_RESET = 1700;  // number of cycles in UVP mode (hm_power_limit == 0) before UVP sleep mode is activated (must be an even number)
const int MW_PLUG_TIMER = 60;  // number of secs after which Meanwell is automatically turned off (unless "keep alive" command resets timer)
const int HM_PLUG_TIMER = 900;  // number of secs after which Hoymiles AC side is automatically turned off (unless "keep alive" command resets timer)
const int DDNS_UPDATE_INTERVAL = 60;  // DDNS IP address check interval (in secs)
const int EM_RESET_INTERVAL = 600;  // EM internal data reset interval (in secs)
const int READCOMMAND_TIMEOUT = 4;  // max waiting time (in secs) for terminal input
const String GET_ASTRO_TIME = "03:30";  // time at which astro times (sunrise/sunset) will be calculated (after a possible DST change, before sunrise)

// URLs
const String EM_SETTINGS = "http://" + EM_ADDR + "/settings";
const String EM_STATUS = "http://" + EM_ADDR + "/status";
const String EM_RESET = "http://" + EM_ADDR + "/reset_data";
const String PM_STATUS = "http://" + PM_ADDR + "/rpc/Switch.GetStatus?id=0";
const String PM_ECO_ON = "http://" + PM_ADDR + "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":true}}";
const String PM_ECO_OFF = "http://" + PM_ADDR + "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":false}}";
const String MWPLUG_ON = "http://" + MWPLUG_ADDR + "/relay/0?turn=on&timer=" + String(MW_PLUG_TIMER);
const String MWPLUG_OFF = "http://" + MWPLUG_ADDR + "/relay/0?turn=off";
const String HMPLUG_ON = "http://" + HMPLUG_ADDR + "/relay/0?turn=on";
const String HMPLUG_OFF = "http://" + HMPLUG_ADDR + "/relay/0?turn=off";
const String PUBLIC_IP_SERVER = "http://api.ipify.org";  // public service for obtaining WiFi routers public IP address
const String DDNS_SERVER_UPDATE = "http://***:***@dynupdate.no-ip.com/nic/update?hostname=***.ddns.net&myip=";  // public DDNS service

// Power settings
const int POWER_TARGET_DEFAULT = 5;  // System is aiming for this amount of watts to be drawn from grid
const int POWER_TARGET_DEVIATION = 10;  // Max allowed deviation (+/-) from target power
const int POWER_RAMPDOWN_RATE = 40; // Max power decrease per polling interval, MUST BE EQUAL OR HIGHER THAN -MW_MIN_POWER
const int POWER_FILTER_CYCLES = 10;  // Number of cycles during which power spikes are filtered out
const float POWER_LIMIT_RAMPDOWN = 0.67;  // Power rampdown rate when CELL_OVP or CELL_UVP is reached

// BMS/batt voltage protection settings in millivolts
const int ESS_BMS_OVP_DIFF = 50;  // min difference between ESS and BMS OVP settings
const int ESS_BMS_UVP_DIFF = 100;  // min difference between ESS and BMS UVP settings
const int ESS_OVP = 3550;  // one cell above this voltage: ramp down charging power (BMS_OVP - ESS_OVP >= ESS_BMS_OVP_DIFF)
const int ESS_OVPR = 3450;  // all cells below this voltage: re-enable charging (should be the same as BMS Balancer Start Voltage)
const int ESS_UVP = 3200;  // one cell below this voltage: ramp down discharging power (ESS_UVP - BMS_UVP >= ESS_BMS_UVP_DIFF)
const int ESS_UVPR = 3250;  // all cells above this voltage: re-enable discharging
const int ESS_FULL = 27100;  // batt voltage at which ESS is considered "full" (>80%)
const int ESS_EMPTY = 25600;  // batt voltage at which ESS is considered "empty" (<20%)

// PWM params for Meanwell power control
#define PWM_CHANNEL 0
#define PWM_FREQ 250
#define PWM_RESOLUTION 10
const unsigned long PWM_DUTY_CYCLE_MAX = pow(2,PWM_RESOLUTION)-1;

// Errors
const String ERROR_TYPE[] = {"WIFI", "DDNS", "BMS", "RF24", "3EM", "1PM", "MWPLUG"};  // error messages correspond with these types! changes here also need changed error messages
const int ERROR_TYPES = sizeof(ERROR_TYPE)/sizeof(ERROR_TYPE[0]);
const int ERROR_LIMIT = 20;  // number of consecutive erroneous cycles before error is considered persistent and system is halted
const int UNCRITICAL_ERROR_TYPES = 2;  // ERROR_LIMIT doesn't apply to first ... error types

// Symbols for a nice telnet frontend
const String ESS_FLOW_SYMBOL[] = {"╴×╶","╴\033[32m◀\033[0m╶","╴\033[32m▶\033[0m╶"};  // green flow symbols
const String PV_FLOW_SYMBOL[] = {"╴×╶","╴\033[33m◀\033[0m╶","╴\033[33m▶\033[0m╶"};  // yellow flow symbols
const String GRID_FLOW_SYMBOL[] = {"───","╴\033[31m◀\033[0m╶","╴\033[31m▶\033[0m╶"};  // red flow symbols
const String DIFF_SYMBOL[] = {" ▼"," ▲"};
const String ESS_LEVEL_SYMBOL[] = {"─\033[33m⢀\033[0m🔋 ","─\033[32m⢀\033[0m🔋 ","─\033[32m⢠\033[0m🔋 ","─\033[32m⢰\033[0m🔋 ","─\033[32m⢸\033[0m🔋 "};
const int ESS_LEVELS = sizeof(ESS_LEVEL_SYMBOL)/sizeof(ESS_LEVEL_SYMBOL[0]);
const String SUN_SYMBOL[] = {" ☀️╶"," ☀️­╶"};
const String MOON_SYMBOL[] = {" 🌙╶─"," 🌙╶─"};
const String PV_LEVEL_SYMBOL[] = {" ☁️╶─"," ☁️­╶─"," ⛅╶─"," ⛅╶─"," 🌤╶─"," 🌤️­╶─"," ☀️╶─"," ☀️­╶─"};
const int PV_LEVELS = sizeof(PV_LEVEL_SYMBOL)/sizeof(PV_LEVEL_SYMBOL[0])/2;
const String CABLE_SYMBOL = "─";
const String PV_CABLE_SYMBOL = "─┐";
const String ESS_CABLE_SYMBOL = "┌─";
const String ESS_SYMBOL = "─🔋 ";
const String HOUSE_SYMBOL = "             🏠";
const String GRID_SYMBOL = " 🏭╶";
const String GRID_CABLE_SYMBOL = "─┘";
const String CONS_CABLE_SYMBOL = "└─";
const String CONS_SYMBOL = "╴📺 ";
const String OPS_SYMBOL[] = {" 🏃"," 🧍"," 💤🛌"};
const String POWERFILTER_SYMBOL[] = {" ✋🕛"," ✋🕐"," ✋🕑"," ✋🕒"," ✋🕓"," ✋🕔"," ✋🕕"," ✋🕖"," ✋🕗"," ✋🕘"," ✋🕙"," ✋🕚"};
const String RAMPDOWN_SYMBOL = "🪜";
const String OVP_LIMIT_SYMBOL = "                     ▁▁";
const String UVP_LIMIT_SYMBOL = "      ▔▔";
const String MODE_SYMBOL[] = {""," 👆"," 🔌"};
const String WIFI_SYMBOL[] = {" ⚠️­"," 📶"};
const String ERROR_SYMBOL = "❌";
const String BALANCER_SYMBOL = " 🔄";

