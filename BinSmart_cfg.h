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

// Hoymiles power parameters
const int HM_MIN_POWER = -12;  // Hoymiles turned off above min_power
const int HM_MAX_POWER = -200;  // Hoymiles discharging power limit
// tests have shown that Hoymiles power output is non-linear, the following formulas correct it
// the formulas needs adapting for different inverters and/or different battery voltages
// this is an excellent site for determining the formula parameters: https://www.arndt-bruenner.de/mathe/scripts/regrnl.htm
// formulas include conversion to positive power values in deciwatts
const int HM_LOW_POWER_THRESHOLD = -90;  // power below this threshold needs a special formula
#define HM_LOW_POWER_FORMULA -0.00072275*power*power*power-0.136224*power*power-16.484*power-9.45
const int HM_HIGH_POWER_THRESHOLD = -160;  // power above this value needs a special formula
#define HM_HIGH_POWER_FORMULA -0.00019255*power*power*power-0.1629286114853008*power*power-48.3149403751306*power-2761.25
#define HM_OFF 0
#define HM_ON 1

// Meanwell power parameters
const int MW_MIN_POWER = 12;  // Meanwell turned off below min_power
const int MW_RECHARGE_POWER = 200;  // Meanwell power setting for automatic recharging (to prevent BMS turnoff): MW operates at highest efficiency
// the following formulas are the result of Meanwell power output tests, they need adapting for different chargers
// translating MW charging power to PWM value: higher PWM value means less power
#define MW_PWM_FORMULA PWM_DUTY_CYCLE_MAX*(0.9636-77.057*power/vbat)  // PWM signal controls Meanwell charging current; for correct charging power, vbat needs to be included in PWM formula
const int MW_LOW_POWER_THRESHOLD = 25;  // Meanwell power output below this threshold is not linear
#define MW_LOW_POWER_FORMULA PWM_DUTY_CYCLE_MAX*(129119.635*power/vbat*power/vbat-321.337*power/vbat+1.08)
#define MW_POWER_LIMIT_FORMULA vbat/77.057*(0.9636-1.0/PWM_DUTY_CYCLE_MAX)  // MW max charging power also depends on vbat

// Hoymiles/RF24 comms
const byte RF24_CHANNEL = 03; // Possible RF24 channles for Hoymiles comms are 03, 23, 40, 61, 75; frequency in MHz is 2400 + channel
const byte RF24_PALEVEL = RF24_PA_MIN; // Possible RF24 PA levels are RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
const byte HM_SN[] = {0x**, 0x**, 0x**, 0x**, 0x**, 0x**};  // serial number of Hoymiles inverter
const byte HM_RADIO_ID[5] = {0x01, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5]};
byte hm_turnon[15] =  {0x51, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5], 0x80, 0x17, 0x41, 0x72, 0x81, 0x00, 0x00, 0xB0, 0x01, 0x00};
byte hm_turnoff[15] = {0x51, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5], 0x80, 0x17, 0x41, 0x72, 0x81, 0x01, 0x00, 0x20, 0x00, 0x00};
byte hm_power[19] =   {0x51, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5], 0x80, 0x17, 0x41, 0x72, 0x81, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const byte RF24_CHANNELS[] = {03, 23, 40, 61, 75};  // Frequency is 2400 + RF24_CHANNELS [MHz]
const byte RF24_PALEVELS[] = {RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX};
const int RF24_TX_TIMEOUT = 2000;  // max time (in ms) for sending RF24 data and receiving an ACK packet

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
const int UVP_SLEEP_DELAY = 30000;  // cycle duration (in msecs) during UVP sleep mode
const int MW_PLUG_TIMER = 60;  // number of secs after which Meanwell plug is automatically turned off (safety feature if system fails)
const int DDNS_UPDATE_INTERVAL = 60;  // DDNS IP address check interval (in secs)
const int EM_RESET_INTERVAL = 600;  // EM internal data reset interval (in secs)
const int READCOMMAND_TIMEOUT = 4;  // max waiting time (in secs) for terminal input
const int HTTP_TIMEOUT = 6;  // max waiting time (in secs) for HTTP responses
const int RF24_TIMEOUT = 4;  // max waiting time (in secs) for RF24 responses
const int ESS_TIMEZONE = +1;  // ESS is installed in this timezone (relative to UTC)
const float ESS_LATITUDE = 46.**;  // geo coordinates of ESS
const float ESS_LONGITUDE = 13.**;
const String GET_ASTRO_TIME = "03:30";  // time at which astro times (sunrise/sunset) will be calculated (after a possible DST change, before sunrise)

// URLs
const String EM_STATUS = "http://" + EM_ADDR + "/status";
const String EM_RESET = "http://" + EM_ADDR + "/reset_data";
const String PM_STATUS = "http://" + PM_ADDR + "/rpc/Switch.GetStatus?id=0";
const String PM_RESET = "http://" + PM_ADDR + "/rpc/Switch.ResetCounters?id=0";
const String PM_ECO_ON = "http://" + PM_ADDR + "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":true}}";
const String PM_ECO_OFF = "http://" + PM_ADDR + "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":false}}";
const String MWPLUG_STATUS = "http://" + MWPLUG_ADDR + "/rpc/Switch.GetStatus?id=0";
const String MWPLUG_RESET = "http://" + MWPLUG_ADDR + "/rpc/Switch.ResetCounters?id=0";
const String MWPLUG_ON = "http://" + MWPLUG_ADDR + "/relay/0?turn=on&timer=" + String(MW_PLUG_TIMER);
const String MWPLUG_OFF = "http://" + MWPLUG_ADDR + "/relay/0?turn=off";
const String MWPLUG_ECO_ON = "http://" + MWPLUG_ADDR + "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":true}}";
const String MWPLUG_ECO_OFF = "http://" + MWPLUG_ADDR + "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":false}}";
const String HMPLUG_ON = "http://" + HMPLUG_ADDR + "/relay/0?turn=on";
const String HMPLUG_OFF = "http://" + HMPLUG_ADDR + "/relay/0?turn=off";
const String HMPLUG_STATUS = "http://" + HMPLUG_ADDR + "/status";
const String HMPLUG_RESET = "http://" + HMPLUG_ADDR + "/reboot";
const String PUBLIC_IP_SERVER = "http://api.ipify.org";  // public service for obtaining WiFi routers public IP address
// const String PUBLIC_IP_SERVER = "http://ifconfig.me/ip";  // alternative service
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
const String ESS_FLOW_SYMBOL[] = {"───","╴\033[32m◀\033[0m╶","╴\033[32m▶\033[0m╶"};  // green flow symbols
const String PV_FLOW_SYMBOL[] = {"───","╴\033[33m◀\033[0m╶","╴\033[33m▶\033[0m╶"};  // yellow flow symbols
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
const String HOUSE_SYMBOL = "              🏠";
const String GRID_SYMBOL = " 🏭╶";
const String GRID_CABLE_SYMBOL = "─┘";
const String CONS_CABLE_SYMBOL = "└─";
const String CONS_SYMBOL = "╴📺 ";
const String OPS_SYMBOL[] = {" 🏃"," 🧍"," 💤🛌"};
const String POWERFILTER_SYMBOL[] = {" 🕛"," 🕐"," 🕑"," 🕒"," 🕓"," 🕔"," 🕕"," 🕖"," 🕗"," 🕘"," 🕙"," 🕚"};
const String RAMPDOWN_SYMBOL = "🪜";
const String OVP_LIMIT_SYMBOL = "                       ▁▁";
const String UVP_LIMIT_SYMBOL = "       ▔▔";
const String MODE_SYMBOL[] = {""," 👈"," 🔌"};
const String WIFI_SYMBOL[] = {" ⚠️­"," 📶"};
const String ERROR_SYMBOL = "❌";
const String BALANCER_SYMBOL = " 🔄";
const String CLEAR_SCREEN = "\033[0H\033[0J";
const String SHOW_CURSOR = "\033[?25h";
const String HIDE_CURSOR = "\033[?25l";
