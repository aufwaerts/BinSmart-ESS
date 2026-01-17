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
IPAddress EM_ADDR(*,*,*,*);  // Shelly 3EM
IPAddress PM1_ADDR(*,*,*,*);  // Shelly Plus 1PM, connecting Maxeon solar panel to AC
IPAddress PM2_ADDR(*,*,*,*);  // Shelly 2PM Gen3, connecting Meanwell charger and Hoymiles inverter to AC
const char* JKBMS_MAC_ADDR = "*:*:*:*:*:*";  // MAC (= BLE) address of JKBMS

// Power settings
const int PV_MAX_POWER = 360;  // PV module/inverter max AC output
const int POWER_TARGET_DEFAULT = 5;  // System is aiming for this amount of watts to be drawn from grid
const int POWER_TARGET_TOLERANCE = 5;  // Max tolerated deviation (+/-) from target power
const int HM_LOW_POWER_TOLERANCE = 15;  // Max tolerated positive deviation when Hoymiles is below HM_LOW_POWER_THRESHOLD
const int POWER_RAMPDOWN_RATE = -40; // Max power decrease per cycle
const int POWER_FILTER_CYCLES = 12;  // Number of cycles during which power spikes are filtered out
const float POWER_LIMIT_RAMPDOWN = 0.67;  // Power rampdown rate when ESS_OVP or ESS_UVP is reached

// Time/Timer settings
const unsigned long PROCESSING_DELAY = 2000;  // minimum delay (in ms) for power changes to take effect
const unsigned long DDNS_UPDATE_INTERVAL = 60000;  // DDNS IP address check interval (in ms)
const unsigned long READINPUT_TIMEOUT = 4000;  // max waiting time (in ms) for terminal input
const unsigned int HTTP_SHELLY_TIMEOUT = 4000;  // max waiting time (in ms) for Shelly HTTP responses
const unsigned int HTTP_DDNS_TIMEOUT = 1000;  // max waiting time (in ms) for DDNS/public IP responses
const unsigned long RF24_TIMEOUT = 4000;  // max waiting time (in ms) for RF24 responses
const unsigned long RF24_KEEPALIVE = 30000;  // number of ms after which Hoymiles RF24 interface receives "keep alive" message
const unsigned long BLE_TIMEOUT = 2;  // max waiting time (in secs) for JKBMS BLE server connection
const unsigned long MW_TIMER = 60000;  // number of ms after which Meanwell is automatically turned off (unless keep-alive message is received)
const int ESS_TIMEZONE = +1;  // ESS is installed in this timezone (relative to UTC)
const float ESS_LATITUDE = 46.***;  // geo coordinates of ESS
const float ESS_LONGITUDE = 13.***;
const String GET_ASTRO_TIME = "03:30";  // time at which astro times (sunrise/sunset) will be calculated (after a possible DST change, before sunrise)

// Meanwell (charging) power parameters
const int MW_MAX_POWER = 300;  // max power output at minimum voltage (24V)
// max power output ranges from 300 to 350W, depending on vbat
#define MW_POWER_LIMIT_FORMULA vbat/77.057*(0.9636-1.0/PWM_DUTY_CYCLE_MAX)
const int MW_MIN_POWER = 15;  // Meanwell turned off below min_power (power output would be unstable and very inefficient)
const int MW_LOW_POWER_THRESHOLD = 25;  // power output below this threshold is non-linear
const int MW_RECHARGE_POWER = 100;  // power setting for automatic recharging (prevents BMS UVP)
// the following formulas are the results of Meanwell HLG-320 power output tests
// PWM signal controls Meanwell charging current; charging power also depends on vbat
// higher PWM value means less power
#define MW_POWER_FORMULA PWM_DUTY_CYCLE_MAX*(0.9636-77.057*I)
#define MW_LOW_POWER_FORMULA PWM_DUTY_CYCLE_MAX*(129119.635*I*I-321.337*I+1.08)

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

// Shelly http commands
const String EM_STATUS = "/status";
const String EM_RESET = "/reset_data";
const String PM_CH0_STATUS = "/rpc/Switch.GetStatus?id=0";
const String PM_CH1_STATUS = "/rpc/Switch.GetStatus?id=1";
const String PM_ECO_ON = "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":true}}";
const String PM_ECO_OFF = "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":false}}";
const String PM_CH0_ON = "/relay/0?turn=on&timer=" + String(MW_TIMER/1000);
const String PM_CH0_OFF = "/relay/0?turn=off";
const String PM_CH1_ON = "/relay/1?turn=on";
const String PM_CH1_OFF = "/relay/1?turn=off";

// URLs
const String PUBLIC_IP_SERVER = "http://api.ipify.org";  // public service for obtaining WiFi routers public IP address
// const String PUBLIC_IP_SERVER = "http://ifconfig.me/ip";  // alternative service
const String DDNS_SERVER_UPDATE = "http://***:***@dynupdate.no-ip.com/nic/update?hostname=***.ddns.net&myip=";  // public DDNS service

// BMS/ESS voltage protection settings in millivolts
const int ESS_OVP = 3500;  // one cell above this voltage: ramp down charging power
const int ESS_OVPR = 3450;  // all cells below this voltage: re-enable charging (should be the same as BMS Balancer Start Voltage)
const int ESS_UVP = 3150;  // one cell below this voltage: ramp down discharging power
const int ESS_UVPR = 3200;  // all cells above this voltage: re-enable discharging
const int ESS_BMS_OVP_DIFF = 100;  // min difference between ESS and BMS OVP settings (bms_ovp - ESS_OVP >= ESS_BMS_OVP_DIFF)
const int ESS_BMS_UVP_DIFF = 100;  // min difference between ESS and BMS UVP settings (ESS_UVP - bms_uvp >= ESS_BMS_UVP_DIFF)
const int BAT_FULL = 27600;  // voltage at which battery is considered full
const int BAT_EMPTY = 8*ESS_UVP;  // voltage at which battery is considered empty

// BMS definitions and commands
#define RS485_COMMAND_LEN 21
#define RS485_COMMAND_POS 8
#define WRITE_DATA 0x02
#define READ_DATA 0x03
#define READ_ALL 0x06
#define DATA_ID_POS 11
#define VCELLS_ID 0x79
#define CURRENT_ID 0x84
#define WARNINGS_POS 67
#define WARNINGS_ID 0x8B
#define OVP_POS 79
#define OVP_ID 0x90
#define UVP_POS 88
#define UVP_ID 0x93
#define SBAL_POS 112
#define SBAL_ID 0x9B
#define TBAL_POS 115
#define TBAL_ID 0x9C
#define RS485_1 0x4E
#define RS485_2 0x57
const byte READ_SETTINGS[RS485_COMMAND_LEN] = {RS485_1, RS485_2, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, READ_ALL, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x29};
const byte READ_VOLTAGES[RS485_COMMAND_LEN] = {RS485_1, RS485_2, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, READ_DATA, 0x03, 0x00, VCELLS_ID, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x9F};
const byte READ_CURRENT[RS485_COMMAND_LEN] = {RS485_1, RS485_2, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, READ_DATA, 0x03, 0x00, CURRENT_ID, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0xAA};
#define BLE_COMMAND_LEN 20
#define BLE_COMMAND_POS 4
#define BLE_SETTING_POS 6
#define BLE_INFO 0x97
#define BLE_DATA 0x96
#define BLE_BAL_SWITCH 0x1F
#define BLE_1 0xAA
#define BLE_2 0x55
#define BLE_3 0x90
#define BLE_4 0xEB
const byte GET_INFO[BLE_COMMAND_LEN] = {BLE_1, BLE_2, BLE_3, BLE_4, BLE_INFO, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11};
const byte GET_DATA[BLE_COMMAND_LEN] = {BLE_1, BLE_2, BLE_3, BLE_4, BLE_DATA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10};
const byte BAL_OFF[BLE_COMMAND_LEN] = {BLE_1, BLE_2, BLE_3, BLE_4, BLE_BAL_SWITCH, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9D};
const byte BAL_ON[BLE_COMMAND_LEN] = {BLE_1, BLE_2, BLE_3, BLE_4, BLE_BAL_SWITCH, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9E};

// Errors
const String ERROR_TYPE[] = {"WIFI", "DDNS", "BMS", "RF24", "3EM", "1PM", "2PM"};  // error messages correspond with these types! changes here also need changed error messages
const int ERROR_TYPES = sizeof(ERROR_TYPE)/sizeof(ERROR_TYPE[0]);
const int ERROR_LIMIT = 20;  // number of consecutive erroneous cycles before error is considered persistent and system is halted
const int UNCRITICAL_ERROR_TYPES = 2;  // ERROR_LIMIT doesn't apply to first ... error types

// Symbols for a nice telnet frontend
const String PV_FLOW_SYMBOL[3] = {"────", "╴\033[33m▶\033[0m╶─", "╴\033[33m▶▶\033[0m╶"};
const String ESS_FLOW_SYMBOL[3] = {"────", "─╴▷┃", "─┃◁╶"};
const String MW_FLOW_SYMBOL[3][2] = {{"─╴\033[33m▶\033[0m╶", "─╴\033[31m▶\033[0m╶"}, {"─╴\033[33m▶\033[0m╏", "─╴\033[31m▶\033[0m╏"}, {"╴\033[33m▶▶\033[0m╶", "╴\033[31m▶▶\033[0m╶"}};
const String HM_FLOW_SYMBOL[3] = {"─╴\033[32m◀\033[0m╶", "─╏\033[32m◀\033[0m╶", "╴\033[32m◀◀\033[0m╶"};
const String GRID_FLOW_SYMBOL[3][2] = {{"────", "────"}, {"╴\033[31m▶\033[0m╶─", "╴\033[31m▶\033[0m╶─"}, {"╴\033[33m◀\033[0m╶─", "╴\033[32m◀\033[0m╶─"}};
const String CONS_FLOW_SYMBOL[3] = {"─╴\033[31m▶\033[0m╶", "─╴\033[32m▶\033[0m╶", "─╴\033[33m▶\033[0m╶"};
const String DIFF_SYMBOL[] = {" ▲"," ▼"," ▼🪜"};
const String BAT_LEVEL_SYMBOL[] = {"─🔋\033[33m⡀\033[0m ","─🔋\033[32m⡀\033[0m ","─🔋\033[32m⣀\033[0m ","─🔋\033[32m⣄\033[0m ","─🔋\033[32m⣤\033[0m ","─🔋\033[32m⣦\033[0m ","─🔋\033[32m⣶\033[0m ","─🔋\033[32m⣷\033[0m ","─🔋\033[32m⣿\033[0m "};
const int BAT_LEVELS = sizeof(BAT_LEVEL_SYMBOL)/sizeof(BAT_LEVEL_SYMBOL[0]);
const String BAT_OVP_SYMBOL[3] = {" ", "                     ▁ ▁", "                     ▁▁▁"};
const String BAT_UVP_SYMBOL[3] = {" ", "      ▔ ▔", "      ▔▔▔"};
const String NIGHT_DAY_SYMBOL[2] = {" 🌙╶"," 🌞╶"};
const String PV_CABLE_SYMBOL = "─┐";
const String ESS_CABLE_SYMBOL = "┌─";
const String ESS_SYMBOL = "─🔋 ";
const String HOUSE_SYMBOL = "             🏠";
const String GRID_SYMBOL = " 🏭╶";
const String GRID_CABLE_SYMBOL = "─┘";
const String CONS_CABLE_SYMBOL = "└─";
const String CONS_SYMBOL = "╴📺 ";
const String OPS_SYMBOL[3] = {" 🏃"," 🧍"," 💤🛌"};
const String POWERFILTER_SYMBOL = " ⏳";
const String MANUAL_MODE_SYMBOL = " 👈";
const String AUTO_RECHARGE_SYMBOL = " ⚡";
const String WIFI_SYMBOL[2] = {"⚠️­","📶"};
const String ERROR_SYMBOL = "❌ ";
const String BALANCER_SYMBOL = " 🔄";
const String CLEAR_SCREEN = "\033[0H\033[0J";
const String SHOW_CURSOR = "\033[?25h";
const String HIDE_CURSOR = "\033[?25l";
