// ESS configuration

// ESP32 pin definitions
#define LED_PIN 12
#define PWM_OUTPUT_PIN 13
#define RF24_CE_PIN 32
#define RF24_CSN_PIN 33
#define UART_RX_PIN 16
#define UART_TX_PIN 17

// Networking
#define WIFI_SSID "***"
#define WIFI_PWD "***"
#define GOOD_WIFI_RSSI -70  // RSSI above this value is considered "good enough"
#define HTTP_PORT 80  // standard HTTP port
#define TELNET_PORT ***  // TCP port for communication with terminal
#define JKBMS_MAC_ADDR "*:*:*:*:*:*"  // MAC (= BLE) address of JKBMS
IPAddress ESP32_ADDR(*,*,*,*);  // Local IP address of ESS
IPAddress ROUTER_ADDR(*,*,*,*);   // Local WiFi router
IPAddress SUBNET(*,*,*,*);   // WiFi subnet
IPAddress DNS_SERVER1(8,8,8,8);  // Google DNS resolver
IPAddress DNS_SERVER2(9,9,9,9);  // Quad9 DNS server
IPAddress EM_ADDR(*,*,*,*);  // Shelly 3EM
IPAddress PM1_ADDR(*,*,*,*);  // Shelly Plus 1PM, connecting Maxeon solar panel to AC
IPAddress PM2_ADDR(*,*,*,*);  // Shelly 2PM Gen3, connecting Meanwell charger and Hoymiles inverter to AC

// Power settings
const int PV_MAX_POWER = 359;  // PV module/inverter max AC output
const int POWER_TARGET_DEFAULT = 5;  // System is aiming for this amount of watts to be drawn from grid
const int POWER_TARGET_TOLERANCE = 5;  // Max tolerated deviation (+/-) from target power
const int POWER_RAMPDOWN_RATE = -40; // Max power decrease per cycle
const int POWER_FILTER_CYCLES = 12;  // Number of cycles during which power spikes are filtered out
const float POWER_LIMIT_RAMPDOWN = 0.67;  // Power rampdown rate when ESS_OVP or ESS_UVP is reached
const float PM2_MW_POWER_CORR = 1.006;  // Power correction factor for MW power readings with PM2

// Time/timer/astro settings
const int PROCESSING_DELAY = 2000;  // minimum delay (in msecs) for power changes to take effect
const int HTTP_SHELLY_TIMEOUT = 4000;  // max waiting time (in msecs) for Shelly HTTP responses
const int HTTP_DDNS_TIMEOUT = 1000;  // max waiting time (in msecs) for DDNS/public IP responses
const int DDNS_UPDATE_INTERVAL = 120;  // DDNS IP address check interval (in secs)
const int MW_KEEPALIVE = 40;  // number of secs after which Shelly 2PM receives "keep alive" message (must be less than corresponding Shelly timer)
const int RF24_WAIT = 50;  // min waiting time (in msecs) for Hoymiles RF24 interface to be ready
const int RF24_TIMEOUT = 1000;  // max waiting time (in msecs) for RF24 ACKs after writeFast()
const int RF24_KEEPALIVE = 30;  // number of secs after which Hoymiles RF24 interface receives "keep alive" message
const int BMS_WAIT = 50;  // min waiting time (in msecs) for BMS to be ready
const int BMS_TIMEOUT = 20;  // max waiting time (in msecs) for BMS response
const int BLE_TIMEOUT = 2;  // max waiting time (in secs) for JKBMS BLE server connection
const int USERIO_TIMEOUT = 4000;  // max waiting time (in msecs) for terminal input/output
const float LATITUDE = **.***;  // latitude of ESS
const float LONGITUDE = **.**;  // longitude of ESS
const int TIMEZONE = *;  // timezone (relative to UTC) of ESS

// PWM params for Meanwell power control
#define PWM_CHANNEL 0
#define PWM_FREQ 250
#define PWM_RESOLUTION 10
#define DUTY_CYCLE_MIN 4
#define DUTY_CYCLE_MAX 1023

// Meanwell power parameters
const int MW_MIN_POWER = 15;  // Meanwell turned off below min_power (power output would be unstable and inefficient)
const int MW_MAX_POWER = 300;  // theoretical max AC power at vbat < 24000
#define MW_MAX_POWER_FORMULA vbat/77.648*(0.984-float(DUTY_CYCLE_MIN)/DUTY_CYCLE_MAX)  // actual max AC power is between 300 and 360W, depending on vbat
#define MW_POWER_FORMULA DUTY_CYCLE_MAX*(0.984-77.648*mw_power/vbat)  // converts current to PWM duty cycle (result of Meanwell HLG-320 tests)
#define MW_LOW_POWER_THRESHOLD 31  // power output below this threshold is non-linear (result of HLG-320 tests):
#define MW_LOW_POWER_FORMULA DUTY_CYCLE_MAX*(94523.2839*mw_power/vbat*mw_power/vbat-266.5076*mw_power/vbat+1.075)

// Hoymiles/RF24 comms parameters
const byte RF24_CHANNEL = 03; // Possible RF24 channles for Hoymiles comms are 03, 23, 40, 61, 75; frequency in MHz is 2400 + channel
const byte RF24_PALEVEL = RF24_PA_MIN; // Possible RF24 PA levels are RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
const byte HM_SN[6] = {0x11, 0x21, 0x83, 0x81, 0x95, 0x16};  // serial number of Hoymiles inverter
const byte HM_RADIO_ID[5] = {0x01, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5]};
const byte HM_POWER_ON = 0x00;
const byte HM_POWER_OFF = 0x01;
const byte HM_SWITCH[2][15] = {{0x51, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5], 0x80, 0x17, 0x41, 0x72, 0x81, HM_POWER_ON, 0x00, 0xB0, 0x01, 0x44},
                               {0x51, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5], 0x80, 0x17, 0x41, 0x72, 0x81, HM_POWER_OFF, 0x00, 0x20, 0x00, 0xD4}};

// Hoymiles power parameters
const int HM_MIN_POWER = -15;  // Hoymiles turned off above min_power (power would be too unstable)
const int HM_MAX_POWER = -180;  // Limit of linear power output range
#define HM_LOW_POWER_THRESHOLD -61  // Hoymiles power output above this threshold is unstable
#define HM_LOW_POWER_TOLERANCE 15  // Max tolerated positive deviation from target power when Hoymiles is below HM_LOW_POWER_THRESHOLD

// Shelly webhooks
const char EM_STATUS[] = "/status";
const char EM_RESET[] = "/reset_data";
const char PM_CONFIG[] = "/rpc/Shelly.GetConfig";
const char PM_STATUS[] = "/rpc/Switch.GetStatus?id=";
const char MW_RELAY[] = "/relay/0?turn=";
const char HM_RELAY[] = "/relay/1?turn=";
const char ECO_MODE[] = "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":";

// URLs
const char PUBLIC_IP_URL[] = "http://api.ipify.org";  // public service for obtaining WiFi router public IP address
// const char PUBLIC_IP_URL[] = "http://ifconfig.me/ip";  // alternative service
const char DDNS_SERVER_URL[] = "http://***:***@dynupdate.no-ip.com/nic/update?hostname=***.***.***&myip=";  // public DDNS service

// BMS/ESS voltage protection settings in millivolts
const int ESS_OVP = 3500;  // one cell above this voltage: ramp down charging power
const int ESS_OVPR = 3450;  // all cells below this voltage: re-enable charging (should be the same as BMS Balancer Start Voltage)
const int ESS_UVP = 3150;  // one cell below this voltage: ramp down discharging power
const int ESS_UVPR = 3250;  // all cells above this voltage: re-enable discharging
const int ESS_BMS_OVP_DIFF = 100;  // min difference between ESS and BMS OVP settings (BMS_OVP - ESS_OVP >= ESS_BMS_OVP_DIFF)
const int ESS_BMS_UVP_DIFF = 100;  // min difference between ESS and BMS UVP settings (ESS_UVP - BMS_UVP >= ESS_BMS_UVP_DIFF)
const int BMS_BAL_ON = 3145;  // one cell at or below this voltage: activate cell bottom balancing
const int BMS_BAL_OFF = 3160;  // all cells at or above this voltage: deactivate cell balancing
const int BAT_FULL = 27600;  // voltage at which battery is considered full
const int BAT_EMPTY = 8*ESS_UVP;  // voltage at which battery is considered empty
const int BAT_LEVELS = 9;  // number of different battery levels that can be visualized

// BMS definitions and commands
#define RS485_1 0x4E
#define RS485_2 0x57
#define RS485_LEN_POS 3
#define RS485_COMMAND_POS 8
#define WRITE_DATA 0x02
#define READ_DATA 0x03
#define READ_ALL 0x06
#define DATA_ID_POS 11
#define VCELLS_ID 0x79
#define CURRENT_ID 0x84
#define DISCH_SW_ID 0xAC
#define WARNINGS_POS 68
#define OVP_POS 80
#define UVP_POS 89
#define BAL_ST_POS 113
#define BAL_TR_POS 116
#define BAL_SW_POS 119
#define DISCH_SW_POS 163
const byte READ_SETTINGS[] = {RS485_1, RS485_2, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, READ_ALL, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x29};
const byte READ_VOLTAGES[] = {RS485_1, RS485_2, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, READ_DATA, 0x03, 0x00, VCELLS_ID, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x9F};
const byte READ_CURRENT[] = {RS485_1, RS485_2, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, READ_DATA, 0x03, 0x00, CURRENT_ID, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0xAA};
const byte DISCH_ON[] = {RS485_1, RS485_2, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, WRITE_DATA, 0x03, 0x00, DISCH_SW_ID, 0x01, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0xD3};
const byte DISCH_OFF[] = {RS485_1, RS485_2, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, WRITE_DATA, 0x03, 0x00, DISCH_SW_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0xD2};
#define BLE_1 0xAA
#define BLE_2 0x55
#define BLE_3 0x90
#define BLE_4 0xEB
#define BLE_COMMAND_LEN 20
#define BLE_COMMAND_POS 4
#define BLE_SETTING_POS 6
#define BLE_INFO 0x97
#define BLE_DATA 0x96
#define BLE_BAL_SWITCH 0x1F
const byte GET_INFO[BLE_COMMAND_LEN] = {BLE_1, BLE_2, BLE_3, BLE_4, BLE_INFO, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11};
const byte GET_DATA[BLE_COMMAND_LEN] = {BLE_1, BLE_2, BLE_3, BLE_4, BLE_DATA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10};
const byte BAL_OFF[BLE_COMMAND_LEN] = {BLE_1, BLE_2, BLE_3, BLE_4, BLE_BAL_SWITCH, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9D};
const byte BAL_ON[BLE_COMMAND_LEN] = {BLE_1, BLE_2, BLE_3, BLE_4, BLE_BAL_SWITCH, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9E};

// Error settings
const int ERROR_TYPES = 7;
const int UNCRITICAL_ERROR_TYPES = 2;  // ERROR_LIMIT doesn't apply to first ... error types
const String ERROR_TYPE[ERROR_TYPES][5] = {"WIFI", "DDNS", "BMS", "RF24", "3EM", "1PM", "2PM"};  // error messages correspond with these types! changes here also need changed error messages
const int ERROR_LIMIT = 20;  // number of consecutive erroneous cycles before error is considered persistent and system is halted

// Symbols for a nice telnet frontend
const char PV_FLOW_SYMBOL[3][30] = {"────", "╴\033[33m▶\033[0m╶─", "╴\033[33m▶▶\033[0m╶"};
const char ESS_FLOW_SYMBOL[3][15] = {"────", "─╴▷┃", "─┃◁╶"};
const char MW_FLOW_SYMBOL[3][2][30] = {{"─╴\033[33m▶\033[0m╶", "─╴\033[31m▶\033[0m╶"}, {"─╴\033[33m▶\033[0m╏", "─╴\033[31m▶\033[0m╏"}, {"╴\033[33m▶▶\033[0m╶", "╴\033[31m▶▶\033[0m╶"}};
const char HM_FLOW_SYMBOL[3][30] = {"─╴\033[32m◀\033[0m╶", "─╏\033[32m◀\033[0m╶", "╴\033[32m◀◀\033[0m╶"};
const char GRID_FLOW_SYMBOL[3][2][30] = {{"────", "────"}, {"╴\033[31m▶\033[0m╶─", "╴\033[31m▶\033[0m╶─"}, {"╴\033[33m◀\033[0m╶─", "╴\033[32m◀\033[0m╶─"}};
const char CONS_FLOW_SYMBOL[3][30] = {"─╴\033[31m▶\033[0m╶", "─╴\033[32m▶\033[0m╶", "─╴\033[33m▶\033[0m╶"};
const char DIFF_SYMBOL[3][10] = {" ▲"," ▼"," ▼🪜"};
const char BAT_LEVEL_SYMBOL[BAT_LEVELS][30] = {"─🔋\033[33m⡀\033[0m ","─🔋\033[32m⡀\033[0m ","─🔋\033[32m⣀\033[0m ","─🔋\033[32m⣄\033[0m ","─🔋\033[32m⣤\033[0m ","─🔋\033[32m⣦\033[0m ","─🔋\033[32m⣶\033[0m ","─🔋\033[32m⣷\033[0m ","─🔋\033[32m⣿\033[0m "};
const char BAT_OVP_SYMBOL[3][32] = {"", "                     ▁ ▁", "                     ▁▁▁"};
const char BAT_UVP_SYMBOL[3][20] = {"", "      ▔ ▔", "      ▔▔▔"};
const char NIGHT_DAY_SYMBOL[2][8] = {"🌙╶","🌞╶"};
const char CABLE_SYMBOL[] = "─";
const char PV_CABLE_SYMBOL[] = "┐";
const char ESS_CABLE_SYMBOL[] = "┌";
const char ESS_SYMBOL[] = "─🔋";
const char HOUSE_SYMBOL[] = "             🏠";
const char GRID_SYMBOL[] = "🏭╶";
const char GRID_CABLE_SYMBOL[] = "┘";
const char CONS_CABLE_SYMBOL[] = "└";
const char CONS_SYMBOL[] = "╴📺";
const char OPS_SYMBOL[3][10] = {" 🏃"," 🧍"," 💤🛌"};
const char POWERFILTER_SYMBOL[] = " ⏳";
const char MANUAL_MODE_SYMBOL[] = " 👈";
const char AUTO_RECHARGE_SYMBOL[] = " ⚡";
const char WIFI_SYMBOL[2][10] = {"⚠️­","📶"};
const char ERROR_SYMBOL[] = "❌ ";
const char BALANCER_SYMBOL[] = " 🔄";
const char CLEAR_SCREEN[] = "\033[0H\033[0J";
const char CMD_PROMPT[] = "Enter command or [h] for help: ";
