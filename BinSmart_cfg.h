// ESS configuration

// Networking
IPAddress DNS_SERVER1(8,8,8,8);  // Google DNS resolver
IPAddress DNS_SERVER2(9,9,9,9);  // Quad9 DNS server
const int GOOD_WIFI_RSSI = -70;  // RSSI above this value is considered "good enough"
const int HTTP_PORT = 80;  // standard HTTP port
const char HTTP_OK[] = "200 OK";  // http success return code
const char DDNS_SERVER[] = "dynupdate.no-ip.com";  // public DDNS service
const char DDNS_URL[] = "/nic/update?hostname=";
const char DDNS_IP[] = "&myip=";
const char PUBLIC_IP_SERVER[] = "api.ipify.org";  // public service for obtaining public IP address
const char PUBLIC_IP_URL[] = "/";
// const char PUBLIC_IP_SERVER[] = "ifconfig.me";  // alternative public IP service
// const char PUBLIC_IP_SERVER_URL[] = "/ip";

// Shelly webhooks
const char EM_STATUS[] = "/status";
const char EM_RESET[] = "/reset_data";
const char PM_CONFIG[] = "/rpc/Shelly.GetConfig";
const char PM_STATUS[] = "/rpc/Switch.GetStatus?id=";
const char MW_RELAY[] = "/relay/0?turn=";
const char ECO_MODE[] = "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":";

// ESP32 pin definitions
const int LED_PIN = 12;
const int PWM_OUTPUT_PIN = 13;
const int RF24_CE_PIN = 32;
const int RF24_CSN_PIN = 33;
const int UART_RX_PIN = 16;
const int UART_TX_PIN = 17;

// Power settings
const int PV_MAX_POWER = 359;  // PV module/inverter max AC output
const int POWER_TARGET_DEFAULT = 5;  // System is aiming for this amount of watts to be drawn from grid
const int POWER_TARGET_TOLERANCE = 5;  // Max tolerated deviation (+/-) from target power
const int POWER_RAMPDOWN_RATE = -40; // Max power decrease per cycle
const int POWER_FILTER_CYCLES = 12;  // Number of cycles during which power spikes are filtered out
const float POWER_LIMIT_RAMPDOWN = 0.67;  // Power rampdown rate when ESS_OVP or ESS_UVP is reached
const float PM2_MW_POWER_CORR = 1.006;  // Power correction factor for MW power readings with PM2

// Time/timer settings
const int PROCESSING_DELAY = 2000;  // minimum delay (in msecs) for power changes to take effect
const int HTTP_TIMEOUT = 3000;  // max waiting time (in msecs) during http requests
const int DDNS_UPDATE_INTERVAL = 60;  // DDNS IP address check interval (in secs)
const int MW_KEEPALIVE = 40;  // number of secs after which Shelly 2PM receives "keep alive" message (must be less than corresponding Shelly timer)
const int RF24_WAIT = 50;  // min Hoymiles RF24 waiting time (in msecs) after previous response
const int RF24_TIMEOUT = 1000;  // max waiting time (in msecs) for RF24 ACKs after writeFast()
const int RF24_KEEPALIVE = 30;  // number of secs after which Hoymiles RF24 interface receives "keep alive" message
const int BMS_WAIT = 50;  // min BMS waiting time (in msecs) after previous response
const int BMS_TIMEOUT1 = 20;  // max waiting time (in msecs) for BMS response
const int BMS_TIMEOUT2 = 3;  // max additional waiting time (in msecs) if response is incomplete
const int BLE_TIMEOUT = 2;  // max waiting time (in secs) for JKBMS BLE server connection
const int USERIO_TIMEOUT = 4000;  // max waiting time (in msecs) for terminal input/output
const int BATT_RECHARGE_TIMEOUT = 96;  // number of hours in BMS UVP mode without charging, before auto recharge is activated

// PWM params for Meanwell power control
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 250;
const int PWM_RESOLUTION = 10;
const int DUTY_CYCLE_MIN = 2;
const int DUTY_CYCLE_MAX = pow(2,PWM_RESOLUTION)-1;

// Meanwell power parameters
const int MW_MIN_POWER = 15;  // Meanwell turned off below min_power (power output would be unstable and inefficient)
const int MW_MAX_POWER = 300;  // theoretical max AC power at vbat < 24000
const int MW_LOW_POWER_THRESHOLD = 31;  // power output below MW_LOW_POWER_THRESHOLD is non-linear
#define MW_MAX_POWER_FORMULA vbat/77.0*(0.98-float(DUTY_CYCLE_MIN)/DUTY_CYCLE_MAX)  // actual max AC power is between 300 and 360W, depending on vbat
#define MW_POWER_FORMULA DUTY_CYCLE_MAX*(0.98-77.0*mw_power/vbat)  // converts current to PWM duty cycle (result of Meanwell HLG-320 tests)
#define MW_LOW_POWER_FORMULA DUTY_CYCLE_MAX*(94523.28*mw_power/vbat*mw_power/vbat-266.51*mw_power/vbat+1.07)  // non-linear formula for low power

// Hoymiles/RF24 comms parameters
const byte RF24_CHANNEL = 03; // Possible RF24 channles for Hoymiles comms are 03, 23, 40, 61, 75; frequency in MHz is 2400 + channel
const byte RF24_PALEVEL = RF24_PA_MIN; // Possible RF24 PA levels are RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
const byte HM_RADIO_ID[] = {0x01, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5]};
const byte HM_POWER_ON = 0x00;
const byte HM_POWER_OFF = 0x01;
const byte HM_SWITCH[2][15] = {{0x51, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5], 0x80, 0x17, 0x41, 0x72, 0x81, HM_POWER_ON, 0x00, 0xB0, 0x01, 0x44},
                               {0x51, HM_SN[2], HM_SN[3], HM_SN[4], HM_SN[5], 0x80, 0x17, 0x41, 0x72, 0x81, HM_POWER_OFF, 0x00, 0x20, 0x00, 0xD4}};

// Hoymiles power parameters
const int HM_MIN_POWER = -15;  // Hoymiles turned off above min_power (power would be too unstable)
const int HM_MAX_POWER = -180;  // Limit of linear power output range
const int HM_LOW_POWER_THRESHOLD = -61;  // Hoymiles power output above this threshold is unstable
const int HM_LOW_POWER_TOLERANCE = 15;  // Max tolerated positive deviation from target power when Hoymiles is below HM_LOW_POWER_THRESHOLD

// BMS/ESS voltage settings in millivolts
const int ESS_OVP_OFFSET = 10;  // vcell_ovp/vcell_ovpr are ESS_OVP_OFFSET below BMS OVP/OVPR
const int ESS_UVP_OFFSET = 5;  // vcell_uvp/vcell_uvpr are ESS_UVP_OFFSET above BMS UVP/UVPR
const int BMS_BAL_ON = 3150;  // one cell at or below this voltage: activate bottom balancing
const int BMS_BAL_OFF = 3170;  // all cells at or above this voltage: deactivate bottom balancing

// BMS definitions and commands
const byte RS485_ID1 = 0x4E;
const byte RS485_ID2 = 0x57;
const byte RS485_ACTIVATE = 0x01;
const byte RS485_WRITE_DATA = 0x02;
const byte RS485_READ_DATA = 0x03;
const byte RS485_READ_ALL = 0x06;
const byte RS485_VCELLS_ID = 0x79;
const byte RS485_CURRENT_ID = 0x84;
const byte RS485_DISCH_SW_ID = 0xAC;
const int RS485_LEN_POS = 3;
const int RS485_COMMAND_POS = 8;
const int RS485_DATA_ID_POS = 11;
const int RS485_WARNINGS_POS = 68;
const int RS485_OVP_POS = 80;
const int RS485_OVPR_POS = 83;
const int RS485_UVP_POS = 89;
const int RS485_UVPR_POS = 92;
const int RS485_BAL_ST_POS = 113;
const int RS485_BAL_TR_POS = 116;
const int RS485_BAL_SW_POS = 119;
const int RS485_DISCH_SW_POS = 163;
const byte RS485_WAKEUP[] = {RS485_ID1, RS485_ID2, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, RS485_ACTIVATE, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x24};
const byte RS485_READ_SETTINGS[] = {RS485_ID1, RS485_ID2, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, RS485_READ_ALL, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x29};
const byte RS485_READ_VOLTAGES[] = {RS485_ID1, RS485_ID2, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, RS485_READ_DATA, 0x03, 0x00, RS485_VCELLS_ID, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x9F};
const byte RS485_READ_CURRENT[] = {RS485_ID1, RS485_ID2, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, RS485_READ_DATA, 0x03, 0x00, RS485_CURRENT_ID, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0xAA};
const byte RS485_DISCH_ON[] = {RS485_ID1, RS485_ID2, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, RS485_WRITE_DATA, 0x03, 0x00, RS485_DISCH_SW_ID, 0x01, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0xD3};
const byte RS485_DISCH_OFF[] = {RS485_ID1, RS485_ID2, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, RS485_WRITE_DATA, 0x03, 0x00, RS485_DISCH_SW_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0xD2};
const byte BLE_ID1 = 0xAA;
const byte BLE_ID2 = 0x55;
const byte BLE_ID3 = 0x90;
const byte BLE_ID4 = 0xEB;
const byte BLE_INFO = 0x97;
const byte BLE_DATA = 0x96;
const byte BLE_BAL_SWITCH = 0x1F;
const int BLE_COMMAND_LEN = 20;
const int BLE_COMMAND_POS = 4;
const int BLE_SETTING_POS = 6;
const byte BLE_GET_INFO[] = {BLE_ID1, BLE_ID2, BLE_ID3, BLE_ID4, BLE_INFO, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11};
const byte BLE_GET_DATA[] = {BLE_ID1, BLE_ID2, BLE_ID3, BLE_ID4, BLE_DATA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10};
const byte BLE_BAL_OFF[] = {BLE_ID1, BLE_ID2, BLE_ID3, BLE_ID4, BLE_BAL_SWITCH, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9D};
const byte BLE_BAL_ON[] = {BLE_ID1, BLE_ID2, BLE_ID3, BLE_ID4, BLE_BAL_SWITCH, 0x04, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9E};

// Error definitions
const int ERROR_TYPES = 7;
const int UNCRITICAL_ERROR_TYPES = 2;  // ERROR_LIMIT doesn't apply to first ... error types
const char ERROR_TYPE[ERROR_TYPES][5] = {"WIFI", "DDNS", "BMS", "RF24", "3EM", "1PM", "2PM"};  // error messages correspond with these types! changes here also need changed error messages
const int ERROR_LIMIT = 20;  // number of consecutive erroneous cycles before error is considered persistent and system is halted

// Symbols for a nice telnet frontend
const char PV_FLOW_SYMBOL[3][30] = {"────", "╴\033[33m▶\033[0m╶─", "╴\033[33m▶▶\033[0m╶"};
const char ESS_FLOW_SYMBOL[3][15] = {"────", "─╴▷┃", "─┃◁╶"};
const char MW_FLOW_SYMBOL[3][2][30] = {{"─╴\033[33m▶\033[0m╶", "─╴\033[31m▶\033[0m╶"}, {"─╴\033[33m▶\033[0m╏", "─╴\033[31m▶\033[0m╏"}, {"╴\033[33m▶▶\033[0m╶", "╴\033[31m▶▶\033[0m╶"}};
const char HM_FLOW_SYMBOL[3][30] = {"─╴\033[32m◀\033[0m╶", "─╏\033[32m◀\033[0m╶", "╴\033[32m◀◀\033[0m╶"};
const char GRID_FLOW_SYMBOL[3][2][30] = {{"────", "────"}, {"╴\033[31m▶\033[0m╶─", "╴\033[31m▶\033[0m╶─"}, {"╴\033[33m◀\033[0m╶─", "╴\033[32m◀\033[0m╶─"}};
const char CONS_FLOW_SYMBOL[3][30] = {"─╴\033[31m▶\033[0m╶", "─╴\033[32m▶\033[0m╶", "─╴\033[33m▶\033[0m╶"};
const char DIFF_SYMBOL[3][10] = {" ▲"," ▼"," ▼🪜"};
const int  BAT_LEVELS = 9;  // number of different battery levels that can be visualized
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
