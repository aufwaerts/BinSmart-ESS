// *** The following definitions MUST be checked/adapted

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
const int TELNET_PORT = ***;  // TCP port for communication with terminal
const String DDNS_UPDATE = "http://***@dynupdate.no-ip.com/nic/update?hostname=***&myip=";  // public DynDNS server
IPAddress ESP32_ADDR(***,***,***,***);  // Local IP address of ESS
IPAddress ROUTER_ADDR(***,***,***,***);   // Local WiFi router
IPAddress SUBNET(***,***,***,***);   // WiFi subnet
IPAddress DNS_SERVER1(1,1,1,1);  // Cloudflare DNS resolver
IPAddress DNS_SERVER2(1,0,0,1);  // Backup DNS server
const String EM_ADDR = "***.***.***.***";  // Shelly 3EM
const String PM_ADDR = "***.***.***.***";  // Shelly Plus 1PM
const String MWPLUG_ADDR = "***.***.***.***";  // Shelly Plus Plug, connecting Meanwell charger to AC
const String HMPLUG_ADDR = "***.***.***.***";  // Shelly Plus Plug, connecting Hoymiles inverter to AC

// Geo params
const int ESS_TIMEZONE = +1;  // timezone (UTC...) of ESS location
const float ESS_LATITUDE = ***.***;  // geo coordinates of ESS location
const float ESS_LONGITUDE = ***.***;


// *** The following definitions MAY be changed

const String GET_ASTRO_TIME = "03:30";  // time at which astro times (sunrise/sunset) will be calculated (after a possible DST change, before sunrise)
const int GOOD_WIFI_RSSI = -70;  // RSSI above this value is considered "good enough"
const String PUBLIC_IP = "http://api.ipify.org/";  // public service for obtaining WiFi routers public IP address
const int ERROR_LIMIT = 20;  // number of consecutive erroneous cycles before system is halted

// Timer settings
const int PROCESSING_DELAY = 2000;  // minimum delay (in msecs) for power changes to take effect
const int UVP_SLEEP_DELAY = 20000;  // cycle duration (in msecs) during UVP sleep mode
const int UVP_WAKEUP_RESET = 1700;  // number of cycles in UVP mode (hm_power_limit == 0) before UVP sleep mode is activated (must be an even number)
const int NO_COMMAND_TIMER = 200;  // duration (in msecs) before end of cycle, when no user command should be given
const int MW_PLUG_TIMER = 180;  // number of secs after which Meanwell is automatically turned off (unless "keep alive" command resets timer)
const int HM_PLUG_TIMER = 900;  // number of secs after which Hoymiles is automatically turned off (unless "keep alive" command resets timer)
const int DDNS_UPDATE_INTERVAL = 60;  // DDNS IP address check interval (in secs)
const int EM_RESET_INTERVAL = 600;  // EM internal data reset interval (in secs)
const int READCOMMAND_TIMEOUT = 4;  // max waiting time (in secs) for terminal input
const int HTTP_TIMEOUT = 7;  //  max waiting time (in secs) for http response

// Power settings
const int POWER_TARGET_DEFAULT = 5;  // System is aiming for this amount of watts to be drawn from grid
const int EM_MAX_TARGET_DEVIATION = 5;  // Max allowed deviation (+/-) from target power
const int PV_SUN_THRESHOLD = 50;  // Everthing above this value is considered (partly) sunshine
const int PV_MAX_POWER = 350;  // Max PV output
const int HM_MIN_POWER = -5;  // Hoymiles turned off below min_power
const int HM_MAX_POWER = -300;  // Hoymiles power limit (HM-300)
const int MW_MIN_POWER = 10;  // Meanwell turned off above min_power
const int MW_MAX_POWER = 340;  // Meanwell charging power limit (HLG-320H-30AB)
const int MW_RECHARGE_POWER = 200;  // Meanwell power setting for automatic recharging (to prevent BMS turnoff): MW operates at highest efficiency
const int POWER_RAMPDOWN_RATE = 40; // Max power decrease per polling interval, MUST BE EQUAL OR HIGHER THAN ABS(HM_MIN_POWER)
const int POWERFILTER_CYCLES = 9;  // Number of cycles during which power spikes are filtered out
const float POWER_LIMIT_RAMPDOWN = 0.667;  // Power rampdown rate when CELL_OVP or CELL_UVP is reached

// BMS/batt voltages
const int CELL_OVP = 3560;  // battery full voltage [mV] (must be lower than BMS setting)
const int CELL_OVPR = 3450;  // recovery voltage after battery full [mV] (should be higher than BMS setting)
const int CELL_UVP = 3200;  // battery low voltage [mV] (must be higher than BMS setting)
const int CELL_UVPR = 3250;  // recovery voltage after battery low [mV] (should be lower than BMS setting)
const int CELL_UUVP = 3000;  // automatic battery recharge trigger voltage (prevents BMS turnoff) [mV] (independent from BMS settings)
const int BALANCER_THRESHOLD = 3450;  // BMS balancer starts working when at least one cell is above this threshold [mV] (equals BMS setting)
const int BALANCER_CELL_DIFF = 3;  // BMS balancer starts working when difference between cell voltages is larger than this value [mV] (equals BMS setting)

// Meanwell/PWM
#define PWM_CHANNEL 0
#define PWM_FREQ 250
#define PWM_RESOLUTION 10
const int PWM_MAX_DUTY_CYCLE = pow(2,PWM_RESOLUTION)-1;
const int PWM_MAX_LINEAR = round(PWM_MAX_DUTY_CYCLE*0.9);


// *** The following definitions MUST NOT be changed (unless different ESS hardware components are used)

// URLs
const String EM_STATUS = "http://" + EM_ADDR + "/status";
const String EM_RESET = "http://" + EM_ADDR + "/reset_data";
const String PM_STATUS = "http://" + PM_ADDR + "/rpc/Switch.GetStatus?id=0";
const String PM_RESET = "http://" + PM_ADDR + "/rpc/Switch.ResetCounters?id=0";
const String PM_CONFIG = "http://" + PM_ADDR + "/rpc/Shelly.GetConfig";
const String PM_ECO_ON = "http://" + PM_ADDR + "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":true}}";
const String PM_ECO_OFF = "http://" + PM_ADDR + "/rpc/Sys.SetConfig?config={\"device\":{\"eco_mode\":false}}";
const String PM_REBOOT = "http://" + PM_ADDR + "/rpc/Shelly.Reboot?delay_ms=500";
const String MWPLUG_STATUS = "http://" + MWPLUG_ADDR + "/rpc/Switch.GetStatus?id=0";
const String MWPLUG_RESET = "http://" + MWPLUG_ADDR + "/rpc/Switch.ResetCounters?id=0";
const String MWPLUG_ON = "http://" + MWPLUG_ADDR + "/relay/0?turn=on&timer=" + String(MW_PLUG_TIMER);
const String MWPLUG_OFF = "http://" + MWPLUG_ADDR + "/relay/0?turn=off";
const String HMPLUG_STATUS = "http://" + HMPLUG_ADDR + "/rpc/Switch.GetStatus?id=0";
const String HMPLUG_RESET = "http://" + HMPLUG_ADDR + "/rpc/Switch.ResetCounters?id=0";
const String HMPLUG_ON = "http://" + HMPLUG_ADDR + "/relay/0?turn=on&timer=" + String(HM_PLUG_TIMER);
const String HMPLUG_OFF = "http://" + HMPLUG_ADDR + "/relay/0?turn=off";

// BMS comms
#define BMS_RESPONSE_SIZE 46
const byte BMS_READ_VOLTAGES[] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x79, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x9F};

// Hoymiles/RF24 comms
const byte HM_SERIAL[] = {0x11, 0x21, 0x82, 0x85, 0x00, 0x12};  // serial number of Hoymiles inverter
const byte RF24_CHANNELS[] = {03, 23, 40, 61, 75};  // Frequency is 2400 + RF24_CHANNELS [MHz]
const byte RF24_PALEVELS[] = {RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX};
const int RF24_TX_TIMEOUT = 2000;  // max time (in ms) for sending RF24 data and receiving an ACK packet
const int RF24_MIN_DELAY = 20;  // Minimum time gap (in ms) between two consecutive Hoymiles commands

// Errors
const String ERROR_TYPE[] = {"WIFI", "DDNS", "3EM", "1PM", "BMS", "MWPLUG", "HMPLUG", "RF24"};  // error messages correspond with these types, do not change!
const int ERROR_TYPES = sizeof(ERROR_TYPE)/sizeof(ERROR_TYPE[0]);

// Symbols
const String FLOW_SYMBOL[] = {"───","╴◀╶","╴▶╶","╴◀◀","╴▶▶","┇◀╶","╴▶┇","┃◁╶","╴▷┃"};
const String DOWN_SYMBOL = "▼";
const String UP_SYMBOL = "▲";
const String BATT_LEVEL_SYMBOL[] = {"⡀ ","⣀ ","⣄ ","⣤ ","⣦ ","⣶ ","⣷ ","⣿ "};
const int BATT_LEVELS = sizeof(BATT_LEVEL_SYMBOL)/sizeof(BATT_LEVEL_SYMBOL[0]);
const String PV_SYMBOL[] = {" 🌛 ▦"," ☁­ ▦"," ⛅ ▦"," 🌤­ ▦"," ☀­ ▦"};
const int PV_LEVELS = sizeof(PV_SYMBOL)/sizeof(PV_SYMBOL[0]);
const String PV_CABLE_SYMBOL = "─┐┌─";
const String CONS_SYMBOL = "─╴📺 ";
const String CONS_SYMBOL_SHORT = "╴📺 ";
const String HOUSE_SYMBOL = "🏠";
const String GRID_SYMBOL = " 🏭╶─";
const String GRID_CABLE_SYMBOL = "─┘└─";
const String ESS_SYMBOL = "─🔋";
const String NORMAL_OPS_SYMBOL = " 🏃";
const String UVP_SLEEP_SYMBOL = " 💤🛌";
const String POWERFILTER_SYMBOL[] = {" 🕛0"," 🕐1"," 🕑2"," 🕒3"," 🕓4"," 🕔5"," 🕕6"," 🕖7"," 🕗8"," 🕘9"," 🕙10"," 🕚11"};
const String OVP_LIMIT_SYMBOL = "     ▁▁▁";
const String UVP_LIMIT_SYMBOL = "                    ▔▔▔";
const String RAMPDOWN_SYMBOL = " 🪜";
const String CHARGING_SYMBOL = " ⚡";
const String GOOD_WIFI_SYMBOL = "  📶";
const String BAD_WIFI_SYMBOL = "  ⚠ ";
const String ERROR_SYMBOL = "❌";

