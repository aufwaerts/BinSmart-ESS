const String SW_VERSION = "v1.04";

// Definitions, global variables

// Included libraries
#include <WiFi.h>
#include <HTTPClient.h>
#include <AsyncElegantOTA.h>
#include <ESPAsyncWebServer.h>
#include <RF24.h>
#include <CRC8.h>
#include <CRC16.h>
#include <TimeLib.h>
#include <Dusk2Dawn.h>

// ESP32 pin definitions
#define LED_PIN 12
#define PWM_OUTPUT_PIN 13
#define RF24_CE_PIN 32
#define RF24_CSN_PIN 33
#define UART_RX_PIN 16
#define UART_TX_PIN 17

// WIFI credentials and classes
const String WIFI_SSID = "***";
const String WIFI_PWD = "***";
const int GOOD_WIFI_RSSI = -70;
WiFiServer server(***);
AsyncWebServer OTAserver(80);
WiFiClient telnet;
HTTPClient http;
String http_response;

// IP addresses
IPAddress ESP32_ADDR(***,***,***,***);
IPAddress ROUTER_ADDR(***,***,***,***);
IPAddress SUBNET(***,***,***,***);
IPAddress DNS_SERVER1(1,1,1,1);
IPAddress DNS_SERVER2(1,0,0,1);
const String EM_ADDR = "***.***.***.***";
const String PM_ADDR = "***.***.***.***";
const String MWPLUG_ADDR = "***.***.***.***";
const String HMPLUG_ADDR = "***.***.***.***";
String DDNS_address = "0.0.0.0";

// Timer settings
const int PROCESSING_DELAY = 2000;  // minimum delay (in msecs) for power changes to take effect
const int UVP_SLEEP_DELAY = 20000;  // cycle duration (in msecs) during UVP sleep mode
const int UVP_WAKEUP_RESET = 1700;  // number of cycles in UVP mode (hm_power_limit == 0) before UVP sleep mode is activated (must be an even number)
const int NO_COMMAND_TIMER = 200;  // duration (in msecs) before end of cycle, when no user command should be given
const int MW_PLUG_TIMER = 180;  // number of secs after which Meanwell is automatically turned off (unless "keep alive" command resets timer)
const int HM_PLUG_TIMER = 900;  // number of secs after which Hoymiles is automatically turned off (unless "keep alive" command resets timer)
const int DDNS_UPDATE_INTERVAL = 60;  // DDNS IP address check interval (in secs)
const int EM_RESET_INTERVAL = 600;  // EM internal data reset interval (in secs)
const int READCOMMAND_TIMEOUT = 4;  // max waiting time (in sesc) for terminal input
const int HTTP_TIMEOUT = 7;  //  max waiting time (in sesc) for http response

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
const String PUBLIC_IP = "http://api.ipify.org/";
const String DDNS_UPDATE = "http://***:***@dynupdate.no-ip.com/nic/update?hostname=***&myip=";

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

// BMS comms and voltages
const byte BMS_READ_VOLTAGES[] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x79, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x9F};
#define BMS_RESPONSE_SIZE 46
byte BMS_response[BMS_RESPONSE_SIZE];
const int CELL_OVP = 3560;  // battery full voltage (in mV)
const int CELL_OVPR = 3450;  // recovery voltage after battery full
const int CELL_UVP = 3200;  // battery low voltage
const int CELL_UVPR = 3250;  // recovery voltage after battery low
const int CELL_UUVP = 3000;  // automatic battery recharge trigger voltage (prevents BMS turnoff)

// Meanwell/PWM
#define PWM_CHANNEL 0
#define PWM_FREQ 250
#define PWM_RESOLUTION 10
const int PWM_MAX_DUTY_CYCLE = pow(2,PWM_RESOLUTION)-1;
const int PWM_MAX_LINEAR = round(PWM_MAX_DUTY_CYCLE*0.9);

// Errors
const String ERROR_TYPE[] = {"WIFI", "DDNS", "3EM", "1PM", "BMS", "MWPLUG", "HMPLUG", "RF24"};
const int ERROR_TYPES = sizeof(ERROR_TYPE)/sizeof(ERROR_TYPE[0]);
const int ERROR_LIMIT = 20;  // number of consecutive erroneous cycles before system is halted
int error_counter[ERROR_TYPES];
unsigned long errortime[ERROR_TYPES];
int errors_consecutive = 0;
String error_msg;
bool error_flag = false;

// Hoymiles/RF24
const byte RF24_CHANNELS[] = {03, 23, 40, 61, 75};  // Frequency is 2400 + RF24_CHANNELS [MHz]
const byte RF24_PALEVELS[] = {RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX};
const byte RF24_RADIO_ID[] = {0x01, 0x82, 0x85, 0x00, 0x12};  // derived from HM serial number
const int RF24_TX_TIMEOUT = 2000;  // max time (in milliseconds) for sending RF24 data and receiving an ACK packet
const int RF24_TX_BUFFERSIZE = 100;  // size of buffer for tx failure data
const int RF24_MIN_DELAY = 20;  // Minimum time gap (in milliseconds) between two consecutive Hoymiles commands
byte HM_TURNON[] =  {0x51, 0x82, 0x85, 0x00, 0x12, 0x80, 0x17, 0x41, 0x72, 0x81, 0x00, 0x00, 0xB0, 0x01, 0xD0};
byte HM_TURNOFF[] = {0x51, 0x82, 0x85, 0x00, 0x12, 0x80, 0x17, 0x41, 0x72, 0x81, 0x01, 0x00, 0x20, 0x00, 0x40};
byte HM_POWER[] =   {0x51, 0x82, 0x85, 0x00, 0x12, 0x80, 0x17, 0x41, 0x72, 0x81, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
RF24 radio(RF24_CE_PIN, RF24_CSN_PIN);
CRC8 crc8;
CRC16 crc16;

// Time definitions and global variables
const int TIMEZONE = +1;  // timezone (UTC...) of ESS location
Dusk2Dawn ESS_location(***, ***, TIMEZONE);  // geo coordinates of ESS location
const String GET_ASTRO_TIME = "03:30";  // time at which astro times (sunrise/sunset) will be calculated (after a possible DST change, before sunrise)
String current_time = "00:00", sunrise = "00:00", sunset = "00:00";  // will be read from Shelly 3EM or replaced by calculation
bool daytime;  // current time is day or night
int UTC_offset;  // time difference (in hours) between UTC and current time
String datetime_string = "00/00/0000 00:00:00";  // temp string for formatting date and time info
String ct_string = "00:00:00 +";  // temp string for formatting current time incl. seconds
unsigned long unixtime, starttime, resettime_errors, resettime_energy; // epoch times (will be read from Shelly 3EM)
unsigned long minpower_time = 0;  // time of last lowest power consumption reading
unsigned long DDNS_time = 0;  // time of last DDNS update
unsigned long ts_cycle = 0, ts_power = 0, ts_HM_ON = 0, ts_MW_ON = 0, ts_DDNS = 0, ts_EM = 0, ts_EM_reset = 0;  // various timestamps

// Special characters (Font: Cascadia Code)
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

// Other definitions and global variables
String power_string = "0000", energy_string = "000.000";
int power_grid = 0, power_grid_min = 10000, power_pv = 0, power_new = 0, power_old = 0, filter_cycles = POWERFILTER_CYCLES, power_manual = 0;
int power_target = POWER_TARGET_DEFAULT;
int vcell_min = 0, vcell_max = 10000, vbat = 27000;  // Cell min/max voltages and batt voltage in millivolts
int hm_power_limit = HM_MAX_POWER, mw_power_limit = MW_MAX_POWER, uvp_countdown = UVP_WAKEUP_RESET;
float from_pv, from_ess, to_ess;  // energy counters (will be read from Shelly 1PM and Shelly plugs)
float from_grid = 0, grid_to_ess = 0, pv_to_grid = 0, ess_to_grid = 0;  // energy counters (will be calculated)
bool manual_mode = false, auto_recharge = false, pm_eco_mode = true, rampdown = false, limited = false;
String cycle_msg, ota_msg, cmd_msg, filter_symbol;
char received_command = ' ';
char repeat_command = ' ';

void setup() {

    //Reduce CPU clock to 80 MHz (30% of max is enough for this application)
    setCpuFrequencyMhz(80);
    delay(500);

    // reserve memory for global Strings
    filter_symbol.reserve(10);
    ota_msg.reserve(100);
    error_msg.reserve(200);
    cmd_msg.reserve(700);
    cycle_msg.reserve(1000);
    if (!http_response.reserve(1800)) error_msg = "String memory allocation failed";
    else error_msg = cmd_msg = "";

    // extend http response timeout (default is 5000 ms)
    http.setTimeout(HTTP_TIMEOUT*1000);
    http.setReuse(true);

    // Assign LED Pin
    pinMode(LED_PIN, OUTPUT);

    // Init PWM generator (to adjust Meanwell power)
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM_OUTPUT_PIN, PWM_CHANNEL);

    // Init WiFi
    WiFi.config(ESP32_ADDR, ROUTER_ADDR, SUBNET, DNS_SERVER1, DNS_SERVER2);
    WiFi.setTxPower(WIFI_POWER_7dBm);
    WiFi.begin(WIFI_SSID, WIFI_PWD);
    do {
        FlashLED(false); delay(PROCESSING_DELAY);   // if WiFi unavailable or wrong SSID/PWD, system keeps flashing LED in endless loop
    } while (WiFi.status() != WL_CONNECTED);

    // Start OTA software update service
    ota_msg = "BinSmart ESS ";
    ota_msg += SW_VERSION;
    ota_msg += "\nEnter \"";
    ota_msg += ESP32_ADDR.toString();
    ota_msg += "/update\" for OTA firmware update";
    OTAserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { request->send(200, "text/plain", ota_msg); });
    AsyncElegantOTA.begin(&OTAserver);
    OTAserver.begin();

    // Start wifi server and wait for terminal to connect
    server.begin();
    while (!telnet) telnet = server.available();

    // Print startup cycle_msg
    cycle_msg = "BinSmart ESS ";
    cycle_msg += SW_VERSION;
    cycle_msg += "\r\n\r\n";
    telnet.print(cycle_msg);
    delay(1000);
    cycle_msg = "WiFi connected to ";
    cycle_msg += WIFI_SSID;
    cycle_msg += "   RSSI: ";
    cycle_msg += WiFi.RSSI();
    cycle_msg += "   TxPower: ";
    cycle_msg += WiFi.getTxPower();
    cycle_msg += "\r\n";
    telnet.print(cycle_msg);

    // Init RS485 communication with BMS
    Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    if (ReadBMSVoltages()) telnet.print("RS485 communication with JKBMS OK\r\n");
    
    // Init RF24 radio communication with Hoymiles
    if (radio.begin()) {
        radio.setPALevel(RF24_PALEVELS[0]);
        radio.setChannel(RF24_CHANNELS[0]);
        radio.setDataRate (RF24_250KBPS);
        radio.setCRCLength(RF24_CRC_16);
        radio.setAddressWidth(sizeof(RF24_RADIO_ID));
        radio.enableDynamicPayloads();
        radio.stopListening();
        radio.openWritingPipe(RF24_RADIO_ID);
        crc8.setPolynome(0x01);
        crc8.setStartXOR(0);
        crc8.setEndXOR(0);
        crc16.setPolynome((uint16_t)0x18005);
        crc16.setStartXOR(0xFFFF);
        crc16.setEndXOR(0x0000);
        crc16.setReverseIn(true);
        crc16.setReverseOut(true);
        // Turn off Hoymiles (DC side)
        if (HoymilesCommand(HM_TURNOFF, sizeof(HM_TURNOFF))) telnet.print("RF24 communication with Hoymiles OK\r\n");
    }
    else error_msg = "RF24 radio init failed";

    // Turn off Meanwell plug, reset energy counter
    if (ShellyCommand(MWPLUG_OFF))
        if (ShellyCommand(MWPLUG_RESET)) telnet.print("Meanwell Shelly plug found\r\n");

    // Turn on Hoymiles AC plug, reset energy counter
    if (ShellyCommand(HMPLUG_ON))
        if (ShellyCommand(HMPLUG_RESET)) telnet.print("Hoymiles Shelly plug found\r\n");

     // Read Shelly 1PM eco setting, reset energy counter
    if (ShellyCommand(PM_CONFIG))
        if (ShellyCommand(PM_RESET)) telnet.print("Shelly 1PM found\r\n");

    // Read time values from Shelly 3EM, clear internal memory
    if (ShellyCommand(EM_STATUS)) {
        starttime = resettime_errors = resettime_energy = unixtime;
        GetAstroTimes();
        daytime = ((current_time >= sunrise) && (current_time < sunset));
        if (ShellyCommand(EM_RESET)) telnet.print("Shelly 3EM found\r\n");
    }

    // No erros during setup: Zero error counters, set timestamps, start polling cycle
    if (error_msg == "") {
        for (int i=0; i<ERROR_TYPES; i++) error_counter[i] = errortime[i] = 0;
        telnet.print("\r\nNo errors during setup, start polling cycle ...");
        delay(PROCESSING_DELAY);
        ts_cycle = millis();
    }
    else {
        if (telnet.available()) telnet.flush();
        telnet.print("\r\n");
        telnet.print(ERROR_SYMBOL);
        telnet.print(" ");
        telnet.print(error_msg);
        telnet.print("\r\nSetup error(s) occured, system halted");
        telnet.print("\r\n\r\nPress any key to restart: ");
        while (!telnet.available()) FlashLED(false);
        telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
        delay(3000);
        telnet.stop();
        delay(500);
        ESP.restart();
    }
}

void loop() {

    error_msg = "";  // assume there will be no errors this cycle
    ReadBMSVoltages();  // Read cell voltages from BMS, set OVP/UVP power limits
    ShellyCommand(EM_STATUS);  // Read current time, unixtime and grid power
    SetNewPower();  // Calculate and apply new charging/discharging power setting
    FinishCycle();  // Update energy counters, print cycle info, error info and command response, keep Shelly plugs alive, update DDNS
    CheckErrors();  // Check errors, halt system if an error is persistent, or show completed cycle and error status by flashing LED

    // Read user command while waiting for power changes to take effect (UVP sleep mode: extend waiting time)
    int cycle_delay = max(PROCESSING_DELAY, (!uvp_countdown)*UVP_SLEEP_DELAY)-NO_COMMAND_TIMER;
    while (millis()-ts_power < cycle_delay) if (ReadCommand(true)) break;  // read user command
    if (millis()-ts_power >= cycle_delay) ReadCommand(false);  // no user command: repeat previous command (if command was to be repeated)
    http.begin(EM_STATUS); http.connect();  // initiate "long polling" for http EM_STATUS request in next cycle
    while (millis()-ts_power < PROCESSING_DELAY);  // wait for end of processing delay after command was processed
}

bool ReadBMSVoltages() {

    uint16_t bytesum = 0, value, i;
    
    // Send command to BMS via RS485
    Serial2.write(BMS_READ_VOLTAGES, sizeof(BMS_READ_VOLTAGES));
    Serial2.flush();
    // Wait for response
    for (i=0; i<1000; i++) if (Serial2.available()) break;
    
    i = BMS_RESPONSE_SIZE-1; BMS_response[i] = 0x00;
    // Fill response buffer with BMS response in reversed byte order
    while (Serial2.available()) {
          BMS_response[i] = Serial2.read();
          if (i>1) bytesum += BMS_response[i];
          if (i>0) i--;
    }

    // Check validity of BMS response
    memcpy(&value, &BMS_response[BMS_RESPONSE_SIZE-2], 2);
    if (value == 0x4E57) {
        memcpy(&value, &BMS_response[BMS_RESPONSE_SIZE-4], 2);
        if (value == BMS_RESPONSE_SIZE-2) {
            memcpy(&value, &BMS_response[0], 2);
            if (value == bytesum) {

                // Read voltages of batt cells, determine min and max voltages
                vcell_min = 10000; vcell_max = vbat = 0;
                for (i=30; i>=9; i-=3) {
                    memcpy(&value, &BMS_response[i], 2);
                    if (value < vcell_min) vcell_min = value;
                    if (value > vcell_max) vcell_max = value;
                    vbat += value;
                }

                // Turn on/off automatic recharge feature (prevents BMS shutdown at very low cell voltage)
                if (vcell_min <= CELL_UUVP) auto_recharge = true;
                if (vcell_min >= CELL_UVP) auto_recharge = false;

                // Set OVP/OVPR, depending on max cell voltage
                if (vcell_max <= CELL_OVPR) mw_power_limit = MW_MAX_POWER;  // exit OVP mode
                if (vcell_max >= CELL_OVP) {
                    if (mw_power_limit) {
                        mw_power_limit = int(power_new*POWER_LIMIT_RAMPDOWN);  // ramp down charging power limit softly
                        if (mw_power_limit < MW_MIN_POWER) mw_power_limit = 0;  // enter OVP mode
                    }
                }

                // Set UVP/UVPR, depending on min cell voltage
                if (vcell_min >= CELL_UVPR) hm_power_limit = HM_MAX_POWER;  // exit UVP mode
                if (vcell_min <= CELL_UVP) {
                    if (hm_power_limit) {
                        hm_power_limit = int(power_new*POWER_LIMIT_RAMPDOWN);  // ramp down discharging power limit softly
                        if (hm_power_limit > HM_MIN_POWER) hm_power_limit = 0;  // enter UVP mode
                    }
                }
                
                // If in UVP mode, handle uvp_countdown
                if (!hm_power_limit) {  // UVP mode (HM disabled)
                    if (daytime) uvp_countdown = max(uvp_countdown-1, 0);  // daytime: decrease uvp_countdown
                    else uvp_countdown = max(uvp_countdown-(UVP_WAKEUP_RESET/5), 0);  // nighttime: enter UVP sleep mode next cycle
                }

                return true;
            }
        } 
    }
    error_msg = "Could not read batt voltages from BMS";
    return false;
}

bool ShellyCommand(const String URL) {

    if (WiFi.status() != WL_CONNECTED) {
        error_msg = "WIFI";
        return false;
    }

    // Handle commands for Shelly energy meter, power meter or plug
    if (!http.connected()) http.begin(URL);
    if (http.GET() == HTTP_CODE_OK) {
        http_response = http.getString();
        http.end();
        if (URL == EM_STATUS) {
            ts_EM = millis();  // remember exact time when Shelly response was read (allows precise calculations of current time)
            int index = http_response.indexOf("time",140);
            memcpy(&current_time[0], &http_response[index+7], 5);  // current_time considers timezone and DST
            unixtime = atol(&http_response[index+25]);  // epoch time (UTC timezone)
            UTC_offset = (current_time.toInt()+24-hour(unixtime))%24;  // timezone_offset + DST_offset
            unixtime += UTC_offset*3600;  // adjust unixtime to match current_time
            power_grid = round(http_response.substring(http_response.indexOf("total_power",750)+13).toFloat());
            return true;
        }
        if (URL == MWPLUG_ON) {
            ts_MW_ON = millis();
            return true;
        }
        if (URL == HMPLUG_ON) {
            ts_HM_ON = millis();
            return true;
        }
        if (URL == EM_RESET) {
            ts_EM_reset = millis();
            return true;
        }
        if (URL == PM_REBOOT) {
            delay(4000); // wait for Shelly 1PM to reboot
            return true;
        }
        if (URL == PM_STATUS) {
            power_pv = round(http_response.substring(http_response.indexOf("apower",35)+8).toFloat());
            from_pv = http_response.substring(http_response.indexOf("total",80)+7).toFloat()/1000;
            return true;
        }
        if (URL == HMPLUG_STATUS) {
            from_ess = http_response.substring(http_response.indexOf("total",80)+7).toFloat()/1000;
            return true;
        }
        if (URL == MWPLUG_STATUS) {
            to_ess = http_response.substring(http_response.indexOf("total",80)+7).toFloat()/1000;
            return true;
        }
        if (URL == PM_CONFIG) {
            pm_eco_mode = (http_response.indexOf("eco_mode\":true",750)>0);
            return true;
        }
        return true;
    }
    // Shelly command failed
    http.end();
    if (URL.indexOf(EM_ADDR) > 0) error_msg = "3EM";
    if (URL.indexOf(PM_ADDR) > 0) error_msg = "1PM";
    if (URL.indexOf(MWPLUG_ADDR) > 0) error_msg = "MWPLUG";
    if (URL.indexOf(HMPLUG_ADDR) > 0) error_msg = "HMPLUG";
    error_msg += " cmd failed: ";
    error_msg += URL;
    return false;
}

void SetNewPower() {

    // Save power setting from previous cycle as basis for new power calculation
    power_old = power_new;
    // Calculate new power setting
    power_new = power_old - power_grid + power_target;
    if (abs(power_new - power_old) <= EM_MAX_TARGET_DEVIATION) power_new = power_old;

    if (manual_mode) power_new = power_manual;   // Manual mode: manual power setting overrides calculation
    if (auto_recharge) power_new = max(MW_RECHARGE_POWER, power_new);  // Low voltage auto-recharge overrides manual power setting

    // Make sure lower/upper power limits are not exceeded
    limited = false;
    if (power_new < 0) {
        // Hoymiles inverter
        if (power_new < hm_power_limit) {
            power_new = hm_power_limit;
            limited = true;
        }
        if (power_new > HM_MIN_POWER) power_new = 0;
    }
    if (power_new > 0) {
        // Meanwell charger
        if (power_new > mw_power_limit) {
            power_new = mw_power_limit;
            limited = true;
        }
        if (power_new < MW_MIN_POWER) power_new = 0;
    }
    
    // Filter out power spikes and slowly increase discharging power (reduces battery power loss to grid when consumer is suddenly turned  off)
    rampdown = false;
    filter_symbol = "";
    if (!manual_mode && (power_old - power_new > POWER_RAMPDOWN_RATE)) {
        if (power_old > 0) {
            power_new = max(power_old - POWER_RAMPDOWN_RATE, 0);  // MW was charging: ramp down to zero without delay
            rampdown = true;
        }
        else {  // power was off or HM was discharging: ramp down only after filtering out power spikes
            if (!filter_cycles) {
                power_new = power_old - POWER_RAMPDOWN_RATE;
                rampdown = true;
            }
            else {
                power_new = power_old;
                filter_symbol = POWERFILTER_SYMBOL[filter_cycles];
                filter_cycles--;
            }
        }
    }
    else filter_cycles = POWERFILTER_CYCLES;

    int power_n = power_new;  // temp value for decision tree
    // Apply power setting
    if (power_n == 0) {  // turn charging or discharging off
        if (power_old <= 0) if (!HoymilesCommand(HM_TURNOFF, sizeof(HM_TURNOFF))) power_new = power_old;  // repeat command every cycle, just to make sure HM is really turned off
        if (power_old > 0) if (!ShellyCommand(MWPLUG_OFF)) power_new = power_old;
    }
    if (power_n > 0) {  // set new charging power
        if (power_new != power_old) SetMWPower();  // set new PWM value only if power setting has changed
        if (power_old == 0) if (!ShellyCommand(MWPLUG_ON)) power_new = 0;
        if (power_old < 0) {
            if (!HoymilesCommand(HM_TURNOFF, sizeof(HM_TURNOFF))) power_new = power_old;
            else if (!ShellyCommand(MWPLUG_ON)) power_new = 0;
        }
        else HoymilesCommand(HM_TURNOFF, sizeof(HM_TURNOFF));  // repeat command every cycle, just to make sure HM is really turned off
    }
    if (power_n < 0) {  // set new discharging power
        if (!HoymilesCommand(HM_POWER, sizeof(HM_POWER))) power_new = power_old;  // send HM power setting every cycle, even if power remains unchanged
        else {
            if (power_old == 0) {
                delay(RF24_MIN_DELAY);
                if (!HoymilesCommand(HM_TURNON, sizeof(HM_TURNON))) power_new = 0;
            }
            if (power_old > 0) {
                if (!ShellyCommand(MWPLUG_OFF)) power_new = power_old;  // takes longer than RF24_MIN_DELAY
                else if (!HoymilesCommand(HM_TURNON, sizeof(HM_TURNON))) power_new = 0;
            }
        }
    }
    ts_power = millis();
}

void FinishCycle() {

    bool skip_maintenance_tasks = false;

    // Calculate cycle time, reset cycle timestamp
    float seconds_cycle = (millis()-ts_cycle)/1000.0;
    ts_cycle = millis();

    // Update grid energy counters
    if (power_grid > 0) from_grid += power_grid*seconds_cycle/3600000;
    if (power_grid < 0) {
        pv_to_grid -= max(power_grid, -power_pv)*seconds_cycle/3600000;
        ess_to_grid -= (power_grid - max(power_grid, -power_pv))*seconds_cycle/3600000;
    }
    if (power_old > power_pv) grid_to_ess += (power_old-power_pv)*seconds_cycle/3600000;

    // Set Shelly 1PM eco mode, read PV power
    if (daytime) {
        if (pm_eco_mode) {
            skip_maintenance_tasks = true;
            if (ShellyCommand(PM_ECO_OFF))
                if (ShellyCommand(PM_REBOOT)) pm_eco_mode = false;
        }
        else ShellyCommand(PM_STATUS);  // reads power_pv from Shelly 1PM
    }
    else {
        if (!pm_eco_mode) {
            skip_maintenance_tasks = true;
            if (ShellyCommand(PM_ECO_ON))
                if (ShellyCommand(PM_REBOOT)) pm_eco_mode = true;
        }
        power_pv = 0;
    }

    // Calculate sunrise/sunset and daytime flag
    if (current_time == GET_ASTRO_TIME) GetAstroTimes();  // calculate during first cycle or at GET_ASTRO_TIME
    daytime = ((current_time >= sunrise) && (current_time < sunset));

    // MW is charging or HM is discharging or PV is producing: wake up up ESS (or keep it awake)
    if (power_new || power_pv) uvp_countdown = UVP_WAKEUP_RESET;

    // UVP sleep mode and no PV production: check for new EM power minimum
    if (!uvp_countdown && !power_pv && (power_grid < power_grid_min)) {
        power_grid_min = power_grid;
        minpower_time = unixtime;
    }

    // Print cycle data
    if (telnet) {

        // Clear terminal screen, print SW version
        cycle_msg = "\033[0H\033[0J";  // clear entire terminal screen
        cycle_msg += "BinSmart ESS ";
        cycle_msg += SW_VERSION;
        if (error_flag) {
            cycle_msg += "  ";
            cycle_msg += ERROR_SYMBOL;
        }
        if (WiFi.RSSI() >= GOOD_WIFI_RSSI) cycle_msg += GOOD_WIFI_SYMBOL;
        else cycle_msg += BAD_WIFI_SYMBOL;
        cycle_msg += "\r\n";

        // Time, cycle time, daytime
        unsigned long ct = unixtime + (millis()-ts_EM)/1000 + 1;
        sprintf(&ct_string[0],"%02d:%02d:%02d +",hour(ct),minute(ct),second(ct));
        cycle_msg += ct_string;
        cycle_msg += String(seconds_cycle,3);
        if (uvp_countdown) cycle_msg += NORMAL_OPS_SYMBOL;
        else cycle_msg += UVP_SLEEP_SYMBOL;
        cycle_msg += "\r\n\r\n";
        
        // Power flows
        sprintf(&power_string[0],"%4d",power_pv);
        cycle_msg += power_string;
        if (!daytime) cycle_msg += PV_SYMBOL[0];
        else {
            if (power_pv < PV_SUN_THRESHOLD) cycle_msg += PV_SYMBOL[1];
            else cycle_msg += PV_SYMBOL[min((power_pv-PV_SUN_THRESHOLD)*(PV_LEVELS-2)/(PV_MAX_POWER-PV_SUN_THRESHOLD)+2, PV_LEVELS-1)];
        }
        cycle_msg += FLOW_SYMBOL[2*(power_pv>0)];
        cycle_msg += PV_CABLE_SYMBOL;
        cycle_msg += FLOW_SYMBOL[2];  // power consumption is always > 0
        cycle_msg += CONS_SYMBOL;
        cycle_msg += power_pv + power_grid - power_old;
        cycle_msg += "\r\n             ";
        cycle_msg += HOUSE_SYMBOL;
        if (mw_power_limit < MW_MAX_POWER) cycle_msg += OVP_LIMIT_SYMBOL;
        cycle_msg += "\r\n";
        sprintf(&power_string[0],"%4d",power_grid);
        cycle_msg += power_string;
        cycle_msg += GRID_SYMBOL;
        cycle_msg += FLOW_SYMBOL[(power_grid<0)+2*(power_grid>0)];
        cycle_msg += GRID_CABLE_SYMBOL;
        if (!limited)
            cycle_msg += FLOW_SYMBOL[(power_old<0)+2*(power_old>0)];
        else {
            if (power_old <= hm_power_limit)
                cycle_msg += FLOW_SYMBOL[3*(hm_power_limit == HM_MAX_POWER) + 5*(hm_power_limit > HM_MAX_POWER) + 2*(!hm_power_limit)];
            if (power_old >= mw_power_limit)
                cycle_msg += FLOW_SYMBOL[4*(mw_power_limit == MW_MAX_POWER) + 6*(mw_power_limit < MW_MAX_POWER) + 2*(!mw_power_limit)];
        }
        cycle_msg += ESS_SYMBOL;
        if (vbat >= CELL_UVP*8) cycle_msg += BATT_LEVEL_SYMBOL[min((vbat-CELL_UVP*8)*(BATT_LEVELS-2)/((CELL_OVPR-CELL_UVP)*8)+1, BATT_LEVELS-1)];
        else cycle_msg += BATT_LEVEL_SYMBOL[0];
        cycle_msg += power_old;

        if (power_new != power_old) {
            if (rampdown) cycle_msg += RAMPDOWN_SYMBOL;
            else cycle_msg += " ";
            if (power_new < power_old) cycle_msg += DOWN_SYMBOL;
            else cycle_msg += UP_SYMBOL;
            cycle_msg += abs(power_new - power_old);
        }

        cycle_msg += filter_symbol;
        if (auto_recharge) cycle_msg += CHARGING_SYMBOL;
        cycle_msg += "\r\n";
        if (hm_power_limit > HM_MAX_POWER) cycle_msg += UVP_LIMIT_SYMBOL;
        cycle_msg += "\r\n";

        if (error_msg != "") {
            cycle_msg += ERROR_SYMBOL;
            cycle_msg += " ";
            cycle_msg += error_msg;
            cycle_msg += "\r\n\r\n";
        }

        if (cmd_msg != "") cycle_msg += cmd_msg;

        cycle_msg += "Enter command or press [CR]: ";
        if (telnet.available()) {
            received_command = telnet.read();
            if (telnet.available()) telnet.flush();  // discard any input after first character
        }
        telnet.print(cycle_msg);
    }
    else  {  // telnet connection lost: clear command response and command repeat flag
        cmd_msg = "";
        repeat_command = ' ';
    }

    if (skip_maintenance_tasks) return;  // Skip the following tasks if Shelly 1PM eco mode was changed (which took at least 4 seconds)

    // Carry out (max) one maintenance task per cycle (to avoid extension of PROCESSING_DELAY)
    if ((power_new > 0) && ((millis()-ts_MW_ON)/1000 >= MW_PLUG_TIMER-30)) {
        ShellyCommand(MWPLUG_ON);  // Charging: Keep alive Meanwell (30 secs before plug timer expires)
        return;
    }
    if (hm_power_limit && ((millis()-ts_HM_ON)/1000 >= HM_PLUG_TIMER-30)) {
        ShellyCommand(HMPLUG_ON);  // Not in UVP mode: Keep alive Hoymiles AC-side (30 secs before plug timer expires)
        return;
    }
    if ((millis()-ts_DDNS)/1000 >= DDNS_UPDATE_INTERVAL) {
        UpdateDDNS();   // Check if public IP address was changed, update DDNS server entry
        return;
    }
    if ((millis()-ts_EM_reset)/1000 >= EM_RESET_INTERVAL) {
        if (minute(unixtime) > 1) {  // prevents clash with internal memory reorg
            ShellyCommand(EM_RESET);   // Clear data held in internal 3EM memory (prevents http timeout due to mem reorg)
            return;
        }
    }
}

void CheckErrors() {

    if (error_msg == "") {
        errors_consecutive = 0;
        FlashLED(true);
        return;
    }

    // Increase error counters
    for (int i=0; i<ERROR_TYPES; i++)
        if (error_msg.indexOf(ERROR_TYPE[i]) > -1) {
            error_counter[i]++;
            errortime[i] = unixtime;
        }
    error_flag = true;
    errors_consecutive++;
    FlashLED(false);
    if (errors_consecutive < ERROR_LIMIT) return;

    // Error is persistent: Halt the system
    HoymilesCommand(HM_TURNOFF, sizeof(HM_TURNOFF));
    ShellyCommand(MWPLUG_OFF);
    ShellyCommand(HMPLUG_OFF);
    hm_power_limit = power_grid = power_new = power_old = 0;
    cycle_msg = "\033[0H\033[0J";  // clear entire terminal screen
    cycle_msg += "BinSmart ESS ";
    cycle_msg += SW_VERSION;
    cycle_msg += "\r\nSystem halted at ";
    sprintf(&datetime_string[0],"%02d/%02d/%04d %02d:%02d:%02d",day(unixtime),month(unixtime),year(unixtime),hour(unixtime),minute(unixtime),second(unixtime));
    cycle_msg += datetime_string;
    cycle_msg += "\r\nafter ";
    cycle_msg += ERROR_LIMIT;
    cycle_msg += " consecutive errors\r\nLast error:\r\n";
    cycle_msg += ERROR_SYMBOL;
    cycle_msg += " ";
    cycle_msg += error_msg;
    cycle_msg += "\r\n\r\nPress [r] to restart: ";
    while (true) {
        telnet.print(cycle_msg);
        if ((millis()-ts_DDNS)/1000 >= DDNS_UPDATE_INTERVAL) UpdateDDNS();  // keep updating DDNS in order to be reachable via Internet
        FlashLED(false);
        ts_cycle = millis();
        while (millis()-ts_cycle < UVP_SLEEP_DELAY) ReadCommand(true);
    }
}

bool ReadCommand(bool keyboard) {

    if (!telnet) {
        telnet = server.available();
        if (telnet) {
            // new telnet connection established: check for user command next cycle
            received_command = ' ';
            repeat_command = ' ';
            return true;
        }
        else return false;
    }

    char command;

    if (keyboard) {
        if (received_command != ' ') {
            command = received_command;  // use previously received command
            received_command = ' ';
        }
        else {
            // check if keyboard input available
            if (!telnet.available()) return false;
            command = telnet.read();
        }
        if (telnet.available()) telnet.flush();  // discard any input after first character
        repeat_command = ' ';
    }
    else {
        // repeat previous command (if command was to be repeated)
        if (repeat_command == ' ') return false;
        command = repeat_command;
        repeat_command = ' ';
    }

    unsigned long ts;
    String input;

    switch (command) {
        case 'm':
            telnet.print("\r\nEnter ESS power (negative for discharging): ");
            ts = millis();
            while ((millis()-ts)/1000 < READCOMMAND_TIMEOUT)
                if (telnet.available()) {
                    input = telnet.readString();
                    if ((input[0] == '0') || input.toInt()) {
                        manual_mode = true;
                        power_manual = input.toInt();
                        cmd_msg = "ESS power manually set to ";
                        cmd_msg += power_manual;
                        cmd_msg += " W\r\n\r\n";
                        return true;
                    }
                    break;
                }
            if (!manual_mode) cmd_msg = "Automatic mode remains activated\r\n\r\n";
            else {
                cmd_msg = "ESS power remains at ";
                cmd_msg += power_manual;
                cmd_msg += " W\r\n\r\n";
            }
            break;
        case 'a':
            manual_mode = false;
            cmd_msg = "Automatic mode activated\r\n\r\n";
            break;
        case 'g':
            telnet.print("\r\nEnter grid power target: ");
            ts = millis();
            while ((millis()-ts)/1000 < READCOMMAND_TIMEOUT)
                if (telnet.available()) {
                    input = telnet.readString();
                    if ((input[0] == '0') || input.toInt()) {
                        power_target = input.toInt();
                        cmd_msg = "Grid power target set to ";
                        cmd_msg += power_target;
                        cmd_msg += " W\r\n\r\n";
                        return true;
                    }
                    break;
                }
            cmd_msg = "Grid power target remains at ";
            cmd_msg += power_target;
            cmd_msg += " W\r\n\r\n";
            break;
        case 'b':
            repeat_command = 'b';
            cmd_msg = "Batt voltage   : ";
            cmd_msg += String(vbat/1000.0,2);
            cmd_msg += "\r\nCell voltages  : ";
            cmd_msg += String(vcell_min/1000.0,3);
            cmd_msg += " - ";
            cmd_msg += String(vcell_max/1000.0,3);
            cmd_msg += "\r\nVoltage diff   : ";
            cmd_msg += String((vcell_max-vcell_min)/1000.0,3);
            cmd_msg += "\r\nActive balancer: ";
            if ((vcell_max >= CELL_OVPR) && (vcell_max-vcell_min >= 3)) cmd_msg += "ON";
            else cmd_msg += "OFF";
            if (hm_power_limit > HM_MAX_POWER) {
                cmd_msg += "\r\nUVP power limit: ";
                cmd_msg += hm_power_limit;
                cmd_msg += " W";
            }
            if (mw_power_limit < MW_MAX_POWER) {
                cmd_msg += "\r\nOVP power limit: ";
                cmd_msg += mw_power_limit;
                cmd_msg += " W";
            }
            cmd_msg += "\r\n\r\n";
            break;
        case 'w':
            repeat_command = 'w';
            cmd_msg = "WiFi RSSI: ";
            cmd_msg += WiFi.RSSI();
            cmd_msg += " dBm\r\n\r\n";
            break;
        case 't':
            repeat_command = 't';
            cmd_msg = "Current time : ";
            ts = unixtime + (millis()- ts_EM + NO_COMMAND_TIMER + 100)/1000 + 1;
            sprintf(&datetime_string[0],"%02d/%02d/%04d %02d:%02d:%02d",day(ts),month(ts),year(ts),hour(ts),minute(ts),second(ts));
            cmd_msg += datetime_string;
            cmd_msg += "\r\nUTC offset   : +";
            cmd_msg += UTC_offset;
            if (UTC_offset == TIMEZONE+1) cmd_msg += " (DST)";
            cmd_msg += "\r\nESS started  : ";
            sprintf(&datetime_string[0],"%02d/%02d/%04d %02d:%02d:%02d",day(starttime),month(starttime),year(starttime),hour(starttime),minute(starttime),second(starttime));
            cmd_msg += datetime_string;
            cmd_msg += "\r\nESS uptime   : ";
            sprintf(&datetime_string[0],"%03dd %02dh %02dm %02ds   ",elapsedDays(ts-starttime),numberOfHours(ts-starttime),numberOfMinutes(ts-starttime),numberOfSeconds(ts-starttime));
            cmd_msg += datetime_string;
            cmd_msg += "\r\nSunrise today: ";
            cmd_msg += sunrise;
            cmd_msg += "\r\nSunset today : ";
            cmd_msg += sunset;
            cmd_msg += "\r\n\r\n";
            break;
        case 'd':
            cmd_msg = "DDNS address of ESS: ";
            cmd_msg += DDNS_address;
            cmd_msg += "\r\nLast DDNS update: ";
            sprintf(&datetime_string[0],"%02d/%02d/%04d %02d:%02d\r\n",day(DDNS_time),month(DDNS_time),year(DDNS_time),hour(DDNS_time),minute(DDNS_time));
            cmd_msg += datetime_string;
            cmd_msg += "\r\n";
            break;
        case 'l':
            cmd_msg = "Lowest consumption since ";
            sprintf(&datetime_string[0],"%02d/%02d/%04d:      \r\n",day(starttime),month(starttime),year(starttime));
            cmd_msg += datetime_string;
            if (!minpower_time) cmd_msg += "Not yet measured\r\n\r\n";
            else {
                cmd_msg += power_grid_min;
                cmd_msg += " W (measured ";
                sprintf(&datetime_string[0],"%02d/%02d/%04d %02d:%02d)\r\n",day(minpower_time),month(minpower_time),year(minpower_time),hour(minpower_time),minute(minpower_time));
                cmd_msg += datetime_string;
                cmd_msg += "\r\n";
            }
            break;
        case 'n':
            telnet.print("\r\nReading Shelly energy counters ...");
            ShellyCommand(HMPLUG_STATUS);  // reads Hoymiles energy counter from HM plug
            ShellyCommand(MWPLUG_STATUS);  // reads Meanwell energy counter from MW plug
            ShellyCommand(PM_STATUS);  // reads PV energy counter from Shelly 1PM
            cmd_msg = "Energy [kWh] since ";
            sprintf(&datetime_string[0],"%02d/%02d/%04d:      \r\n",day(resettime_energy),month(resettime_energy),year(resettime_energy));
            cmd_msg += datetime_string;
            cmd_msg += "From PV  :";
            if (!from_pv) cmd_msg += " 0\r\n";
            else {
                cmd_msg += "\r\n";
                sprintf(&energy_string[0],"%7.3f",from_pv);
                cmd_msg += energy_string;
                cmd_msg += PV_SYMBOL[2];
                cmd_msg += FLOW_SYMBOL[2];
                cmd_msg += PV_CABLE_SYMBOL;
                cmd_msg += FLOW_SYMBOL[2];
                cmd_msg += CONS_SYMBOL_SHORT;
                sprintf(&energy_string[0],"%5.1f %%",(from_pv-(to_ess-grid_to_ess)-pv_to_grid)/from_pv*100);
                cmd_msg += energy_string;
                cmd_msg += "\r\n                ";
                cmd_msg += HOUSE_SYMBOL;
                cmd_msg += "\r\n";
                sprintf(&energy_string[0],"%5.1f %%",pv_to_grid/from_pv*100);
                cmd_msg += energy_string;
                cmd_msg += GRID_SYMBOL;
                cmd_msg += FLOW_SYMBOL[1];
                cmd_msg += GRID_CABLE_SYMBOL;
                cmd_msg += FLOW_SYMBOL[2];
                cmd_msg += ESS_SYMBOL;
                cmd_msg += " ";
                sprintf(&energy_string[0],"%5.1f %%",(to_ess-grid_to_ess)/from_pv*100);
                cmd_msg += energy_string;
                cmd_msg += "\r\n";
            }
            cmd_msg += "From Grid:";
            if (!from_grid) cmd_msg += " 0\r\n";
            else {
                cmd_msg += "\r\n       ";
                cmd_msg += PV_SYMBOL[2];
                cmd_msg += FLOW_SYMBOL[7];
                cmd_msg += PV_CABLE_SYMBOL;
                cmd_msg += FLOW_SYMBOL[2];
                cmd_msg += CONS_SYMBOL_SHORT;
                sprintf(&energy_string[0],"%5.1f %%",(from_grid-grid_to_ess)/from_grid*100);
                cmd_msg += energy_string;
                cmd_msg += "\r\n                ";
                cmd_msg += HOUSE_SYMBOL;
                cmd_msg += "\r\n";
                sprintf(&energy_string[0],"%7.3f",from_grid);
                cmd_msg += energy_string;
                cmd_msg += GRID_SYMBOL;
                cmd_msg += FLOW_SYMBOL[2];
                cmd_msg += GRID_CABLE_SYMBOL;
                cmd_msg += FLOW_SYMBOL[2];
                cmd_msg += ESS_SYMBOL;
                cmd_msg += " ";
                sprintf(&energy_string[0],"%5.1f %%",grid_to_ess/from_grid*100);
                cmd_msg += energy_string;
                cmd_msg += "\r\n";
            }
            cmd_msg += "From ESS :";
            if (!from_ess) cmd_msg += " 0\r\n";
            else {
                cmd_msg += "\r\n       ";
                cmd_msg += PV_SYMBOL[2];
                cmd_msg += FLOW_SYMBOL[7];
                cmd_msg += PV_CABLE_SYMBOL;
                cmd_msg += FLOW_SYMBOL[2];
                cmd_msg += CONS_SYMBOL_SHORT;
                sprintf(&energy_string[0],"%5.1f %%",(from_ess-ess_to_grid)/from_ess*100);
                cmd_msg += energy_string;
                cmd_msg += "\r\n                ";
                cmd_msg += HOUSE_SYMBOL;
                cmd_msg += "\r\n";
                sprintf(&energy_string[0],"%5.1f %%",ess_to_grid/from_ess*100);
                cmd_msg += energy_string;
                cmd_msg += GRID_SYMBOL;
                cmd_msg += FLOW_SYMBOL[1];
                cmd_msg += GRID_CABLE_SYMBOL;
                cmd_msg += FLOW_SYMBOL[1];
                cmd_msg += ESS_SYMBOL;
                cmd_msg += " ";
                sprintf(&energy_string[0],"%7.3f",from_ess);
                cmd_msg += energy_string;
                cmd_msg += "\r\n\r\n";
            }
            break;
        case 'e':
            cmd_msg = "Errors since ";
            sprintf(&datetime_string[0],"%02d/%02d/%04d:      \r\n",day(resettime_errors),month(resettime_errors),year(resettime_errors));
            cmd_msg += datetime_string;
            for (int i=0; i<ERROR_TYPES; i++) {
                cmd_msg += ERROR_TYPE[i];
                cmd_msg += "\t";
                cmd_msg += error_counter[i];
                if (error_counter[i]) {
                    cmd_msg += " (last error: ";
                    sprintf(&datetime_string[0],"%02d/%02d/%04d %02d:%02d)  ",day(errortime[i]),month(errortime[i]),year(errortime[i]),hour(errortime[i]),minute(errortime[i]));
                    cmd_msg += datetime_string;
                }
                cmd_msg += "\r\n";
            }
            cmd_msg += "\r\n";
            error_flag = false;
            break;
        case 'z':
            telnet.print("\r\nPress [e] or [n] to reset error or energy stats: ");
            ts = millis();
            while ((millis()-ts)/1000 < READCOMMAND_TIMEOUT)
                if (telnet.available()) {
                    command = telnet.read();
                    if (command == 'e') {
                        for (int i=0; i<ERROR_TYPES; i++) error_counter[i] = errortime[i] = 0;
                        error_flag = false;
                        resettime_errors = unixtime;
                        cmd_msg = "Error stats reset to zero\r\n\r\n";
                        if (telnet.available()) telnet.flush();
                        return true;
                    }
                    if (command == 'n') {
                        from_grid = grid_to_ess = pv_to_grid = ess_to_grid = 0;
                        ShellyCommand(HMPLUG_RESET);  // resets HM plug energy counter
                        ShellyCommand(MWPLUG_RESET);  // resets MW plug energy counter
                        ShellyCommand(PM_RESET);  // resets Shelly 1PM energy counter
                        resettime_energy = unixtime;
                        ts_cycle = millis();
                        cmd_msg = "Energy stats reset to zero\r\n\r\n";
                        if (telnet.available()) telnet.flush();
                        return true;
                    }
                    break;
                }
            cmd_msg = "No stats reset\r\n\r\n";
            break;
        case 'c':
            cmd_msg = "";
            break;
        case 'r':
            telnet.print("\r\nPress [y] to confirm system reboot: ");
            ts = millis();
            while ((millis()-ts)/1000 < READCOMMAND_TIMEOUT)
                if (telnet.available()) {
                    if (telnet.read() == 'y') {
                        telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
                        delay(3000);
                        telnet.stop();
                        delay(500);
                        ESP.restart();
                    }
                    break;
                }
            cmd_msg = "System not rebooted\r\n\r\n";
            break;
        default:
            cmd_msg = "Command options:\r\n";
            cmd_msg += "[m] - Manual power mode\r\n";
            cmd_msg += "[a] - Automatic power mode\r\n";
            cmd_msg += "[g] - Grid power target\r\n";
            cmd_msg += "[b] - Battery info\r\n";
            cmd_msg += "[w] - WiFi RSSI\r\n";
            cmd_msg += "[t] - Time, uptime, astro times\r\n";
            cmd_msg += "[d] - DDNS address of ESS\r\n";
            cmd_msg += "[l] - Lowest power consumption\r\n";
            cmd_msg += "[n] - Energy stats\r\n";
            cmd_msg += "[e] - Error stats\r\n";
            cmd_msg += "[z] - Reset stats to zero\r\n";
            cmd_msg += "[c] - Clear command response\r\n";
            cmd_msg += "[r] - Reboot system\r\n\r\n";
            break;
    }
    if (telnet.available()) telnet.flush();
    return true;
}

void FlashLED(bool no_error) {

    if (!no_error) { digitalWrite(LED_PIN, HIGH); delay(1000); digitalWrite(LED_PIN, LOW); }
    else {
        int flashes = 1+(power_new>0)+2*(power_new<0);
        for (int i=0; i<flashes; i++) {
            if (i) delay(200);
            digitalWrite(LED_PIN, HIGH); delay(20); digitalWrite(LED_PIN, LOW);
        }
    }
}

void SetMWPower() {

    int duty_cycle = max(int((0.959-power_new*76.646/vbat)*PWM_MAX_DUTY_CYCLE+0.5), 1);  // Higher duty cycle means lower charging power
    if (duty_cycle > PWM_MAX_LINEAR) duty_cycle = min((duty_cycle*3)-(PWM_MAX_LINEAR*2), PWM_MAX_DUTY_CYCLE);
    ledcWrite(PWM_CHANNEL, duty_cycle);
}

bool HoymilesCommand(byte command[], int size) {

    if ((command[10] == 0x0B) && (power_new != power_old)) {
        // Hoymiles power limit command: Update command string with calculated power and CRCs
        int power = -power_new;
        uint16_t limit = 10*power;
        // Compensate non-linearity of Hoymiles power control in lower and upper power regions
        if (power < 90) limit = round(10*(0.0001308*power*power*power - 0.01891*power*power + 1.71*power - 0.9));
        if (power > 180) limit = round(10*(-0.00186*power*power + 1.288*power + 9.2));
        command[12] = (limit >> 8) & 0xff;
        command[13] = limit & 0xff;
        crc16.restart();
        crc16.add(&command[10], 6);
        uint16_t c = crc16.getCRC();
        command[16] = (c>>8) & 0xFF;
        command[17] = c & 0xFF;
        crc8.restart();
        crc8.add (command, 18);
        command[18] = crc8.getCRC();
    }

    // Send command to Hoymiles via RF24
    if (radio.writeFast(command, size))
        if (radio.txStandBy(RF24_TX_TIMEOUT, true)) return true;

    // tx failed
    error_msg = "Hoymiles RF24 command 0x0";
    error_msg += String(command[10],HEX);
    error_msg += " failed";
    return false;
}

bool UpdateDDNS() {

    if (WiFi.status() != WL_CONNECTED) {
        error_msg = "WIFI";
        return false;
    }
    http.begin(PUBLIC_IP);
    if (http.GET() == HTTP_CODE_OK) {
        http_response = http.getString();
        http.end();
        if (http_response != DDNS_address) {
            http.begin(DDNS_UPDATE + http_response);
            if (http.GET() == HTTP_CODE_OK) {
                http.end();
                DDNS_address = http_response;
                DDNS_time = unixtime;
                ts_DDNS = millis();
                return true;
            }
            http.end();
            error_msg = "Could not update DDNS IP address";
            return false;
        }
        ts_DDNS = millis();
        return true;
    }
    http.end();
    error_msg = "Could not obtain DDNS IP address";
    return false;
}

void GetAstroTimes() {

    // Calculate sunrise/sunset times for current day
    int mins_after_midnight = ESS_location.sunrise(year(unixtime), month(unixtime), day(unixtime), (UTC_offset == TIMEZONE+1));
    sprintf(&sunrise[0], "%02d:%02d", mins_after_midnight/60, mins_after_midnight%60);
    mins_after_midnight = ESS_location.sunset(year(unixtime), month(unixtime), day(unixtime), (UTC_offset == TIMEZONE+1));
    sprintf(&sunset[0], "%02d:%02d", mins_after_midnight/60, mins_after_midnight%60);
}

