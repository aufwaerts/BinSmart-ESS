#define SW_VERSION "v2.63"

#include <WiFi.h>  // standard Arduino/ESP32
#include <HTTPClient.h>  // standard Arduino/ESP32
#include <ESPAsyncWebServer.h>  // https://github.com/lacamera/ESPAsyncWebServer
#include <AsyncElegantOTA.h>  // superseded by https://github.com/ayushsharma82/ElegantOTA (I prefer the old version)
#include <RF24.h>  // https://nrf24.github.io/RF24/
#include <CRC8.h>  // https://github.com/RobTillaart/CRC
#include <CRC16.h>  // https://github.com/RobTillaart/CRC
#include <TimeLib.h>  // https://playground.arduino.cc/Code/Time/
#include <Dusk2Dawn.h>  // https://github.com/dmkishi/Dusk2Dawn
#include <NimBLEDevice.h>  // https://github.com/h2zero/NimBLE-Arduino
#include "BinSmart_cfg.h"
#include "BinSmart_var.h"

void setup() {

    //Reduce CPU clock to 80 MHz (enough for this application)
    setCpuFrequencyMhz(80);

    // Assign LED Pin, turn LED on to signal start of setup()
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    delay(500);

    // Init WiFi
    WiFi.config(ESP32_ADDR, ROUTER_ADDR, SUBNET, DNS_SERVER1, DNS_SERVER2);
    WiFi.setTxPower(WIFI_POWER_5dBm);
    WiFi.setSleep(WIFI_PS_MAX_MODEM);
    WiFi.begin(WIFI_SSID, WIFI_PWD);
    while (WiFi.status() != WL_CONNECTED);   // if WiFi unavailable or wrong SSID/PWD, system stops here and LED remains on
    digitalWrite(LED_PIN, LOW);

    // Start OTA software update service
    AsyncElegantOTA.begin(&OTAserver);
    OTAserver.begin();

    // Start telnet server and wait for terminal to connect
    server.begin();
    while (!telnet) telnet = server.available();

    // Print startup message
    telnet.printf("%sBinSmart ESS %s\r\n\nWiFi connected to %s   RSSI: %d   TxPower: %d\r\n", CLEAR_SCREEN, SW_VERSION, WIFI_SSID, WiFi.RSSI(), WiFi.getTxPower());
    delay(2000);

    // Init PWM generator (for adjusting Meanwell power)
    if (!ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION)) strcpy(error_msg, "PWM generator init failed");
    CheckErrors();
    ledcAttachPin(PWM_OUTPUT_PIN, PWM_CHANNEL);
    SetMWPower(MW_MIN_POWER);
    telnet.println("PWM generator initialized");
    
    // Init RS485 communication with BMS, read OVP/UVP/balancer settings and cell voltages
    Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    BMSCommand(READ_SETTINGS);
    CheckErrors();
    telnet.println("RS485 communication with JKBMS OK");

    // Init BLE communication with BMS
    NimBLEDevice::init("");
    pClient = NimBLEDevice::createClient(serverAddress);
    if (!pClient) strcpy(error_msg, "BLE init failed");
    CheckErrors();
    pClient->setConnectionParams(12, 12, 0, 12);
    pClient->setConnectTimeout(BLE_TIMEOUT);
    telnet.println("BLE communication with JKBMS initialized");

    // Init RF24 radio communication with Hoymiles
    if (!radio.begin()) strcpy(error_msg, "RF24 radio init failed");
    CheckErrors();
    radio.setPALevel(RF24_PALEVEL);
    radio.setChannel(RF24_CHANNEL);
    radio.setDataRate(RF24_250KBPS);
    radio.setCRCLength(RF24_CRC_16);
    radio.setAddressWidth(sizeof(HM_RADIO_ID));
    radio.enableDynamicPayloads();
    radio.openWritingPipe(HM_RADIO_ID);
    radio.stopListening();  // radio will only be used in TX mode
    crc16.setPolynome((uint16_t)0x18005);
    crc16.setStartXOR(0xFFFF);
    crc16.setEndXOR(0x0000);
    crc16.setReverseIn(true);
    crc16.setReverseOut(true);
    crc8.setPolynome(0x01);
    crc8.setStartXOR(0);
    crc8.setEndXOR(0);
    telnet.println("RF24 radio initialized");

    // Turn off Hoymiles DC side
    HoymilesCommand(HM_OFF);
    CheckErrors();
    telnet.println("RF24 communication with Hoymiles OK");

    // Shelly 1PM: Read eco mode setting
    ShellyCommand(PM1_ADDR, PM_CONFIG);
    CheckErrors();
    telnet.println("Shelly 1PM found");

    // Shelly 2PM: Read eco mode setting, turn off Meanwell, turn on Hoymiles AC side
    if (ShellyCommand(PM2_ADDR, PM_CONFIG))
        if (ShellyCommand(PM2_ADDR, MW_OFF)) ShellyCommand(PM2_ADDR, HM_AC_ON);
    CheckErrors();
    telnet.println("Shelly 2PM found");
    
    // Shelly 3EM: Read geo coordinates, grid power and current time
    if (ShellyCommand(EM_ADDR, EM_SETTINGS)) ShellyCommand(EM_ADDR, EM_STATUS);
    CheckErrors();
    CalcAstroTimes();
    telnet.println("Shelly 3EM found");

    // Update DDNS address
    if (!UpdateDDNS()) strcpy(error_msg, "DDNS update failed");
    CheckErrors();
    telnet.println("DDNS address updated");
    
    // No erros during setup: set timestamps, start polling cycle
    telnet.print("\nNo errors during setup, start polling cycle ...");
    starttime = resettime_errors = resettime_energy = unixtime;
    delay(PROCESSING_DELAY*1000);
    ts_power = millis();
}

void loop() {

    ShellyCommand(PM1_ADDR, PM_STATUS);  // Read PV power from Shelly 1PM
    BMSCommand(READ_VOLTAGES);  // Read cell voltages from BMS, set charging/discharging power limits
    ShellyCommand(PM2_ADDR, PM_STATUS); // Read ESS power from Shelly 2PM
    ShellyCommand(EM_ADDR, EM_STATUS);  // Read time and grid power from Shelly 3EM
    BMSCommand(READ_CURRENT);  // Read DC charging/discharging current and power from BMS, determine batt charge level
    SetNewPower();  // Calculate and apply new charging/discharging power
    MaintenanceTasks();  // Update energy counters, carry out routine and maintenance tasks
    CheckErrors();  // Check for comms errors, show error status by flashing LED, halt system if an error is persistent
    FinishCycle();  // Print cycle info and user command response, wait for power changes to take effect, read user command
}

bool ShellyCommand(IPAddress shelly_addr, const char shelly_command[]) {

    // Prepare Shelly http command
    strcpy(http_command, "http://");
    strcat(http_command, shelly_addr.toString().c_str());
    strcat(http_command, shelly_command);

    // read PV or ESS power: check if power_pv or power_ess can be set without Shelly http command; if not: determine PM channel
    if (!strcmp(shelly_command, PM_STATUS)) {
        if ((shelly_addr == PM1_ADDR) && !daytime) {
            power_pv = 0;  // PV power at nighttime is zero
            return true;
        }
        if (shelly_addr == PM2_ADDR) {
            power_ess = power_new;  // assumption: ESS power reading equals power setting
            if (!power_new) return true;  // Hoymiles and Meanwell turned off: ESS power is zero
            if (power_new < 0) http_command[strlen(http_command)-1] = '1';  // Discharging: read ch1 (Hoymiles) instead of ch0 (Meanwell) 
        }
    }

    if (WiFi.status() != WL_CONNECTED) {
        strcpy(error_msg, "WIFI");
        return false;
    }

    // Send http command to Shelly
    http.setTimeout(HTTP_SHELLY_TIMEOUT*1000);
    http.begin(http_command);
    if (http.GET() == HTTP_CODE_OK) {
        WiFiClient* stream = http.getStreamPtr();
        if (!strcmp(shelly_command, EM_SETTINGS)) {
            // read geo coordinates and timezone from Shelly 3EM
            stream->find("lat");
            latitude = stream->parseFloat();
            longitude = stream->parseFloat();
            timezone = round(longitude/15);
        }
        if (!strcmp(shelly_command, EM_STATUS)) {
            // read time and grid power from Shelly 3EM
            for (int i=0; i<140; i++) stream->read();  // fast forward to "time"
            stream->find("time");
            min_of_day = stream->parseInt()*60 + stream->parseInt();  // read local time (minutes after midnight)
            daytime = ((min_of_day > sunrise) && (min_of_day < sunset));
            long unixtime_utc = stream->parseInt();  // read epoch time (UTC)
            int utc_offset = (min_of_day/60+24-hour(unixtime_utc))%24;  // UTC offset = timezone + dst
            unixtime = unixtime_utc + utc_offset*3600;  // unixtime matches local time
            dst = (utc_offset != timezone);
            for (int i=0; i<550; i++) stream->read();  // fast forward to "total_power"
            stream->find("total_power");
            power_grid = stream->parseFloat();  // read grid power from http response
        }
        if (!strcmp(shelly_command, PM_STATUS)) {
            // read power from Shelly 1PM or 2PM
            stream->find("apower");
            if (shelly_addr == PM1_ADDR) power_pv = stream->parseFloat();
            if (shelly_addr == PM2_ADDR) power_ess = stream->parseFloat();
        }
        if (!strcmp(shelly_command, MW_ON)) {
            // Meanwell turned on
            ts_MW = millis();
            if (!mw_on) mw_counter++;
            mw_on = true;
        }
        if (!strcmp(shelly_command, MW_OFF)) {
            // Meanwell turned off
            ts_MW = millis();
            if (mw_on) mw_counter++;
            mw_on = false;
        }
        if (!strcmp(shelly_command, PM_CONFIG)) {
            bool eco_mode = stream->findUntil("eco_mode\":true", "eco_mode\":false");
            if (shelly_addr == PM1_ADDR) pm1_eco_mode = eco_mode;
            else pm2_eco_mode = eco_mode;
        }
        http.end();
        return true;
    }
    // Shelly command failed
    http.end();
    if (shelly_addr == EM_ADDR) sprintf(error_msg, "3EM cmd %s failed", http_command);
    if (shelly_addr == PM1_ADDR) sprintf(error_msg, "1PM cmd %s failed", http_command);
    if (shelly_addr == PM2_ADDR) sprintf(error_msg, "2PM cmd %s failed", http_command);
    return false;
}

bool BMSCommand(const byte BMS_command[]) {

    while (millis()-ts_BMS < 50);  // minimum delay after previous BMS response

    if (BMS_command[0] == BLE_1) {

        // Send BMS command via BLE
        if (pClient->connect()) {
            NimBLERemoteService* pService = pClient->getService("FFE0");
            NimBLERemoteCharacteristic* pChar = pService->getCharacteristic("FFE1");
            if (pChar->writeValue(GET_INFO, BLE_COMMAND_LEN, false)) {
                delay(500);
                if (pChar->writeValue(GET_DATA, BLE_COMMAND_LEN, false)) {
                    delay(500);
                    if (pChar->writeValue(BMS_command, BLE_COMMAND_LEN, false)) {
                        pClient->disconnect();
                        ts_BMS = millis();
                        return true;
                    }
                }
            }
            pClient->disconnect();
        }
        ts_BMS = millis();
        sprintf(error_msg, "BMS BLE command 0x%02X 0x%02X failed", BMS_command[BLE_COMMAND_POS], BMS_command[BLE_SETTING_POS]);
        return false;
    }

    // Send BMS command via RS485
    Serial2.write(BMS_command, BMS_command[RS485_LEN_POS]+2);
    Serial2.flush();

    bms_resp[0] = 0;  // will result in failed validity check if no response
    int len = 0, sum = 0;

    // Wait for BMS response (max waiting time: 100 ms)
    for (int i=0; i<100; i++) {
        if (Serial2.available()) break;
        delay(1);
    }
    if (BMS_command[RS485_COMMAND_POS] == READ_ALL) delay(20);  // allow some more time to fetch long response
    
    // fill response buffer with BMS response
    while (Serial2.available()) {
        bms_resp[len] = Serial2.read();
        sum += bms_resp[len];
        len++;
    }
    ts_BMS = millis();

    // check validity of BMS response
    if ((bms_resp[0] == RS485_1) && (bms_resp[1] == RS485_2)) {  // start of response frame
         if ((bms_resp[2]<<8|bms_resp[3]) == len-2) {  // response length
             if ((bms_resp[len-2]<<8|bms_resp[len-1]) == sum-bms_resp[len-2]-bms_resp[len-1]) {  // response checksum

                // process BMS response
                if (bms_resp[RS485_COMMAND_POS] == READ_DATA) {
                    if (bms_resp[DATA_ID_POS] == VCELLS_ID) {
                        // read voltages of battery cells, determine min and max voltages
                        vbat = vcell_max = 0; vcell_min = 4000;
                        for (int vcells_pos = DATA_ID_POS+3; vcells_pos <= DATA_ID_POS+bms_resp[DATA_ID_POS+1]; vcells_pos+=3) {
                            int vcell = bms_resp[vcells_pos]<<8|bms_resp[vcells_pos+1];
                            if (vcell < vcell_min) vcell_min = vcell;
                            if (vcell > vcell_max) vcell_max = vcell;
                            vbat += vcell;
                        }
                        // Save old charging power limit, update limit (depending on vcell_max)
                        mw_limit_old = mw_limit;
                        if ((vcell_max <= ESS_OVPR) || (mw_limit >= MW_MAX_POWER)) mw_limit = round(MW_POWER_LIMIT_FORMULA);  // exit OVP mode (update MW power limit)
                        if (vcell_max >= ESS_OVP) {  // OVP cell voltage reached
                            if ((mw_limit <= MW_MIN_POWER) || !power_new) mw_limit = 0;  // enter (or remain in) OVP mode
                            else mw_limit = max(int(round(power_new*POWER_LIMIT_RAMPDOWN)), MW_MIN_POWER);  // ramp down charging power limit softly
                        }
                        // Save old discharging power limit, update limit (depending on vcell_min)
                        hm_limit_old = hm_limit;
                        if (vcell_min >= ESS_UVPR) hm_limit = HM_MAX_POWER;  // exit UVP mode (reset HM power limit)
                        if (vcell_min <= ESS_UVP) {  // UVP cell voltage reached
                            if ((hm_limit >= HM_MIN_POWER) || !power_new) hm_limit = 0;  // enter (or remain in) UVP mode
                            else hm_limit = min(int(round(power_new*POWER_LIMIT_RAMPDOWN)), HM_MIN_POWER);  // ramp up discharging power limit softly
                        }
                        return true;
                    }
                    
                    if (bms_resp[DATA_ID_POS] == CURRENT_ID) {
                        // Read charging/discharging DC current, calculate DC power
                        cbat = (bms_resp[DATA_ID_POS+1]&0x7F)<<8|bms_resp[DATA_ID_POS+2];
                        if (!(bms_resp[DATA_ID_POS+1]&0x80)) cbat = -cbat; // charging current is positive, discharging current is negative
                        pbat = (vbat/1000.0)*(cbat/100.0);
                        // Calculate DC/AC (or AC/DC) efficiency
                        power_eff = 0;
                        if ((pbat > 0) && (power_ess > 0)) power_eff = pbat/power_ess*100;  // charging: power_ess (AC) greater than pbat (DC)
                        if ((pbat < 0) && (power_ess < 0)) power_eff = -power_ess/pbat*100;  // discharging: pbat (DC) greater than power_ess (AC)
                        // Update batt charge level
                        int vbat_idle = vbat-round(cbat/16.0);  // voltage at cbat=0
                        if (vbat_idle > BAT_EMPTY) bat_level = min(((vbat_idle-BAT_EMPTY)*(BAT_LEVELS-2))/(BAT_FULL-BAT_EMPTY) + 1, BAT_LEVELS-1);
                        else bat_level = 0;
                        return true;               
                    }
                }

                if (bms_resp[RS485_COMMAND_POS] == READ_ALL) {
                    // check for BMS warnings
                    if (bms_resp[WARNINGS_POS] || bms_resp[WARNINGS_POS+1]) {
                        sprintf(error_msg, "BMS has warnings 0x%02X 0x%02X", bms_resp[WARNINGS_POS], bms_resp[WARNINGS_POS+1]);
                        return false;
                    }
                    // check OVP and UVP setting
                    if ((bms_resp[OVP_POS]<<8|bms_resp[OVP_POS+1]) - ESS_OVP < ESS_BMS_OVP_DIFF) {
                        sprintf(error_msg, "ESS_OVP less than %d mV below BMS_OVP (%d mV)", ESS_BMS_OVP_DIFF, bms_resp[OVP_POS]<<8|bms_resp[OVP_POS+1]);
                        return false;
                    }
                    bms_uvp = bms_resp[UVP_POS]<<8|bms_resp[UVP_POS+1];
                    if (ESS_UVP - bms_uvp < ESS_BMS_UVP_DIFF) {
                        sprintf(error_msg, "ESS_UVP less than %d mV above BMS_UVP (%d mV)", ESS_BMS_UVP_DIFF, bms_uvp);
                        return false;
                    }
                    // read balancer settings
                    bms_balancer_start = bms_resp[BAL_ST_POS]<<8|bms_resp[BAL_ST_POS+1];
                    if (ESS_UVP < bms_balancer_start) {
                        sprintf(error_msg, "BMS Balancer Start Voltage higher than ESS_UVP (%d mV)", ESS_UVP);
                        return false;
                    }
                    bms_balancer_trigger = bms_resp[BAL_TR_POS]<<8|bms_resp[BAL_TR_POS+1];
                    bms_bal_on = bms_resp[BAL_SW_POS];
                    return true;

                }
            }
        } 
    }
    sprintf(error_msg, "BMS RS485 command 0x%02X 0x%02X failed", BMS_command[RS485_COMMAND_POS], BMS_command[DATA_ID_POS]);
    return false;
}

void SetNewPower() {

    // Save power settings from previous cycle as baseline for new power calculation
    power_old = power_new;

    // Calculate new power setting
    int target_deviation = round(power_grid - power_target);
    int positive_tolerance = POWER_TARGET_TOLERANCE;
    if ((power_old < 0) && (power_old > HM_LOW_POWER_THRESHOLD)) positive_tolerance = HM_LOW_POWER_TOLERANCE;  // HM operating with low power: higher positive tolerance
    if ((target_deviation < -POWER_TARGET_TOLERANCE) || (target_deviation > positive_tolerance)) power_new = power_old - target_deviation;

    // Filter out power spikes and ramp down ESS power (reduces power loss to grid when consumer is quickly switched on and off)
    if (max(power_new, hm_limit) - power_old < POWER_RAMPDOWN_RATE) {
        if (power_old > 0) filter_cycles = 0;  // charging: start rampdown immediately
        else filter_cycles = max(filter_cycles-1, 0);  // countdown filter cycles
        if (filter_cycles) power_new = power_old;  // filter out power spikes
        else {
            power_new = power_old + POWER_RAMPDOWN_RATE;  // ramp down ESS power after filtering out power spikes
            if ((power_old > 0) && (power_new < 0)) power_new = 0;  // don't skip zero power during rampdown
            if ((power_new == POWER_RAMPDOWN_RATE) && (HM_MIN_POWER < POWER_RAMPDOWN_RATE)) power_new = HM_MIN_POWER;  // make sure rampdown gets past HM_MIN_POWER
            if ((power_new >= 0) && (power_new < MW_MIN_POWER)) filter_cycles = POWER_FILTER_CYCLES;  // reset filter cycle countdown before discharging
        }
    }
    else filter_cycles = POWER_FILTER_CYCLES;  // reset filter cycle countdown

    if (manual_mode) {
        power_new = power_manual;  // Manual power setting overrides calculation
        filter_cycles = POWER_FILTER_CYCLES;
    }
    
    // Make sure charging/discharging power limits are not exceeded
    if (power_new > mw_limit) power_new = mw_limit;
    if (power_new < hm_limit) power_new = hm_limit;

    // Turn ESS off if power is between MW and HM operating ranges
    if ((power_new > HM_MIN_POWER) && (power_new < MW_MIN_POWER)) power_new = 0;

    // Set automatic battery recharge power (to prevent BMS turnoff)
    if (auto_recharge) {
        manual_mode = false;
        filter_cycles = POWER_FILTER_CYCLES;
        if (vcell_min >= ESS_UVP) {
            power_new = 0;
            auto_recharge = false;
            vcell_min = ESS_UVP;  // keeps BMS balancer turned on
        }
        else power_new = MW_MAX_POWER/2;
    }
    if ((vcell_min <= (bms_uvp + ESS_UVP)/2)) auto_recharge = true;

    // Apply new power setting
    if (power_new == 0) {  // turn charging or discharging off
        if (power_old < 0)
            if (!HoymilesCommand(HM_OFF)) power_new = power_old;
        if (power_old > 0) {
            SetMWPower(MW_MIN_POWER);  // extends Shelly relay lifetime
            if (!ShellyCommand(PM2_ADDR, MW_OFF)) power_new = MW_MIN_POWER;
        }
    }
    if (power_new > 0) {  // set new charging power, (if necessary) turn discharging off, turn charging on
        if (power_old > 0) SetMWPower(power_new);  // re-calculate even if power setting remains unchanged (vbat might have changed)
        if (power_old == 0) {
            SetMWPower(power_new);
            if (!ShellyCommand(PM2_ADDR, MW_ON)) power_new = 0;
        }
        if (power_old < 0) {
            if (!HoymilesCommand(HM_OFF)) power_new = power_old;
            else {
                SetMWPower(power_new);
                if (!ShellyCommand(PM2_ADDR, MW_ON)) power_new = 0;
            }
        }
    }
    if (power_new < 0) {  // set new discharging power, (if necessary) turn charging off, turn discharging on
        if (power_old < 0) {
            if (power_new != power_old)
                if (!HoymilesCommand(power_new)) power_new = power_old;
        }
        if (power_old == 0) {
            if (!HoymilesCommand(power_new)) power_new = 0;
            else if (!HoymilesCommand(HM_ON)) power_new = 0;
        }
        if (power_old > 0) {
            SetMWPower(MW_MIN_POWER);  // extends Shelly relay lifetime
            if (!ShellyCommand(PM2_ADDR, MW_OFF)) power_new = MW_MIN_POWER;
            else {
                if (!HoymilesCommand(power_new)) power_new = 0;
                else if (!HoymilesCommand(HM_ON)) power_new = 0;
            }
        }
    }
    
    // Calculate cycle duration and reset power timestamp
    secs_cycle = (millis()-ts_power)/1000.0;
    ts_power = millis();
}

void MaintenanceTasks() {

    // Calculate AC power flows
    power_from_grid = (power_grid > 0) * power_grid;
    power_to_grid = (power_grid < 0) * -power_grid;
    power_from_ess = (power_ess < 0) * -power_ess;
    power_to_ess = (power_ess > 0) * power_ess;
    power_cons = power_pv - power_ess + power_grid;
    power_pv_to_ess = min(power_to_ess, power_pv);
    power_pv_to_cons = min(power_cons, power_pv - power_pv_to_ess);
    power_pv_to_grid = power_pv - power_pv_to_cons - power_pv_to_ess;
    power_ess_to_cons = min(power_cons - power_pv_to_cons, power_from_ess);
    power_ess_to_grid = power_from_ess - power_ess_to_cons;
    power_grid_to_ess = power_to_ess - power_pv_to_ess;
    power_grid_to_cons = power_from_grid - power_grid_to_ess;

    // Update AC energy counters
    float hrs_cycle = secs_cycle/3600;
    en_from_pv += hrs_cycle*power_pv;
    en_from_ess += hrs_cycle*power_from_ess;
    en_to_ess += hrs_cycle*power_to_ess;
    en_from_grid += hrs_cycle*power_from_grid;
    en_pv_to_cons += hrs_cycle*power_pv_to_cons;
    en_pv_to_ess += hrs_cycle*power_pv_to_ess;
    en_pv_to_grid = en_from_pv - en_pv_to_cons - en_pv_to_ess;
    en_grid_to_cons += hrs_cycle*power_grid_to_cons;
    en_grid_to_ess = en_to_ess - en_pv_to_ess;
    en_ess_to_cons += hrs_cycle*power_ess_to_cons;
    en_ess_to_grid = en_from_ess - en_ess_to_cons;
    if (power_ess > 0) en_pv_wasted += hrs_cycle*(1-0.9*pbat/power_ess)*min(power_from_grid, power_pv_to_ess);  // PV energy wasted to ESS, due to AC/DC/AC conversion losses
    
    // Update DC energy counters
    en_from_batt += hrs_cycle * (pbat < 0) * -pbat;
    en_to_batt += hrs_cycle * (pbat > 0) * pbat;

    // No ESS power output and no PV production: check for new power_grid_min (i.e. min household consumption)
    if (!power_old && !power_pv && (power_grid < power_grid_min)) {
        power_grid_min = power_grid;
        minpower_time = unixtime;
    }

    // Re-calculate astro times once per night (after a potential DST change, before sunrise)
    if (min_of_day == 210) CalcAstroTimes;

    // Keep alive Hoymiles RF24 interface
    if ((millis()-ts_HM)/1000 >= RF24_KEEPALIVE) {
        if (power_new < 0) HoymilesCommand(HM_ON);
        else HoymilesCommand(HM_OFF);
    }

    // Keep alive Meanwell relay (if Meanwell is turned on)
    if (mw_on && ((millis()-ts_MW)/1000 >= MW_KEEPALIVE)) ShellyCommand(PM2_ADDR, MW_ON);
    // Turn off Meanwell PWM optocoupler (if Meanwell is turned off)
    if (!mw_on && ((millis()-ts_MW)/2000 >= PROCESSING_DELAY)) ledcWrite(PWM_CHANNEL, 0);

    // Set Shelly 1PM eco mode (turn off at sunrise, turn on at sunset)
    if ((min_of_day >= sunrise) && (min_of_day <= sunset) && pm1_eco_mode) pm1_eco_mode = !ShellyCommand(PM1_ADDR, ECO_OFF);
    if (((min_of_day < sunrise) || (min_of_day > sunset)) && !pm1_eco_mode) pm1_eco_mode = ShellyCommand(PM1_ADDR, ECO_ON);

    // Set Shelly 2PM eco mode (turn off when charging/discharging active or possible, turn on when charging/discharging inactive and impossible)
    if ((power_new || auto_recharge) && pm2_eco_mode) pm2_eco_mode = !ShellyCommand(PM2_ADDR, ECO_OFF);
    if ((power_pv < MW_MIN_POWER) && !power_old && !hm_limit && !auto_recharge && !pm2_eco_mode) pm2_eco_mode = ShellyCommand(PM2_ADDR, ECO_ON);

    // Clear Shelly 3EM energy data at midnight (prevents HTTP timeouts at 00:00 UTC due to internal data reorgs)
    if (!min_of_day && !em_data_cleared) em_data_cleared = ShellyCommand(EM_ADDR, EM_RESET);
    if (min_of_day) em_data_cleared = false;

    // Turn off/on JKBMS balancer (disable/enable bottom balancing), depending on lowest cell voltage 
    if ((vcell_min >= ESS_UVPR) && bms_bal_on) bms_bal_on = !BMSCommand(BAL_OFF);
    if ((vcell_min < ESS_UVP) && !power_old && !bms_bal_on) bms_bal_on = BMSCommand(BAL_ON);

    // Check if public IP address was changed, if yes: update DDNS server entry
    if ((millis()-ts_pubip)/1000 >= DDNS_UPDATE_INTERVAL) UpdateDDNS();
}

void CheckErrors() {

    digitalWrite(LED_PIN, HIGH);  // signal end of cycle
    delay(20);
    if (error_msg[0] == '\0') {  // no errors this cycle
        digitalWrite(LED_PIN, LOW);  // turn off LED (if error: LED remains on during next cycle)
        errors_consecutive = 0;  // reset counter for consecutive errors
        if (!starttime) delay(500);  // add a little waiting time during setup()
        return;
    }

    if (!starttime) {
        // Error occured during system setup
        telnet.printf("%s%s%s\r\n\nPress any key to restart: ", SHOW_CURSOR, ERROR_SYMBOL, error_msg);
        while (!telnet.available());
        telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
        delay(3000);
        telnet.stop();
        ESP.restart();
    }
 
    // Increase error counter
    int error_index;
    for (error_index=0; error_index<ERROR_TYPES; error_index++)
        if (strstr(error_msg, ERROR_TYPE[error_index])) {
            error_counter[error_index]++;
            errortime[error_index] = unixtime;
            strcpy(last_error_msg, error_msg);
            break;
        }
    if (error_index >= UNCRITICAL_ERROR_TYPES) errors_consecutive++;  // if error was WIFI related: allow unlimited erroneous cycles
    if (errors_consecutive < ERROR_LIMIT) {
        error_flag = true;  // new unread error
        error_msg[0] = '\0';  // reset error message for next cycle
        return;
    }

    // Error is persistent: Halt the system
    HoymilesCommand(HM_OFF);
    SetMWPower(MW_MIN_POWER);
    ShellyCommand(PM2_ADDR, HM_AC_OFF);
    ShellyCommand(PM2_ADDR, MW_OFF);
    
    // System halted: prepare exception cycle message
    sprintf(cycle_msg, "%sBinSmart ESS %s\r\n\n%sSystem halted at %02d/%02d/%04d %02d:%02d:%02d\r\n   after %d consecutive errors", CLEAR_SCREEN, SW_VERSION, ERROR_SYMBOL, day(unixtime), month(unixtime), year(unixtime), hour(unixtime), minute(unixtime), second(unixtime), ERROR_LIMIT);
    strcat(cycle_msg, "\r\n\nLast error: ");
    strcat(cycle_msg, last_error_msg);
    strcat(cycle_msg, "\r\n\nPress any key to restart: ");
    command = '\0';

    while (true) {  // wait for user to confirm restart
        if ((millis()-ts_pubip)/1000 >= DDNS_UPDATE_INTERVAL) UpdateDDNS();  // keep updating DDNS in order to be reachable via Internet
        if (!telnet) telnet = server.available();
        if (telnet) telnet.print(cycle_msg);
        delay(PROCESSING_DELAY*1000);
        ledcWrite(PWM_CHANNEL, 0);  // turn off Meanwell PWM optocoupler
        if (telnet.available()) {
            telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
            delay(3000);
            telnet.stop();
            ESP.restart();
        }
    }
}

void FinishCycle() {

    if (!telnet) {
        cmd_resp[0] = command = '\0';  // telnet session ended: clear user command response
        telnet = server.available();  // new telnet session?
    }
    if (telnet) {
        // Prepare and print cycle message and command response (including command prompt)
        PrepareCycleMsg();  // prepare cycle status message
        PrepareCmdResp();  // react to user command from previous cycle(s)
        telnet.printf("%s%s", cycle_msg, cmd_resp);  // print cycle message and command response (including command prompt)
    }
    while ((millis()-ts_power)/1000 < PROCESSING_DELAY);  // wait for power changes to take effect
    if (telnet.available()) command = telnet.read();  // read new user command        if (telnet.available()) command = telnet.read();  // read new user command
}

void SetMWPower(int power) {

    // convert power setting to PWM value
    float I = float(power)/vbat;
    int pwm_duty_cycle = round(MW_POWER_FORMULA);
    if (power < MW_LOW_POWER_THRESHOLD) pwm_duty_cycle = round(MW_LOW_POWER_FORMULA);  //  compensate non-linear power output in low power range

    // Make sure upper/lower PWM limits are not exceeded
    if (pwm_duty_cycle > PWM_DUTY_CYCLE_MAX) pwm_duty_cycle = PWM_DUTY_CYCLE_MAX;
    if (pwm_duty_cycle < PWM_DUTY_CYCLE_MIN) pwm_duty_cycle = PWM_DUTY_CYCLE_MIN;
    ledcWrite(PWM_CHANNEL, pwm_duty_cycle);
}

bool HoymilesCommand(int hm_command) {

    while (millis()-ts_HM < 50);  // minimum delay between two consecutive HM commands

    if ((hm_command == HM_OFF) || (hm_command == HM_ON)) {  // switch command
        if (radio.writeFast(HM_SWITCH[hm_command], sizeof(HM_SWITCH[hm_command])))
            if (radio.txStandBy(RF24_TIMEOUT*1000)) {
                ts_HM = millis();
                return true;
            }
        ts_HM = millis();
        if (hm_command == HM_OFF) strcpy(error_msg, "RF24 Hoymiles turn off command failed");
        else strcpy(error_msg, "RF24 Hoymiles turn on command failed");
        return false;
    }

    // power limit command
    unsigned int limit = -10 * hm_command;
    hm_power[12] = highByte(limit);
    hm_power[13] = lowByte(limit);
    crc16.restart();
    crc16.add(&hm_power[10],6);
    unsigned int crc = crc16.getCRC();
    hm_power[16] = highByte(crc);
    hm_power[17] = lowByte(crc);
    crc8.restart();
    crc8.add(hm_power, 18);
    hm_power[18] = crc8.getCRC();

    if (radio.writeFast(hm_power, sizeof(hm_power)))
        if (radio.txStandBy(RF24_TIMEOUT*1000)) {
                ts_HM = millis();
                return true;
        }
    ts_HM = millis();
    strcpy(error_msg, "RF24 Hoymiles power command failed");
    return false;
}

void PrepareCycleMsg() {

    // Cycle message header
    strcpy(cycle_msg, CLEAR_SCREEN);
    strcat(cycle_msg, "BinSmart ESS ");
    strcat(cycle_msg, SW_VERSION);
    strcat(cycle_msg, " ");
    if (error_flag) strcat(cycle_msg, ERROR_SYMBOL);
    strcat(cycle_msg, WIFI_SYMBOL[WiFi.RSSI() >= GOOD_WIFI_RSSI]);

    // Time, cycle time, daytime
    sprintf(cycle_msg + strlen(cycle_msg), "\r\n%02d:%02d:%02d %+.3f%s\r\n", hour(unixtime), minute(unixtime), second(unixtime), secs_cycle, OPS_SYMBOL[!power_new + (pm1_eco_mode && pm2_eco_mode)]);

    // OVP symbol above battery symbol
    strcat(cycle_msg, BAT_OVP_SYMBOL[(mw_limit_old < MW_MAX_POWER) + !mw_limit_old]);

    // PV power
    sprintf(cycle_msg + strlen(cycle_msg), "\r\n%4d ", int(round(power_pv)));
    strcat(cycle_msg, NIGHT_DAY_SYMBOL[daytime]);
    strcat(cycle_msg, PV_FLOW_SYMBOL[(round(power_pv) > 0) + (round(power_pv) >= PV_MAX_POWER)]);
    strcat(cycle_msg, CABLE_SYMBOL);
    strcat(cycle_msg, PV_CABLE_SYMBOL);

    // ESS power
    strcat(cycle_msg, ESS_CABLE_SYMBOL);
    strcat(cycle_msg, CABLE_SYMBOL);
    if (!power_old)
        strcat(cycle_msg, ESS_FLOW_SYMBOL[!mw_limit_old + 2*(!hm_limit_old)]);
    if (power_old > 0)
        strcat(cycle_msg, MW_FLOW_SYMBOL[(power_old == mw_limit_old) + ((power_old == mw_limit_old) && (mw_limit_old >= MW_MAX_POWER))][power_grid_to_ess > power_pv_to_ess]);
    if (power_old < 0)
        strcat(cycle_msg, HM_FLOW_SYMBOL[(power_old == hm_limit_old) + (power_old == HM_MAX_POWER)]);
    strcat(cycle_msg, BAT_LEVEL_SYMBOL[bat_level]);
    sprintf(cycle_msg + strlen(cycle_msg), "%d", int(round(power_ess)));
    if (power_new != power_old) strcat(cycle_msg, DIFF_SYMBOL[(power_new < power_old) + !filter_cycles]);
    else if (filter_cycles && (filter_cycles < POWER_FILTER_CYCLES)) strcat(cycle_msg, POWERFILTER_SYMBOL);
    if (manual_mode) strcat(cycle_msg, MANUAL_MODE_SYMBOL);
    if (auto_recharge) strcat(cycle_msg, AUTO_RECHARGE_SYMBOL);
    strcat(cycle_msg, "\r\n");

    // House symbol
    strcat(cycle_msg, HOUSE_SYMBOL);

    // UVP symbol below battery symbol
    strcat(cycle_msg, BAT_UVP_SYMBOL[(hm_limit_old > HM_MAX_POWER) + !hm_limit_old]);

    // Grid power
    sprintf(cycle_msg + strlen(cycle_msg), "\r\n%4d ", int(round(power_grid)));
    strcat(cycle_msg, GRID_SYMBOL);
    strcat(cycle_msg, GRID_FLOW_SYMBOL[(int(round(power_grid)) > 0)  + 2*(int(round(power_grid)) < 0)][power_ess_to_grid > power_pv_to_grid]);
    strcat(cycle_msg, CABLE_SYMBOL);
    strcat(cycle_msg, GRID_CABLE_SYMBOL);

    // Power to consumers
    strcat(cycle_msg, CONS_CABLE_SYMBOL);
    strcat(cycle_msg, CABLE_SYMBOL);
    if (power_grid_to_cons >= max(power_pv_to_cons, power_ess_to_cons)) strcat(cycle_msg, CONS_FLOW_SYMBOL[0]);
    if (power_ess_to_cons >= max(power_grid_to_cons, power_pv_to_cons)) strcat(cycle_msg, CONS_FLOW_SYMBOL[1]);
    if (power_pv_to_cons >= max(power_grid_to_cons, power_ess_to_cons)) strcat(cycle_msg, CONS_FLOW_SYMBOL[2]);
    strcat(cycle_msg, CONS_SYMBOL);
    sprintf(cycle_msg + strlen(cycle_msg), " %.0f\r\n\n", power_cons);

    // Append error info (if error occured)
    if (error_msg[0] != '\0') {
        strcat(cycle_msg, ERROR_SYMBOL);
        strcat(cycle_msg, error_msg);
        strcat(cycle_msg, "\r\n\n");
    }
}

void PrepareCmdResp() {

    switch (command) {
        case 'm':
            command = '\0';  // respond to command only once
            telnet.printf("%s%s", cycle_msg, "Enter ESS power: ");
            ts_input = millis();
            while (!telnet.available() && ((millis()-ts_input)/1000 < READINPUT_TIMEOUT));
            if (telnet.available()) {
                power_manual = telnet.readString().toInt();
                manual_mode = true;
                sprintf(cmd_resp, "ESS power manually set to %d W\r\n\n", power_manual);
                break;
            }
            if (!manual_mode) strcpy(cmd_resp, "Automatic mode remains activated\r\n\n");
            else sprintf(cmd_resp, "ESS power remains at %d W\r\n\n", power_manual);
            break;
        case 'a':
            command = '\0';  // respond to command only once
            manual_mode = false;
            strcpy(cmd_resp, "Automatic mode reactivated\r\n\n");
            break;
        case 'g':
            command = '\0';  // respond to command only once
            telnet.printf("%s%s", cycle_msg, "Enter grid power target: ");
            ts_input = millis();
            while (!telnet.available() && ((millis()-ts_input)/1000 < READINPUT_TIMEOUT));
            if (telnet.available()) {
                power_target = telnet.readString().toInt();
                sprintf(cmd_resp, "Grid power target set to %d W\r\n\n", power_target);
                break;
            }
            sprintf(cmd_resp, "Grid power target remains at %d W\r\n\n", power_target);
            break;
        case 'b':
            sprintf(cmd_resp, "Batt voltage    : %.3f V\r\nCell voltages   : %d - %d mV\r\nMax cell diff   : %d mV", vbat/1000.0, vcell_min, vcell_max, vcell_max-vcell_min);
            if ((vcell_max-vcell_min >= bms_balancer_trigger) && (vcell_max >= bms_balancer_start) && bms_bal_on) strcat(cmd_resp, BALANCER_SYMBOL);
            sprintf(cmd_resp + strlen(cmd_resp), "\r\nBatt current    : %.2f A\r\nAC power set to : %d W\r\nAC power reading: %.1f W\r\nDC power reading: %.1f W\r\n", cbat/100.0, power_old, power_ess, pbat);
            if (power_eff > 0) sprintf(cmd_resp + strlen(cmd_resp), "MW power eff.   : %.1f %%\r\n", power_eff);
            if (power_eff < 0) sprintf(cmd_resp + strlen(cmd_resp), "HM power eff.   : %.1f %%\r\n", -power_eff);
            sprintf(cmd_resp + strlen(cmd_resp), "MW power limit  : %d W\r\nHM power limit  : %d W\r\n\n", mw_limit, hm_limit);
            break;
        case 'd':
            sprintf(cmd_resp, "ESS public IP address: %s\r\nLast address check   : %02d/%02d/%04d %02d:%02d\r\n", pubip_addr.toString().c_str(), day(pubip_time), month(pubip_time), year(pubip_time), hour(pubip_time), minute(pubip_time));
            sprintf(cmd_resp + strlen(cmd_resp), "Last DDNS update     : %02d/%02d/%04d %02d:%02d\r\n\n", day(ddns_time), month(ddns_time), year(ddns_time), hour(ddns_time), minute(ddns_time));
            break;
        case 'w':
            sprintf(cmd_resp, "WiFi RSSI: %d dBm\r\nChip temp: %.1f °C\r\n\n", WiFi.RSSI(), temperatureRead());
            break;
        case 't':
            sprintf(cmd_resp, "Local time    : %02d/%02d/%04d %02d:%02d:%02d", day(unixtime), month(unixtime), year(unixtime), hour(unixtime), minute(unixtime), second(unixtime));
            sprintf(cmd_resp + strlen(cmd_resp), "\r\nTimezone      : UTC%+d\r\nUTC offset    : ", timezone);
            if (dst) sprintf(cmd_resp + strlen(cmd_resp), "%+d (DST)", timezone+dst);
            else sprintf(cmd_resp + strlen(cmd_resp), "%+d (SDT)", timezone);
            sprintf(cmd_resp + strlen(cmd_resp), "\r\nESS started   : %02d/%02d/%04d %02d:%02d:%02d", day(starttime), month(starttime), year(starttime), hour(starttime), minute(starttime), second(starttime));
            sprintf(cmd_resp + strlen(cmd_resp), "\r\nESS uptime    : %03dd %02dh %02dm %02ds", elapsedDays(unixtime-starttime), numberOfHours(unixtime-starttime), numberOfMinutes(unixtime-starttime), numberOfSeconds(unixtime-starttime));
            sprintf(cmd_resp + strlen(cmd_resp), "\r\nSunrise today : %02d:%02d\r\nSunset today  : %02d:%02d\r\n\n", sunrise/60, sunrise%60, sunset/60, sunset%60); 
            break;
        case 'l':
            sprintf(cmd_resp, "Lowest consumption since %02d/%02d/%04d %02d:%02d\r\n\n", day(starttime), month(starttime), year(starttime), hour(starttime), minute(starttime));
            if (!minpower_time) strcat(cmd_resp, "Not yet measured\r\n\n");
            else sprintf(cmd_resp + strlen(cmd_resp), "%.1f W (measured %02d/%02d/%04d %02d:%02d)\r\n\n", power_grid_min, day(minpower_time), month(minpower_time), year(minpower_time), hour(minpower_time), minute(minpower_time));
            break;
        case 'n':
            sprintf(cmd_resp, "Energy flows [kWh] since %02d/%02d/%04d %02d:%02d\r\n\n", day(resettime_energy), month(resettime_energy), year(resettime_energy), hour(resettime_energy), minute(resettime_energy));
            // PV energy
            sprintf(cmd_resp + strlen(cmd_resp), "%7.3f %s%s%s%s%s%s %.1f﹪+%.1f﹪\r\n", en_from_pv/1000, NIGHT_DAY_SYMBOL[1], PV_FLOW_SYMBOL[1], PV_CABLE_SYMBOL, ESS_CABLE_SYMBOL, MW_FLOW_SYMBOL[0][0], ESS_SYMBOL, (en_pv_to_ess-en_pv_wasted)/en_from_pv*100, en_pv_wasted/en_from_pv*100);
            sprintf(cmd_resp + strlen(cmd_resp), "  %s\r\n", HOUSE_SYMBOL);
            sprintf(cmd_resp + strlen(cmd_resp), "%6.1f﹪%s%s%s%s%s%s %.1f﹪\r\n\n", en_pv_to_grid/en_from_pv*100, GRID_SYMBOL, GRID_FLOW_SYMBOL[2][0], GRID_CABLE_SYMBOL, CONS_CABLE_SYMBOL, CONS_FLOW_SYMBOL[2], CONS_SYMBOL, en_pv_to_cons/en_from_pv*100);
            // Grid energy
            sprintf(cmd_resp + strlen(cmd_resp), "                %s%s%s %.1f﹪\r\n", ESS_CABLE_SYMBOL, MW_FLOW_SYMBOL[0][1], ESS_SYMBOL, en_grid_to_ess/en_from_grid*100);
            sprintf(cmd_resp + strlen(cmd_resp), "  %s\r\n", HOUSE_SYMBOL);
            sprintf(cmd_resp + strlen(cmd_resp), "%7.3f %s%s%s%s%s%s %.1f﹪\r\n\n", en_from_grid/1000, GRID_SYMBOL, GRID_FLOW_SYMBOL[1][0], GRID_CABLE_SYMBOL, CONS_CABLE_SYMBOL, CONS_FLOW_SYMBOL[0], CONS_SYMBOL, en_grid_to_cons/en_from_grid*100);
             // ESS energy
            sprintf(cmd_resp + strlen(cmd_resp), "                %s%s%s %.3f\r\n", ESS_CABLE_SYMBOL, HM_FLOW_SYMBOL[0], ESS_SYMBOL, en_from_ess/1000);
            sprintf(cmd_resp + strlen(cmd_resp), "  %s\r\n", HOUSE_SYMBOL);
            sprintf(cmd_resp + strlen(cmd_resp), "%6.1f﹪%s%s%s%s%s%s %.1f﹪\r\n\n", en_ess_to_grid/en_from_ess*100, GRID_SYMBOL, GRID_FLOW_SYMBOL[2][1], GRID_CABLE_SYMBOL, CONS_CABLE_SYMBOL, CONS_FLOW_SYMBOL[1], CONS_SYMBOL, en_ess_to_cons/en_from_ess*100);
            // Meanwell/Hoymiles/ESS efficiency
            sprintf(cmd_resp + strlen(cmd_resp), "MW AC▸DC eff.: %.1f﹪\r\nHM DC▸AC eff.: %.1f﹪\r\nAC▸DC▸AC eff.: %.1f﹪\r\n\n", en_to_batt/en_to_ess*100, en_from_ess/en_from_batt*100, en_from_ess/en_to_ess*100);
            break;
        case 'e':
            sprintf(cmd_resp, "Errors since %02d/%02d/%04d %02d:%02d\r\n\n", day(resettime_errors), month(resettime_errors), year(resettime_errors), hour(resettime_errors), minute(resettime_errors));
            for (int i=0; i<ERROR_TYPES; i++) {
                sprintf(cmd_resp + strlen(cmd_resp), "%s\t%d", ERROR_TYPE[i], error_counter[i]);
                if (error_counter[i]) sprintf(cmd_resp + strlen(cmd_resp), " (last: %02d/%02d/%04d %02d:%02d)", day(errortime[i]), month(errortime[i]), year(errortime[i]), hour(errortime[i]), minute(errortime[i]));
                strcat(cmd_resp, "\r\n");
            }
            strcat(cmd_resp, "\r\n");
            if (last_error_msg[0] != '\0') sprintf(cmd_resp + strlen(cmd_resp), "Last error: %s\r\n\n", last_error_msg);
            error_flag = false;
            break;
        case 's':
            sprintf(cmd_resp, "MW relay ops since %02d/%02d/%04d %02d:%02d\r\n\n", day(starttime), month(starttime), year(starttime), hour(starttime), minute(starttime));
            sprintf(cmd_resp + strlen(cmd_resp), "%d (%d/day)\r\n\n", mw_counter, mw_counter/(elapsedDays(unixtime-starttime)+1));
            break;
        case 'z':
            command = '\0';  // respond to command only once
            telnet.printf("%s%s", cycle_msg, "Reset [e]rror or e[n]ergy stats: ");
            ts_input = millis();
            while (!telnet.available() && ((millis()-ts_input)/1000 < READINPUT_TIMEOUT));
            if (telnet.available()) {
                char input = telnet.readString()[0];
                if (input == 'n') {
                    en_from_pv = en_pv_to_cons = en_pv_to_ess = en_pv_to_grid = en_pv_wasted = 0;  // Reset PV energy counters
                    en_from_grid = en_grid_to_cons = en_grid_to_ess = 0;  // Reset grid energy counters
                    en_from_ess = en_to_ess = en_ess_to_cons = en_ess_to_grid = 0;  // Reset ESS energy counters
                    en_from_batt = en_to_batt = 0;  // Reset ESS DC energy counters
                    resettime_energy = unixtime;
                    strcpy(cmd_resp, "Energy stats reset to zero\r\n\n");
                    break;
                }
                if (input == 'e') {
                    for (int i=0; i<ERROR_TYPES; i++) error_counter[i] = errortime[i] = 0;
                    errors_consecutive = 0;
                    last_error_msg[0] = '\0';
                    error_flag = false;
                    resettime_errors = unixtime;
                    strcpy(cmd_resp, "Error stats reset to zero\r\n\n");
                    break;
                }
            }
            strcpy(cmd_resp, "No stats reset\r\n\n");
            break;
        case 'c':
            cmd_resp[0] = command = '\0';  // clear command response from previous cycle(s)
            break;
        case 'r':
            command = '\0';  // respond to command only once
            telnet.printf("%s%s", cycle_msg, "Enter [y] to confirm system reboot: ");
            ts_input = millis();
            while (!telnet.available() && ((millis()-ts_input)/1000 < READINPUT_TIMEOUT));
            if (telnet.available()) {
                if (telnet.readString()[0] == 'y') {
                    telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
                    delay(3000);
                    telnet.stop();
                    ESP.restart();
                }
            }
            strcpy(cmd_resp, "System not rebooted\r\n\n");
            break;
        case 'h':
            strcpy(cmd_resp, "Command options:\r\n");
            strcat(cmd_resp, "[m] - Manual power mode\r\n");
            strcat(cmd_resp, "[a] - Automatic power mode\r\n");
            strcat(cmd_resp, "[g] - Grid power target\r\n");
            strcat(cmd_resp, "[b] - Battery info\r\n");
            strcat(cmd_resp, "[d] - DDNS info\r\n");
            strcat(cmd_resp, "[w] - WiFi RSSI\r\n");
            strcat(cmd_resp, "[t] - Time, uptime, astro times\r\n");
            strcat(cmd_resp, "[l] - Lowest household consumption\r\n");
            strcat(cmd_resp, "[n] - Energy stats\r\n");
            strcat(cmd_resp, "[e] - Error stats\r\n");
            strcat(cmd_resp, "[s] - Shelly relay counter\r\n");
            strcat(cmd_resp, "[z] - Reset stats to zero\r\n");
            strcat(cmd_resp, "[c] - Clear command response\r\n");
            strcat(cmd_resp, "[r] - Reboot system\r\n\n");
            break;
        default:
            if (cmd_resp[0] == '\0') strcpy(cmd_resp, "Enter command or [h] for help: ");
            return;
    }
    strcat(cmd_resp, "Enter command or [h] for help: ");
}

bool UpdateDDNS() {

    if (WiFi.status() != WL_CONNECTED) {
        strcpy(error_msg, "WIFI");
        return false;
    }

    // read public IP from server (with shorter timeout)
    http.setTimeout(HTTP_DDNS_TIMEOUT*1000);
    http.begin(PUBLIC_IP_URL);
    if (http.GET() == HTTP_CODE_OK) {
        pubip_addr.fromString(http.getString());
        http.end();
        ts_pubip = millis();
        pubip_time = unixtime;
        if (ddns_addr == pubip_addr) return true;  // public IP unchanged

        // update DDNS server entry
        strcpy(http_command, DDNS_SERVER_URL);
        strcat(http_command, pubip_addr.toString().c_str());
        http.begin(http_command);
        if (http.GET() == HTTP_CODE_OK) {
            http.end();
            ddns_addr = pubip_addr;
            ddns_time = unixtime;
            return true;
        }
        http.end();
        strcpy(error_msg, "DDNS update failed");
        return false;
    }
    http.end();
    // Read IP command frequently fails, no big deal, so don't report
    return false;
}

void CalcAstroTimes() {
    Dusk2Dawn ess_location(latitude, longitude, timezone);  // calculate sunrise and sunset of current day
    sunrise = ess_location.sunrise(year(unixtime), month(unixtime), day(unixtime), dst);
    sunset = ess_location.sunset(year(unixtime), month(unixtime), day(unixtime), dst);
    daytime = ((min_of_day > sunrise) && (min_of_day < sunset));
}