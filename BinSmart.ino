const String SW_VERSION = "v2.04";

#include <WiFi.h>  // standard Arduino/ESP32
#include <HTTPClient.h>  // standard Arduino/ESP32
#include <ESPAsyncWebServer.h>  // https://github.com/lacamera/ESPAsyncWebServer
#include <AsyncElegantOTA.h>  // superseded by https://github.com/ayushsharma82/ElegantOTA (I prefer the old version)
#include <RF24.h>  // https://nrf24.github.io/RF24/
#include <CRC8.h>  // https://github.com/RobTillaart/CRC
#include <CRC16.h>  // https://github.com/RobTillaart/CRC
#include <TimeLib.h>  // https://playground.arduino.cc/Code/Time/
#include <Dusk2Dawn.h>  // https://github.com/dmkishi/Dusk2Dawn
#include "BinSmart_cfg.h"
#include "BinSmart_var.h"

void setup() {

    //Reduce CPU clock to 80 MHz (30% of max is enough for this application)
    setCpuFrequencyMhz(80);
    delay(500);

    // reserve memory for global Strings
    ota_msg.reserve(100);
    error_msg.reserve(200);
    last_error_msg.reserve(200);
    cmd_resp.reserve(700);
    cycle_msg.reserve(1000);
    if (!http_resp.reserve(2400)) error_msg = "String memory allocation failed";
    else error_msg = cmd_resp = "";

    // Assign LED Pin
    pinMode(LED_PIN, OUTPUT);

    // Init PWM generator (to adjust Meanwell power)
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM_OUTPUT_PIN, PWM_CHANNEL);
    SetMWPower(MW_MIN_POWER);

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
    ota_msg += "\r\nEnter \"";
    ota_msg += ESP32_ADDR.toString();
    ota_msg += "/update\" for OTA firmware update";
    OTAserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { request->send(200, "text/plain", ota_msg); });
    AsyncElegantOTA.begin(&OTAserver);
    OTAserver.begin();

    // Start wifi server and wait for terminal to connect
    server.begin();
    while (!telnet) telnet = server.available();

    // Print startup cycle_msg
    cycle_msg = "\033[0H\033[0J";  // clear entire terminal screen
    cycle_msg += "BinSmart ESS ";
    cycle_msg += SW_VERSION;
    cycle_msg += "\r\n\n";
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
    delay(1000);

    // Init RS485 communication with BMS, read OVP/UVP/balancer settings
    Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial2.write(BMS_SETTINGS, sizeof(BMS_SETTINGS));
    if (ReadBMSResponse()) telnet.print("RS485 communication with JKBMS OK\r\n");
    delay(1000);
    
    // Init RF24 radio communication with Hoymiles
    if (radio.begin()) {
        radio.setPALevel(RF24_PALEVEL);
        radio.setChannel(RF24_CHANNEL);
        radio.setDataRate(RF24_250KBPS);
        radio.setCRCLength(RF24_CRC_16);
        radio.setAddressWidth(sizeof(HM_RADIO_ID));
        radio.enableDynamicPayloads();
        radio.openWritingPipe(HM_RADIO_ID);
        radio.stopListening();

        crc16.setPolynome((uint16_t)0x18005);
        crc16.setStartXOR(0xFFFF);
        crc16.setEndXOR(0x0000);
        crc16.setReverseIn(true);
        crc16.setReverseOut(true);
        crc8.setPolynome(0x01);
        crc8.setStartXOR(0);
        crc8.setEndXOR(0);

        // CRC8 need to be calculated only once for hm_turnon and hm_turnoff commands
        crc8.restart();
        crc8.add(hm_turnon, 14);
        hm_turnon[14] = crc8.getCRC();
        crc8.restart();
        crc8.add(hm_turnoff, 14);
        hm_turnoff[14] = crc8.getCRC();

        // Turn off Hoymiles (DC side)
        if (HoymilesCommand(HM_OFF)) telnet.print("RF24 communication with Hoymiles OK\r\n");
    }
    else error_msg = "RF24 radio init failed";
    delay(1000);

    // Set timeout for http requests
    http.setTimeout(HTTP_TIMEOUT*1000);

    // Shelly 1PM: Read PV power, reset energy counter, disable eco mode
    if (ShellyCommand(PM_STATUS)) telnet.print("Shelly 1PM found\r\n");
    ShellyCommand(PM_RESET);
    ShellyCommand(PM_ECO_OFF);

    // Shelly Meanwell plug: Turn off, reset energy counter, disable eco mode
    if (ShellyCommand(MWPLUG_OFF)) telnet.print("Meanwell plug found\r\n");
    ShellyCommand(MWPLUG_RESET);
    ShellyCommand(MWPLUG_ECO_OFF);

    // Shelly Hoymiles plug: Turn on, reset energy counter
    if (ShellyCommand(HMPLUG_ON)) telnet.print("Hoymiles plug found\r\n");
    ShellyCommand(HMPLUG_RESET);

    // Shelly 3EM: Read grid power and current time, reset energy counters
    if (ShellyCommand(EM_STATUS)) telnet.print("Shelly 3EM found\r\n");
    ShellyCommand(EM_RESET);
    
    // No erros during setup: zero error counters, set timestamps, start polling cycle
    if (error_msg == "") {
        for (int i=0; i<ERROR_TYPES; i++) error_counter[i] = errortime[i] = 0;
        starttime = resettime_errors = resettime_energy = local_unixtime;
        ts_power = millis();
        telnet.print("\r\nNo errors during setup, start polling cycle ...");
        delay(PROCESSING_DELAY);
    }
    else {
        telnet.print("\r\n");
        telnet.print(ERROR_SYMBOL);
        telnet.print(" ");
        telnet.print(error_msg);
        telnet.print("\r\nSetup error(s) occured, system halted");
        telnet.print("\r\n\nEnter [r] to restart: ");
        while (!telnet.available()) FlashLED(false);
        telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
        delay(3000);
        telnet.stop();
        delay(500);
        ESP.restart();
    }
}

void loop() {

    Serial2.write(BMS_VOLTAGES, sizeof(BMS_VOLTAGES));  // Ask BMS for battery cell voltages
    ShellyCommand(EM_STATUS);  // Read local time and grid power from Shelly 3EM (while BMS is fetching cell voltages)
    ReadBMSResponse();  // Read cell voltages from BMS
    SetNewPower();  // Set power limits, calculate and apply new charging/discharging power
    UserCommand();  // Process user command from previous cycle(s)
    FinishCycle();  // Read PV power, update energy counters, print cycle info, carry out maintenance tasks
    CheckErrors();  // Check errors, halt system if an error is persistent, or show completed cycle and error status by flashing LED

    telnet.print("\033[?25h");  // show cursor
    // read user command while waiting for power changes to take effect (UVP sleep mode: extend waiting time)
    while (millis()-ts_power < max(PROCESSING_DELAY, uvp_sleep_mode * UVP_SLEEP_DELAY))
        if (ReadCommand() && (millis()-ts_power >= PROCESSING_DELAY)) break;
    telnet.print("\033[?25l");  // hide cursor
}

bool ReadBMSResponse() {

    byte BMS_resp[300] = {0};
    int len = 0, sum = 0;

    // Wait for BMS to respond to previously sent command (max waiting time: 100 ms)
    for (int i=0; i<100; i++) {
        if (Serial2.available()) break;
        delay(1);
    }
    
    // fill response buffer with BMS response
    while (Serial2.available()) {
        BMS_resp[len] = Serial2.read();
        sum += BMS_resp[len];
        len++;
    }
    if (BMS_resp[BMS_COMMAND_POS] == BMS_READ_ALL) {
        // extra long response: wait a little, then read the rest of the response bytes
        delay(20);
        while (Serial2.available()) {
            BMS_resp[len] = Serial2.read();
            sum += BMS_resp[len];
            len++;
        }
    }

    // check validity of BMS response
    if ((BMS_resp[0] == BMS_STX_1) && (BMS_resp[1] == BMS_STX_2)) {  // start of response frame
         if ((BMS_resp[2]<<8|BMS_resp[3]) == len-2) {  // response length
             if ((BMS_resp[len-2]<<8|BMS_resp[len-1]) == sum-BMS_resp[len-2]-BMS_resp[len-1]) {  // response checksum

                if (BMS_resp[BMS_COMMAND_POS] == BMS_READ_DATA) {
                    if (BMS_resp[BMS_DATA_ID_POS] == BMS_VCELLS_ID) {
                        // read voltages of batt cells, determine min and max voltages
                        vbat = vcell_max = 0; vcell_min = 4000;
                        for (int vcells_pos = BMS_DATA_ID_POS+3; vcells_pos <= BMS_DATA_ID_POS+BMS_resp[BMS_DATA_ID_POS+1]; vcells_pos+=3) {
                            int vcell = BMS_resp[vcells_pos]<<8|BMS_resp[vcells_pos+1];
                            if (vcell < vcell_min) vcell_min = vcell;
                            if (vcell > vcell_max) vcell_max = vcell;
                            vbat += vcell;
                        }
                        return true;
                    }
                    if (BMS_resp[BMS_DATA_ID_POS] == BMS_CURRENT_ID) {
                    // Read charging/discharging batt current, calculate batt power
                        int cbat = (BMS_resp[BMS_DATA_ID_POS+1]&0x7F)<<8|BMS_resp[BMS_DATA_ID_POS+2];
                        if (!(BMS_resp[BMS_DATA_ID_POS+1]&0x80)) cbat = -cbat; // charging current is positive, discharging current is negative
                        pbat = vbat/1000.0*cbat/100.0;  
                        return true;               
                    }
                }

                if (BMS_resp[BMS_COMMAND_POS] == BMS_READ_ALL) {
                    // check for BMS warnings
                    if (BMS_resp[BMS_WARNINGS_POS] != BMS_WARNINGS_ID) {
                        error_msg = "Could not read BMS warnings";
                        return false;
                    }
                    if (BMS_resp[BMS_WARNINGS_POS+1] || BMS_resp[BMS_WARNINGS_POS+2]) {
                        error_msg = "BMS has warnings 0x";
                        if (BMS_resp[BMS_WARNINGS_POS+1] < 0x10) error_msg += "0";
                        error_msg += String(BMS_resp[BMS_WARNINGS_POS+1],HEX);
                        if (BMS_resp[BMS_WARNINGS_POS+2] < 0x10) error_msg += "0";
                        error_msg += String(BMS_resp[BMS_WARNINGS_POS+2],HEX);
                        return false;
                    }
                    // check OVP and UVP setting
                    if (BMS_resp[BMS_OVP_POS] != BMS_OVP_ID) {
                        error_msg = "Could not read BMS Cell OVP";
                        return false;
                    }
                    if ((BMS_resp[BMS_OVP_POS+1]<<8|BMS_resp[BMS_OVP_POS+2]) - ESS_OVP < ESS_BMS_OVP_DIFF) {
                        error_msg = "ESS_OVP less than ";
                        error_msg += ESS_BMS_OVP_DIFF;
                        error_msg += " mV below BMS Cell OVP (";
                        error_msg += (BMS_resp[BMS_OVP_POS+1]<<8|BMS_resp[BMS_OVP_POS+2]);
                        error_msg += " mV)";
                        return false;
                    }
                    if (BMS_resp[BMS_UVP_POS] != BMS_UVP_ID) {
                        error_msg = "Could not read BMS Cell UVP";
                        return false;
                    }
                    bms_uvp = BMS_resp[BMS_UVP_POS+1]<<8|BMS_resp[BMS_UVP_POS+2];
                    if (ESS_UVP - bms_uvp < ESS_BMS_UVP_DIFF) {
                        error_msg = "ESS_UVP less than ";
                        error_msg += ESS_BMS_UVP_DIFF;
                        error_msg += " mV above BMS Cell UVP (";
                        error_msg += bms_uvp;
                        error_msg += " mV)";
                        return false;
                    }
                    // read balancer settings
                    if (BMS_resp[BMS_SBAL_POS] != BMS_SBAL_ID) {
                        error_msg = "Could not read BMS Balancer Start Voltage";
                        return false;
                    }
                    BMS_balancer_start = BMS_resp[BMS_SBAL_POS+1]<<8|BMS_resp[BMS_SBAL_POS+2];
                    if (BMS_resp[BMS_TBAL_POS] != BMS_TBAL_ID) {
                        error_msg = "Could not read BMS Balancer Trigger Voltage";
                        return false;
                    }
                    BMS_balancer_trigger = BMS_resp[BMS_TBAL_POS+1]<<8|BMS_resp[BMS_TBAL_POS+2];
                    return true;
                }
            }
        } 
    }
    // BMS communication failed
    error_msg = "BMS command failed";
    return false;
}

bool ShellyCommand(const String URL) {

    if (WiFi.status() != WL_CONNECTED) {
        error_msg = "WIFI";
        return false;
    }

    // Handle commands for Shelly energy meter, power meter or plug
    http.begin(URL);
    http_resp_code = http.GET();
    if (http_resp_code == HTTP_CODE_OK) {
        http_resp = http.getString();
        http.end();
        if (URL == EM_STATUS) {
            power_grid = round(atof(&http_resp[http_resp.indexOf("total_power",700)+13]));  // read grid power from http response
            int index = http_resp.indexOf("\"time\"");  // read local time from http response
            String local_time = http_resp.substring(index+8, index+13);  // local_time includes timezone and DST
            long unixtime = atol(&http_resp[index+26]);  // read epoch time (UTC) from http response
            int UTC_offset = (local_time.toInt()+24-hour(unixtime))%24;  // timezone_offset + DST_offset
            local_unixtime = unixtime + UTC_offset*3600;  // adjust unixtime to match local_time
            dst = (UTC_offset == ESS_TIMEZONE+1);  // daylight saving time
            if ((local_time == GET_ASTRO_TIME) || !starttime) {  // Calculate sunrise/sunset times for current day
                int mins_after_midnight = ess_location.sunrise(year(local_unixtime), month(local_unixtime), day(local_unixtime), dst);
                sprintf(&sunrise[0], "%02d:%02d", mins_after_midnight/60, mins_after_midnight%60);
                mins_after_midnight = ess_location.sunset(year(local_unixtime), month(local_unixtime), day(local_unixtime), dst);
                sprintf(&sunset[0], "%02d:%02d", mins_after_midnight/60, mins_after_midnight%60);
            }
            daytime = ((local_time >= sunrise) && (local_time < sunset));  // set daytime/nighttime flag
            return true;
        }
        if (URL == PM_STATUS) {
            power_pv = round(atof(&http_resp[http_resp.indexOf("apower")+8]));
            from_pv = atof(&http_resp[http_resp.indexOf("total")+7]);
            return true;
        }
        if (URL == MWPLUG_ON) {
            ts_MW = millis();
            if (!mwplug_on) mw_counter++;
            mwplug_on = true;
            return true;
        }
        if (URL == MWPLUG_OFF) {
            if (mwplug_on) mw_counter++;
            mwplug_on = false;
            return true;
        }
        if (URL == PM_ECO_ON) {
            pm_eco_mode = true;
            return true;
        }
        if (URL == PM_ECO_OFF) {
            pm_eco_mode = false;
            return true;
        }
        if (URL == MWPLUG_ECO_ON) {
            mwplug_eco_mode = true;
            return true;
        }
        if (URL == MWPLUG_ECO_OFF) {
            mwplug_eco_mode = false;
            return true;
        }
        return true;
    }
    // Shelly command failed
    http.end();
    if (URL.indexOf(EM_ADDR) > -1) error_msg = "3EM";
    if (URL.indexOf(PM_ADDR) > -1) error_msg = "1PM";
    if (URL.indexOf(MWPLUG_ADDR) > -1) error_msg = "MWPLUG";
    if (URL.indexOf(HMPLUG_ADDR) > -1) error_msg = "HMPLUG";
    error_msg += " cmd failed: ";
    error_msg += URL;
    error_msg += " (http code ";
    error_msg += http_resp_code;
    error_msg += ")";
    return false;
}

void SetNewPower() {

    // Save power setting from previous cycle as baseline for new power calculations
    power_old = power_new;

    // Update Meanwell hardware power limit, depending on vbat
    bool mw_limit_was_max = (mw_power_limit == mw_max_power);
    mw_max_power = round(MW_POWER_LIMIT_FORMULA);

    // Update OVP max charging power, depending on vbat and max cell voltage
    if ((vcell_max <= ESS_OVPR) || mw_limit_was_max) mw_power_limit = mw_max_power;  // exit OVP mode or update Meanwell power limit
    if (vcell_max >= ESS_OVP) {
        mw_power_limit = round(power_old*POWER_LIMIT_RAMPDOWN);  // ramp down charging power limit softly
        if (mw_power_limit < MW_MIN_POWER) mw_power_limit = 0;  // enter OVP mode
    }

    // Update UVP max discharging power, depending on min cell voltage
    if (vcell_min >= ESS_UVPR) hm_power_limit = HM_MAX_POWER;  // exit UVP mode
    if (vcell_min <= ESS_UVP) {
        hm_power_limit = round(power_old*POWER_LIMIT_RAMPDOWN);  // ramp down discharging power limit softly
        if (hm_power_limit > HM_MIN_POWER) hm_power_limit = 0;  // enter UVP mode
    }

    // Turn on/off automatic recharge feature (prevents BMS Cell UVP shutdown)
    if (vcell_min <= (bms_uvp + ESS_UVP)/2) auto_recharge = true;
    if (vcell_min >= ESS_UVP) auto_recharge = false;
    
    // Calculate new power setting
    power_new = power_old - power_grid + power_target;
    if (abs(power_new - power_old) <= POWER_TARGET_DEVIATION) power_new = power_old;

    // Filter out power spikes and slowly increase discharging power (reduces power loss to grid when consumer is suddenly turned  off)
    rampdown = false;
    if (power_old - max(power_new, hm_power_limit) > POWER_RAMPDOWN_RATE) {
        filter_cycles = max(filter_cycles-1, 0);  // countdown filter cycles
        if (filter_cycles) power_new = power_old;  // filter out power spikes
        else {
            power_new = power_old - POWER_RAMPDOWN_RATE;  // ramp down ESS power after filtering out power spikes
            rampdown = true;
        }
    }
    else filter_cycles = POWER_FILTER_CYCLES;  // reset filter cycle countdown

    if (manual_mode) {
        power_new = power_manual;  // Manual power setting overrides calculation
        rampdown = false;
        filter_cycles = POWER_FILTER_CYCLES;
    }
    if (auto_recharge) {
        power_new = max(MW_RECHARGE_POWER, power_new);  // Auto recharge power overrides calculation and manual setting
        rampdown = false;
        filter_cycles = POWER_FILTER_CYCLES;
    }

    // Make sure charging/discharging power limits are not exceeded
    if (power_new > mw_power_limit) power_new = mw_power_limit;
    if (power_new < hm_power_limit) power_new = hm_power_limit;

    // Turn ESS off if power is between MW and HM operating ranges
    if ((power_new > HM_MIN_POWER) && (power_new < MW_MIN_POWER)) power_new = 0;

    // If power setting has changed: apply new power setting
    if (power_new != power_old) {
        if (power_new == 0) {  // turn charging or discharging off
            if (power_old < 0)
                if (!HoymilesCommand(HM_OFF)) power_new = power_old;
            if (power_old > 0)
                if (!ShellyCommand(MWPLUG_OFF)) power_new = power_old;
        }
        if (power_new > 0) {  // set new charging power, (if necessary) turn discharging off, turn charging on
            if (power_old > 0) SetMWPower(power_new);
            if (power_old == 0) {
                SetMWPower(power_new);
                if (!ShellyCommand(MWPLUG_ON)) power_new = 0;
            }
            if (power_old < 0) {
                if (!HoymilesCommand(HM_OFF)) power_new = power_old;
                else {
                    SetMWPower(power_new);
                    if (!ShellyCommand(MWPLUG_ON)) power_new = 0;
                }
            }
        }
        if (power_new < 0) {  // set new discharging power, (if necessary) turn charging off, turn discharging on
            if (power_old < 0)
                if (!HoymilesCommand(power_new)) power_new = power_old;
            if (power_old == 0) {
                if (!HoymilesCommand(power_new)) power_new = 0;
                else if (!HoymilesCommand(HM_ON)) power_new = 0;
            }
            if (power_old > 0) {
                if (!ShellyCommand(MWPLUG_OFF)) power_new = power_old;
                else {
                    if (!HoymilesCommand(power_new)) power_new = 0;
                    else if (!HoymilesCommand(HM_ON)) power_new = 0;
                }
            }
        }
    }
    secs_cycle = (millis()-ts_power)/1000.0;
    ts_power = millis();
}

void FinishCycle() {

    // Daytime (Shelly 1PM eco mode turned off): Read PV power from Shelly 1PM (at nighttime power_pv is zero)
    if (!pm_eco_mode) ShellyCommand(PM_STATUS);

    // ESS is asleep: check for new min grid power (min household consumption)
    if (uvp_sleep_mode && (power_grid < power_grid_min)) {
        power_grid_min = power_grid;
        minpower_time = local_unixtime;
    }

    // Discharging disabled (UVP active) and low PV production: enable sleep mode
    uvp_sleep_mode = !hm_power_limit && mwplug_eco_mode && !power_old;
    
    // Calculate power flows that aren't measured by Shellies (or set by power_old)
    int power_cons = power_pv - power_old + power_grid;
    int power_from_grid = (power_grid > 0) * power_grid;
    int power_to_grid = (power_grid < 0) * -power_grid;
    int power_from_ess = (power_old < 0) * -power_old;
    int power_to_ess = (power_old > 0) * power_old;
    int power_pv_to_ess = min(power_pv, power_to_ess);
    int power_pv_to_cons = min(power_pv - power_pv_to_ess, power_cons);
    int power_pv_to_grid = power_pv - power_pv_to_cons - power_pv_to_ess;
    int power_ess_to_cons = min(power_from_ess, power_cons - power_pv_to_cons);
    int power_ess_to_grid = power_from_ess - power_ess_to_cons;
    int power_grid_to_ess = min(power_from_grid, power_to_ess - power_pv_to_ess);
    int power_grid_to_cons = power_from_grid - power_grid_to_ess;

    // Update energy counters
    float hrs_cycle = secs_cycle/3600;
    pv_to_cons += hrs_cycle*power_pv_to_cons;
    pv_to_ess += hrs_cycle*power_pv_to_ess;
    pv_to_grid += hrs_cycle*power_pv_to_grid;
    from_grid += hrs_cycle*power_from_grid;
    to_grid += hrs_cycle*power_to_grid;
    grid_to_cons += hrs_cycle*power_grid_to_cons;
    grid_to_ess += hrs_cycle*power_grid_to_ess;
    from_ess += hrs_cycle*power_from_ess;
    to_ess += hrs_cycle*power_to_ess;
    ess_to_cons += hrs_cycle*power_ess_to_cons;
    ess_to_grid += hrs_cycle*power_ess_to_grid;
    // PV energy consumed by household, directly or via ESS (via ESS: make sure grid_to_ess doesn't subsidize PV energy figure)
    pv_consumed = pv_to_cons + (ess_to_cons > grid_to_ess*0.85) * (ess_to_cons - grid_to_ess*0.85);

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
        cycle_msg += WIFI_SYMBOL[(WiFi.RSSI() >= GOOD_WIFI_RSSI)];
        cycle_msg += "\r\n";

        // Time, cycle time, daytime
        sprintf(buf,"%02d:%02d:%02d %+.3f",hour(local_unixtime),minute(local_unixtime),second(local_unixtime),secs_cycle);
        cycle_msg += buf;
        cycle_msg += OPS_SYMBOL[!power_old + uvp_sleep_mode];
        cycle_msg += "\r\n";

        // OVP limit symbol
        if (mw_power_limit < mw_max_power) cycle_msg += OVP_LIMIT_SYMBOL;
        cycle_msg += "\r\n";
        
        // PV power
        sprintf(buf,"%4d",power_pv);
        cycle_msg += buf;
        if (!daytime) cycle_msg += MOON_SYMBOL[(telnet.remoteIP()==WIN_CLIENT_ADDR)];
        else {
            int pv_level = min(power_pv*PV_LEVELS/PV_MAX_POWER, PV_LEVELS-1);
            cycle_msg += PV_LEVEL_SYMBOL[2*pv_level+(telnet.remoteIP()==WIN_CLIENT_ADDR)];
        }
        cycle_msg += PV_FLOW_SYMBOL[2*(power_pv>0)];
        cycle_msg += PV_CABLE_SYMBOL;

        // ESS power
        cycle_msg += ESS_CABLE_SYMBOL;
        if (power_old > 0) {
            if (power_grid_to_ess > power_pv_to_ess) cycle_msg += GRID_FLOW_SYMBOL[2];
            else cycle_msg += PV_FLOW_SYMBOL[2];
        }
        else  cycle_msg += ESS_FLOW_SYMBOL[(power_old<0)];
        if (vbat > ESS_EMPTY) cycle_msg += ESS_LEVEL_SYMBOL[min(((vbat-ESS_EMPTY)*(ESS_LEVELS-2))/(ESS_FULL-ESS_EMPTY)+1,ESS_LEVELS-1)];
        else cycle_msg += ESS_LEVEL_SYMBOL[0];
        cycle_msg += power_old;
        if (power_new != power_old) {
            cycle_msg += DIFF_SYMBOL[(power_new > power_old)];
            cycle_msg += abs(power_new - power_old);
            if (rampdown) cycle_msg += RAMPDOWN_SYMBOL;
        }
        else if (filter_cycles && (filter_cycles < POWER_FILTER_CYCLES)) cycle_msg += POWERFILTER_SYMBOL[filter_cycles%12];
        cycle_msg += MODE_SYMBOL[manual_mode + 2*auto_recharge];
        cycle_msg += "\r\n";

        // House symbol, UVP limit symbol
        cycle_msg += HOUSE_SYMBOL;
        if (hm_power_limit > HM_MAX_POWER) cycle_msg += UVP_LIMIT_SYMBOL;
        cycle_msg += "\r\n";

        // Grid power
        sprintf(buf,"%4d",power_grid);
        cycle_msg += buf;
        cycle_msg += GRID_SYMBOL;
        cycle_msg += CABLE_SYMBOL;
        if (power_grid < 0) {
            if (power_ess_to_grid > power_pv_to_grid) cycle_msg += ESS_FLOW_SYMBOL[1];
            else cycle_msg += PV_FLOW_SYMBOL[1];
        }
        else  cycle_msg += GRID_FLOW_SYMBOL[2*(power_grid>0)];
        cycle_msg += GRID_CABLE_SYMBOL;

        // Power consumption
        cycle_msg += CONS_CABLE_SYMBOL;
        if (power_grid_to_cons > max(power_pv_to_cons, power_ess_to_cons)) cycle_msg += GRID_FLOW_SYMBOL[2];
        else {
            if (power_ess_to_cons > power_pv_to_cons) cycle_msg += ESS_FLOW_SYMBOL[2];
            else cycle_msg += PV_FLOW_SYMBOL[2];
        }
        cycle_msg += CABLE_SYMBOL;
        cycle_msg += CONS_SYMBOL;
        cycle_msg += power_cons;
        cycle_msg += "\r\n\n";
        
        // Append error info (if error occured)
        if (error_msg != "") {
            cycle_msg += ERROR_SYMBOL;
            cycle_msg += " ";
            cycle_msg += error_msg;
            cycle_msg += "\r\n\n";
        }

        // Append command response to cycle info
        cycle_msg += cmd_resp;
        // Append command prompt to cycle info
        cycle_msg += "Enter command or [h] for help: ";

        // Print cycle info
        telnet.print(cycle_msg);
    }

    // Carry out (a maximum of) one of the following maintenance tasks per cycle (avoids overrun of PROCESSING_DELAY)

    // Keep alive Meanwell plug (if Meanwell is turned on)
    if (mwplug_on && ((millis()-ts_MW)/1000 >= MW_PLUG_TIMER-30)) {
        ShellyCommand(MWPLUG_ON);
        return;
    }

    // Set Shelly Meanwell Plug eco mode (turned off when PV produces more than MW min power, turned on when no PV production)
    if ((power_pv >= MW_MIN_POWER) && mwplug_eco_mode) {
        ShellyCommand(MWPLUG_ECO_OFF);
        return;
    }
    if (!power_pv && !mwplug_eco_mode) {
        ShellyCommand(MWPLUG_ECO_ON);
        return;
    }

    // Set Shelly 1PM eco mode (turned off at daytime, turned on at nighttime)
    if (daytime && pm_eco_mode) {
        ShellyCommand(PM_ECO_OFF);
        return;
    }
    if (!daytime && !pm_eco_mode) {
        ShellyCommand(PM_ECO_ON);
        return;
    }

    // Check if public IP address was changed, update DDNS server entry
    UpdateDDNS();
    return;
}

void CheckErrors() {

    if (error_msg == "") {
        errors_consecutive = 0;
        FlashLED(true);
        return;
    }

    error_flag = true;  // new unread error
    FlashLED(false);  // show error by flashing LED

    // Increase error counter
    int error_index;
    for (error_index=0; error_index<ERROR_TYPES; error_index++)
        if (error_msg.indexOf(ERROR_TYPE[error_index]) > -1) {
            error_counter[error_index]++;
            errortime[error_index] = local_unixtime;
            last_error_msg = error_msg;
            break;
        }
    if (error_index >= UNCRITICAL_ERROR_TYPES) errors_consecutive++;  // if error was WIFI related: allow unlimited erroneous cycles
    if (errors_consecutive < ERROR_LIMIT) {
        error_msg = "";
        return;
    }

    // Error is persistent: Halt the system
    HoymilesCommand(HM_OFF);
    ShellyCommand(MWPLUG_OFF);
    ShellyCommand(HMPLUG_OFF);
    hm_power_limit = power_grid = power_new = power_old = 0;
    cycle_msg = "\033[0H\033[0J";  // clear entire terminal screen
    cycle_msg += "BinSmart ESS ";
    cycle_msg += SW_VERSION;
    cycle_msg += "\r\n\nSystem halted at ";
    sprintf(buf,"%02d/%02d/%04d %02d:%02d:%02d",day(local_unixtime),month(local_unixtime),year(local_unixtime),hour(local_unixtime),minute(local_unixtime),second(local_unixtime));
    cycle_msg += buf;
    cycle_msg += "\r\nafter ";
    cycle_msg += ERROR_LIMIT;
    cycle_msg += " consecutive errors\r\n\nLast error:\r\n";
    cycle_msg += ERROR_SYMBOL;
    cycle_msg += " ";
    cycle_msg += error_msg;
    cycle_msg += "\r\n\nEnter [r] to restart: ";
    while (true) {
        telnet.print(cycle_msg);
        UpdateDDNS();  // keep updating DDNS in order to be reachable via Internet
        FlashLED(false);
        ts_power = millis();
        while (millis()-ts_power < PROCESSING_DELAY) if (ReadCommand()) break;
        if (command == 'r') {
            telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
            delay(3000);
            telnet.stop();
            delay(500);
            ESP.restart();
        }
    }
}

bool ReadCommand() {
    if (!telnet) telnet = server.available();
    if (telnet.available()) {
        command = telnet.read();
        telnet.flush();
        return true;
    }
    return false;
}

void UserCommand() {

    String input;
    unsigned long ts;

    if (!telnet) {
        cmd_resp = "";
        command = 0;
        return;
    }

    switch (command) {
        case 'm':
            command = 0;  // respond to command only once
            telnet.print(cycle_msg);
            telnet.print("\r\nEnter ESS power (negative for discharging): ");
            ts = millis();
            while ((millis()-ts)/1000 < READCOMMAND_TIMEOUT)
                if (telnet.available()) {
                    input = telnet.readString();
                    if ((input[0] == '0') || input.toInt()) {
                        manual_mode = true;
                        power_manual = input.toInt();
                        cmd_resp = "ESS power manually set to ";
                        cmd_resp += power_manual;
                        cmd_resp += " W\r\n\n";
                        return;
                    }
                    break;
                }
            if (!manual_mode) {
                cmd_resp = "Automatic mode remains activated\r\n\n";
            }
            else {
                cmd_resp = "ESS power remains at ";
                cmd_resp += power_manual;
                cmd_resp += " W\r\n\n";
            }
            break;
        case 'a':
            command = 0;  // respond to command only once
            manual_mode = false;
            cmd_resp = "Automatic mode activated\r\n\n";
            break;
        case 'g':
            command = 0;  // respond to command only once
            telnet.print(cycle_msg);
            telnet.print("\r\nEnter grid power target: ");
            ts = millis();
            while ((millis()-ts)/1000 < READCOMMAND_TIMEOUT)
                if (telnet.available()) {
                    input = telnet.readString();
                    if ((input[0] == '0') || input.toInt()) {
                        power_target = input.toInt();
                        cmd_resp = "Grid power target set to ";
                        cmd_resp += power_target;
                        cmd_resp += " W\r\n\n";
                        return;
                    }
                    break;
                }
            cmd_resp = "Grid power target remains at ";
            cmd_resp += power_target;
            cmd_resp += " W\r\n\n";
            break;
        case 'b':
            Serial2.write(BMS_CURRENT, sizeof(BMS_CURRENT));
            ReadBMSResponse();  // Read battery current and DC power from BMS
            cmd_resp = "Batt power DC   : ";
            cmd_resp += String(pbat,1);
            cmd_resp += " W\r\nAC/DC efficiency: ";
            if ((power_old > 0) && (pbat > 0)) cmd_resp += String(pbat/power_old*100,1);
                else if ((power_old < 0) && (pbat < 0)) cmd_resp += String(power_old/pbat*100,1);
                    else cmd_resp += "n/m";
            cmd_resp += " %\r\nBatt voltage    : ";
            cmd_resp += String(vbat/1000.0,3);
            cmd_resp += " V\r\nCell voltages   : ";
            cmd_resp += vcell_min;
            cmd_resp += " - ";
            cmd_resp += vcell_max;
            cmd_resp += " mV\r\nMax cell diff   : ";
            cmd_resp += vcell_max-vcell_min;
            cmd_resp += " mV";
            if ((vcell_max-vcell_min >= BMS_balancer_trigger) && (vcell_max >= BMS_balancer_start)) cmd_resp += BALANCER_SYMBOL;
            cmd_resp += "\r\nMW power limit  : ";
            cmd_resp += mw_max_power;
            cmd_resp += " W";
            if (hm_power_limit > HM_MAX_POWER) {
                cmd_resp += "\r\nUVP power limit : ";
                cmd_resp += hm_power_limit;
                cmd_resp += " W";
            }
            if (mw_power_limit < mw_max_power) {
                cmd_resp += "\r\nOVP power limit : ";
                cmd_resp += mw_power_limit;
                cmd_resp += " W";
            }
            cmd_resp += "\r\n\n";
            break;
        case 'd':
            cmd_resp = "ESS public IP address: ";
            cmd_resp += public_IP;
            cmd_resp += "\r\nLast address check   : ";
            sprintf(buf,"%02d/%02d/%04d %02d:%02d\r\n",day(pubip_time),month(pubip_time),year(pubip_time),hour(pubip_time),minute(pubip_time));
            cmd_resp += buf;
            cmd_resp += "Last DDNS update     : ";
            sprintf(buf,"%02d/%02d/%04d %02d:%02d\r\n\n",day(DDNS_time),month(DDNS_time),year(DDNS_time),hour(DDNS_time),minute(DDNS_time));
            cmd_resp += buf;
            break;
        case 'w':
            cmd_resp = "WiFi RSSI: ";
            cmd_resp += WiFi.RSSI();
            cmd_resp += " dBm\r\n\n";
            break;
        case 't':
            cmd_resp = "Local time    : ";
            sprintf(buf,"%02d/%02d/%04d %02d:%02d:%02d",day(local_unixtime),month(local_unixtime),year(local_unixtime),hour(local_unixtime),minute(local_unixtime),second(local_unixtime));
            cmd_resp += buf;
            cmd_resp += "\r\nTimezone      : ";
            sprintf(buf,"UTC%+d",ESS_TIMEZONE);
            cmd_resp += buf;
            cmd_resp += "\r\nUTC offset    : ";
            if (dst) sprintf(buf,"%+d (DST)",ESS_TIMEZONE+1);
            else sprintf(buf,"%+d (SDT)",ESS_TIMEZONE);
            cmd_resp += buf;
            cmd_resp += "\r\nESS started   : ";
            sprintf(buf,"%02d/%02d/%04d %02d:%02d:%02d",day(starttime),month(starttime),year(starttime),hour(starttime),minute(starttime),second(starttime));
            cmd_resp += buf;
            cmd_resp += "\r\nESS uptime    : ";
            sprintf(buf,"%03dd %02dh %02dm %02ds",elapsedDays(local_unixtime-starttime),numberOfHours(local_unixtime-starttime),numberOfMinutes(local_unixtime-starttime),numberOfSeconds(local_unixtime-starttime));
            cmd_resp += buf;
            cmd_resp += "\r\nSunrise today : ";
            cmd_resp += sunrise;
            cmd_resp += "\r\nSunset today  : ";
            cmd_resp += sunset;
            cmd_resp += "\r\n\n";
            break;
        case 'l':
            cmd_resp = "Lowest consumption since ";
            sprintf(buf,"%02d/%02d/%04d %02d:%02d\r\n",day(starttime),month(starttime),year(starttime),hour(starttime),minute(starttime));
            cmd_resp += buf;
            if (!minpower_time) cmd_resp += "Not yet measured\r\n\n";
            else {
                cmd_resp += power_grid_min;
                cmd_resp += " W (measured ";
                sprintf(buf,"%02d/%02d/%04d %02d:%02d)\r\n\n",day(minpower_time),month(minpower_time),year(minpower_time),hour(minpower_time),minute(minpower_time));
                cmd_resp += buf;
            }
            break;
        case 'n':
            cmd_resp = "Energy flows [kWh] since ";
            sprintf(buf,"%02d/%02d/%04d %02d:%02d\r\n\n",day(resettime_energy),month(resettime_energy),year(resettime_energy),hour(resettime_energy),minute(resettime_energy));
            cmd_resp += buf;

            if (from_pv) {
                sprintf(buf,"%7.3f",from_pv/1000);
                cmd_resp += buf;
                cmd_resp += SUN_SYMBOL[(telnet.remoteIP()==WIN_CLIENT_ADDR)];
                cmd_resp += PV_FLOW_SYMBOL[2];
                cmd_resp += PV_CABLE_SYMBOL;
                cmd_resp += ESS_CABLE_SYMBOL;
                cmd_resp += PV_FLOW_SYMBOL[2];
                cmd_resp += ESS_SYMBOL;
                sprintf(buf,"%.1f %%",pv_to_ess/from_pv*100);
                cmd_resp += buf;
                cmd_resp += "\r\n  ";
                cmd_resp += HOUSE_SYMBOL;
                cmd_resp += "\r\n";
                sprintf(buf,"%5.1f %%",pv_to_grid/from_pv*100);
                cmd_resp += buf;
                cmd_resp += GRID_SYMBOL;
                cmd_resp += PV_FLOW_SYMBOL[1];
                cmd_resp += GRID_CABLE_SYMBOL;
                cmd_resp += CONS_CABLE_SYMBOL;
                cmd_resp += PV_FLOW_SYMBOL[2];
                cmd_resp += CONS_SYMBOL;
                sprintf(buf,"%.1f %%",pv_to_cons/from_pv*100);
                cmd_resp += buf;
                cmd_resp += "\r\n\n";
            }

            if (from_grid) {
                cmd_resp += "                ";
                cmd_resp += ESS_CABLE_SYMBOL;
                cmd_resp += GRID_FLOW_SYMBOL[2];
                cmd_resp += ESS_SYMBOL;
                sprintf(buf,"%.1f %%",grid_to_ess/from_grid*100);
                cmd_resp += buf;
                cmd_resp += "\r\n  ";
                cmd_resp += HOUSE_SYMBOL;
                cmd_resp += "\r\n";
                sprintf(buf,"%7.3f",from_grid/1000);
                cmd_resp += buf;
                cmd_resp += GRID_SYMBOL;
                cmd_resp += GRID_FLOW_SYMBOL[2];
                cmd_resp += GRID_CABLE_SYMBOL;
                cmd_resp += CONS_CABLE_SYMBOL;
                cmd_resp += GRID_FLOW_SYMBOL[2];
                cmd_resp += CONS_SYMBOL;
                sprintf(buf,"%.1f %%",grid_to_cons/from_grid*100);
                cmd_resp += buf;
                cmd_resp += "\r\n\n";
            }
   
            if (from_ess) {
                cmd_resp += "                ";
                cmd_resp += ESS_CABLE_SYMBOL;
                cmd_resp += ESS_FLOW_SYMBOL[1];
                cmd_resp += ESS_SYMBOL;
                sprintf(buf,"%.3f\r\n  ",from_ess/1000);
                cmd_resp += buf;
                cmd_resp += HOUSE_SYMBOL;
                cmd_resp += "\r\n";
                sprintf(buf,"%5.1f %%",ess_to_grid/from_ess*100);
                cmd_resp += buf;
                cmd_resp += GRID_SYMBOL;
                cmd_resp += ESS_FLOW_SYMBOL[1];
                cmd_resp += GRID_CABLE_SYMBOL;
                cmd_resp += CONS_CABLE_SYMBOL;
                cmd_resp += ESS_FLOW_SYMBOL[2];
                cmd_resp += CONS_SYMBOL;
                sprintf(buf,"%.1f %%",ess_to_cons/from_ess*100);
                cmd_resp += buf;
                // ESS efficiency
                cmd_resp += "\r\n\nFrom/to ESS : ";
                cmd_resp += String(from_ess/1000,3);
                cmd_resp += "/";
                cmd_resp += String(to_ess/1000,3);
                if (from_ess <= to_ess) {
                    sprintf(buf," (%.1f %%)",from_ess/to_ess*100);
                    cmd_resp += buf;
                }
                cmd_resp += "\r\n";
            }
            // PV energy consumed in household (directly or via ESS)
            cmd_resp += "PV cons/prod: ";
            cmd_resp += String(pv_consumed/1000,3);
            cmd_resp += "/";
            cmd_resp += String(from_pv/1000,3);
            if (pv_consumed <= from_pv) {
                sprintf(buf," (%.1f %%)",pv_consumed/from_pv*100);
                cmd_resp += buf;
            }
            cmd_resp += "\r\n\n";
            break;
        case 'e':
            cmd_resp = "Errors since ";
            sprintf(buf,"%02d/%02d/%04d %02d:%02d\r\n",day(resettime_errors),month(resettime_errors),year(resettime_errors),hour(resettime_errors),minute(resettime_errors));
            cmd_resp += buf;
            for (int i=0; i<ERROR_TYPES; i++) {
                cmd_resp += ERROR_TYPE[i];
                cmd_resp += "\t";
                cmd_resp += error_counter[i];
                if (error_counter[i]) {
                    cmd_resp += " (last: ";
                    sprintf(buf,"%02d/%02d/%04d %02d:%02d)",day(errortime[i]),month(errortime[i]),year(errortime[i]),hour(errortime[i]),minute(errortime[i]));
                    cmd_resp += buf;
                }
                cmd_resp += "\r\n";
            }
            if (last_error_msg != "") {
                cmd_resp += "Last: ";
                cmd_resp += last_error_msg;
                cmd_resp += "\r\n";
            }
            cmd_resp += "\n";
            error_flag = false;
            break;
        case 's':
            cmd_resp = "MW plug ops since ";
            sprintf(buf,"%02d/%02d/%04d %02d:%02d\r\n",day(starttime),month(starttime),year(starttime),hour(starttime),minute(starttime));
            cmd_resp += buf;
            cmd_resp += mw_counter;
            cmd_resp += " (";
            cmd_resp += String(mw_counter/(elapsedDays(local_unixtime-starttime)+1));
            cmd_resp += "/day)\r\n\n";
            break;
        case 'z':
            command = 0;  // respond to command only once
            telnet.print(cycle_msg);
            telnet.print("\r\nEnter [e] or [n] to reset error or energy stats: ");
            ts = millis();
            while ((millis()-ts)/1000 < READCOMMAND_TIMEOUT)
                if (telnet.available()) {
                    input = telnet.readString();
                    if (input[0] == 'e') {
                        for (int i=0; i<ERROR_TYPES; i++) error_counter[i] = errortime[i] = 0;
                        last_error_msg = "";
                        error_flag = false;
                        resettime_errors = local_unixtime;
                        cmd_resp = "Error stats reset to zero\r\n\n";
                        return;
                    }
                    if (input[0] == 'n') {
                        from_pv = pv_to_cons = pv_to_ess = pv_to_grid = 0;  // Reset PV energy counters
                        ShellyCommand(PM_RESET);
                        from_grid = to_grid = grid_to_cons = grid_to_ess = 0;  // Reset grid energy counters
                        ShellyCommand(EM_RESET);
                        from_ess = to_ess = ess_to_cons = ess_to_grid = 0;  // Reset ESS energy counters
                        ShellyCommand(MWPLUG_RESET);
                        ShellyCommand(HMPLUG_RESET);
                        resettime_energy = local_unixtime;
                        cmd_resp = "Energy stats reset to zero\r\n\n";
                        return;
                    }
                    break;
                }
            cmd_resp = "No stats reset\r\n\n";
            break;
        case 'c':
            command = 0;  // respond to command only once
            cmd_resp = "";
            break;
        case 'r':
            command = 0;  // respond to command only once
            telnet.print(cycle_msg);
            telnet.print("\r\nEnter [y] to confirm system reboot: ");
            ts = millis();
            while ((millis()-ts)/1000 < READCOMMAND_TIMEOUT)
                if (telnet.available()) {
                    if (telnet.readString()[0] == 'y') {
                        telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
                        delay(3000);
                        telnet.stop();
                        delay(500);
                        ESP.restart();
                    }
                    break;
                }
            cmd_resp = "System not rebooted\r\n\n";
            break;
        case 'h':
            cmd_resp = "Command options:\r\n";
            cmd_resp += "[m] - Manual power mode\r\n";
            cmd_resp += "[a] - Automatic power mode\r\n";
            cmd_resp += "[g] - Grid power target\r\n";
            cmd_resp += "[b] - Battery info\r\n";
            cmd_resp += "[d] - DDNS info\r\n";
            cmd_resp += "[w] - WiFi RSSI\r\n";
            cmd_resp += "[t] - Time, uptime, astro times\r\n";
            cmd_resp += "[l] - Lowest household consumption\r\n";
            cmd_resp += "[n] - Energy stats\r\n";
            cmd_resp += "[e] - Error stats\r\n";
            cmd_resp += "[s] - Shelly plug counter\r\n";
            cmd_resp += "[z] - Reset stats to zero\r\n";
            cmd_resp += "[c] - Clear command response\r\n";
            cmd_resp += "[r] - Reboot system\r\n\n";
            return;
    }
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

void SetMWPower(float power) {

    unsigned long duty_cycle;
    // translate power setting to PWM value (compensate non-linearity of Meanwell power output in lower power region)
    if (power <= MW_LOW_POWER_THRESHOLD) duty_cycle = MW_LOW_POWER_FORMULA;
    else duty_cycle = round(MW_PWM_FORMULA);
    // // make sure PWM limits are not exceeded (PWM = 0 would lead to excessive power output)
    if (duty_cycle < 1) duty_cycle = 1;
    if (duty_cycle > PWM_DUTY_CYCLE_MAX) duty_cycle = PWM_DUTY_CYCLE_MAX;
    ledcWrite(PWM_CHANNEL, duty_cycle);
    
}

bool HoymilesCommand(float power) {

    while (millis()-ts_HM < 200);  // minimum delay between two consecutive HM commands: 200 ms

    if (!power) {  // turn off Hoymiles
        if (radio.writeBlocking(hm_turnoff, sizeof(hm_turnoff), 1000))
            if (radio.txStandBy(2000)) {
                ts_HM = millis();
                return true;
            }
        // RF24 command failed
        ts_HM = millis();
        error_msg = "Hoymiles RF24 turn off command failed";
        return false;
    }

    if (power > 0) {  // turn on Hoymiles
        if (radio.writeBlocking(hm_turnon, sizeof(hm_turnon), 1000))
            if (radio.txStandBy(2000)) {
                ts_HM = millis();
                return true;
            }
        // RF24 command failed
        ts_HM = millis();
        error_msg = "Hoymiles RF24 turn on command failed";
        return false;
    }

    if (power < 0) {  // set Hoymiles power limit
        // calculate power limit value for Hoymiles RF24 command
        unsigned int limit = round(-10*power);
        if (power >= HM_LOW_POWER_THRESHOLD) limit = round(HM_LOW_POWER_FORMULA);
        if (power < HM_HIGH_POWER_THRESHOLD) limit = round(HM_HIGH_POWER_FORMULA);
        // update RF24 command with calculated power value and CRCs
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

        if (radio.writeBlocking(hm_power, sizeof(hm_power), 1000))
            if (radio.txStandBy(2000)) {
                ts_HM = millis();
                return true;
            }
        // RF24 command failed
        ts_HM = millis();
        error_msg = "Hoymiles RF24 power command failed";
        return false;
    }
}

bool UpdateDDNS() {

    if (WiFi.status() != WL_CONNECTED) {
        error_msg = "WIFI";
        return false;
    }

    if (public_IP != DDNS_address) {  // public IP was changed: update DDNS
        http.begin(DDNS_SERVER_UPDATE + public_IP);
        http_resp_code = http.GET();
        if (http_resp_code == HTTP_CODE_OK) {
            http_resp = http.getString();
            http.end();
            DDNS_address = public_IP;
            DDNS_time = local_unixtime;
            return true;
        }
        http.end();
        error_msg = "Could not update DDNS address (http code ";
        error_msg += http_resp_code;
        error_msg += ")";
        return false;
    }

    if ((millis()-ts_pubip)/1000 < DDNS_UPDATE_INTERVAL) return true;
    // update interval passed: read public IP from server
    http.begin(PUBLIC_IP_SERVER);
    http_resp_code = http.GET();
    if (http_resp_code == HTTP_CODE_OK) {
        public_IP = http.getString();
        http.end();
        ts_pubip = millis();
        pubip_time = local_unixtime;
        return true;
    }
    http.end();
    return false;
}


