const String SW_VERSION = "v2.14";

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

    // Assign LED Pin, turn on LED as welcome signal
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    delay(500);
    digitalWrite(LED_PIN, HIGH);

    // Init WiFi
    WiFi.config(ESP32_ADDR, ROUTER_ADDR, SUBNET, DNS_SERVER1, DNS_SERVER2);
    WiFi.setTxPower(WIFI_POWER_7dBm);
    WiFi.begin(WIFI_SSID, WIFI_PWD);
    while (WiFi.status() != WL_CONNECTED);   // if WiFi unavailable or wrong SSID/PWD, system stops here and LED remains on
    digitalWrite(LED_PIN, LOW);

    // reserve memory for global Strings
    ota_msg.reserve(100);
    error_msg.reserve(200);
    last_error_msg.reserve(200);
    cmd_resp.reserve(700);
    cycle_msg.reserve(1000);
    if (!http_resp.reserve(2400)) error_msg = "String memory allocation failed";
    else error_msg = cmd_resp = "";

    // Init PWM generator (to adjust Meanwell power)
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM_OUTPUT_PIN, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, PWM_DUTY_CYCLE_MAX);
    
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
    cycle_msg = CLEAR_SCREEN;
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
    if (BMSCommand(BMS_SETTINGS, sizeof(BMS_SETTINGS))) telnet.print("RS485 communication with JKBMS OK\r\n");
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
        radio.setAutoAck(true);
        ts_HM = millis();

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

    // Shelly 1PM: Turn off eco mode
    if (ShellyCommand(PM1_ECO_OFF)) telnet.print("Shelly 1PM found\r\n");

    // Shelly 2PM: Turn off Meanwell relay, turn on Hoymiles relay, turn off eco mode
    if (ShellyCommand(PM2_MW_OFF)) telnet.print("Shelly 2PM found\r\n");
    ShellyCommand(PM2_HM_ON);
    ShellyCommand(PM2_ECO_OFF);

    // Shelly 3EM: Erase energy data, read grid power and current time
    if (ShellyCommand(EM_RESET)) telnet.print("Shelly 3EM found\r\n");
    ShellyCommand(EM_STATUS);
    
    // No erros during setup: zero error counters, set timestamps, start polling cycle
    if (error_msg == "") {
        for (int i=0; i<ERROR_TYPES; i++) error_counter[i] = errortime[i] = 0;
        starttime = resettime_errors = resettime_energy = local_unixtime;
        ts_power = millis();
        telnet.print("\r\nNo errors during setup, start polling cycle ...");
        delay(PROCESSING_DELAY*1000);
    }
    else {
        digitalWrite(LED_PIN, HIGH);
        cycle_msg = "\r\n";
        cycle_msg += ERROR_SYMBOL;
        cycle_msg += " ";
        cycle_msg += error_msg;
        cycle_msg += "\r\nSetup error(s) occured, system halted";
        cycle_msg += "\r\n\nPress any key to restart: ";
        telnet.print(cycle_msg);
        while (!telnet.available());
        telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
        delay(3000);
        telnet.stop();
        delay(500);
        ESP.restart();
    }
}

void loop() {

    telnet.print(HIDE_CURSOR);  // Discourage user commands while system is busy
    BMSCommand(BMS_VOLTAGES, sizeof(BMS_VOLTAGES));  // Read cell voltages from BMS
    if (daytime) ShellyCommand(PM1_STATUS);  // Read PV power from Shelly 1PM
    power_ess = power_new;  // Assumption: charging/discharging power equals power setting
    if (power_new) ShellyCommand(PM2_STATUS[power_new < 0]); // Read actual charging/discharging power from Shelly 2PM
    ShellyCommand(EM_STATUS);  // Read time and grid power from Shelly 3EM
    SetNewPower();  // Set power limits, calculate and apply new charging/discharging power
    FinishCycle();  // Update energy counters, compile cycle info, carry out maintenance tasks
    CheckErrors();  // Check errors, halt system if error is persistent, show cycle status by flashing LED
    UserCommand();  // Compile response to user command from previous cycle(s), print cycle info and user command response

    // Read new user command while waiting for power changes to take effect
    while ((millis()-ts_power)/1000 < PROCESSING_DELAY) ReadCommand();
}

bool BMSCommand(const byte command[], int size) {

    // Send command to BMS via RS485
    Serial2.write(command, size);
    Serial2.flush();

    byte BMS_resp[300] = {0};
    int len = 0, sum = 0;

    // Wait for BMS response (max waiting time: 100 ms)
    for (int i=0; i<100; i++) {
        if (Serial2.available()) break;
        delay(1);
    }
    if (command[BMS_COMMAND_POS] == BMS_READ_ALL) delay(20);  // allow some more time to fetch long response
    
    // fill response buffer with BMS response
    while (Serial2.available()) {
        BMS_resp[len] = Serial2.read();
        sum += BMS_resp[len];
        len++;
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

    // Handle HTTP commands for Shellies
    http.begin(URL);
    http_resp_code = http.GET();
    if (http_resp_code == HTTP_CODE_OK) {
        http_resp = http.getString();
        http.end();
        if (URL == EM_STATUS) {
            power_grid = atof(&http_resp[http_resp.indexOf("total_power",700)+13]);  // read grid power from http response
            int index = http_resp.indexOf("\"time\"");  // read local time from http response
            String local_time = http_resp.substring(index+8, index+13);  // local_time includes timezone and DST
            long unixtime = atol(&http_resp[index+26]);  // read epoch time (UTC) from http response
            int UTC_offset = (local_time.toInt()+24-hour(unixtime))%24;  // UTC_offset = timezone_offset + DST_offset
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
        if (URL == PM1_STATUS) {
            power_pv = atof(&http_resp[http_resp.indexOf("apower")+8]);
            return true;
        }
        if ((URL == PM2_STATUS[0]) || (URL == PM2_STATUS[1])) {
            power_ess = atof(&http_resp[http_resp.indexOf("apower")+8]);
            return true;
        }
        if (URL == PM2_MW_ON) {
            ts_MW = millis();
            if (!mw_on) mw_counter++;
            mw_on = true;
            return true;
        }
        if (URL == PM2_MW_OFF) {
            delay(50); ledcWrite(PWM_CHANNEL, 0);  // turns off LED in optocoupler
            if (mw_on) mw_counter++;
            mw_on = false;
            return true;
        }
        return true;
    }
    // Shelly command failed
    http.end();
    if (URL.indexOf(EM_ADDR) > -1) error_msg = "3EM";
    if (URL.indexOf(PM1_ADDR) > -1) error_msg = "1PM";
    if (URL.indexOf(PM2_ADDR) > -1) error_msg = "2PM";
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
        if (mw_power_limit == MW_MIN_POWER) mw_power_limit = 0;  // enter OVP mode
        else {
            mw_power_limit = round(power_old*POWER_LIMIT_RAMPDOWN);  // ramp down charging power limit softly
            if (mw_power_limit < MW_MIN_POWER) mw_power_limit = MW_MIN_POWER;  // last power limit before OVP mode
        }
    }

    // Update UVP max discharging power, depending on min cell voltage
    if (vcell_min >= ESS_UVPR) hm_power_limit = HM_MAX_POWER;  // exit UVP mode
    if (vcell_min <= ESS_UVP) {
        if (hm_power_limit == HM_MIN_POWER) hm_power_limit = 0;  // enter UVP mode
        else {
            hm_power_limit = round(power_old*POWER_LIMIT_RAMPDOWN);  // ramp up discharging power limit softly
            if (hm_power_limit > HM_MIN_POWER) hm_power_limit = HM_MIN_POWER;  // last power limit before UVP mode
        }
    }

    // Turn on/off automatic recharge feature (prevents BMS Cell UVP shutdown)
    if (vcell_min <= (bms_uvp + ESS_UVP)/2) {
        auto_recharge = true;
        if (pm2_eco_mode) pm2_eco_mode = !ShellyCommand(PM2_ECO_OFF);
    }
    if (vcell_min >= ESS_UVP) auto_recharge = false;

    // Calculate new power setting
    power_new = power_old - int(round(power_grid)) + power_target;
    if (abs(power_new - power_old) <= POWER_TARGET_DEVIATION) power_new = power_old;

    // Filter out power spikes and slowly increase discharging power (reduces power loss to grid when consumer is suddenly turned  off)
    rampdown = false;
    if (max(power_new, hm_power_limit) - power_old < POWER_RAMPDOWN_RATE) {
        filter_cycles = max(filter_cycles-1, 0);  // countdown filter cycles
        if (filter_cycles) power_new = power_old;  // filter out power spikes
        else {
            power_new = power_old + POWER_RAMPDOWN_RATE;  // ramp down ESS power after filtering out power spikes
            if ((power_new == POWER_RAMPDOWN_RATE) && (HM_MIN_POWER < POWER_RAMPDOWN_RATE)) power_new = HM_MIN_POWER;  // make sure the rampdown doesn't get stuck above HM_MIN_POWER
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
                SetMWPower(MW_MIN_POWER); delay(100);  // prolongs relay lifetime
                if (!ShellyCommand(PM2_MW_OFF)) power_new = MW_MIN_POWER;
        }
        if (power_new > 0) {  // set new charging power, (if necessary) turn discharging off, turn charging on
            if (power_old > 0) SetMWPower(power_new);
            if (power_old == 0) {
                SetMWPower(power_new);
                if (!ShellyCommand(PM2_MW_ON)) power_new = 0;
            }
            if (power_old < 0) {
                if (!HoymilesCommand(HM_OFF)) power_new = power_old;
                else {
                    SetMWPower(power_new);
                    if (!ShellyCommand(PM2_MW_ON)) power_new = 0;
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
                SetMWPower(MW_MIN_POWER); delay(100);  // prolongs relay lifetime
                if (!ShellyCommand(PM2_MW_OFF)) power_new = MW_MIN_POWER;
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
    
    // Calculate power flows that aren't measured by Shellies
    float power_cons = power_pv - power_ess + power_grid;
    float power_from_grid = (power_grid > 0) * power_grid;
    float power_to_grid = (power_grid < 0) * -power_grid;
    float power_from_ess = (power_ess < 0) * -power_ess;
    float power_to_ess = (power_ess > 0) * power_ess;
    float power_pv_to_ess = min(power_to_ess, power_pv);
    float power_pv_to_cons = min(power_cons, power_pv - power_pv_to_ess);
    float power_pv_to_grid = power_pv - power_pv_to_cons - power_pv_to_ess;
    float power_ess_to_cons = min(power_cons - power_pv_to_cons, power_from_ess);
    float power_ess_to_grid = power_from_ess - power_ess_to_cons;
    float power_grid_to_ess = power_to_ess - power_pv_to_ess;
    float power_grid_to_cons = power_from_grid - power_grid_to_ess;

    // Update energy counters
    float hrs_cycle = secs_cycle/3600;
    en_to_cons += hrs_cycle*power_cons;
    en_from_pv += hrs_cycle*power_pv;
    en_from_ess += hrs_cycle*power_from_ess;
    en_to_ess += hrs_cycle*power_to_ess;
    en_from_grid += hrs_cycle*power_from_grid;
    en_to_grid += hrs_cycle*power_to_grid;

    en_pv_to_cons += hrs_cycle*power_pv_to_cons;
    en_pv_to_ess += hrs_cycle*power_pv_to_ess;
    en_pv_to_grid = en_from_pv - en_pv_to_cons - en_pv_to_ess;
    en_grid_to_cons += hrs_cycle*power_grid_to_cons;
    en_grid_to_ess += hrs_cycle*power_grid_to_ess;
    en_ess_to_cons += hrs_cycle*power_ess_to_cons;
    en_ess_to_grid = en_from_ess - en_ess_to_cons;
    en_pv_consumed = en_to_cons - en_grid_to_cons;  // PV energy consumed by household, directly or via ESS
    

    // Toggle cycle flag (for moving energy flow arrows)
    odd_cycle = !odd_cycle;

    // Compile cycle message
    cycle_msg = CLEAR_SCREEN;
    cycle_msg += SHOW_CURSOR;
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
    cycle_msg += OPS_SYMBOL[!power_old];
    cycle_msg += "\r\n";

    // OVP limit symbol
    if (mw_power_limit < mw_max_power) cycle_msg += OVP_LIMIT_SYMBOL;
    cycle_msg += "\r\n";
    
    // PV power
    sprintf(buf,"%4.0f",power_pv);
    cycle_msg += buf;
    if (!daytime) cycle_msg += MOON_SYMBOL[(telnet.remoteIP()==WIN_CLIENT_ADDR)];
    else {
        int pv_level = min(int(round(power_pv))*PV_LEVELS/PV_MAX_POWER, PV_LEVELS-1);
        cycle_msg += PV_LEVEL_SYMBOL[2*pv_level+(telnet.remoteIP()==WIN_CLIENT_ADDR)];
    }
    if (odd_cycle) cycle_msg += CABLE_SYMBOL;
    cycle_msg += PV_FLOW_SYMBOL[2*(int(round(power_pv))>0)];
    if (!odd_cycle) cycle_msg += CABLE_SYMBOL;
    cycle_msg += PV_CABLE_SYMBOL;

    // ESS power
    cycle_msg += ESS_CABLE_SYMBOL;
    if (odd_cycle) cycle_msg += CABLE_SYMBOL;
    if (int(round(power_ess)) > 0) {
        if (power_grid_to_ess > power_pv_to_ess) cycle_msg += GRID_FLOW_SYMBOL[2];
        else cycle_msg += PV_FLOW_SYMBOL[2];
    }
    else  cycle_msg += ESS_FLOW_SYMBOL[(power_ess < 0)];
    if (!odd_cycle) cycle_msg += CABLE_SYMBOL;
    if (vbat > ESS_EMPTY) cycle_msg += ESS_LEVEL_SYMBOL[min(((vbat-ESS_EMPTY)*(ESS_LEVELS-2))/(ESS_FULL-ESS_EMPTY)+1,ESS_LEVELS-1)];
    else cycle_msg += ESS_LEVEL_SYMBOL[0];
    cycle_msg += int(round(power_ess));
    if (power_new != power_old) {
        cycle_msg += DIFF_SYMBOL[(power_new > power_old)];
        cycle_msg += abs(power_new - power_old);
        if (rampdown) cycle_msg += RAMPDOWN_SYMBOL;
    }
    else if (filter_cycles && (filter_cycles < POWER_FILTER_CYCLES)) cycle_msg += POWERFILTER_SYMBOL[filter_cycles%12];
    cycle_msg += MODE_SYMBOL[manual_mode + 2*auto_recharge];
    cycle_msg += "\r\n ";

    // House symbol, UVP limit symbol
    cycle_msg += HOUSE_SYMBOL;
    if (hm_power_limit > HM_MAX_POWER) cycle_msg += UVP_LIMIT_SYMBOL;
    cycle_msg += "\r\n";

    // Grid power
    sprintf(buf,"%4.0f",power_grid);
    cycle_msg += buf;
    cycle_msg += GRID_SYMBOL;
    cycle_msg += CABLE_SYMBOL;
    if (!odd_cycle) cycle_msg += CABLE_SYMBOL;
    if (int(round(power_grid)) < 0) {
        if (power_ess_to_grid > power_pv_to_grid) cycle_msg += ESS_FLOW_SYMBOL[1];
        else cycle_msg += PV_FLOW_SYMBOL[1];
    }
    else  cycle_msg += GRID_FLOW_SYMBOL[2*(int(round(power_grid))>0)];
    if (odd_cycle) cycle_msg += CABLE_SYMBOL;
    cycle_msg += GRID_CABLE_SYMBOL;

    // Power consumption
    cycle_msg += CONS_CABLE_SYMBOL;
    if (!odd_cycle) cycle_msg += CABLE_SYMBOL;
    if (power_grid_to_cons > max(power_pv_to_cons, power_ess_to_cons)) cycle_msg += GRID_FLOW_SYMBOL[2];
    else {
        if (power_ess_to_cons > power_pv_to_cons) cycle_msg += ESS_FLOW_SYMBOL[2];
        else cycle_msg += PV_FLOW_SYMBOL[2];
    }
    if (odd_cycle) cycle_msg += CABLE_SYMBOL;
    cycle_msg += CABLE_SYMBOL;
    cycle_msg += CONS_SYMBOL;
    cycle_msg += int(round(power_cons));
    cycle_msg += "\r\n\n";
    
    // Append error info (if error occured)
    if (error_msg != "") {
        cycle_msg += ERROR_SYMBOL;
        cycle_msg += " ";
        cycle_msg += error_msg;
        cycle_msg += "\r\n\n";
    }

    // Keep alive Hoymiles inverter
    if ((millis()-ts_HM)/1000 >= HOYMILES_KEEPALIVE) HoymilesCommand(power_new < 0);

    // Keep alive Meanwell relay (if Meanwell is turned on)
    if (mw_on && ((millis()-ts_MW)/1000 >= MW_TIMER-30)) ShellyCommand(PM2_MW_ON);

    // ESS is asleep: check for new min grid power (min household consumption)
    if (pm2_eco_mode && (power_grid < power_grid_min)) {
        power_grid_min = power_grid;
        minpower_time = local_unixtime;
    }

    // Set Shelly 1PM eco mode (turn off at daytime, turn on at nighttime)
    if (daytime && pm1_eco_mode) pm1_eco_mode = !ShellyCommand(PM1_ECO_OFF);
    if (!daytime && !pm1_eco_mode) {
        pm1_eco_mode = ShellyCommand(PM1_ECO_ON);
        ShellyCommand(EM_RESET);  // Reset Shelly 3EM energy data (prevents HTTP timeouts during data reorgs)
    }

    // Set Shelly 2PM eco mode (turn off when charging/discharging active or possible, turn on when charging/discharging not possible)
    if (((power_pv > MW_MIN_POWER) || power_new) && pm2_eco_mode) pm2_eco_mode = !ShellyCommand(PM2_ECO_OFF);
    if (!hm_power_limit && (power_pv < 1) && !power_old && !pm2_eco_mode) pm2_eco_mode = ShellyCommand(PM2_ECO_ON);

    // Check if public IP address was changed, update DDNS server entry
    UpdateDDNS();
    return;
}

void CheckErrors() {

    if (error_msg == "") {
        errors_consecutive = 0;
        digitalWrite(LED_PIN, HIGH); delay(20); digitalWrite(LED_PIN, LOW);  // no error: flash LED briefly
        return;
    }

    error_flag = true;  // new unread error
    digitalWrite(LED_PIN, HIGH);  // error: keep LED on during next cycle
 
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
    SetMWPower(MW_MIN_POWER); delay(100);
    ShellyCommand(PM2_MW_OFF);
    ShellyCommand(PM2_HM_OFF);
    hm_power_limit = power_grid = power_new = power_old = 0;
    cycle_msg = CLEAR_SCREEN;
    cycle_msg += SHOW_CURSOR;
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
    command = 0;
    while (true) {
        telnet.print(cycle_msg);
        UpdateDDNS();  // keep updating DDNS in order to be reachable via Internet
        ts_power = millis();
        while ((millis()-ts_power)/1000 < PROCESSING_DELAY) if (ReadCommand()) break;
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
    if (!telnet) telnet = server.available();  // new telnet session?
    if (!telnet.available()) return false;  // user command available?
    command = telnet.readString()[0];  // read user command (discard everything after first character)
    return true;
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
            command = 0;  // respond to command only once, skip telnet output at the end of 
            cycle_msg += "Enter ESS power (negative for discharging): ";
            telnet.print(cycle_msg);
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
                        if (power_manual && pm2_eco_mode) pm2_eco_mode = !ShellyCommand(PM2_ECO_OFF);
                        return;
                    }
                    break;
                }
            if (!manual_mode) cmd_resp = "Automatic mode remains activated\r\n\n";
            else {
                cmd_resp = "ESS power remains at ";
                cmd_resp += power_manual;
                cmd_resp += " W\r\n\n";
            }
            return;
        case 'a':
            command = 0;  // respond to command only once
            manual_mode = false;
            cmd_resp = "Automatic mode activated\r\n\n";
            break;
        case 'g':
            command = 0;  // respond to command only once
            cycle_msg += "Enter grid power target: ";
            telnet.print(cycle_msg);
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
            return;
        case 'b':
            BMSCommand(BMS_CURRENT, sizeof(BMS_CURRENT));  // Read battery current and DC power from BMS
            cmd_resp = "Batt power DC   : ";
            cmd_resp += String(pbat,1);
            cmd_resp += " W\r\nAC/DC efficiency: ";
            if ((power_ess > 0) && (pbat > 0)) cmd_resp += String(pbat/power_ess*100,1);
                else if ((power_ess < 0) && (pbat < 0)) cmd_resp += String(power_ess/pbat*100,1);
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
                sprintf(buf,"%.1f W (measured ",power_grid_min);
                cmd_resp += buf;
                sprintf(buf,"%02d/%02d/%04d %02d:%02d)\r\n\n",day(minpower_time),month(minpower_time),year(minpower_time),hour(minpower_time),minute(minpower_time));
                cmd_resp += buf;
            }
            break;
        case 'n':
            cmd_resp = "Energy flows [kWh] since ";
            sprintf(buf,"%02d/%02d/%04d %02d:%02d\r\n\n",day(resettime_energy),month(resettime_energy),year(resettime_energy),hour(resettime_energy),minute(resettime_energy));
            cmd_resp += buf;

            if (en_from_pv) {
                sprintf(buf,"%7.3f",en_from_pv/1000);
                cmd_resp += buf;
                cmd_resp += SUN_SYMBOL[(telnet.remoteIP()==WIN_CLIENT_ADDR)];
                cmd_resp += PV_FLOW_SYMBOL[2];
                cmd_resp += PV_CABLE_SYMBOL;
                cmd_resp += ESS_CABLE_SYMBOL;
                cmd_resp += PV_FLOW_SYMBOL[2];
                cmd_resp += ESS_SYMBOL;
                sprintf(buf,"%.1f %%",en_pv_to_ess/en_from_pv*100);
                cmd_resp += buf;
                cmd_resp += "\r\n  ";
                cmd_resp += HOUSE_SYMBOL;
                cmd_resp += "\r\n";
                sprintf(buf,"%5.1f %%",en_pv_to_grid/en_from_pv*100);
                cmd_resp += buf;
                cmd_resp += GRID_SYMBOL;
                cmd_resp += PV_FLOW_SYMBOL[1];
                cmd_resp += GRID_CABLE_SYMBOL;
                cmd_resp += CONS_CABLE_SYMBOL;
                cmd_resp += PV_FLOW_SYMBOL[2];
                cmd_resp += CONS_SYMBOL;
                sprintf(buf,"%.1f %%",en_pv_to_cons/en_from_pv*100);
                cmd_resp += buf;
                cmd_resp += "\r\n\n";
            }

            if (en_from_grid) {
                cmd_resp += "                ";
                cmd_resp += ESS_CABLE_SYMBOL;
                cmd_resp += GRID_FLOW_SYMBOL[2];
                cmd_resp += ESS_SYMBOL;
                sprintf(buf,"%.1f %%",en_grid_to_ess/en_from_grid*100);
                cmd_resp += buf;
                cmd_resp += "\r\n  ";
                cmd_resp += HOUSE_SYMBOL;
                cmd_resp += "\r\n";
                sprintf(buf,"%7.3f",en_from_grid/1000);
                cmd_resp += buf;
                cmd_resp += GRID_SYMBOL;
                cmd_resp += GRID_FLOW_SYMBOL[2];
                cmd_resp += GRID_CABLE_SYMBOL;
                cmd_resp += CONS_CABLE_SYMBOL;
                cmd_resp += GRID_FLOW_SYMBOL[2];
                cmd_resp += CONS_SYMBOL;
                sprintf(buf,"%.1f %%",en_grid_to_cons/en_from_grid*100);
                cmd_resp += buf;
                cmd_resp += "\r\n\n";
            }
   
            if (en_from_ess) {
                cmd_resp += "                ";
                cmd_resp += ESS_CABLE_SYMBOL;
                cmd_resp += ESS_FLOW_SYMBOL[1];
                cmd_resp += ESS_SYMBOL;
                sprintf(buf,"%.3f\r\n  ",en_from_ess/1000);
                cmd_resp += buf;
                cmd_resp += HOUSE_SYMBOL;
                cmd_resp += "\r\n";
                sprintf(buf,"%5.1f %%",en_ess_to_grid/en_from_ess*100);
                cmd_resp += buf;
                cmd_resp += GRID_SYMBOL;
                cmd_resp += ESS_FLOW_SYMBOL[1];
                cmd_resp += GRID_CABLE_SYMBOL;
                cmd_resp += CONS_CABLE_SYMBOL;
                cmd_resp += ESS_FLOW_SYMBOL[2];
                cmd_resp += CONS_SYMBOL;
                sprintf(buf,"%.1f %%",en_ess_to_cons/en_from_ess*100);
                cmd_resp += buf;
                // ESS efficiency
                cmd_resp += "\r\n\nFrom/to ESS : ";
                cmd_resp += String(en_from_ess/1000,3);
                cmd_resp += "/";
                cmd_resp += String(en_to_ess/1000,3);
                if (en_from_ess <= en_to_ess) {
                    sprintf(buf," (%.1f %%)",en_from_ess/en_to_ess*100);
                    cmd_resp += buf;
                }
                cmd_resp += "\r\n";
            }
            // PV energy consumed in household (directly or via ESS)
            cmd_resp += "PV cons/prod: ";
            cmd_resp += String(en_pv_consumed/1000,3);
            cmd_resp += "/";
            cmd_resp += String(en_from_pv/1000,3);
            if (en_pv_consumed <= en_from_pv) {
                sprintf(buf," (%.1f %%)",en_pv_consumed/en_from_pv*100);
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
            cmd_resp = "MW relay ops since ";
            sprintf(buf,"%02d/%02d/%04d %02d:%02d\r\n",day(starttime),month(starttime),year(starttime),hour(starttime),minute(starttime));
            cmd_resp += buf;
            cmd_resp += mw_counter;
            cmd_resp += " (";
            cmd_resp += String(mw_counter/(elapsedDays(local_unixtime-starttime)+1));
            cmd_resp += "/day)\r\n\n";
            break;
        case 'z':
            command = 0;  // respond to command only once
            cycle_msg += "Enter [e] or [n] to reset error or energy stats: ";
            telnet.print(cycle_msg);
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
                        en_from_pv = en_pv_to_cons = en_pv_to_ess = en_pv_to_grid = 0;  // Reset PV energy counters
                        en_from_grid = en_to_grid = en_grid_to_cons = en_grid_to_ess = 0;  // Reset grid energy counters
                        en_from_ess = en_to_ess = en_ess_to_cons = en_ess_to_grid = 0;  // Reset ESS energy counters
                        resettime_energy = local_unixtime;
                        cmd_resp = "Energy stats reset to zero\r\n\n";
                        return;
                    }
                    break;
                }
            cmd_resp = "No stats reset\r\n\n";
            return;
        case 'c':
            command = 0;  // respond to command only once
            cmd_resp = "";
            break;
        case 'r':
            command = 0;  // respond to command only once
            cycle_msg += "Enter [y] to confirm system reboot: ";
            telnet.print(cycle_msg);
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
            return;
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
            cmd_resp += "[s] - Shelly relay counter\r\n";
            cmd_resp += "[z] - Reset stats to zero\r\n";
            cmd_resp += "[c] - Clear command response\r\n";
            cmd_resp += "[r] - Reboot system\r\n\n";
            break;
    }

    // Print cycle message and user command response
    cycle_msg += cmd_resp;
    cycle_msg += "Enter command or [h] for help: ";
    telnet.print(cycle_msg);
}

void SetMWPower(float power) {

    unsigned long duty_cycle;
    // translate power setting to PWM value (compensate non-linearity of Meanwell power output in lower power region)
    if (power <= MW_LOW_POWER_THRESHOLD) duty_cycle = MW_LOW_POWER_FORMULA;
    else duty_cycle = round(MW_POWER_FORMULA);
    // // make sure PWM limits are not exceeded
    if (duty_cycle < 1) duty_cycle = 1;  // 0 would lead to excessive power output
    if (duty_cycle > PWM_DUTY_CYCLE_MAX) duty_cycle = PWM_DUTY_CYCLE_MAX;
    ledcWrite(PWM_CHANNEL, duty_cycle);
    
}

bool HoymilesCommand(int power) {

    while (millis()-ts_HM < 50);  // minimum delay between two consecutive HM commands: 50 ms
    if (power == HM_OFF) {  // turn off Hoymiles
        if (radio.writeFast(hm_turnoff, sizeof(hm_turnoff)))
            if (radio.txStandBy(RF24_TIMEOUT*1000)) {
                ts_HM = millis();
                return true;
            }
        // RF24 command failed
        ts_HM = millis();
        error_msg = "Hoymiles RF24 turnoff command failed";
        return false;
    }

    if (power == HM_ON) {  // turn on Hoymiles
        if (radio.writeFast(hm_turnon, sizeof(hm_turnon)))
            if (radio.txStandBy(RF24_TIMEOUT*1000)) {
                ts_HM = millis();
                return true;
            }
        // RF24 command failed
        ts_HM = millis();
        error_msg = "Hoymiles RF24 turnon command failed";
        return false;
    }

    // set Hoymiles power limit
    // calculate power limit value for Hoymiles RF24 command
    unsigned int limit = round(HM_POWER_FORMULA);
    if (power > HM_LOW_POWER_THRESHOLD) limit = round(HM_LOW_POWER_FORMULA);
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

    if (radio.writeFast(hm_power, sizeof(hm_power)))
        if (radio.txStandBy(RF24_TIMEOUT*1000)) {
            ts_HM = millis();
            return true;
        }
    // RF24 command failed
    ts_HM = millis();
    error_msg = "Hoymiles RF24 power command failed";
    return false;
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


