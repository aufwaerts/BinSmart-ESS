const String SW_VERSION = "v2.32";

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

    // Start OTA software update service
    ota_msg.reserve(100);
    ota_msg = "BinSmart ESS ";
    ota_msg += SW_VERSION;
    ota_msg += "\r\nEnter \"";
    ota_msg += ESP32_ADDR.toString();
    ota_msg += "/update\" for OTA firmware update";
    OTAserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { request->send(200, "text/plain", ota_msg); });
    AsyncElegantOTA.begin(&OTAserver);
    OTAserver.begin();

    // Start telnet server and wait for terminal to connect
    server.begin();
    while (!telnet) telnet = server.available();

    // reserve memory for global Strings
    input_str.reserve(50);
    error_msg.reserve(200);
    last_error_msg.reserve(200);
    cmd_resp.reserve(700);
    cycle_msg.reserve(1000);
    if (!http_resp.reserve(2400)) {
        error_msg = "String memory allocation failed";
        telnet.println(ERROR_SYMBOL + error_msg);
    }
    else input_str = error_msg = last_error_msg = cmd_resp = "";

    // Print startup cycle_msg
    cycle_msg = CLEAR_SCREEN;
    cycle_msg += HIDE_CURSOR;
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
    telnet.println(cycle_msg);
    delay(1000);

    // Init PWM generator (for adjusting Meanwell power)
    if (!ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION)) {
        error_msg = "PWM generator init failed";
        telnet.println(ERROR_SYMBOL + error_msg);
    }
    else {
        ledcAttachPin(PWM_OUTPUT_PIN, PWM_CHANNEL);
        ledcWrite(PWM_CHANNEL, PWM_DUTY_CYCLE_MAX);
        telnet.println("PWM generator initialized");
    }
    delay(1000);
    
    // Init RS485 communication with BMS, read OVP/UVP/balancer settings and cell voltages
    Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    if (!BMSCommand(READ_SETTINGS)) telnet.println(ERROR_SYMBOL + error_msg);
    else telnet.println("RS485 communication with JKBMS OK");
    if (!BMSCommand(READ_VOLTAGES)) telnet.println(ERROR_SYMBOL + error_msg);
    delay(1000);

    // Init BLE communication with BMS, turn off balancer switch
    NimBLEDevice::init("");
    if (!BMSCommand(BAL_OFF)) telnet.println(ERROR_SYMBOL + error_msg);
    else telnet.println("BLE communication with JKBMS OK");
    delay(1000);

    // Init RF24 radio communication with Hoymiles
    radio.begin();
    radio.setPALevel(RF24_PALEVEL);
    radio.setChannel(RF24_CHANNEL);
    radio.setDataRate(RF24_250KBPS);
    radio.setCRCLength(RF24_CRC_16);
    radio.setAddressWidth(sizeof(HM_RADIO_ID));
    radio.enableDynamicPayloads();
    radio.openWritingPipe(HM_RADIO_ID);
    radio.setAutoAck(true);
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
    // Turn off Hoymiles DC side
    if (!HoymilesCommand(HM_OFF)) telnet.println(ERROR_SYMBOL + error_msg);
    else telnet.println("RF24 communication with Hoymiles OK");
    delay(1000);

    // Set timeout for http requests
    http.setTimeout(HTTP_TIMEOUT);

    // Shelly 1PM: Turn off eco mode
    if (!ShellyCommand(PM1_ECO_OFF)) telnet.println(ERROR_SYMBOL + error_msg);
    else telnet.println("Shelly 1PM found");

    // Shelly 2PM: Turn off Meanwell relay, turn on Hoymiles relay, turn off eco mode
    if (!ShellyCommand(PM2_MW_OFF)) telnet.println(ERROR_SYMBOL + error_msg);
    else telnet.println("Shelly 2PM found"); 
    if (!ShellyCommand(PM2_HM_ON)) telnet.println(ERROR_SYMBOL + error_msg);
    if (!ShellyCommand(PM2_ECO_OFF)) telnet.println(ERROR_SYMBOL + error_msg);

    // Shelly 3EM: Erase energy data, read grid power and current time
    if (!ShellyCommand(EM_RESET)) telnet.println(ERROR_SYMBOL + error_msg);
    else telnet.println("Shelly 3EM found");
    if (!ShellyCommand(EM_STATUS)) telnet.println(ERROR_SYMBOL + error_msg);

    // No erros during setup: zero error counters, set timestamps, start polling cycle
    if (error_msg == "") {
        for (int i=0; i<ERROR_TYPES; i++) error_counter[i] = errortime[i] = 0;
        starttime = resettime_errors = resettime_energy = local_unixtime;
        ts_power = millis();
        telnet.print("\nNo errors during setup, start polling cycle ...");
        delay(PROCESSING_DELAY);
    }
    else {
        digitalWrite(LED_PIN, HIGH);
        telnet.println("\nSetup error(s) occured, system halted");
        telnet.print("\nPress any key to restart: ");
        telnet.print(SHOW_CURSOR);
        while (!telnet.available());
        telnet.print(HIDE_CURSOR);
        telnet.flush();
        telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
        delay(3000);
        telnet.stop();
        delay(500);
        ESP.restart();
    }
}

void loop() {

    if (daytime) ShellyCommand(PM1_STATUS);  // Read AC PV power from Shelly 1PM
    BMSCommand(READ_VOLTAGES);  // Read cell voltages from BMS
    if (power_new > 0) ShellyCommand(PM2_MW_STATUS); // Read Meanwell AC charging power from Shelly 2PM
    if (power_new < 0) ShellyCommand(PM2_HM_STATUS); // Read Hoymiles AC discharging power from Shelly 2PM
    ShellyCommand(EM_STATUS);  // Read time and AC grid power from Shelly 3EM
    BMSCommand(READ_CURRENT);  // Read DC charging/discharging current from BMS
    SetNewPower();  // Set power limits, calculate and apply new charging/discharging power
    FinishCycle();  // Update energy counters, compile cycle info, carry out maintenance tasks
    CheckErrors();  // Check errors, halt system if error is persistent, show cycle status by flashing LED
    ProcessUserCommand();  // Process user command from previous cycle(s), print cycle info and user command response
    ReadCommand();  // Read user command while waiting for power changes to take effect
}

bool BMSCommand(const byte command[]) {

    if (command[0] == BLE_1) {
        // Send command to BMS via BLE
        if (!pClient) {
            pClient = NimBLEDevice::createClient(serverAddress);
            pClient->setConnectionParams(12, 12, 0, 12);
            pClient->setConnectTimeout(BLE_TIMEOUT);
        }
        pClient->connect();

        if (pClient->isConnected()) {
            NimBLERemoteService* pService = pClient->getService("FFE0");
            NimBLERemoteCharacteristic* pChar = pService->getCharacteristic("FFE1");
            if (pChar->writeValue(GET_INFO, BLE_COMMAND_LEN, false)) {
                delay(500);
                if (pChar->writeValue(GET_DATA, BLE_COMMAND_LEN, false)) {
                    delay(500);
                    if (pChar->writeValue(command, BLE_COMMAND_LEN, false)) {
                        pClient->disconnect();
                        return true;
                    }
                }
            }
            pClient->disconnect();
        }
        error_msg = "BMS BLE command 0x";
        error_msg += String(command[BLE_COMMAND_POS],HEX);
        error_msg += " failed";
        return false;
    }

    // Send command to BMS via RS485
    while (millis()-ts_BMS < 100);  // minimum delay between RS485 packets: 100 ms
    Serial2.write(command, RS485_COMMAND_LEN);
    Serial2.flush();

    BMS_resp[0] = 0;  // will result in failed validity check if no response
    int len = 0, sum = 0;

    // Wait for BMS response (max waiting time: 100 ms)
    for (int i=0; i<100; i++) {
        if (Serial2.available()) break;
        delay(1);
    }
    if (command[RS485_COMMAND_POS] == READ_ALL) delay(20);  // allow some more time to fetch long response
    
    // fill response buffer with BMS response
    while (Serial2.available()) {
        BMS_resp[len] = Serial2.read();
        sum += BMS_resp[len];
        len++;
    }
    ts_BMS = millis();

    // check validity of BMS response
    if ((BMS_resp[0] == RS485_1) && (BMS_resp[1] == RS485_2)) {  // start of response frame
         if ((BMS_resp[2]<<8|BMS_resp[3]) == len-2) {  // response length
             if ((BMS_resp[len-2]<<8|BMS_resp[len-1]) == sum-BMS_resp[len-2]-BMS_resp[len-1]) {  // response checksum

                // process BMS response
                if (BMS_resp[RS485_COMMAND_POS] == READ_DATA) {
                    if (BMS_resp[DATA_ID_POS] == VCELLS_ID) {
                        // read voltages of battery cells, determine min and max voltages
                        vbat = vcell_max = 0; vcell_min = 4000;
                        for (int vcells_pos = DATA_ID_POS+3; vcells_pos <= DATA_ID_POS+BMS_resp[DATA_ID_POS+1]; vcells_pos+=3) {
                            int vcell = BMS_resp[vcells_pos]<<8|BMS_resp[vcells_pos+1];
                            if (vcell < vcell_min) vcell_min = vcell;
                            if (vcell > vcell_max) vcell_max = vcell;
                            vbat += vcell;
                        }
                        // Update charging power limit (depending on vcell_max)
                        if ((vcell_max <= ESS_OVPR) || (mw_power_limit >= MW_MAX_POWER)) mw_power_limit = round(MW_POWER_LIMIT_FORMULA);  // exit OVP mode (update MW power limit)
                        if (vcell_max >= ESS_OVP) {  // OVP cell voltage reached: reduce MW power limit
                            if (mw_power_limit == MW_MIN_POWER) mw_power_limit = 1;  // enter OVP mode
                            else {
                                mw_power_limit = round(power_new*POWER_LIMIT_RAMPDOWN);  // ramp down charging power limit softly
                                if (!mw_power_limit) mw_power_limit = 1;  // applies if vcell_max >= ESS_OVP when ESS was started
                                if ((mw_power_limit < MW_MIN_POWER) && (mw_power_limit > 1)) mw_power_limit = MW_MIN_POWER;  // always last power limit before OVP
                            }
                        }
                        // Update discharging power limit (depending on vcell_min)
                        if (vcell_min >= ESS_UVPR) hm_power_limit = HM_MAX_POWER;  // exit UVP mode (reset HM power limit)
                        if (vcell_min <= ESS_UVP) {  // UVP cell voltage reached: reduce HM power limit
                            if (hm_power_limit == HM_MIN_POWER) hm_power_limit = -1;  // enter UVP mode
                            else {
                                hm_power_limit = round(power_new*POWER_LIMIT_RAMPDOWN);  // ramp up discharging power limit softly
                                if (!hm_power_limit) hm_power_limit = -1;  // applies if vcell_min <= ESS_UVP when ESS was started
                                if ((hm_power_limit > HM_MIN_POWER) && (hm_power_limit < -1)) hm_power_limit = HM_MIN_POWER;  // always last power limit before UVP
                            }
                        }
                        return true;
                    }
                    
                    if (BMS_resp[DATA_ID_POS] == CURRENT_ID) {
                        // Read charging/discharging DC current, calculate DC power
                        cbat = (BMS_resp[DATA_ID_POS+1]&0x7F)<<8|BMS_resp[DATA_ID_POS+2];
                        if (!(BMS_resp[DATA_ID_POS+1]&0x80)) cbat = -cbat; // charging current is positive, discharging current is negative
                        pbat = vbat/1000.0*cbat/100.0;
                        // Update batt charge level
                        int vbat_idle = vbat-round(cbat/16.0);  // voltage at cbat=0
                        if (vbat_idle > BAT_EMPTY) bat_level = min(((vbat_idle-BAT_EMPTY)*(BAT_LEVELS-2))/(BAT_FULL-BAT_EMPTY) + 1, BAT_LEVELS-1);
                        else bat_level = 0;
                        return true;               
                    }
                }

                if (BMS_resp[RS485_COMMAND_POS] == READ_ALL) {
                    // check for BMS warnings
                    if (BMS_resp[WARNINGS_POS] != WARNINGS_ID) {
                        error_msg = "Could not read BMS warnings";
                        return false;
                    }
                    if (BMS_resp[WARNINGS_POS+1] || BMS_resp[WARNINGS_POS+2]) {
                        error_msg = "BMS has warnings 0x";
                        if (BMS_resp[WARNINGS_POS+1] < 0x10) error_msg += "0";
                        error_msg += String(BMS_resp[WARNINGS_POS+1],HEX);
                        if (BMS_resp[WARNINGS_POS+2] < 0x10) error_msg += "0";
                        error_msg += String(BMS_resp[WARNINGS_POS+2],HEX);
                        return false;
                    }
                    // check OVP and UVP setting
                    if (BMS_resp[OVP_POS] != OVP_ID) {
                        error_msg = "Could not read BMS Cell OVP";
                        return false;
                    }
                    if ((BMS_resp[OVP_POS+1]<<8|BMS_resp[OVP_POS+2]) - ESS_OVP < ESS_BMS_OVP_DIFF) {
                        error_msg = "ESS_OVP less than ";
                        error_msg += ESS_BMS_OVP_DIFF;
                        error_msg += " mV below BMS Cell OVP (";
                        error_msg += (BMS_resp[OVP_POS+1]<<8|BMS_resp[OVP_POS+2]);
                        error_msg += " mV)";
                        return false;
                    }
                    if (BMS_resp[UVP_POS] != UVP_ID) {
                        error_msg = "Could not read BMS Cell UVP";
                        return false;
                    }
                    bms_uvp = BMS_resp[UVP_POS+1]<<8|BMS_resp[UVP_POS+2];
                    if (ESS_UVP - bms_uvp < ESS_BMS_UVP_DIFF) {
                        error_msg = "ESS_UVP less than ";
                        error_msg += ESS_BMS_UVP_DIFF;
                        error_msg += " mV above BMS Cell UVP (";
                        error_msg += bms_uvp;
                        error_msg += " mV)";
                        return false;
                    }
                    // read balancer settings
                    if (BMS_resp[SBAL_POS] != SBAL_ID) {
                        error_msg = "Could not read BMS Balancer Start Voltage";
                        return false;
                    }
                    BMS_balancer_start = BMS_resp[SBAL_POS+1]<<8|BMS_resp[SBAL_POS+2];
                    if (ESS_UVP < BMS_balancer_start) {
                        error_msg = "BMS Balancer Start Voltage higher than ESS_UVP (";
                        error_msg += ESS_UVP;
                        error_msg += " mV)";
                        return false;
                    }
                    if (BMS_resp[TBAL_POS] != TBAL_ID) {
                        error_msg = "Could not read BMS Balancer Trigger Voltage";
                        return false;
                    }
                    BMS_balancer_trigger = BMS_resp[TBAL_POS+1]<<8|BMS_resp[TBAL_POS+2];
                    return true;
                }
            }
        } 
    }
    error_msg = "BMS RS485 command 0x";
    error_msg += String(command[DATA_ID_POS],HEX);
    error_msg += " failed";
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
        if (URL == PM2_MW_STATUS) {
            power_ess = atof(&http_resp[http_resp.indexOf("apower")+8]);
            return true;
        }
        if (URL == PM2_HM_STATUS) {
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

    // Save power settings from previous cycle as baseline for new power calculation
    power_old = power_new;
    power_old_limit = power_new_limit;

    // Calculate new power setting
    int deviation = round(power_grid - power_target);
    if ((power_old >= 0) || (power_old <= HM_LOW_POWER_THRESHOLD)) {
        // normal ESS operating range: normal target power tolerance (+/-) applies
        if (abs(deviation) > POWER_TARGET_TOLERANCE) power_new = round(power_old - power_grid + power_target);
    }
    else {
        // HM operating above HM_LOW_POWER_THRESHOLD: normal negative power tolerance, but higher positive power tolerance
        if (deviation < -POWER_TARGET_TOLERANCE) power_new = round(power_old - power_grid + power_target);
        if (deviation > HM_LOW_POWER_TOLERANCE) power_new = round(power_old - power_grid + power_target);
    }

    // Filter out power spikes and ramp down ESS power (reduces power loss to grid when consumer is quickly switched on and off)
    if (max(power_new, hm_power_limit) - power_old < POWER_RAMPDOWN_RATE) {
        if (power_old > 0) filter_cycles = 0;  // charging: start ramping down immediately
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
    if (auto_recharge) power_new = MW_RECHARGE_POWER;  // Auto recharge power overrides calculation and manual setting

    // Make sure charging/discharging power limits are not exceeded
    power_new_limit = 0;// Assumption: New power setting is unlimited
    if (power_new > mw_power_limit) power_new = power_new_limit = mw_power_limit;
    if (power_new < hm_power_limit) power_new = power_new_limit = hm_power_limit;

    // Turn ESS off if power is between MW and HM operating ranges
    if ((power_new > HM_MIN_POWER) && (power_new < MW_MIN_POWER)) power_new = 0;

    // Apply new power setting
    if (power_new == 0) {  // turn charging or discharging off
        if (power_old < 0)
            if (!HoymilesCommand(HM_OFF)) power_new = power_old;
        if (power_old > 0) {
            if (power_old > MW_MIN_POWER) { SetMWPower(MW_MIN_POWER); delay(100); }  // prolongs relay lifetime
            if (!ShellyCommand(PM2_MW_OFF)) power_new = MW_MIN_POWER;
        }
    }
    if (power_new > 0) {  // set new charging power, (if necessary) turn discharging off, turn charging on
        if (power_old > 0) SetMWPower(power_new);  // re-calculate even if power setting remains unchanged (vbat might have changed)
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
        if (power_old < 0) {
            if (power_new != power_old)
                if (!HoymilesCommand(power_new)) power_new = power_old;
        }
        if (power_old == 0) {
            if (!HoymilesCommand(power_new)) power_new = 0;
            else if (!HoymilesCommand(HM_ON)) power_new = 0;
        }
        if (power_old > 0) {
            if (power_old > MW_MIN_POWER) { SetMWPower(MW_MIN_POWER); delay(100); }  // prolongs relay lifetime
            if (!ShellyCommand(PM2_MW_OFF)) power_new = MW_MIN_POWER;
            else {
                if (!HoymilesCommand(power_new)) power_new = 0;
                else if (!HoymilesCommand(HM_ON)) power_new = 0;
            }
        }
    }

    // Calculate cycle duration and reset power setting timestamp
    secs_cycle = (millis()-ts_power)/1000.0;
    ts_power = millis();
}

void FinishCycle() {
    
    // Calculate AC power flows
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

    // Update AC energy counters
    float hrs_cycle = secs_cycle/3600;
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
    en_pv_consumed = en_pv_to_cons + en_ess_to_cons - en_grid_to_ess*0.8;  // PV energy consumed by household, directly or via ESS
    en_pv_wasted += 0.2*hrs_cycle*min(power_from_grid, power_pv_to_ess);  // PV energy wasted in ESS, due to AC/DC/AC conversion losses of ~20%
    
    // Update DC energy counters
    en_from_batt += hrs_cycle * (pbat < 0) * -pbat;
    en_to_batt += hrs_cycle * (pbat > 0) * pbat;

    // Compile cycle message
    cycle_msg = CLEAR_SCREEN;
    cycle_msg += SHOW_CURSOR;
    cycle_msg += "BinSmart ESS ";
    cycle_msg += SW_VERSION;
    if (error_flag) cycle_msg += ERROR_SYMBOL;
    else cycle_msg += " ";
    cycle_msg += WIFI_SYMBOL[(WiFi.RSSI() >= GOOD_WIFI_RSSI)];
    cycle_msg += "\r\n";

    // Time, cycle time, daytime
    sprintf(buf,"%02d:%02d:%02d %+.3f",hour(local_unixtime),minute(local_unixtime),second(local_unixtime),secs_cycle);
    cycle_msg += buf;
    cycle_msg += OPS_SYMBOL[!power_new + (pm1_eco_mode && pm2_eco_mode)];
    cycle_msg += "\r\n";

    // OVP symbol above battery symbol
    cycle_msg += BAT_OVP_SYMBOL[(mw_power_limit < MW_MAX_POWER) + (mw_power_limit < MW_MIN_POWER)];

    // PV power
    sprintf(buf,"\r\n%4d",int(round(power_pv)));
    cycle_msg += buf;
    cycle_msg += NIGHT_DAY_SYMBOL[daytime];
    cycle_msg += PV_FLOW_SYMBOL[(int(round(power_pv)) > 0) + (int(round(power_pv)) >= PV_MAX_POWER)];
    cycle_msg += PV_CABLE_SYMBOL;

    // ESS power
    cycle_msg += ESS_CABLE_SYMBOL;
    if (int(round(power_ess)) == 0)
        cycle_msg += ESS_FLOW_SYMBOL[(power_old_limit > 0) + 2*(power_old_limit < 0)];
    if (int(round(power_ess)) > 0)
        cycle_msg += MW_FLOW_SYMBOL[(power_old_limit > 0) + (power_old_limit >= MW_MAX_POWER)][power_grid_to_ess > power_pv_to_ess];
    if (int(round(power_ess)) < 0)
        cycle_msg += HM_FLOW_SYMBOL[(power_old_limit < 0) + (power_old_limit <= HM_MAX_POWER)];
    cycle_msg += BAT_LEVEL_SYMBOL[bat_level];
    sprintf(buf,"%d",int(round(power_ess)));
    cycle_msg += buf;
    if (power_new != power_old) cycle_msg += DIFF_SYMBOL[(power_new < power_old) + !filter_cycles];
    else if (filter_cycles && (filter_cycles < POWER_FILTER_CYCLES)) cycle_msg += POWERFILTER_SYMBOL;
    if (manual_mode) cycle_msg += MANUAL_MODE_SYMBOL;
    if (auto_recharge && (power_old == MW_RECHARGE_POWER)) cycle_msg += AUTO_RECHARGE_SYMBOL;
    cycle_msg += "\r\n";

    // House symbol
    cycle_msg += HOUSE_SYMBOL;

    // UVP symbol below battery symbol
    cycle_msg += BAT_UVP_SYMBOL[(hm_power_limit > HM_MAX_POWER) + (hm_power_limit > HM_MIN_POWER)];

    // Grid power
    sprintf(buf,"\r\n%4d",int(round(power_grid)));
    cycle_msg += buf;
    cycle_msg += GRID_SYMBOL;
    cycle_msg += GRID_FLOW_SYMBOL[(int(round(power_grid)) > 0)  + 2*(int(round(power_grid)) < 0)][power_ess_to_grid > power_pv_to_grid];
    cycle_msg += GRID_CABLE_SYMBOL;

    // Power to consumers
    cycle_msg += CONS_CABLE_SYMBOL;
    if (power_grid_to_cons >= max(power_pv_to_cons, power_ess_to_cons)) cycle_msg += CONS_FLOW_SYMBOL[0];
    if (power_ess_to_cons >= max(power_grid_to_cons, power_pv_to_cons)) cycle_msg += CONS_FLOW_SYMBOL[1];
    if (power_pv_to_cons >= max(power_grid_to_cons, power_ess_to_cons)) cycle_msg += CONS_FLOW_SYMBOL[2];
    cycle_msg += CONS_SYMBOL;
    sprintf(buf,"%.0f",power_cons);
    cycle_msg += buf;
    cycle_msg += "\r\n\n";
    
    // Append error info (if error occured)
    if (error_msg != "") {
        cycle_msg += ERROR_SYMBOL;
        cycle_msg += error_msg;
        cycle_msg += "\r\n\n";
    }

    // If ESS is asleep: check for new min grid power (min household consumption)
    if (pm1_eco_mode && pm2_eco_mode && (power_grid < power_grid_min)) {
        power_grid_min = power_grid;
        minpower_time = local_unixtime;
    }

    // Turn on/off automatic recharge feature (prevents BMS turnoff)
    if (vcell_min <= (bms_uvp + ESS_UVP)/2) auto_recharge = true;
    if (vcell_min >= ESS_UVP) auto_recharge = false;
    
    // Keep alive Hoymiles RF24 interface
    if (millis()-ts_HM >= RF24_KEEPALIVE) {
        HoymilesCommand(power_new < 0);
        return;
    }

    // Keep alive Meanwell relay (if Meanwell is turned on)
    if (mw_on && (millis()-ts_MW >= MW_TIMER-10*PROCESSING_DELAY)) {
        ShellyCommand(PM2_MW_ON);
        return;
    }

    // Set Shelly 1PM eco mode (turn off at daytime, turn on at nighttime)
    if (daytime && pm1_eco_mode) {
        pm1_eco_mode = !ShellyCommand(PM1_ECO_OFF);
        return;
    }
    if (!daytime && !pm1_eco_mode) {
        pm1_eco_mode = ShellyCommand(PM1_ECO_ON);
        power_pv = 0;
        ShellyCommand(EM_RESET);  // Reset Shelly 3EM energy data (prevents HTTP timeouts during data reorgs)
        return;
    }

    // Set Shelly 2PM eco mode (turn off when charging/discharging active or possible, turn on when charging/discharging inactive and impossible)
    if (((power_pv > MW_MIN_POWER) || power_new) && pm2_eco_mode) {
        pm2_eco_mode = !ShellyCommand(PM2_ECO_OFF);
        return;
    }
    if (!hm_power_limit && (power_pv < 1) && !power_old && !pm2_eco_mode) {
        pm2_eco_mode = ShellyCommand(PM2_ECO_ON);
        return;
    }

    // Check if public IP address was changed, and if yes, update DDNS server entry
    if (millis()-ts_pubip >= DDNS_UPDATE_INTERVAL) {
        UpdateDDNS();
        return;
    }

    // Turn on/off JKBMS balancer (enable/disable bottom balancing), depend an UVP state
    if ((hm_power_limit == -1) && !bms_bal_on) bms_bal_on = BMSCommand(BAL_ON);
    if ((hm_power_limit == HM_MAX_POWER) && bms_bal_on) bms_bal_on = BMSCommand(BAL_OFF);
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
    power_grid = power_pv = power_ess = power_new = power_old = 0;
    
    while (true) {
        UpdateDDNS();  // keep updating DDNS in order to be reachable via Internet
        cycle_msg = CLEAR_SCREEN;
        cycle_msg += SHOW_CURSOR;
        cycle_msg += "BinSmart ESS ";
        cycle_msg += SW_VERSION;
        cycle_msg += "\r\n\n";
        cycle_msg += ERROR_SYMBOL;
        cycle_msg += "System halted at ";
        sprintf(buf,"%02d/%02d/%04d %02d:%02d:%02d",day(local_unixtime),month(local_unixtime),year(local_unixtime),hour(local_unixtime),minute(local_unixtime),second(local_unixtime));
        cycle_msg += buf;
        cycle_msg += "\r\nafter ";
        cycle_msg += ERROR_LIMIT;
        cycle_msg += " consecutive errors\r\n\n";
        command = 'e';
        ProcessUserCommand();

        ts_power = millis();
        command = 0;
        ReadCommand();
        if (command) {
            telnet.print(HIDE_CURSOR);
            telnet.flush();
            telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
            delay(3000);
            telnet.stop();
            delay(500);
            ESP.restart();
        }
    }
}

void ProcessUserCommand() {

    if (!telnet) {
        cmd_resp = "";
        command = 0;
    }

    switch (command) {
        case 'm':
            command = 0;  // respond to command only once, skip telnet output at the end of 
            cycle_msg += "Enter ESS power (negative for discharging): ";
            telnet.print(cycle_msg);
            ts_input = millis();
            while (millis()-ts_input < READINPUT_TIMEOUT)
                if (telnet.available()) {
                    input_str = telnet.readString();
                    if ((input_str[0] == '0') || input_str.toInt()) {
                        manual_mode = true;
                        power_manual = input_str.toInt();
                        cmd_resp = "ESS power manually set to ";
                        cmd_resp += power_manual;
                        cmd_resp += " W\r\n\n";
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
            ts_input = millis();
            while (millis()-ts_input < READINPUT_TIMEOUT)
                if (telnet.available()) {
                    input_str = telnet.readString();
                    if ((input_str[0] == '0') || input_str.toInt()) {
                        power_target = input_str.toInt();
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
            cmd_resp = "Batt voltage    : ";
            cmd_resp += String(vbat/1000.0,3);
            cmd_resp += " V\r\nCell voltages   : ";
            cmd_resp += vcell_min;
            cmd_resp += " - ";
            cmd_resp += vcell_max;
            cmd_resp += " mV\r\nMax cell diff   : ";
            cmd_resp += vcell_max-vcell_min;
            cmd_resp += " mV";
            if ((vcell_max-vcell_min >= BMS_balancer_trigger) && (vcell_max >= BMS_balancer_start) && bms_bal_on) cmd_resp += BALANCER_SYMBOL;
            cmd_resp += "\r\nBatt current    : ";
            cmd_resp += String(cbat/100.0,2);
            cmd_resp += " A\r\nAC power set to : ";
            cmd_resp += power_old;
            cmd_resp += " W\r\nAC power reading: ";
            cmd_resp += String(power_ess,1);
            cmd_resp += " W\r\nDC power reading: ";
            cmd_resp += String(pbat,1);
            cmd_resp += " W\r\nAC/DC efficiency: ";
            if ((power_ess > 0) && (pbat > 0)) cmd_resp += String(pbat/power_ess*100,1);
                else if ((power_ess < 0) && (pbat < 0)) cmd_resp += String(power_ess/pbat*100,1);
                    else cmd_resp += "n/m";
            cmd_resp += " %\r\nMW power limit  : ";
            cmd_resp += mw_power_limit;
            cmd_resp += " W";
            cmd_resp += "\r\nHM power limit  : ";
            cmd_resp += hm_power_limit;
            cmd_resp += " W\r\n\n";
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

            // PV energy
            sprintf(buf,"%7.3f",en_from_pv/1000);
            cmd_resp += buf;
            cmd_resp += NIGHT_DAY_SYMBOL[1];
            cmd_resp += PV_FLOW_SYMBOL[1];
            cmd_resp += PV_CABLE_SYMBOL;
            cmd_resp += ESS_CABLE_SYMBOL;
            cmd_resp += MW_FLOW_SYMBOL[0][0];
            cmd_resp += ESS_SYMBOL;
            sprintf(buf,"%.1f %%\r\n   ",en_pv_to_ess/en_from_pv*100);
            cmd_resp += buf;
            cmd_resp += HOUSE_SYMBOL;
            cmd_resp += "\r\n";
            sprintf(buf,"%5.1f %%",en_pv_to_grid/en_from_pv*100);
            cmd_resp += buf;
            cmd_resp += GRID_SYMBOL;
            cmd_resp += GRID_FLOW_SYMBOL[2][0];
            cmd_resp += GRID_CABLE_SYMBOL;
            cmd_resp += CONS_CABLE_SYMBOL;
            cmd_resp += CONS_FLOW_SYMBOL[2];
            cmd_resp += CONS_SYMBOL;
            sprintf(buf,"%.1f %%\r\n\n",en_pv_to_cons/en_from_pv*100);
            cmd_resp += buf;

            // Grid energy
            cmd_resp += "                 ";
            cmd_resp += ESS_CABLE_SYMBOL;
            cmd_resp += MW_FLOW_SYMBOL[0][1];
            cmd_resp += ESS_SYMBOL;
            sprintf(buf,"%.1f %%\r\n   ",en_grid_to_ess/en_from_grid*100);
            cmd_resp += buf;
            cmd_resp += HOUSE_SYMBOL;
            cmd_resp += "\r\n";
            sprintf(buf,"%7.3f",en_from_grid/1000);
            cmd_resp += buf;
            cmd_resp += GRID_SYMBOL;
            cmd_resp += GRID_FLOW_SYMBOL[1][0];
            cmd_resp += GRID_CABLE_SYMBOL;
            cmd_resp += CONS_CABLE_SYMBOL;
            cmd_resp += CONS_FLOW_SYMBOL[0];
            cmd_resp += CONS_SYMBOL;
            sprintf(buf,"%.1f %%\r\n\n",en_grid_to_cons/en_from_grid*100);
            cmd_resp += buf;
   
            // ESS energy
            cmd_resp += "                 ";
            cmd_resp += ESS_CABLE_SYMBOL;
            cmd_resp += HM_FLOW_SYMBOL[0];
            cmd_resp += ESS_SYMBOL;
            sprintf(buf,"%.3f\r\n   ",en_from_ess/1000);
            cmd_resp += buf;
            cmd_resp += HOUSE_SYMBOL;
            cmd_resp += "\r\n";
            sprintf(buf,"%5.1f %%",en_ess_to_grid/en_from_ess*100);
            cmd_resp += buf;
            cmd_resp += GRID_SYMBOL;
            cmd_resp += GRID_FLOW_SYMBOL[2][1];
            cmd_resp += GRID_CABLE_SYMBOL;
            cmd_resp += CONS_CABLE_SYMBOL;
            cmd_resp += CONS_FLOW_SYMBOL[1];
            cmd_resp += CONS_SYMBOL;
            sprintf(buf,"%.1f %%\r\n\n",en_ess_to_cons/en_from_ess*100);
            cmd_resp += buf;

            // PV energy wasted in ESS (20% of energy sent to ESS instead of consumer, despite grid energy being drawn)
            if (en_pv_to_grid+en_pv_wasted < en_from_pv) {
                cmd_resp += "PV energy wasted: ";
                sprintf(buf,"%.1f %%\r\n",(en_pv_to_grid+en_pv_wasted)/en_from_pv*100);
                cmd_resp += buf;
            }
            // Total PV energy consumed in household (directly or via ESS)
            if (en_pv_consumed < en_from_pv) {
                cmd_resp += "PV energy cons. : ";
                sprintf(buf,"%.1f %%\r\n",en_pv_consumed/en_from_pv*100);
                cmd_resp += buf;
            }
            // Meanwell charging efficiency
            if (en_to_batt < en_to_ess) {
                cmd_resp += "MW efficiency   : ";
                sprintf(buf,"%.1f %%\r\n",en_to_batt/en_to_ess*100);
                cmd_resp += buf;
            }
            // Hoymiles charging efficiency
            if (en_from_ess < en_from_batt) {
                cmd_resp += "HM efficiency   : ";
                sprintf(buf,"%.1f %%\r\n",en_from_ess/en_from_batt*100);
                cmd_resp += buf;
            }
            // ESS AC-to-AC total efficiency
            if (en_from_ess < en_to_ess) {
                cmd_resp += "ESS efficiency  : ";
                sprintf(buf,"%.1f %%\r\n",en_from_ess/en_to_ess*100);
                cmd_resp += buf;
            }
            cmd_resp += "\r\n";
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
            cmd_resp += mw_counter/(elapsedDays(local_unixtime-starttime)+1);
            cmd_resp += "/day)\r\n\n";
            break;
        case 'z':
            command = 0;  // respond to command only once
            cycle_msg += "Enter [e] or [n] to reset error or energy stats: ";
            telnet.print(cycle_msg);
            ts_input = millis();
            while (millis()-ts_input < READINPUT_TIMEOUT)
                if (telnet.available()) {
                    input_str = telnet.readString();
                    if (input_str[0] == 'e') {
                        for (int i=0; i<ERROR_TYPES; i++) error_counter[i] = errortime[i] = 0;
                        last_error_msg = "";
                        error_flag = false;
                        resettime_errors = local_unixtime;
                        cmd_resp = "Error stats reset to zero\r\n\n";
                        return;
                    }
                    if (input_str[0] == 'n') {
                        en_from_pv = en_pv_to_cons = en_pv_to_ess = en_pv_to_grid = 0, en_pv_consumed = 0;  // Reset PV energy counters
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
            ts_input = millis();
            while (millis()-ts_input < READINPUT_TIMEOUT)
                if (telnet.available()) {
                    input_str = telnet.readString();
                    if (input_str[0] == 'y') {
                        telnet.print(HIDE_CURSOR);
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
    if (errors_consecutive < ERROR_LIMIT) cycle_msg += "Enter command or [h] for help: ";
    else cycle_msg += "Press any key to restart: ";
    telnet.print(cycle_msg);
    // Assumption for next cycle: charging/discharging power equals power setting
    power_ess = power_new;
}

void ReadCommand() {
    while (millis()-ts_power < PROCESSING_DELAY) {
        if (!telnet) {
            telnet = server.available();  // new telnet session?
            if (telnet) telnet.print(cycle_msg);
        }
        if (telnet.available()) {  // user command entered?
            command = telnet.read();  // read user command
            telnet.flush();  // discard everything after first character
        }
    }
    telnet.print(HIDE_CURSOR);  // Discourage user commands while system is busy
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

bool HoymilesCommand(int power) {

    while (millis()-ts_HM < 50);  // minimum delay between two consecutive HM commands: 50 ms
    radio.stopListening();  // switch to TX mode
    if (power == HM_OFF) {  // turn off Hoymiles
        if (radio.writeFast(hm_turnoff, sizeof(hm_turnoff)))
            if (radio.txStandBy(RF24_TIMEOUT)) {
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
            if (radio.txStandBy(RF24_TIMEOUT)) {
                ts_HM = millis();
                return true;
            }
        // RF24 command failed
        ts_HM = millis();
        error_msg = "Hoymiles RF24 turnon command failed";
        return false;
    }

    // set Hoymiles power limit
    unsigned int limit = -10*power;
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
        if (radio.txStandBy(RF24_TIMEOUT)) {
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

    // read public IP from server
    http.begin(PUBLIC_IP_SERVER);
    http_resp_code = http.GET();
    if (http_resp_code == HTTP_CODE_OK) {
        public_IP = http.getString();
        http.end();
        ts_pubip = millis();
        pubip_time = local_unixtime;
        if (public_IP == DDNS_address) return true;  // public IP unchanged

        http.begin(DDNS_SERVER_UPDATE + public_IP);  // update DDNS server entry
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
    http.end();
    error_msg = "Could not read public IP address (http code ";
    error_msg += http_resp_code;
    error_msg += ")";
    return false;
}


