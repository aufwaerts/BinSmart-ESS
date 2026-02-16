#define SW_VERSION "v2.82"

#include <WiFi.h>  // standard Arduino/ESP32
#include <HTTPClient.h>  // standard Arduino/ESP32
#include <WebServer.h>  // standard Arduino/ESP32
#include <ElegantOTA.h>  // https://github.com/ayushsharma82/ElegantOTA (I prefer v2.2.9)
#include <RF24.h>  // https://nrf24.github.io/RF24/
#include <CRC8.h>  // https://github.com/RobTillaart/CRC
#include <CRC16.h>  // https://github.com/RobTillaart/CRC
#include <TimeLib.h>  // https://playground.arduino.cc/Code/Time/
#include <Dusk2Dawn.h>  // https://github.com/dmkishi/Dusk2Dawn
#include <NimBLEDevice.h>  // https://github.com/h2zero/NimBLE-Arduino
#include "BinSmart_cfg.h"
#include "BinSmart_var.h"

void setup() {

    // Reduce CPU clock to 80 MHz (enough for this application)
    // setCpuFrequencyMhz(80);  // REPLACED BY ARDUINO COMPILER SETTING

    // Assign LED Pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Init WiFi
    WiFi.config(ESP32_ADDR, ROUTER_ADDR, SUBNET, DNS_SERVER1, DNS_SERVER2);
    WiFi.setTxPower(WIFI_POWER_5dBm);
    WiFi.begin(WIFI_SSID, WIFI_PWD);
    while (WiFi.status() != WL_CONNECTED);   // if WiFi unavailable or wrong SSID/PWD, system stops here and LED remains on
    digitalWrite(LED_PIN, LOW);

    // Start OTA software update service
    ElegantOTA.begin(&OTA_server);
    OTA_server.begin();

    // Start telnet server and wait for terminal to connect
    telnet_server.begin();
    while (!telnet) {
        OTA_server.handleClient();  // check for OTA software update
        telnet = telnet_server.available();
        delay(100);
    }

    // Print startup message
    telnet.printf("%sBinSmart ESS %s   CPU freq: %d MHz\r\n\nWiFi connected to %s   RSSI: %d\r\n", CLEAR_SCREEN, SW_VERSION, getCpuFrequencyMhz(), WIFI_SSID, WiFi.RSSI());
    delay(2000);

    // Init PWM generator (for adjusting Meanwell power)
    if (!ledcAttachChannel(PWM_OUTPUT_PIN, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL)) strcpy(error_str, "PWM generator init failed");
    CheckErrors();
    SetMWPower(MW_MIN_POWER);
    telnet.println("PWM generator initialized");
    
    // Init RS485 communication with BMS, read OVP/UVP/balancer/switch settings
    Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    BMSCommand(READ_SETTINGS);
    CheckErrors();
    telnet.println("RS485 communication with JKBMS OK");

    // Init BLE communication with BMS
    NimBLEDevice::init("");
    pClient = NimBLEDevice::createClient(serverAddress);
    if (!pClient) strcpy(error_str, "BLE init failed");
    CheckErrors();
    pClient->setConnectionParams(12, 12, 0, 12);
    pClient->setConnectTimeout(BLE_TIMEOUT);
    telnet.println("BLE communication with JKBMS initialized");

    // Init RF24 radio communication with Hoymiles
    if (!radio.begin()) strcpy(error_str, "RF24 radio init failed");
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

    // Turn off Hoymiles power (if Hoymiles is awake)
    if (hm_awake) {
        HoymilesCommand(HM_POWER_OFF);
        CheckErrors();
        telnet.println("RF24 communication with Hoymiles OK");
    }
    else telnet.println("Hoymiles is asleep");

    // Shelly 1PM: Read eco mode setting and PV power
    if (ShellyCommand(PM1_ADDR, PM_CONFIG, "")) ShellyCommand(PM1_ADDR, PM_STATUS, "0");
    CheckErrors();
    telnet.println("Shelly 1PM found");

    // Shelly 2PM: Read eco mode setting, turn off Meanwell relay, make sure Hoymiles relay state matches BMS discharging switch
    if (ShellyCommand(PM2_ADDR, PM_CONFIG, ""))
        if (ShellyCommand(PM2_ADDR, MW_RELAY, "off")) {
            if (hm_awake) ShellyCommand(PM2_ADDR, HM_RELAY, "on");
            else ShellyCommand(PM2_ADDR, HM_RELAY, "off");
        }
    CheckErrors();
    telnet.println("Shelly 2PM found");
    
    // Shelly 3EM: Read grid power and current time
    ShellyCommand(EM_ADDR, EM_STATUS, "");
    CheckErrors();
    telnet.println("Shelly 3EM found");

    // Update DDNS address
    if (!UpdateDDNS()) strcpy(error_str, "DDNS update failed");
    CheckErrors();
    telnet.println("DDNS address updated");
    
    // no errors during setup: set timestamps, start polling cycle
    telnet.print("\nNo errors during setup, start polling cycle ...");
    delay(PROCESSING_DELAY);
    starttime = resettime_errors = resettime_energy = unixtime;
    mw_counter = hm_counter = 0;
    ts_power = millis();
}

void loop() {

    BMSCommand(READ_VOLTAGES);  // read cell voltages from BMS, set charging/discharging power limits
    if (power_new) ShellyCommand(PM2_ADDR, PM_STATUS, (power_new > 0) ? "0" : "1"); // read ESS power from Shelly 2PM
    ShellyCommand(EM_ADDR, EM_STATUS, "");  // read time and grid power from Shelly 3EM
    BMSCommand(READ_CURRENT);  // read DC charging/discharging current and power from BMS
    SetNewPower();  // calculate and apply new charging/discharging power setting
    FinishCycle();  // update energy stats, do maintenance tasks, print status info, handle user command, read PV power, flash LED
    CheckErrors();  // check for comms errors, halt system if an error is persistent
}

bool ShellyCommand(IPAddress ip_addr, const char command[], const char params[]) {

    // Prepare Shelly http command
    strcpy(http_command, "http://");
    strcat(http_command, ip_addr.toString().c_str());
    strcat(http_command, command);
    strcat(http_command, params);

    if (WiFi.status() != WL_CONNECTED) {
        strcpy(error_str, "WIFI");
        return false;
    }

    // Send http command to Shelly
    http.setTimeout(HTTP_SHELLY_TIMEOUT);
    http.begin(http_command);
    if (http.GET() == HTTP_CODE_OK) {
        shelly_resp = http.getStream();
        if (!strcmp(command, EM_STATUS)) {
            // read time and grid power from Shelly 3EM
            for (int i=0; i<140; i++) shelly_resp.read();  // fast forward to "time"
            shelly_resp.find("time");
            min_of_day = shelly_resp.parseInt()*60 + shelly_resp.parseInt();  // read local time (minutes after midnight)
            unsigned long unixtime_utc = shelly_resp.parseInt();  // read epoch time (UTC)
            for (int i=0; i<550; i++) shelly_resp.read();  // fast forward to "total_power"
            shelly_resp.find("total_power");
            power_grid = shelly_resp.parseFloat();  // read grid power from http response
            http.end();
            utc_offset = (min_of_day/60+24-hour(unixtime_utc))%24;  // UTC offset = timezone + dst
            unixtime = unixtime_utc + utc_offset*3600;  // unixtime equals local time
            if (!starttime || min_of_day == 210) {  // calculate sunrise/sunset times during setup() or at 03:30 local time
                sunrise = ess_location.sunrise(year(unixtime), month(unixtime), day(unixtime), utc_offset-TIMEZONE);
                sunset = ess_location.sunset(year(unixtime), month(unixtime), day(unixtime), utc_offset-TIMEZONE);
            }
            return true;
        }
        if (!strcmp(command, PM_STATUS)) {
            // read power from Shelly 1PM or 2PM
            shelly_resp.find("apower");
            if (ip_addr == PM1_ADDR) power_pv = shelly_resp.parseFloat();
            if (ip_addr == PM2_ADDR) {
                power_ess = shelly_resp.parseFloat();
                if (power_new > 0) power_ess = power_ess*PM2_MW_POWER_CORR;  // power correction for positive (charging) power
            }
            http.end();
            return true;
        }
        if (!strcmp(command, MW_RELAY)) {  // Meanwell relay turned on or off
            http.end();
            ts_MW = millis();
            if (mw_on == !strcmp(params, "off")) {  // relay state changed
                mw_counter++;
                mw_on = !mw_on;
            }
            delay(400);  // allow a little more time for power change to stabilize
            if (!mw_on) ledcWrite(PWM_OUTPUT_PIN, 0);  // if MW was turned off, also turn off optocoupler LED
            return true;
        }
        if (!strcmp(command, HM_RELAY)) {  // Hoymiles relay turned on or off
            http.end();
            ts_HM = millis();
            if (hm_on == !strcmp(params, "off")) {  // relay state changed
                hm_counter++;
                hm_on = !hm_on;
            }
            // Hoymiles AC side turned on/off: turn on/off DC side, too (after a short delay)
            delay(1000);
            if (!strcmp(params, "off")) return BMSCommand(DISCH_OFF);
            else return BMSCommand(DISCH_ON);
        }
        if (!strcmp(command, PM_CONFIG)) {
            bool eco_mode = shelly_resp.findUntil("eco_mode\":true", "eco_mode\":false");
            http.end();
            if (ip_addr == PM1_ADDR) pm1_eco_mode = eco_mode;
            if (ip_addr == PM2_ADDR) pm2_eco_mode = eco_mode;
            return true;
        }
        http.end();
        return true;
    }
    // Shelly command failed
    http.end();
    if (ip_addr == EM_ADDR) sprintf(error_str, "3EM cmd %s failed", http_command);
    if (ip_addr == PM1_ADDR) sprintf(error_str, "1PM cmd %s failed", http_command);
    if (ip_addr == PM2_ADDR) sprintf(error_str, "2PM cmd %s failed", http_command);
    return false;
}

bool BMSCommand(const byte command[]) {

    while (millis()-ts_BMS < BMS_WAIT);  // minimum delay after previous BMS response

    if (command[0] == BLE_1) {

        // Send BMS command via BLE
        if (pClient->connect()) {
            NimBLERemoteService* pService = pClient->getService("FFE0");
            NimBLERemoteCharacteristic* pChar = pService->getCharacteristic("FFE1");
            if (pChar->writeValue(GET_INFO, BLE_COMMAND_LEN, false)) {
                delay(500);
                if (pChar->writeValue(GET_DATA, BLE_COMMAND_LEN, false)) {
                    delay(500);
                    if (pChar->writeValue(command, BLE_COMMAND_LEN, false)) {
                        pClient->disconnect();
                        ts_BMS = millis();  // set "end of BMS response" timestamp
                        return true;
                    }
                }
            }
            pClient->disconnect();
        }
        ts_BMS = millis();  // set "end of BMS response" timestamp
        sprintf(error_str, "BMS BLE command 0x%02X 0x%02X failed", command[BLE_COMMAND_POS], command[BLE_SETTING_POS]);
        return false;
    }

    // Send BMS command via RS485
    Serial2.write(command, command[RS485_LEN_POS]+2);
    Serial2.flush();

    memset(bms_resp, 0, sizeof(bms_resp));  // will result in failed validity check if no response

    // Wait for BMS response
    ts_BMS = millis();
    while ((millis()-ts_BMS < BMS_TIMEOUT) && !Serial2.available());
    
    // fill response buffer with BMS response
    int len = 0, sum = 0;
    while (Serial2.available()) {
        bms_resp[len] = Serial2.read();
        sum += bms_resp[len];
        len++;
        if (!Serial2.available()) {
            if (len < (bms_resp[2]<<8|bms_resp[3])+2) {
                bms_resp_wait_counter++;  // response not complete yet: wait for next response byte to arrive
                ts_BMS = millis();
                while ((millis()-ts_BMS < BMS_TIMEOUT) && !Serial2.available());
            }
        }
    }
    ts_BMS = millis();  // set "end of BMS response" timestamp

    // check validity of BMS response
    if ((bms_resp[0] == RS485_1) && (bms_resp[1] == RS485_2)) {  // start of response frame
         if ((bms_resp[2]<<8|bms_resp[3])+2 == len) {  // response length
             if ((bms_resp[len-2]<<8|bms_resp[len-1]) == sum-bms_resp[len-2]-bms_resp[len-1]) {  // response checksum
                if (bms_resp[RS485_COMMAND_POS] == WRITE_DATA) return true;  // no need to inspect the response to a write command

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
                        // determine if cell balancer is active
                        bms_bal_active = bms_bal_on && (vcell_max-vcell_min >= bms_balancer_trigger) && (vcell_max >= bms_balancer_start);
                        // Save old charging power limit, update limit (depending on vcell_max)
                        mw_limit_old = mw_limit;
                        if ((vcell_max <= ESS_OVPR) || (mw_limit >= MW_MAX_POWER)) mw_limit = round(MW_MAX_POWER_FORMULA);  // exit OVP mode (update MW power limit)
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
                        sprintf(error_str, "BMS has warnings 0x%02X 0x%02X", bms_resp[WARNINGS_POS], bms_resp[WARNINGS_POS+1]);
                        return false;
                    }
                    // check OVP and UVP setting
                    if ((bms_resp[OVP_POS]<<8|bms_resp[OVP_POS+1]) - ESS_OVP < ESS_BMS_OVP_DIFF) {
                        sprintf(error_str, "ESS_OVP less than %d mV below BMS_OVP (%d mV)", ESS_BMS_OVP_DIFF, bms_resp[OVP_POS]<<8|bms_resp[OVP_POS+1]);
                        return false;
                    }
                    bms_uvp = bms_resp[UVP_POS]<<8|bms_resp[UVP_POS+1];
                    if (ESS_UVP - bms_uvp < ESS_BMS_UVP_DIFF) {
                        sprintf(error_str, "ESS_UVP less than %d mV above BMS_UVP (%d mV)", ESS_BMS_UVP_DIFF, bms_uvp);
                        return false;
                    }
                    // read balancer settings
                    bms_balancer_start = bms_resp[BAL_ST_POS]<<8|bms_resp[BAL_ST_POS+1];
                    if (ESS_UVP < bms_balancer_start) {
                        sprintf(error_str, "BMS Balancer Start Voltage higher than ESS_UVP (%d mV)", ESS_UVP);
                        return false;
                    }
                    bms_balancer_trigger = bms_resp[BAL_TR_POS]<<8|bms_resp[BAL_TR_POS+1];
                    bms_bal_on = bms_resp[BAL_SW_POS];
                    hm_awake = bms_resp[DISCH_SW_POS];
                    if (!hm_awake) hm_limit = 0;  // if BMS discharging switch turned off, prevent any attempts to turn on Hoymiles
                    return true;
                }
            }
        } 
    }
    sprintf(error_str, "BMS RS485 command 0x%02X 0x%02X failed", command[RS485_COMMAND_POS], command[DATA_ID_POS]);
    // for (int i=0; i<len; i++) telnet.printf("0x%02X ", bms_resp[i]);
    // telnet.println();
    // while (!telnet.available());
    // telnet.read();
    return false;
}

bool HoymilesCommand(int hm_command) {

    while (millis()-ts_HM < RF24_WAIT);  // minimum delay between two consecutive Hoymiles RF24 commands

    if ((hm_command == HM_POWER_OFF) || (hm_command == HM_POWER_ON)) {  // switch command
        if (radio.writeFast(HM_SWITCH[hm_command], sizeof(HM_SWITCH[hm_command])))
            if (radio.txStandBy(RF24_TIMEOUT)) {
                ts_HM = millis();
                delay(400);  // allow a little more time for power change to stabilize
                return true;
            }
        ts_HM = millis();
        if (hm_command == HM_POWER_OFF) strcpy(error_str, "RF24 Hoymiles turn off command failed");
        else strcpy(error_str, "RF24 Hoymiles turn on command failed");
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
        if (radio.txStandBy(RF24_TIMEOUT)) {
                ts_HM = millis();
                return true;
        }
    ts_HM = millis();
    strcpy(error_str, "RF24 Hoymiles power command failed");
    return false;
}

void SetNewPower() {

    // Save power settings from previous cycle as baseline for new power calculation
    power_old = power_new;

    // Calculate new power setting
    int target_deviation = round(power_grid - power_grid_target);
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
            if ((power_old > MW_MIN_POWER) && (power_new < MW_MIN_POWER)) power_new = MW_MIN_POWER;  // don't skip MW_MIN_POWER during power rampdown
            if (power_old == MW_MIN_POWER) {
                power_new = 0;  // don't skip zero during power rampdown
                filter_cycles = POWER_FILTER_CYCLES;  // reset filter cycle countdown before discharging
            }
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
        }
        else power_new = MW_MAX_POWER/2;
    }
    if ((vcell_min <= (bms_uvp + ESS_UVP)/2)) auto_recharge = true;

    // Apply new power setting
    if (power_new == 0) {  // turn charging or discharging off
        if (power_old < 0)
            if (!HoymilesCommand(HM_POWER_OFF)) power_new = power_old;
        if (power_old > 0) {
            SetMWPower(MW_MIN_POWER);  // extends Shelly relay lifetime
            if (!ShellyCommand(PM2_ADDR, MW_RELAY, "off")) power_new = MW_MIN_POWER;
        }
    }
    if (power_new > 0) {  // set new charging power, (if necessary) turn discharging off, turn charging on
        if (power_old > 0) SetMWPower(power_new);  // re-calculate even if power setting remains unchanged (vbat might have changed)
        if (power_old == 0) {
            SetMWPower(power_new);
            if (!ShellyCommand(PM2_ADDR, MW_RELAY, "on&timer=60")) power_new = 0;
        }
        if (power_old < 0) {
            if (!HoymilesCommand(HM_POWER_OFF)) power_new = power_old;
            else {
                SetMWPower(power_new);
                if (!ShellyCommand(PM2_ADDR, MW_RELAY, "on&timer=60")) power_new = 0;
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
            else if (!HoymilesCommand(HM_POWER_ON)) power_new = 0;
        }
        if (power_old > 0) {
            SetMWPower(MW_MIN_POWER);  // extends Shelly relay lifetime
            if (!ShellyCommand(PM2_ADDR, MW_RELAY, "off")) power_new = MW_MIN_POWER;
            else {
                if (!HoymilesCommand(power_new)) power_new = 0;
                else if (!HoymilesCommand(HM_POWER_ON)) power_new = 0;
            }
        }
    }
    
    // Calculate cycle duration and reset power timestamp
    msecs_cycle = millis()-ts_power;
    ts_power = millis();
}

void FinishCycle() {

    // Calculate AC power flows
    power_from_grid = (power_grid > 0) * power_grid;
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
    float hrs_cycle = msecs_cycle/3600000.0;
    en_from_pv += hrs_cycle*power_pv;
    en_from_ess += hrs_cycle*power_from_ess;
    en_to_ess += hrs_cycle*power_to_ess;
    en_from_grid += hrs_cycle*power_from_grid;
    en_pv_to_cons += hrs_cycle*power_pv_to_cons;
    en_pv_to_ess += hrs_cycle*power_pv_to_ess;
    en_pv_to_grid += hrs_cycle*power_pv_to_grid;
    en_grid_to_cons += hrs_cycle*power_grid_to_cons;
    en_grid_to_ess += hrs_cycle*power_grid_to_ess;
    en_ess_to_cons += hrs_cycle*power_ess_to_cons;
    en_ess_to_grid += hrs_cycle*power_ess_to_grid;
    if (power_ess > 0) en_pv_wasted += hrs_cycle*(1-0.9*pbat/power_ess)*min(power_from_grid, power_pv_to_ess);  // PV energy wasted to ESS, due to AC/DC/AC conversion losses
    
    // Update DC energy counters
    en_from_batt += hrs_cycle * (pbat < 0) * -pbat;
    en_to_batt += hrs_cycle * (pbat > 0) * pbat;

    // No ESS power output and no PV production: check for new power_grid_min (i.e. min household consumption)
    if (!power_ess && !power_pv && (power_grid < power_grid_min)) {
        power_grid_min = power_grid;
        minpower_time = unixtime;
    }

    // Keep alive Hoymiles RF24 interface
    if ((millis()-ts_HM >= RF24_KEEPALIVE*1000) && hm_awake)
        if (power_new < 0) HoymilesCommand(HM_POWER_ON);
        else HoymilesCommand(HM_POWER_OFF);

    // Keep alive Meanwell relay (if Meanwell is turned on)
    if (mw_on && (millis()-ts_MW >= MW_KEEPALIVE*1000)) ShellyCommand(PM2_ADDR, MW_RELAY, "on&timer=60");

    // Turn off/on BMS balancer (disable/enable bottom balancing), depending on lowest cell voltage 
    if ((vcell_min >= BMS_BAL_OFF) && bms_bal_on) bms_bal_on = !BMSCommand(BAL_OFF);
    if ((vcell_min <= BMS_BAL_ON) && !power_old && !bms_bal_on) bms_bal_on = BMSCommand(BAL_ON);

    // Set Shelly 1PM eco mode (turn off just before sunrise, turn on at sunset)
    if ((min_of_day >= sunrise-1) && (min_of_day < sunset) && pm1_eco_mode) pm1_eco_mode = !ShellyCommand(PM1_ADDR, ECO_MODE, "false}}");
    if (((min_of_day < sunrise-1) || (min_of_day >= sunset)) && !pm1_eco_mode) pm1_eco_mode = ShellyCommand(PM1_ADDR, ECO_MODE, "true}}");

    // Set Shelly 2PM eco mode (turn off when charging/discharging, turn on when charging/discharging inactive and impossible)
    if (power_new && pm2_eco_mode) pm2_eco_mode = !ShellyCommand(PM2_ADDR, ECO_MODE, "false}}");
    if (!power_new && (power_pv < MW_MIN_POWER-power_grid_target) && !hm_awake && !pm2_eco_mode) pm2_eco_mode = ShellyCommand(PM2_ADDR, ECO_MODE, "true}}");

    // Make Hoymiles fall asleep or wake it up, depending on hm_limit and vcell_min
    if ((vcell_min <= ESS_UVP) && !hm_limit && hm_awake) hm_awake = !ShellyCommand(PM2_ADDR, HM_RELAY, "off");
    if ((vcell_min >= ESS_UVPR-10) && !hm_awake) hm_awake = ShellyCommand(PM2_ADDR, HM_RELAY, "on");  // wake up HM before reaching ESS_UVPR, to allow time for AC sync procedure

    // Clear Shelly 3EM energy data at 23:00 UTC (prevents HTTP timeouts at 00:00 UTC due to internal data reorgs)
    if ((min_of_day/60 == (23+utc_offset)%24) && !em_data_cleared) em_data_cleared = ShellyCommand(EM_ADDR, EM_RESET, "");
    if (min_of_day/60 == utc_offset) em_data_cleared = false;

    // Check if public IP address was changed, if yes: update DDNS server entry
    if (millis()-ts_pubip >= DDNS_UPDATE_INTERVAL*1000) UpdateDDNS();

    // flash LED to indicate end of cycle
    digitalWrite(LED_PIN, HIGH);  delay(20); digitalWrite(LED_PIN, LOW);

    // calculate end-of-cycle waiting time (consider ESS sleep mode and time since power change, allow time for userIO and PV power reading)
    int cycle_delay = PROCESSING_DELAY*(1+(pm1_eco_mode && pm2_eco_mode))-(millis()-ts_power)-100;
    if (cycle_delay > 0) delay(cycle_delay);

    // check for OTA software update
    OTA_server.handleClient();

    // check for telnet session
    if (!telnet) telnet = telnet_server.available();
    if (telnet) UserIO();  // if telnet session exists, print cycle info and handle user command
    else command = '\0';  // no telnet session: clear user command response

    // daytime: read PV power from Shelly 1PM (should take less than 100 ms)
    if ((min_of_day >= sunrise) && (min_of_day < sunset)) ShellyCommand(PM1_ADDR, PM_STATUS, "0");
    else power_pv = 0;

    // wait until power change has stabilized
    cycle_delay = PROCESSING_DELAY*(1+(pm1_eco_mode && pm2_eco_mode))-(millis()-ts_power);
    if (cycle_delay > 0) delay(cycle_delay);

    // assumption for next cycle: ESS power reading equals power setting
    power_ess = power_new;
}

void CheckErrors() {

    if (error_str[0] == '\0') {  // no errors this cycle
        errors_consecutive = 0;  // reset counter for consecutive errors
        return;
    }

    error_flag = true;  // new unread error
    digitalWrite(LED_PIN, HIGH);  // LED turned on for one cycle
    strcpy(last_error_str, error_str);
    int error_index;
    for (error_index=0; error_index<ERROR_TYPES; error_index++)
        if (strstr(error_str, ERROR_TYPE[error_index])) {
            error_counter[error_index]++;    // increase error counter
            errortime[error_index] = unixtime;
            break;
        }
    error_str[0] = '\0';  // assumption: no errors during next cycle
    if (error_index >= UNCRITICAL_ERROR_TYPES) errors_consecutive++;  // if error was WIFI related: allow unlimited erroneous cycles
    if ((errors_consecutive < ERROR_LIMIT) && starttime) return;  // continue with next cycle if below ERROR_LIMIT and no error during setup()

    // Error is persistent or error during setup(): Halt the system
    if (!HoymilesCommand(HM_POWER_OFF)) ShellyCommand(PM2_ADDR, HM_RELAY, "off");
    SetMWPower(MW_MIN_POWER);
    ShellyCommand(PM2_ADDR, MW_RELAY, "off");
    
    // System halted: prepare continuous error message
    sprintf(tn_str, "%sBinSmart ESS %s\r\n\n%sSystem halted ", CLEAR_SCREEN, SW_VERSION, ERROR_SYMBOL);
    if (starttime) sprintf(tn_str + strlen(tn_str), "at %02d/%02d/%04d %02d:%02d:%02d\r\n   after %d consecutive errors\r\n\n", day(unixtime), month(unixtime), year(unixtime), hour(unixtime), minute(unixtime), second(unixtime), errors_consecutive);
    else strcat(tn_str, "during setup\r\n\n");
    strcat(tn_str, "Last error: ");
    strcat(tn_str, last_error_str);
    strcat(tn_str, "\r\n\nPress any key to restart: ");

    while (true) {  // wait for user to confirm restart
        ts_power = millis();
        OTA_server.handleClient();  // Check for OTA software update
        if (millis()-ts_pubip >= DDNS_UPDATE_INTERVAL*1000) UpdateDDNS();  // keep updating DDNS in order to be reachable via Internet
        if (!telnet) telnet = telnet_server.available();
        if (telnet) telnet.print(tn_str);
        for (int i=0; i<10; i++) {
            digitalWrite(LED_PIN, HIGH);  delay(20); digitalWrite(LED_PIN, LOW); delay(40); // indicates halted system
        }
        while (millis()-ts_power < PROCESSING_DELAY*2);  // wait for two PROCESSING_DELAYS
        if (telnet.available()) {
            telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
            delay(2000);
            telnet.stop();
            delay(1000);
            ESP.restart();
        }
    }
}

void SetMWPower(int mw_power) {

    int duty_cycle;

    // convert power setting to PWM value
    if (mw_power >= MW_LOW_POWER_THRESHOLD) duty_cycle = round(MW_POWER_FORMULA);
    else duty_cycle = round(MW_LOW_POWER_FORMULA);

    // Make sure upper/lower PWM limits are not exceeded
    if (duty_cycle > DUTY_CYCLE_MAX) duty_cycle = DUTY_CYCLE_MAX;
    if (duty_cycle < DUTY_CYCLE_MIN) duty_cycle = DUTY_CYCLE_MIN;
    ledcWrite(PWM_OUTPUT_PIN, duty_cycle);
}

void UserIO() {

    // Prepare telnet status message
    strcpy(tn_str, CLEAR_SCREEN);
    strcat(tn_str, "BinSmart ESS ");
    strcat(tn_str, SW_VERSION);
    strcat(tn_str, " ");
    if (error_flag && (command != 'e')) strcat(tn_str, ERROR_SYMBOL);  // "unread error" symbol
    strcat(tn_str, WIFI_SYMBOL[WiFi.RSSI() >= GOOD_WIFI_RSSI]);

    // Time, cycle time, operations symbol
    sprintf(tn_str + strlen(tn_str), "\r\n%02d:%02d:%02d +%d.%03d%s\r\n", hour(unixtime), minute(unixtime), second(unixtime), msecs_cycle/1000, msecs_cycle%1000, OPS_SYMBOL[!power_new + (pm1_eco_mode && pm2_eco_mode)]);

    // OVP symbol above battery symbol
    strcat(tn_str, BAT_OVP_SYMBOL[(mw_limit_old < MW_MAX_POWER) + !mw_limit_old]);

    // PV power, nighttime/daytime symbol
    sprintf(tn_str + strlen(tn_str), "\r\n%4d ", int(round(power_pv)));
    strcat(tn_str, NIGHT_DAY_SYMBOL[(min_of_day >= sunrise) && (min_of_day < sunset)]);
    strcat(tn_str, PV_FLOW_SYMBOL[(round(power_pv) > 0) + (round(power_pv) >= PV_MAX_POWER)]);
    strcat(tn_str, CABLE_SYMBOL);
    strcat(tn_str, PV_CABLE_SYMBOL);

    // ESS power
    strcat(tn_str, ESS_CABLE_SYMBOL);
    strcat(tn_str, CABLE_SYMBOL);
    if (!power_old)
        strcat(tn_str, ESS_FLOW_SYMBOL[!mw_limit_old + 2*(!hm_limit_old)]);
    if (power_old > 0)
        strcat(tn_str, MW_FLOW_SYMBOL[(power_old == mw_limit_old) + ((power_old == mw_limit_old) && (mw_limit_old >= MW_MAX_POWER))][power_grid_to_ess > power_pv_to_ess]);
    if (power_old < 0)
        strcat(tn_str, HM_FLOW_SYMBOL[(power_old == hm_limit_old) + (power_old == HM_MAX_POWER)]);
    strcat(tn_str, BAT_LEVEL_SYMBOL[bat_level]);
    sprintf(tn_str + strlen(tn_str), "%d", int(round(power_ess)));
    if (power_new != power_old) strcat(tn_str, DIFF_SYMBOL[(power_new < power_old) + !filter_cycles]);
    else if (filter_cycles && (filter_cycles < POWER_FILTER_CYCLES)) strcat(tn_str, POWERFILTER_SYMBOL);
    if (manual_mode) strcat(tn_str, MANUAL_MODE_SYMBOL);
    if (auto_recharge) strcat(tn_str, AUTO_RECHARGE_SYMBOL);
    strcat(tn_str, "\r\n");

    // House symbol
    strcat(tn_str, HOUSE_SYMBOL);

    // UVP symbol below battery symbol
    strcat(tn_str, BAT_UVP_SYMBOL[(hm_limit_old > HM_MAX_POWER) + !hm_limit_old]);

    // Grid power
    sprintf(tn_str + strlen(tn_str), "\r\n%4d ", int(round(power_grid)));
    strcat(tn_str, GRID_SYMBOL);
    strcat(tn_str, GRID_FLOW_SYMBOL[(int(round(power_grid)) > 0)  + 2*(int(round(power_grid)) < 0)][power_ess_to_grid > power_pv_to_grid]);
    strcat(tn_str, CABLE_SYMBOL);
    strcat(tn_str, GRID_CABLE_SYMBOL);

    // Power to consumers
    strcat(tn_str, CONS_CABLE_SYMBOL);
    strcat(tn_str, CABLE_SYMBOL);
    if (power_grid_to_cons >= max(power_pv_to_cons, power_ess_to_cons)) strcat(tn_str, CONS_FLOW_SYMBOL[0]);
    if (power_ess_to_cons >= max(power_grid_to_cons, power_pv_to_cons)) strcat(tn_str, CONS_FLOW_SYMBOL[1]);
    if (power_pv_to_cons >= max(power_grid_to_cons, power_ess_to_cons)) strcat(tn_str, CONS_FLOW_SYMBOL[2]);
    strcat(tn_str, CONS_SYMBOL);
    sprintf(tn_str + strlen(tn_str), " %.0f\r\n\n", power_cons);

    // Read user command
    if (telnet.available()) command = telnet.read();

    // Append user command response to telnet message
    switch (command) {
        case 'm':
            command = '\0';  // ask for user input only once
            telnet.printf("%s%s", tn_str, "Enter ESS power: ");
            ts_userio = millis();
            while (!telnet.available() && (millis()-ts_userio < USERIO_TIMEOUT));
            if (telnet.available()) {
                power_manual = telnet.parseInt();
                manual_mode = true;
                sprintf(resp_str, "ESS power manually set to %d W\r\n\n", power_manual);
            }
            else {
                if (!manual_mode) strcpy(resp_str, "Automatic mode remains activated\r\n\n");
                else sprintf(resp_str, "ESS power remains at %d W\r\n\n", power_manual);
            }
            ts_userio = millis();
            break;
        case 'a':
            command = '\0';  // switch to automatic mode only once
            manual_mode = false;
            strcpy(resp_str, "Automatic mode activated\r\n\n");
            ts_userio = millis();
            break;
        case 'g':
            command = '\0';  // ask for user input only once
            telnet.printf("%s%s", tn_str, "Enter grid power target: ");
            ts_userio = millis();
            while (!telnet.available() && (millis()-ts_userio < USERIO_TIMEOUT));
            if (telnet.available()) {
                power_grid_target = telnet.parseInt();
                sprintf(resp_str, "Grid power target set to %d W\r\n\n", power_grid_target);
            }
            else sprintf(resp_str, "Grid power target remains at %d W\r\n\n", power_grid_target);
            ts_userio = millis();
            break;
        case 'b':
            // DC (batt) status
            sprintf(tn_str + strlen(tn_str), "Cell voltages: %d - %d mV\r\nMax cell diff: %d mV%s", vcell_min, vcell_max, vcell_max-vcell_min, (bms_bal_active) ? BALANCER_SYMBOL : "");
            sprintf(tn_str + strlen(tn_str), "\r\nBatt voltage : %.3f V\r\nBatt current : %.2f A\r\nBatt power   : %.1f W\r\n", vbat/1000.0, cbat/100.0, pbat);
            sprintf(tn_str + strlen(tn_str), "RS485 retries: %d\r\n\n", bms_resp_wait_counter);
            // AC status
            sprintf(tn_str + strlen(tn_str), "AC power setting: %d W\r\nAC power reading: %.1f W\r\n", power_old, power_ess);
            // AC/DC (or DC/AC) power conversion efficiency of Meanwell or Hoymiles
            sprintf(tn_str + strlen(tn_str), "AC/DC conv. eff.: %.1f %%\r\n", (power_ess >= 0) ? pbat/power_ess*100 : power_ess/pbat*100);
            // Meanwell/Hoymiles power limit settings
            sprintf(tn_str + strlen(tn_str), "MW power limit  : %d W\r\nHM power limit  : %d W\r\n\n", mw_limit, hm_limit);
            resp_str[0] = '\0';
            break;
        case 'd':
            sprintf(tn_str + strlen(tn_str), "ESS public IP address: %s\r\nLast address check   : %02d/%02d/%04d %02d:%02d\r\n", pubip_addr.toString().c_str(), day(pubip_time), month(pubip_time), year(pubip_time), hour(pubip_time), minute(pubip_time));
            sprintf(tn_str + strlen(tn_str), "Last DDNS update     : %02d/%02d/%04d %02d:%02d\r\n\n", day(ddns_time), month(ddns_time), year(ddns_time), hour(ddns_time), minute(ddns_time));
            resp_str[0] = '\0';
            break;
        case 'w':
            sprintf(tn_str + strlen(tn_str), "WiFi RSSI: %d dBm\r\nChip temp: %.1f °C\r\n\n", WiFi.RSSI(), temperatureRead());
            resp_str[0] = '\0';
            break;
        case 't':
            sprintf(tn_str + strlen(tn_str), "Local time   : %02d/%02d/%04d %02d:%02d:%02d", day(unixtime), month(unixtime), year(unixtime), hour(unixtime), minute(unixtime), second(unixtime));
            sprintf(tn_str + strlen(tn_str), "\r\nTimezone     : UTC%+d\r\nSDT/DST      : %s", TIMEZONE, (utc_offset>TIMEZONE)?"DST":"SDT");
            sprintf(tn_str + strlen(tn_str), "\r\nESS started  : %02d/%02d/%04d %02d:%02d:%02d", day(starttime), month(starttime), year(starttime), hour(starttime), minute(starttime), second(starttime));
            sprintf(tn_str + strlen(tn_str), "\r\nESS uptime   : %03dd %02dh %02dm %02ds", (unixtime-starttime)/86400, ((unixtime-starttime)%86400)/3600, ((unixtime-starttime)/60)%60, (unixtime-starttime)%60);
            sprintf(tn_str + strlen(tn_str), "\r\nSunrise today: %02d:%02d\r\nSunset today : %02d:%02d\r\n\n", sunrise/60, sunrise%60, sunset/60, sunset%60); 
            resp_str[0] = '\0';
            break;
        case 'l':
            sprintf(tn_str + strlen(tn_str), "Lowest consumption since %02d/%02d/%04d %02d:%02d\r\n\n", day(starttime), month(starttime), year(starttime), hour(starttime), minute(starttime));
            if (!minpower_time) strcat(tn_str, "Not yet measured\r\n\n");
            else sprintf(tn_str + strlen(tn_str), "%.1f W (measured %02d/%02d/%04d %02d:%02d)\r\n\n", power_grid_min, day(minpower_time), month(minpower_time), year(minpower_time), hour(minpower_time), minute(minpower_time));
            resp_str[0] = '\0';
            break;
        case 'n':
            sprintf(tn_str + strlen(tn_str), "Energy flows [kWh] since %02d/%02d/%04d %02d:%02d\r\n\n", day(resettime_energy), month(resettime_energy), year(resettime_energy), hour(resettime_energy), minute(resettime_energy));
            // PV energy
            sprintf(tn_str + strlen(tn_str), "%7.3f %s%s%s%s%s%s %.1f﹪+%.1f﹪\r\n", en_from_pv/1000, NIGHT_DAY_SYMBOL[1], PV_FLOW_SYMBOL[1], PV_CABLE_SYMBOL, ESS_CABLE_SYMBOL, MW_FLOW_SYMBOL[0][0], ESS_SYMBOL, (en_pv_to_ess-en_pv_wasted)/en_from_pv*100, en_pv_wasted/en_from_pv*100);
            sprintf(tn_str + strlen(tn_str), "  %s\r\n", HOUSE_SYMBOL);
            sprintf(tn_str + strlen(tn_str), "%6.1f﹪%s%s%s%s%s%s %.1f﹪\r\n\n", en_pv_to_grid/en_from_pv*100, GRID_SYMBOL, GRID_FLOW_SYMBOL[2][0], GRID_CABLE_SYMBOL, CONS_CABLE_SYMBOL, CONS_FLOW_SYMBOL[2], CONS_SYMBOL, en_pv_to_cons/en_from_pv*100);
            // ESS energy
            sprintf(tn_str + strlen(tn_str), "                %s%s%s %.3f\r\n", ESS_CABLE_SYMBOL, HM_FLOW_SYMBOL[0], ESS_SYMBOL, en_from_ess/1000);
            sprintf(tn_str + strlen(tn_str), "  %s\r\n", HOUSE_SYMBOL);
            sprintf(tn_str + strlen(tn_str), "%6.1f﹪%s%s%s%s%s%s %.1f﹪\r\n\n", en_ess_to_grid/en_from_ess*100, GRID_SYMBOL, GRID_FLOW_SYMBOL[2][1], GRID_CABLE_SYMBOL, CONS_CABLE_SYMBOL, CONS_FLOW_SYMBOL[1], CONS_SYMBOL, en_ess_to_cons/en_from_ess*100);
            // Grid energy
            sprintf(tn_str + strlen(tn_str), "                %s%s%s %.1f﹪\r\n", ESS_CABLE_SYMBOL, MW_FLOW_SYMBOL[0][1], ESS_SYMBOL, en_grid_to_ess/en_from_grid*100);
            sprintf(tn_str + strlen(tn_str), "  %s\r\n", HOUSE_SYMBOL);
            sprintf(tn_str + strlen(tn_str), "%7.3f %s%s%s%s%s%s %.1f﹪\r\n\n", en_from_grid/1000, GRID_SYMBOL, GRID_FLOW_SYMBOL[1][0], GRID_CABLE_SYMBOL, CONS_CABLE_SYMBOL, CONS_FLOW_SYMBOL[0], CONS_SYMBOL, en_grid_to_cons/en_from_grid*100);
            // Meanwell/Hoymiles/ESS efficiency
            sprintf(tn_str + strlen(tn_str), "MW AC▸DC eff.: %.1f﹪\r\nHM DC▸AC eff.: %.1f﹪\r\nAC▸DC▸AC eff.: %.1f﹪\r\n\n", en_to_batt/en_to_ess*100, en_from_ess/en_from_batt*100, en_from_ess/en_to_ess*100);
            resp_str[0] = '\0';
            break;
        case 'e':
            sprintf(tn_str + strlen(tn_str), "Errors since %02d/%02d/%04d %02d:%02d\r\n\n", day(resettime_errors), month(resettime_errors), year(resettime_errors), hour(resettime_errors), minute(resettime_errors));
            for (int i=0; i<ERROR_TYPES; i++) {
                sprintf(tn_str + strlen(tn_str), "%-4s: %3d", ERROR_TYPE[i], error_counter[i]);
                if (error_counter[i]) sprintf(tn_str + strlen(tn_str), " (last: %02d/%02d/%04d %02d:%02d)", day(errortime[i]), month(errortime[i]), year(errortime[i]), hour(errortime[i]), minute(errortime[i]));
                strcat(tn_str, "\r\n");
            }
            strcat(tn_str, "\n");
            if (error_str[0] != '\0') sprintf(tn_str + strlen(tn_str), "%s%s\r\n\n", ERROR_SYMBOL, error_str);  // error has just occured: show ERROR_SYMBOL
            else if (last_error_str[0] != '\0') sprintf(tn_str + strlen(tn_str), "Last: %s\r\n\n", last_error_str);
            error_flag = false;
            resp_str[0] = '\0';
            break;
        case 's':
            sprintf(tn_str + strlen(tn_str), "Shelly relay ops since %02d/%02d/%04d %02d:%02d\r\n\n", day(starttime), month(starttime), year(starttime), hour(starttime), minute(starttime));
            sprintf(tn_str + strlen(tn_str), "Meanwell relay: %d (%d/day)\r\nHoymiles relay: %d (%d/day)\r\n\n", mw_counter, mw_counter/((unixtime-starttime)/86400+1), hm_counter, hm_counter/((unixtime-starttime)/86400+1));
            resp_str[0] = '\0';
            break;
        case 'z':
            command = '\0';  // ask for user input only once
            telnet.printf("%s%s", tn_str, "Reset [e]rror or e[n]ergy stats: ");
            ts_userio = millis();
            while (!telnet.available() && (millis()-ts_userio < USERIO_TIMEOUT));
            if (telnet.available()) {
                char input = telnet.read();
                if (input == 'n') {
                    en_from_pv = en_pv_to_cons = en_pv_to_ess = en_pv_to_grid = en_pv_wasted = 0;  // Reset PV energy counters
                    en_from_grid = en_grid_to_cons = en_grid_to_ess = 0;  // Reset grid energy counters
                    en_from_ess = en_to_ess = en_ess_to_cons = en_ess_to_grid = 0;  // Reset ESS energy counters
                    en_from_batt = en_to_batt = 0;  // Reset ESS DC energy counters
                    resettime_energy = unixtime;
                    strcpy(resp_str, "Energy stats reset to zero\r\n\n");
                    ts_userio = millis();
                    break;
                }
                if (input == 'e') {
                    for (int i=0; i<ERROR_TYPES; i++) error_counter[i] = errortime[i] = 0;
                    errors_consecutive = 0;
                    last_error_str[0] = '\0';
                    error_flag = false;
                    resettime_errors = unixtime;
                    strcpy(resp_str, "Error stats reset to zero\r\n\n");
                    ts_userio = millis();
                    break;
                }
            }
            strcpy(resp_str, "No stats reset\r\n\n");
            ts_userio = millis();
            break;
        case 'c':
            command = resp_str[0] = '\0';
            break;
        case 'r':
            command = '\0';  // ask for user input only once
            telnet.printf("%s%s", tn_str, "Enter [y] to confirm system reboot: ");
            ts_userio = millis();
            while (!telnet.available() && (millis()-ts_userio < USERIO_TIMEOUT));
            if (telnet.available()) {
                if (telnet.read() == 'y') {
                    telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
                    delay(2000);
                    telnet.stop();
                    delay(1000);
                    ESP.restart();
                }
            }
            strcpy(resp_str, "System not rebooted\r\n\n");
            ts_userio = millis();
            break;
        case 'h':
            strcat(tn_str, "Command options:\r\n");
            strcat(tn_str, "[m] - Manual power mode\r\n");
            strcat(tn_str, "[a] - Automatic power mode\r\n");
            strcat(tn_str, "[g] - Grid power target\r\n");
            strcat(tn_str, "[b] - Battery info\r\n");
            strcat(tn_str, "[d] - DDNS info\r\n");
            strcat(tn_str, "[w] - WiFi RSSI\r\n");
            strcat(tn_str, "[t] - Time, uptime, astro times\r\n");
            strcat(tn_str, "[l] - Lowest household consumption\r\n");
            strcat(tn_str, "[n] - Energy stats\r\n");
            strcat(tn_str, "[e] - Error stats\r\n");
            strcat(tn_str, "[s] - Shelly relay counter\r\n");
            strcat(tn_str, "[z] - Reset stats to zero\r\n");
            strcat(tn_str, "[c] - Clear command response\r\n");
            strcat(tn_str, "[r] - Reboot system\r\n\n");
            resp_str[0] = '\0';
            break;
    }
    if (millis()-ts_userio > USERIO_TIMEOUT) resp_str[0] = '\0';  // clear command response if shown long enough
    telnet.printf("%s%s%s", tn_str, resp_str, CMD_PROMPT);  // print output message and user command response
}

bool UpdateDDNS() {

    if (WiFi.status() != WL_CONNECTED) {
        strcpy(error_str, "WIFI");
        return false;
    }

    // read public IP from server (with shorter timeout)
    http.setTimeout(HTTP_DDNS_TIMEOUT);
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
        strcpy(error_str, "DDNS update failed");
        return false;
    }
    http.end();
    // Read IP command frequently fails, no big deal, so don't report
    return false;
}
