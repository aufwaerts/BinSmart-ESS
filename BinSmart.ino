const char SW_VERSION[] = "v3.02";

#include <WiFi.h>  // standard Arduino/ESP32
#include <WebServer.h>  // standard Arduino/ESP32
#include <ElegantOTA.h>  // https://github.com/ayushsharma82/ElegantOTA (I prefer v2.2.9)
#include <RF24.h>  // https://nrf24.github.io/RF24/
#include <CRC8.h>  // https://github.com/RobTillaart/CRC
#include <CRC16.h>  // https://github.com/RobTillaart/CRC
#include <TimeLib.h>  // https://playground.arduino.cc/Code/Time/
#include <Dusk2Dawn.h>  // https://github.com/dmkishi/Dusk2Dawn
#include <NimBLEDevice.h>  // https://github.com/h2zero/NimBLE-Arduino
#include "BinSmart_scfg.h"  // software configuration (sensitive data)
#include "BinSmart_cfg.h"  // software and system configuration
#include "BinSmart_var.h"  // global variables

void setup() {

    // Reduce CPU clock to 80 MHz (enough for this application)
    // setCpuFrequencyMhz(80);  // REPLACED BY ARDUINO COMPILER SETTING

    // Assign LED Pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Init WiFi
    WiFi.config(ESP32_ADDR, ROUTER_ADDR, SUBNET, DNS_SERVER1, DNS_SERVER2);
    WiFi.setTxPower(WIFI_POWER_5dBm);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(true);
    WiFi.begin(WIFI_SSID, WIFI_PWD);
    while (WiFi.status() != WL_CONNECTED);   // if WiFi unavailable or wrong SSID/PWD, system stops here and LED remains on
    digitalWrite(LED_PIN, LOW);

    // Start OTA software update service
    ElegantOTA.begin(&OTA_server, OTA_UID, OTA_PWD);
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
    if (ledcAttachChannel(PWM_OUTPUT_PIN, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL) && ledcWrite(PWM_OUTPUT_PIN, DUTY_CYCLE_MAX)) telnet.println("PWM generator initialized");
    else strcpy(error_str, "PWM generator init failed");
    CheckErrors();
    
    // Init RS485 communication with BMS, read OVP/UVP/balancer/switch settings
    Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    if (BMSCommand(RS485_READ_SETTINGS)) telnet.println("RS485 communication with JKBMS OK");
    CheckErrors();

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

    // Turn off Hoymiles power (if Hoymiles isn't asleep)
    if (!bms_disch_on) telnet.println("Hoymiles is asleep");
    else if (HoymilesCommand(HM_TURNOFF)) telnet.println("RF24 communication with Hoymiles OK");
    CheckErrors();

    // Set max waiting time for all http requests
    http.setTimeout(HTTP_TIMEOUT);

    // Shelly 1PM: Read eco mode setting and PV power
    if (ShellyCommand(PM1_ADDR, PM_CONFIG) && ShellyCommand(PM1_ADDR, PM_CH0_STATUS)) telnet.println("Shelly 1PM found");
    CheckErrors();

    // Shelly 2PM: Read eco mode setting, turn off Meanwell relay, make sure Hoymiles relay state matches BMS discharging switch
    if (ShellyCommand(PM2_ADDR, PM_CONFIG) && ShellyCommand(PM2_ADDR, PM_CH0_OFF)) telnet.println("Shelly 2PM found");
    mw_counter = 0;
    CheckErrors();
    if (!bms_disch_on) ShellyCommand(PM2_ADDR, PM_CH1_OFF);
    else ShellyCommand(PM2_ADDR, PM_CH1_ON);
    hm_counter = 0;
    CheckErrors();
    
    // Shelly 3EM: Read local time and grid power, init timestamps
    if (ShellyCommand(EM_ADDR, EM_STATUS)) telnet.println("Shelly 3EM found");
    CheckErrors();

    // Check public IP address and update DDNS server entry
    if (UpdateDDNS()) telnet.println("DDNS server updated");
    CheckErrors();

    telnet.print("\nNo errors during setup, start polling cycle ...");
    start_uxt = errors_uxt = energy_uxt = unixtime;  // called by setup(): init timestamps
    ts_cycle = millis();
    delay(PROCESSING_DELAY);
}

void loop() {

    ShellyCommand(EM_ADDR, EM_STATUS);  // read time and grid power from Shelly 3EM
    BMSCommand(RS485_READ_VOLTAGES);  // read cell voltages from BMS
    ShellyCommand(PM2_ADDR, (power_new > 0) ? PM_CH0_STATUS : PM_CH1_STATUS); // read ESS AC power from Shelly 2PM
    BMSCommand(RS485_READ_CURRENT);  // read charging/discharging current and DC power from BMS
    SetNewPower();  // calculate charging/discharging power limits and setting, apply new power setting
    FinishCycle();  // update energy stats, do maintenance tasks, print status info, handle user command, read PV power, flash LED
    CheckErrors();  // check for comms errors, halt system if an error is persistent
}

bool ShellyCommand(const IPAddress ip_addr, const char command[]) {

    if (ip_addr == PM2_ADDR) {
        if (!strncmp(command, PM_CH0_STATUS, 10)) {  // command is "read ESS power from Shelly 2PM"
            power_ess = power_new;  // assumption: power reading equals power setting
            if (!power_new) return true;  // no need to read ESS power from Shelly 2PM if ESS is turned off
        }
    }

    if (WiFi.status() != WL_CONNECTED) {  // no need to carry on if WiFi is disconnected
        strcpy(error_str, "WIFI disconnected");
        return false;
    }

    // Prepare Shelly http command
    sprintf(http_command, "GET %s HTTP/1.1\r\nHost: %s\r\n\r\n", command, ip_addr.toString().c_str());

    // connect to Shelly and send command
    if (http.connect(ip_addr, HTTP_PORT)) {
        http.write(http_command, strlen(http_command));
        if (http.find(HTTP_OK)) {
            if (!strcmp(command, EM_STATUS)) {
                // read time and grid power from Shelly 3EM
                http.readBytes(http_resp, 220);  // fast forward to "time"
                http.find("time");
                min_of_day = http.parseInt()*60 + http.parseInt();  // read local time (minutes after midnight)
                unsigned long unixtime_utc = http.parseInt();  // read epoch time (UTC)
                utc_offset = (min_of_day/60+24-hour(unixtime_utc))%24;  // UTC offset = timezone + dst
                unixtime = unixtime_utc + utc_offset*3600;  // unixtime equals local time
                http.readBytes(http_resp, 550);  // fast forward to "total_power"
                http.find("total_power");
                power_grid = http.parseFloat();  // read grid power from http response
            }
            if (!strncmp(command, PM_CH0_STATUS, 10)) {
                // read power from Shelly 1PM or 2PM
                http.find("apower");
                if (ip_addr == PM1_ADDR) power_pv = http.parseFloat();
                if (ip_addr == PM2_ADDR) {
                    power_ess = http.parseFloat();
                    if (power_new > 0) power_ess = power_ess*PM2_MW_POWER_CORR;  // power correction for positive (charging) power
                }
            }
            if (!strcmp(command, PM_CH0_ON)) {  // Meanwell relay turned on
                ts_MW = millis();  // reset Shelly MW timer
                mw_counter += !mw_on;
                mw_on = true;
            }
            if (!strcmp(command, PM_CH0_OFF)) {  // Meanwell relay turned off
                mw_counter += mw_on;
                mw_on = false;
                ledcWrite(PWM_OUTPUT_PIN, 0);  // MW turned off: also turn off optocoupler LED
            }
            if (!strcmp(command, PM_CH1_ON)) {  // Hoymiles relay turned on
                hm_counter += !hm_on;
                hm_on = true;
            }
            if (!strcmp(command, PM_CH1_OFF)) {  // Hoymiles relay turned off
                hm_counter += hm_on;
                hm_on = false;
            }
            if (!strcmp(command, PM_CONFIG)) {
                bool eco_mode = http.findUntil("eco_mode\":true", "eco_mode\":false");
                if (ip_addr == PM1_ADDR) pm1_eco_mode = eco_mode;
                if (ip_addr == PM2_ADDR) pm2_eco_mode = eco_mode;
            }
            http.stop();
            return true;
        }
    }
    // Shelly command failed
    http.stop();
    if (ip_addr == EM_ADDR) strcpy(error_str, "3EM");
    if (ip_addr == PM1_ADDR) strcpy(error_str, "1PM");
    if (ip_addr == PM2_ADDR) strcpy(error_str, "2PM");
    sprintf(error_str + strlen(error_str), " cmd failed: %s%s", ip_addr.toString().c_str(), command);
    return false;
}

bool BMSCommand(const byte command[]) {

    if ((command[RS485_DATA_ID_POS] == RS485_CURRENT_ID) && !power_new) {
        cbat = pbat = 0;  // batt current and power is zero if ESS is turned off
        return true;
    }

    while (millis()-ts_BMS < BMS_WAIT);  // minimum delay after previous BMS response

    if (command[0] == BLE_ID1) {
        // Send BMS command via BLE
        if (pClient->connect()) {
            NimBLERemoteService* pService = pClient->getService("FFE0");
            NimBLERemoteCharacteristic* pChar = pService->getCharacteristic("FFE1");
            if (pChar->writeValue(BLE_GET_INFO, BLE_COMMAND_LEN, false)) {
                delay(500);
                if (pChar->writeValue(BLE_GET_DATA, BLE_COMMAND_LEN, false)) {
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
    Serial2.flush(false);  // wait until all tx bytes are sent, clear rx buffer

    memset(bms_resp, 0x00, sizeof(bms_resp));  // will result in failed validity check if no response
    bms_resp[3] = RS485_LEN_POS;  // make sure "response incomplete" check doesn't terminate too early
    int len = 0, checksum = 0;  // response length and checksum

    // Wait for BMS response
    ts_BMS = millis();
    while ((millis()-ts_BMS < BMS_TIMEOUT1) && !Serial2.available());
    
    // fill response buffer with BMS response
    while (Serial2.available()) {
        bms_resp[len] = Serial2.read();
        checksum += bms_resp[len];
        len++;
        if (!Serial2.available() && (len < (bms_resp[2]<<8|bms_resp[3])+2)) {  // response incomplete
            ts_BMS = millis();
            while ((millis()-ts_BMS < BMS_TIMEOUT2) && !Serial2.available());  // wait for next response byte to arrive
        }
    }
    ts_BMS = millis();  // set "end of BMS response" timestamp

    // check validity of BMS response
    if ((bms_resp[0] == RS485_ID1) && (bms_resp[1] == RS485_ID2)) {  // start of response frame
         if ((bms_resp[2]<<8|bms_resp[3])+2 == len) {  // response length
             if ((bms_resp[len-2]<<8|bms_resp[len-1]) == checksum-bms_resp[len-2]-bms_resp[len-1]) {  // response checksum

                if (command[RS485_COMMAND_POS] == RS485_READ_ALL) {  // process response to "read all" command
                    if ((bms_resp[RS485_OVP_POS]<<8|bms_resp[RS485_OVP_POS+1])-(bms_resp[RS485_OVPR_POS]<<8|bms_resp[RS485_OVPR_POS+1]) < VCELL_PROT_OFFSET)
                        sprintf(error_str, "BMS Cell OVPR less than %d mV below BMS Cell OVP", VCELL_PROT_OFFSET);
                    if ((bms_resp[RS485_OVPR_POS]<<8|bms_resp[RS485_OVPR_POS+1])-VCELL_OVP < VCELL_PROT_OFFSET)
                        sprintf(error_str, "ESS OVP voltage less than %d mV below BMS Cell OVPR", VCELL_PROT_OFFSET);
                    if (VCELL_OVP-VCELL_OVPR < VCELL_PROT_OFFSET)
                        sprintf(error_str, "ESS OVPR voltage less than %d mV below ESS OVP voltage", VCELL_PROT_OFFSET);
                    if (VCELL_UVPR-VCELL_UVP < VCELL_PROT_OFFSET)
                        sprintf(error_str, "ESS UVPR voltage less than %d mV above ESS UVP voltage", VCELL_PROT_OFFSET);
                    if (VCELL_UVP-(bms_resp[RS485_UVPR_POS]<<8|bms_resp[RS485_UVPR_POS+1]) < VCELL_PROT_OFFSET)
                        sprintf(error_str, "ESS UVP voltage less than %d mV above BMS Cell UVPR", VCELL_PROT_OFFSET);
                    if ((bms_resp[RS485_UVPR_POS]<<8|bms_resp[RS485_UVPR_POS+1])-(bms_resp[RS485_UVP_POS]<<8|bms_resp[RS485_UVP_POS+1]) < VCELL_PROT_OFFSET)
                        sprintf(error_str, "BMS Cell UVPR less than %d mV above BMS Cell UVP", VCELL_PROT_OFFSET);
                    bms_balancer_start = bms_resp[RS485_BAL_ST_POS]<<8|bms_resp[RS485_BAL_ST_POS+1];  // balancer start voltage
                    bms_balancer_trigger = bms_resp[RS485_BAL_TR_POS]<<8|bms_resp[RS485_BAL_TR_POS+1];  // balancer trigger voltage
                    bms_bal_on = bms_resp[RS485_BAL_SW_POS];  // state of balancer
                    bms_disch_on = bms_resp[RS485_DISCH_SW_POS];  // Hoymiles asleep? (i.e. BMS discharge switch turned off)
                    return true;
                }

                switch (command[RS485_DATA_ID_POS]) {
                    case RS485_DISCH_SW_ID:  // process response to "set discharge switch"
                        ts_HM = millis();  // gives Hoymiles RF24 interface time to boot before (re-)starting RF24 keep alive messages
                        return true;
                    case RS485_VCELLS_ID:  // process response to "read cell voltages"
                        vbat = vcell_max = 0; vcell_min = 4000;
                        for (int i=RS485_DATA_ID_POS+3; i<=RS485_DATA_ID_POS+bms_resp[RS485_DATA_ID_POS+1]; i+=3) {
                            int vcell = bms_resp[i]<<8|bms_resp[i+1];
                            if (vcell < vcell_min) vcell_min = vcell;
                            if (vcell > vcell_max) vcell_max = vcell;
                            vbat += vcell;
                        }
                        return true;
                    case RS485_CURRENT_ID:  // process response to "read battery current"
                        cbat = (bms_resp[RS485_DATA_ID_POS+1]&0x7F)<<8|bms_resp[RS485_DATA_ID_POS+2];
                        if (!(bms_resp[RS485_DATA_ID_POS+1]&0x80)) cbat = -cbat; // charging current is positive, discharging current is negative
                        pbat = (vbat/1000.0)*(cbat/100.0);
                        return true;
                }
            }
        } 
    }
    sprintf(error_str, "BMS RS485 command 0x%02X 0x%02X 0x%02X failed", command[RS485_COMMAND_POS], command[RS485_DATA_ID_POS], command[RS485_DATA_ID_POS+1]);
    return false;
}

bool HoymilesCommand(const byte command) {

    unsigned int crc;  // result of CRC-16/MODBUS calculations

    while (millis()-ts_HM < RF24_WAIT);  // minimum delay between two consecutive Hoymiles RF24 commands

    switch (command) {
        case HM_TURNON:
        case HM_TURNOFF:
            hm_switch_command[10] = command;
            crc16.restart();
            crc16.add(&hm_switch_command[10],2);
            crc = crc16.getCRC();
            hm_switch_command[12] = highByte(crc);
            hm_switch_command[13] = lowByte(crc);
            crc8.restart();
            crc8.add(hm_switch_command, 14);
            hm_switch_command[14] = crc8.getCRC();
            if (radio.writeFast(hm_switch_command, sizeof(hm_switch_command)))
                if (radio.txStandBy(RF24_TIMEOUT)) {
                    ts_HM = millis();
                    return true;
                }
            ts_HM = millis();
            sprintf(error_str, "RF24 Hoymiles turn%s command failed", (command == HM_TURNON) ? "on": "off");
            return false;
        case HM_POWERLIMIT:
            unsigned int limit = -10 * power_new;
            hm_power_command[12] = highByte(limit);
            hm_power_command[13] = lowByte(limit);
            crc16.restart();
            crc16.add(&hm_power_command[10],6);
            crc = crc16.getCRC();
            hm_power_command[16] = highByte(crc);
            hm_power_command[17] = lowByte(crc);
            crc8.restart();
            crc8.add(hm_power_command, 18);
            hm_power_command[18] = crc8.getCRC();
            if (radio.writeFast(hm_power_command, sizeof(hm_power_command)))
                if (radio.txStandBy(RF24_TIMEOUT)) {
                        ts_HM = millis();
                        return true;
                }
            ts_HM = millis();
            strcpy(error_str, "RF24 Hoymiles power command failed");
            return false;
    }
}

void SetNewPower() {

    // Save charging/discharging power limits and power setting from previous cycle,as calc baseline and for UserIO()
    power_old = power_new;
    mw_limit_old = mw_limit;
    hm_limit_old = hm_limit;

    // Re-calculate power limits (depending on vcell_max/vcell_min)
    if ((vcell_max <= VCELL_OVPR) || (mw_limit >= MW_MAX_POWER)) mw_limit = round(MW_MAX_POWER_FORMULA);  // exit OVP mode (update MW power limit)
    if (vcell_max >= VCELL_OVP) {  // OVP cell voltage reached
        if ((mw_limit <= MW_MIN_POWER) || !power_old) mw_limit = 0;  // enter (or remain in) OVP mode
        else mw_limit = max(int(round(power_old*POWER_LIMIT_RAMPDOWN)), MW_MIN_POWER);  // decrease charging power limit softly
    }
    if (vcell_min >= VCELL_UVPR) hm_limit = HM_MAX_POWER;  // exit UVP mode (reset HM power limit)
    if (vcell_min <= VCELL_UVP) {  // UVP cell voltage reached
        if ((hm_limit >= HM_MIN_POWER) || !power_old) hm_limit = 0;  // enter (or remain in) UVP mode
        else hm_limit = min(int(round(power_old*POWER_LIMIT_RAMPDOWN)), HM_MIN_POWER);  // decrease discharging power limit softly
    }
    if (!bms_disch_on) hm_limit = 0;  // prevent discharging if BMS discharging switch is off

    // auto recharging overrides manual power setting and calculation
    if (auto_recharge) power_new = MW_MAX_POWER/2;  // medium charging power, maximum AC/DC efficiency
    else {
        if (manual_mode) power_new = power_manual;  // manual power setting overrides calculation
        else {
            // Calculate new power setting
            int target_deviation = round(power_grid - power_grid_target);
            int positive_tolerance = ((power_old >= 0) || (power_old <= HM_LOW_POWER_THRESHOLD)) ? POWER_TARGET_TOLERANCE : HM_LOW_POWER_TOLERANCE;
            if ((target_deviation < -POWER_TARGET_TOLERANCE) || (target_deviation > positive_tolerance)) power_new = power_old - target_deviation;
        }
    }

    // Make sure charging/discharging power limits are not exceeded
    if (power_new > mw_limit) power_new = mw_limit;
    if (power_new < hm_limit) power_new = hm_limit;

    // filter out power spikes and rampdown ESS power (reduces power loss to grid when consumer is turned on and quickly turned off again)
    if ((power_new - power_old < POWER_RAMPDOWN_RATE) || ((power_old >= MW_MIN_POWER) && (power_new < MW_MIN_POWER))) {
        if (power_old > MW_MIN_POWER) filter_cycles = 0;  // charging power above MW_MIN_POWER: start rampdown immediately
        else filter_cycles = max(filter_cycles-1, 0);  // countdown filter cycles
        if (filter_cycles) power_new = power_old;  // filter out power spikes
        else {
            if (power_new - power_old < POWER_RAMPDOWN_RATE) power_new = power_old + POWER_RAMPDOWN_RATE;  // ramp down ESS power after filtering out power spikes
            if ((power_old > MW_MIN_POWER) && (power_new < MW_MIN_POWER)) {
                power_new = MW_MIN_POWER;  // don't skip MW_MIN_POWER during power rampdown
                filter_cycles = POWER_FILTER_CYCLES;  // start filter cycle countdown before turning off MW (reduces MW relay ops)
            }
        }
    }
    else filter_cycles = POWER_FILTER_CYCLES;  // reset filter cycle countdown

    // Turn ESS off if power is between MW and HM operating ranges
    if ((power_new > HM_MIN_POWER) && (power_new < MW_MIN_POWER)) power_new = 0;

    // Apply new power setting
    if (power_new == 0) {  // turn charging or discharging off
        if (power_old < 0)
            if (!HoymilesCommand(HM_TURNOFF)) power_new = power_old;
        if (power_old > 0)
            if (!ShellyCommand(PM2_ADDR, PM_CH0_OFF)) power_new = MW_MIN_POWER;
    }
    if (power_new > 0) {  // set new charging power, (if necessary) turn discharging off, turn charging on
        if (power_old > 0) SetMWPower(power_new);  // re-calculate even if power setting remains unchanged (vbat might have changed)
        if (power_old == 0) {
            SetMWPower(power_new);
            if (!ShellyCommand(PM2_ADDR, PM_CH0_ON)) power_new = 0;
        }
        if (power_old < 0) {
            if (!HoymilesCommand(HM_TURNOFF)) power_new = power_old;
            else {
                delay(200);
                SetMWPower(power_new);
                if (!ShellyCommand(PM2_ADDR, PM_CH0_ON)) power_new = 0;
            }
        }
    }
    if (power_new < 0) {  // set new discharging power, (if necessary) turn charging off, turn discharging on
        if (power_old < 0) {
            if (power_new != power_old)
                if (!HoymilesCommand(HM_POWERLIMIT)) power_new = power_old;
        }
        if (power_old == 0) {
            if (!HoymilesCommand(HM_POWERLIMIT)) power_new = 0;
            else if (!HoymilesCommand(HM_TURNON)) power_new = 0;
        }
        if (power_old > 0)
            if (!ShellyCommand(PM2_ADDR, PM_CH0_OFF)) power_new = MW_MIN_POWER;
            else {
                delay(200);
                if (!HoymilesCommand(HM_POWERLIMIT)) power_new = 0;
                else if (!HoymilesCommand(HM_TURNON)) power_new = 0;
            }
    }
    
    // Calculate cycle duration and reset cycle timestamp
    msecs_cycle = millis()-ts_cycle;
    ts_cycle = millis();
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

    // Check/set automatic battery recharging (prevents battery damage)
    if (vcell_min <= VCELL_UVP-VCELL_PROT_OFFSET) auto_recharge = true;
    if (vcell_min >= VCELL_UVP) auto_recharge = false;

    if (!sunrise || min_of_day == 210) {  // calculate sunrise/sunset times at 03:30 local time (after a possible SDT/DST change, before sunrise)
        sunrise = ess_location.sunrise(year(unixtime), month(unixtime), day(unixtime), (utc_offset != TIMEZONE));
        sunset = ess_location.sunset(year(unixtime), month(unixtime), day(unixtime), (utc_offset != TIMEZONE));
    }

    // No ESS power output and no PV production: check for new power_grid_min (i.e. min household consumption)
    if (!power_old && !hm_limit_old && !power_pv && (power_grid < power_grid_min)) {
        power_grid_min = power_grid;
        minpower_uxt = unixtime;
    }

    // Keep alive Hoymiles RF24 interface (if Hoymiles is awake)
    if ((millis()-ts_HM >= RF24_KEEPALIVE*1000) && bms_disch_on)
        if (power_new < 0) HoymilesCommand(HM_TURNON);
        else HoymilesCommand(HM_TURNOFF);
    
    // Make sure Hoymiles relay state matches BMS discharging switch (i.e. Hoymiles AC on/off state matches DC on/off state)
    if (hm_on != bms_disch_on) ShellyCommand(PM2_ADDR, (bms_disch_on) ? PM_CH1_ON : PM_CH1_OFF);

    // Keep alive Meanwell relay (if Meanwell is turned on)
    if (mw_on && (millis()-ts_MW >= MW_KEEPALIVE*1000)) ShellyCommand(PM2_ADDR, PM_CH0_ON);

    // Set Shelly 1PM eco mode (turn off just before sunrise, turn on at sunset)
    if ((min_of_day >= sunrise-1) && (min_of_day < sunset) && pm1_eco_mode) pm1_eco_mode = !ShellyCommand(PM1_ADDR, PM_ECO_MODE_OFF);
    if (((min_of_day < sunrise-1) || (min_of_day >= sunset)) && !pm1_eco_mode) pm1_eco_mode = ShellyCommand(PM1_ADDR, PM_ECO_MODE_ON);

    // Set Shelly 2PM eco mode (turn off when charging/discharging, turn on when charging/discharging inactive and impossible)
    if ((power_new || (power_pv >= MW_MIN_POWER-power_grid_target)) && pm2_eco_mode) pm2_eco_mode = !ShellyCommand(PM2_ADDR, PM_ECO_MODE_OFF);
    if (!power_new && !power_pv && !bms_disch_on && !pm2_eco_mode) pm2_eco_mode = ShellyCommand(PM2_ADDR, PM_ECO_MODE_ON);

    // Turn on/off BMS balancer (disable/enable bottom balancing), depending on lowest cell voltage
    if ((vcell_min <= VCELL_UVP) && !bms_bal_on) bms_bal_on = BMSCommand(BLE_BAL_ON);  // turn on balancer when VCELL_UVP is reached while discharging
    if ((vcell_min >= VCELL_UVP + 20) && bms_bal_on) bms_bal_on = !BMSCommand(BLE_BAL_OFF);  // offset of 20 mV makes sure that balancer will only be turned off by charging

    // Turn on/off BMS discharge switch (wake HM or make it fall asleep), depending on lowest cell voltage and hm_limit
    if ((vcell_min >= VCELL_UVPR-10) && !bms_disch_on) bms_disch_on = BMSCommand(RS485_DISCH_ON);  // wake up HM 10 mV below VCELL_UVPR, give HM time to boot and sync AC
    if ((vcell_min <= VCELL_UVP) && !hm_limit && bms_disch_on) bms_disch_on = !BMSCommand(RS485_DISCH_OFF);  // make HM fall asleep after hm_limit reaches zero

    // Clear Shelly 3EM energy data at 23:00 UTC (prevents HTTP timeouts at 00:00 UTC due to internal data reorgs)
    if ((min_of_day/60 == (23+utc_offset)%24) && !em_data_cleared) em_data_cleared = ShellyCommand(EM_ADDR, EM_RESET);
    if (min_of_day/60 == utc_offset) em_data_cleared = false;

    // Check if public IP address was changed, if yes: update DDNS server entry
    if (unixtime-pubip_uxt >= DDNS_UPDATE_INTERVAL) UpdateDDNS();

    // check for OTA software update
    OTA_server.handleClient();

    // check for telnet session
    if (!telnet) telnet = telnet_server.available();
    if (telnet) UserIO();  // if telnet session exists, print cycle info and handle user command
    else command = resp_str[0] = '\0';  // no telnet session: clear user command response

    // flash LED to indicate end of cycle
    digitalWrite(LED_PIN, HIGH);  delay(20); digitalWrite(LED_PIN, LOW);

    // wait until power change has almost stabilized (allow time for PV power reading)
    int cycle_delay = PROCESSING_DELAY*(1+(pm1_eco_mode && pm2_eco_mode))-(millis()-ts_cycle)-100;
    if (cycle_delay > 0) delay(cycle_delay);

    // daytime: read PV power from Shelly 1PM (should take less than 100 ms, including LED flash)
    if ((min_of_day >= sunrise) && (min_of_day < sunset)) ShellyCommand(PM1_ADDR, PM_CH0_STATUS);
    else power_pv = 0;

    // wait until power change has stabilized
    cycle_delay = PROCESSING_DELAY*(1+(pm1_eco_mode && pm2_eco_mode))-(millis()-ts_cycle);
    if (cycle_delay > 0) delay(cycle_delay);
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
    if ((errors_consecutive < ERROR_LIMIT) && start_uxt) return;  // continue with next cycle if below ERROR_LIMIT and no error during setup()

    // Error is persistent or error during setup(): Halt the system
    if (!HoymilesCommand(HM_TURNOFF)) ShellyCommand(PM2_ADDR, PM_CH1_OFF);
    SetMWPower(MW_MIN_POWER); delay(20);
    ShellyCommand(PM2_ADDR, PM_CH0_OFF);
    
    // System halted: prepare continuous error message
    sprintf(cycle_str, "%sBinSmart ESS %s\r\n\n%sSystem halted ", CLEAR_SCREEN, SW_VERSION, ERROR_SYMBOL);
    if (start_uxt) sprintf(cycle_str + strlen(cycle_str), "at %02d/%02d/%04d %02d:%02d:%02d\r\n   after %d consecutive errors\r\n\n", day(unixtime), month(unixtime), year(unixtime), hour(unixtime), minute(unixtime), second(unixtime), errors_consecutive);
    else strcat(cycle_str, "during setup\r\n\n");
    strcat(cycle_str, "Last error: ");
    strcat(cycle_str, last_error_str);
    strcat(cycle_str, "\r\n\nPress any key to restart: ");

    ts_cycle = millis();
    while (true) {  // wait for user to confirm restart
        for (int i=0; i<5; i++) {
            digitalWrite(LED_PIN, HIGH);  delay(20); digitalWrite(LED_PIN, LOW); delay(40); // indicates halted system
        }
        OTA_server.handleClient();  // check for OTA software update
        if (!telnet) telnet = telnet_server.available();  // check for terminal connection
        if (telnet) telnet.print(cycle_str);
        while ((millis()-ts_cycle) % PROCESSING_DELAY);  // standard cycle delay
        if (!((millis()-ts_cycle) % (DDNS_UPDATE_INTERVAL*1000))) UpdateDDNS();  // keep updating DDNS in order to be reachable via Internet
        if (telnet.available()) {
            telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
            delay(2000);
            telnet.stop();
            delay(1000);
            ESP.restart();
        }
    }
}

void SetMWPower(const int mw_power) {

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
    strcpy(cycle_str, CLEAR_SCREEN);
    strcat(cycle_str, "BinSmart ESS ");
    strcat(cycle_str, SW_VERSION);
    strcat(cycle_str, " ");
    if (error_flag && (command != 'e')) strcat(cycle_str, ERROR_SYMBOL);  // "unread error" symbol
    strcat(cycle_str, WIFI_SYMBOL[WiFi.RSSI() >= GOOD_WIFI_RSSI]);

    // Time, cycle time, operations symbol
    sprintf(cycle_str + strlen(cycle_str), "\r\n%02d:%02d:%02d +%d.%03d%s\r\n", hour(unixtime), minute(unixtime), second(unixtime), msecs_cycle/1000, msecs_cycle%1000, OPS_SYMBOL[!power_new + (pm1_eco_mode && pm2_eco_mode)]);

    // PV power, nighttime/daytime symbol
    sprintf(cycle_str + strlen(cycle_str), "\r\n%4d ", int(round(power_pv)));
    strcat(cycle_str, NIGHT_DAY_SYMBOL[(min_of_day >= sunrise) && (min_of_day < sunset)]);
    strcat(cycle_str, PV_FLOW_SYMBOL[(round(power_pv) > 0) + (round(power_pv) >= PV_MAX_POWER)]);
    strcat(cycle_str, CABLE_SYMBOL);
    strcat(cycle_str, PV_CABLE_SYMBOL);

    // ESS power
    strcat(cycle_str, ESS_CABLE_SYMBOL);
    strcat(cycle_str, CABLE_SYMBOL);
    if (!power_old)
        strcat(cycle_str, ESS_FLOW_SYMBOL[!mw_limit_old + 2*(!hm_limit_old)]);
    if (power_old > 0)
        strcat(cycle_str, MW_FLOW_SYMBOL[(power_old == mw_limit_old) + ((power_old == mw_limit_old) && (mw_limit_old >= MW_MAX_POWER))][power_grid_to_ess > power_pv_to_ess]);
    if (power_old < 0)
        strcat(cycle_str, HM_FLOW_SYMBOL[(power_old == hm_limit_old) + (power_old == HM_MAX_POWER)]);
    int vbat_oc = vbat-round(cbat/15.0);  // voltage at cbat=0 (needed for accurate battery charge level)
    strcat(cycle_str, BAT_LEVEL_SYMBOL[(vbat_oc > VCELL_UVP*8) ? min(((vbat_oc/8-VCELL_UVP)*(BAT_LEVELS-2))/(VCELL_OVP-VCELL_UVP)+1, BAT_LEVELS-1) : 0]);
    sprintf(cycle_str + strlen(cycle_str), "%d", int(round(power_ess)));
    if (power_new != power_old) strcat(cycle_str, DIFF_SYMBOL[(power_new < power_old) + !filter_cycles]);
    else if (filter_cycles && (filter_cycles < POWER_FILTER_CYCLES)) strcat(cycle_str, POWERFILTER_SYMBOL);
    if (auto_recharge) strcat(cycle_str, AUTO_RECHARGE_SYMBOL);
    else if (manual_mode) strcat(cycle_str, MANUAL_MODE_SYMBOL);
    strcat(cycle_str, "\r\n");

    // House symbol
    strcat(cycle_str, HOUSE_SYMBOL);

    // Grid power
    sprintf(cycle_str + strlen(cycle_str), "\r\n%4d ", int(round(power_grid)));
    strcat(cycle_str, GRID_SYMBOL);
    strcat(cycle_str, GRID_FLOW_SYMBOL[(int(round(power_grid)) > 0)  + 2*(int(round(power_grid)) < 0)][power_ess_to_grid > power_pv_to_grid]);
    strcat(cycle_str, CABLE_SYMBOL);
    strcat(cycle_str, GRID_CABLE_SYMBOL);

    // Power to consumers
    strcat(cycle_str, CONS_CABLE_SYMBOL);
    strcat(cycle_str, CABLE_SYMBOL);
    if (power_grid_to_cons >= max(power_pv_to_cons, power_ess_to_cons)) strcat(cycle_str, CONS_FLOW_SYMBOL[0]);
    if (power_ess_to_cons >= max(power_grid_to_cons, power_pv_to_cons)) strcat(cycle_str, CONS_FLOW_SYMBOL[1]);
    if (power_pv_to_cons >= max(power_grid_to_cons, power_ess_to_cons)) strcat(cycle_str, CONS_FLOW_SYMBOL[2]);
    strcat(cycle_str, CONS_SYMBOL);
    sprintf(cycle_str + strlen(cycle_str), " %.0f\r\n\n", power_cons);

    // Read user command
    if (telnet.available()) command = telnet.read();

    // Append user command response to telnet message
    switch (command) {
        case 'm':
            command = '\0';  // react to command only once
            telnet.printf("%s%s", cycle_str, "Enter ESS power: ");
            ts_user = millis();
            while (!telnet.available() && (millis()-ts_user < USER_TIMEOUT));
            if (telnet.available()) {
                power_manual = telnet.parseInt();
                manual_mode = true;
                sprintf(resp_str, "ESS power manually set to %d W\r\n\n", power_manual);
            }
            else {
                if (!manual_mode) strcpy(resp_str, "Automatic mode remains activated\r\n\n");
                else sprintf(resp_str, "ESS power remains at %d W\r\n\n", power_manual);
            }
            break;
        case 'a':
            command = '\0';  // react to command only once
            manual_mode = false;
            strcpy(resp_str, "Automatic mode activated\r\n\n");
            break;
        case 'g':
            command = '\0';  // react to command only once
            telnet.printf("%s%s", cycle_str, "Enter grid power target: ");
            ts_user = millis();
            while (!telnet.available() && (millis()-ts_user < USER_TIMEOUT));
            if (telnet.available()) {
                power_grid_target = telnet.parseInt();
                sprintf(resp_str, "Grid power target set to %d W\r\n\n", power_grid_target);
            }
            else sprintf(resp_str, "Grid power target remains at %d W\r\n\n", power_grid_target);
            break;
        case 'p':
            // AC/DC power status
            sprintf(resp_str, "AC power setting: %d W\r\nAC power reading: %.1f W\r\n", power_old, power_ess);
            sprintf(resp_str + strlen(resp_str), "DC power reading: %.1f W\r\nAC/DC conv. eff.: %.1f %%\r\n\n", pbat, (power_ess >= 0) ? pbat/power_ess*100 : power_ess/pbat*100);
            // Meanwell/Hoymiles power limits
            sprintf(resp_str + strlen(resp_str), "MW power limit: %d W\r\nHM power limit: %d W\r\n\n", mw_limit, hm_limit);
            break;
        case 'b':
            // Batt voltage infos
            sprintf(resp_str, "Cell voltages: %d - %d mV\r\nMax cell diff: %d mV%s\r\n", vcell_min, vcell_max, vcell_max-vcell_min, (bms_bal_on && (vcell_max-vcell_min >= bms_balancer_trigger) && (vcell_max >= bms_balancer_start)) ? BALANCER_SYMBOL : "");
            sprintf(resp_str + strlen(resp_str), "Batt voltage : %.3f V", vbat/1000.0);
            sprintf(resp_str + strlen(resp_str), "\r\nBatt current : %.2f A\r\n\n", cbat/100.0);
            // BMS switch status
            sprintf(resp_str + strlen(resp_str), "Balancing switch: %s\r\nDischarge switch: %s\r\n\n", (bms_bal_on)?"on":"off", (bms_disch_on)?"on":"off");
            break;
        case 'd':
            sprintf(resp_str, "ESS public IP address: %s\r\nLast address check   : %02d/%02d/%04d %02d:%02d\r\n", pubip_addr.toString().c_str(), day(pubip_uxt), month(pubip_uxt), year(pubip_uxt), hour(pubip_uxt), minute(pubip_uxt));
            sprintf(resp_str + strlen(resp_str), "Last DDNS update     : %02d/%02d/%04d %02d:%02d\r\n\n", day(ddns_uxt), month(ddns_uxt), year(ddns_uxt), hour(ddns_uxt), minute(ddns_uxt));
            break;
        case 'w':
            sprintf(resp_str, "WiFi RSSI: %d dBm\r\nChip temp: %.1f °C\r\n\n", WiFi.RSSI(), temperatureRead());
            break;
        case 't':
            sprintf(resp_str, "Local time   : %02d/%02d/%04d %02d:%02d:%02d", day(unixtime), month(unixtime), year(unixtime), hour(unixtime), minute(unixtime), second(unixtime));
            sprintf(resp_str + strlen(resp_str), "\r\nTimezone     : UTC%+d\r\nSDT/DST      : %s", TIMEZONE, (utc_offset == TIMEZONE) ? "SDT" : "DST");
            sprintf(resp_str + strlen(resp_str), "\r\nESS started  : %02d/%02d/%04d %02d:%02d:%02d", day(start_uxt), month(start_uxt), year(start_uxt), hour(start_uxt), minute(start_uxt), second(start_uxt));
            sprintf(resp_str + strlen(resp_str), "\r\nESS uptime   : %03dd %02dh %02dm %02ds", (unixtime-start_uxt)/86400, ((unixtime-start_uxt)%86400)/3600, ((unixtime-start_uxt)/60)%60, (unixtime-start_uxt)%60);
            sprintf(resp_str + strlen(resp_str), "\r\nSunrise today: %02d:%02d\r\nSunset today : %02d:%02d\r\n\n", sunrise/60, sunrise%60, sunset/60, sunset%60); 
            break;
        case 'l':
            sprintf(resp_str, "Lowest cons. since %02d/%02d/%04d %02d:%02d\r\n", day(start_uxt), month(start_uxt), year(start_uxt), hour(start_uxt), minute(start_uxt));
            if (!minpower_uxt) strcat(resp_str, "Not yet measured\r\n\n");
            else sprintf(resp_str + strlen(resp_str), "%.1f W (measured %02d/%02d/%04d %02d:%02d)\r\n\n", power_grid_min, day(minpower_uxt), month(minpower_uxt), year(minpower_uxt), hour(minpower_uxt), minute(minpower_uxt));
            break;
        case 'n':
            sprintf(resp_str, "Energy [kWh] since %02d/%02d/%04d %02d:%02d\r\n\n", day(energy_uxt), month(energy_uxt), year(energy_uxt), hour(energy_uxt), minute(energy_uxt));
            // PV energy
            sprintf(resp_str + strlen(resp_str), "%7.3f %s%s%s%s%s%s %.1f﹪+%.1f﹪\r\n", en_from_pv/1000, NIGHT_DAY_SYMBOL[1], PV_FLOW_SYMBOL[1], PV_CABLE_SYMBOL, ESS_CABLE_SYMBOL, MW_FLOW_SYMBOL[0][0], ESS_SYMBOL, (en_pv_to_ess-en_pv_wasted)/en_from_pv*100, en_pv_wasted/en_from_pv*100);
            sprintf(resp_str + strlen(resp_str), "  %s\r\n", HOUSE_SYMBOL);
            sprintf(resp_str + strlen(resp_str), "%6.1f﹪%s%s%s%s%s%s %.1f﹪\r\n\n", en_pv_to_grid/en_from_pv*100, GRID_SYMBOL, GRID_FLOW_SYMBOL[2][0], GRID_CABLE_SYMBOL, CONS_CABLE_SYMBOL, CONS_FLOW_SYMBOL[2], CONS_SYMBOL, en_pv_to_cons/en_from_pv*100);
            // ESS energy
            sprintf(resp_str + strlen(resp_str), "                %s%s%s %.3f\r\n", ESS_CABLE_SYMBOL, HM_FLOW_SYMBOL[0], ESS_SYMBOL, en_from_ess/1000);
            sprintf(resp_str + strlen(resp_str), "  %s\r\n", HOUSE_SYMBOL);
            sprintf(resp_str + strlen(resp_str), "%6.1f﹪%s%s%s%s%s%s %.1f﹪\r\n\n", en_ess_to_grid/en_from_ess*100, GRID_SYMBOL, GRID_FLOW_SYMBOL[2][1], GRID_CABLE_SYMBOL, CONS_CABLE_SYMBOL, CONS_FLOW_SYMBOL[1], CONS_SYMBOL, en_ess_to_cons/en_from_ess*100);
            // Grid energy
            sprintf(resp_str + strlen(resp_str), "                %s%s%s %.1f﹪\r\n", ESS_CABLE_SYMBOL, MW_FLOW_SYMBOL[0][1], ESS_SYMBOL, en_grid_to_ess/en_from_grid*100);
            sprintf(resp_str + strlen(resp_str), "  %s\r\n", HOUSE_SYMBOL);
            sprintf(resp_str + strlen(resp_str), "%7.3f %s%s%s%s%s%s %.1f﹪\r\n\n", en_from_grid/1000, GRID_SYMBOL, GRID_FLOW_SYMBOL[1][0], GRID_CABLE_SYMBOL, CONS_CABLE_SYMBOL, CONS_FLOW_SYMBOL[0], CONS_SYMBOL, en_grid_to_cons/en_from_grid*100);
            // Meanwell/Hoymiles/ESS efficiency
            sprintf(resp_str + strlen(resp_str), "MW AC▸DC eff.: %.1f﹪\r\nHM DC▸AC eff.: %.1f﹪\r\nAC▸DC▸AC eff.: %.1f﹪\r\n\n", en_to_batt/en_to_ess*100, en_from_ess/en_from_batt*100, en_from_ess/en_to_ess*100);
            break;
        case 'e':
            sprintf(resp_str, "Errors since %02d/%02d/%04d %02d:%02d\r\n", day(errors_uxt), month(errors_uxt), year(errors_uxt), hour(errors_uxt), minute(errors_uxt));
            for (int i=0; i<ERROR_TYPES; i++) {
                sprintf(resp_str + strlen(resp_str), "%-4s: %3d", ERROR_TYPE[i], error_counter[i]);
                if (error_counter[i]) sprintf(resp_str + strlen(resp_str), " (last: %02d/%02d/%04d %02d:%02d)", day(errortime[i]), month(errortime[i]), year(errortime[i]), hour(errortime[i]), minute(errortime[i]));
                strcat(resp_str, "\r\n");
            }
            strcat(resp_str, "\n");
            if (error_str[0] != '\0') sprintf(resp_str + strlen(resp_str), "%s%s\r\n\n", ERROR_SYMBOL, error_str);  // error has just occured: show ERROR_SYMBOL
            else if (last_error_str[0] != '\0') sprintf(resp_str + strlen(resp_str), "Last:\r\n%s\r\n\n", last_error_str);
            error_flag = false;
            break;
        case 's':
            sprintf(resp_str, "Shelly relay ops since %02d/%02d/%04d %02d:%02d\r\n", day(start_uxt), month(start_uxt), year(start_uxt), hour(start_uxt), minute(start_uxt));
            sprintf(resp_str + strlen(resp_str), "Meanwell: %d (%d/day)\r\nHoymiles: %d (%d/day)\r\n\n", mw_counter, mw_counter/((unixtime-start_uxt)/86400+1), hm_counter, hm_counter/((unixtime-start_uxt)/86400+1));
            break;
        case 'z':
            command = '\0';  // react to command only once
            telnet.printf("%s%s", cycle_str, "Reset [e]rror or e[n]ergy stats: ");
            ts_user = millis();
            while (!telnet.available() && (millis()-ts_user < USER_TIMEOUT));
            if (telnet.available()) {
                char input = telnet.read();
                if (input == 'n') {
                    en_from_pv = en_pv_to_cons = en_pv_to_ess = en_pv_to_grid = en_pv_wasted = 0;  // Reset PV energy counters
                    en_from_grid = en_grid_to_cons = en_grid_to_ess = 0;  // Reset grid energy counters
                    en_from_ess = en_to_ess = en_ess_to_cons = en_ess_to_grid = 0;  // Reset ESS energy counters
                    en_from_batt = en_to_batt = 0;  // Reset ESS DC energy counters
                    energy_uxt = unixtime;
                    strcpy(resp_str, "Energy stats reset to zero\r\n\n");
                    break;
                }
                if (input == 'e') {
                    for (int i=0; i<ERROR_TYPES; i++) error_counter[i] = errortime[i] = 0;
                    errors_consecutive = 0;
                    last_error_str[0] = '\0';
                    error_flag = false;
                    errors_uxt = unixtime;
                    strcpy(resp_str, "Error stats reset to zero\r\n\n");
                    break;
                }
            }
            strcpy(resp_str, "No stats reset\r\n\n");
            break;
        case 'c':
            command = resp_str[0] = '\0';
            break;
        case 'r':
            command = '\0';  // react to command only once
            telnet.printf("%s%s", cycle_str, "Enter [y] to confirm system reboot: ");
            ts_user = millis();
            while (!telnet.available() && (millis()-ts_user < USER_TIMEOUT));
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
            break;
        case 'h':
            command = '\0';  // react to command only once
            strcpy(resp_str, "Command options:\r\n");
            strcat(resp_str, "[m] - Manual power mode\r\n");
            strcat(resp_str, "[a] - Automatic power mode\r\n");
            strcat(resp_str, "[g] - Grid power target\r\n");
            strcat(resp_str, "[p] - Power status\r\n");
            strcat(resp_str, "[b] - Batt/BMS status\r\n");
            strcat(resp_str, "[d] - DDNS status\r\n");
            strcat(resp_str, "[w] - WiFi RSSI, chip temp\r\n");
            strcat(resp_str, "[t] - Time, uptime, astro times\r\n");
            strcat(resp_str, "[l] - Lowest power consumption\r\n");
            strcat(resp_str, "[n] - Energy stats\r\n");
            strcat(resp_str, "[e] - Error stats\r\n");
            strcat(resp_str, "[s] - Shelly relay counter\r\n");
            strcat(resp_str, "[z] - Reset stats to zero\r\n");
            strcat(resp_str, "[c] - Clear command response\r\n");
            strcat(resp_str, "[r] - Reboot system\r\n\n");
            break;
    }
    telnet.printf("%s%s%s", cycle_str, resp_str, CMD_PROMPT);  // print output message, user command response and command prompt
}

bool UpdateDDNS() {

    if (WiFi.status() != WL_CONNECTED) {  // no need to carry on if WiFi is disconnected
        strcpy(error_str, "WIFI disconnected");
        return false;
    }

    // connect to server and request public IP address
    if (http.connect(PUBLIC_IP_SERVER, HTTP_PORT)) {
        sprintf(http_command, "GET %s HTTP/1.1\r\nHost: %s\r\n\r\n", PUBLIC_IP_URL, PUBLIC_IP_SERVER);
        http.write(http_command, strlen(http_command));
        if (http.find(HTTP_OK)) {
            // read public IP address from router
            http.find("\r\n\r\n");  // start of HTML body
            http.setTimeout(10);  // waiting time max 10 ms if IP address is shorter than 15 bytes
            http_resp[http.readBytes(http_resp, 15)] = '\0';  // read public IP address, append \0
            http.stop();
            http.setTimeout(HTTP_TIMEOUT);  // http timeout back to normal
            pubip_addr.fromString(http_resp);
            pubip_uxt = unixtime;
            if (ddns_addr == pubip_addr) return true;  // public IP unchanged

            // connect to DDNS server and update public IP address
            if (http.connect(DDNS_SERVER, HTTP_PORT)) {
                sprintf(http_command, "GET %s%s%s%s HTTP/1.1\r\nHost: %s\r\nAuthorization: Basic %s\r\n\r\n", DDNS_URL, DDNS_HOSTNAME, DDNS_IP, http_resp, DDNS_SERVER, DDNS_CREDS);
                http.write(http_command, strlen(http_command));
                if (http.find(HTTP_OK)) {
                    if (http.find(http_resp)) {  // DDNS server confirmed updated (or unchanged) public IP address
                        http.stop();
                        ddns_addr = pubip_addr;
                        ddns_uxt = unixtime;
                        return true;
                    }
                }
            }
            strcpy(error_str, "DDNS update command failed");
        }
    }
    http.stop();
    return false;
}
