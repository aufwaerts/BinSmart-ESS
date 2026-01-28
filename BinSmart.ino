#define SW_VERSION "v2.57"

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
    delay(10);
    digitalWrite(LED_PIN, HIGH);

    // Init WiFi
    WiFi.config(ESP32_ADDR, ROUTER_ADDR, SUBNET, DNS_SERVER1, DNS_SERVER2);
    WiFi.setTxPower(WIFI_POWER_7dBm);
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
    telnet.print(CLEAR_SCREEN);
    telnet.print(HIDE_CURSOR);
    telnet.print("BinSmart ESS ");
    telnet.print(SW_VERSION);
    telnet.print("\r\n\nWiFi connected to ");
    telnet.print(WIFI_SSID);
    telnet.print("   RSSI: ");
    telnet.print(WiFi.RSSI());
    telnet.print("   TxPower: ");
    telnet.println(WiFi.getTxPower());
    delay(3000);

    // reserve memory for global Strings
    http_command.reserve(150);
    error_msg.reserve(200);
    last_error_msg.reserve(200);
    cmd_resp.reserve(700);
    if (!cycle_msg.reserve(1000)) error_msg = "String memory allocation failed";
    CheckErrors();

    // Init PWM generator (for adjusting Meanwell power)
    if (!ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION)) error_msg = "PWM generator init failed";
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
    if (!pClient) error_msg = "BLE init failed";
    CheckErrors();
    pClient->setConnectionParams(12, 12, 0, 12);
    pClient->setConnectTimeout(BLE_TIMEOUT);
    telnet.println("BLE communication with JKBMS initialized");

    // Init RF24 radio communication with Hoymiles
    if (!radio.begin()) error_msg = "RF24 radio init failed";
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
        if (ShellyCommand(PM2_ADDR, PM_OFF[0])) ShellyCommand(PM2_ADDR, PM_ON[1]);
    CheckErrors();
    telnet.println("Shelly 2PM found");
    
    // Shelly 3EM: Read geo coordinates, grid power and current time
    if (ShellyCommand(EM_ADDR, EM_SETTINGS)) ShellyCommand(EM_ADDR, EM_STATUS);
    CheckErrors();
    telnet.println("Shelly 3EM found");

    // Update DDNS address
    if (!UpdateDDNS()) error_msg = "DDNS update failed";
    CheckErrors();
    telnet.println("DDNS address updated");
    
    // No erros during setup: set timestamps, start polling cycle
    telnet.print("\nNo errors during setup, start polling cycle ...");
    starttime = resettime_errors = resettime_energy = local_unixtime;
    delay(PROCESSING_DELAY*1000);
    ts_power = millis();
}

void loop() {

    if (daytime) ShellyCommand(PM1_ADDR, PM_STATUS[0]);  // Read PV power from Shelly 1PM
    BMSCommand(READ_VOLTAGES);  // Read cell voltages from BMS, set charging/discharging power limits
    if (power_new) ShellyCommand(PM2_ADDR, PM_STATUS[power_new < 0]); // Read ESS power from Shelly 2PM
    ShellyCommand(EM_ADDR, EM_STATUS);  // Read time and grid power from Shelly 3EM
    BMSCommand(READ_CURRENT);  // Read DC charging/discharging current and power from BMS, determine batt charge level
    SetNewPower();  // Calculate and apply new charging/discharging power
    FinishCycle();  // Update energy counters, compile cycle info, carry out maintenance tasks
    CheckErrors();  // Check errors, halt system if error is persistent, show cycle status by flashing LED
    ProcessUserCommand();  // Process user command from previous cycle(s), print cycle info and user command response
    ReadCommand();  // Read user command while waiting for power changes to take effect
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
            if (!ShellyCommand(PM2_ADDR, PM_OFF[0])) power_new = MW_MIN_POWER;
        }
    }
    if (power_new > 0) {  // set new charging power, (if necessary) turn discharging off, turn charging on
        if (power_old > 0) SetMWPower(power_new);  // re-calculate even if power setting remains unchanged (vbat might have changed)
        if (power_old == 0) {
            SetMWPower(power_new);
            if (!ShellyCommand(PM2_ADDR, PM_ON[0])) power_new = 0;
        }
        if (power_old < 0) {
            if (!HoymilesCommand(HM_OFF)) power_new = power_old;
            else {
                SetMWPower(power_new);
                if (!ShellyCommand(PM2_ADDR, PM_ON[0])) power_new = 0;
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
            if (!ShellyCommand(PM2_ADDR, PM_OFF[0])) power_new = MW_MIN_POWER;
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
        error_msg = "BMS BLE command 0x";
        if (BMS_command[BLE_COMMAND_POS] < 0x10) error_msg += "0";
        error_msg += String(BMS_command[BLE_COMMAND_POS],HEX);
        error_msg += " 0x";
        if (BMS_command[BLE_SETTING_POS] < 0x10) error_msg += "0";
        error_msg += String(BMS_command[BLE_SETTING_POS],HEX);
        error_msg += " failed";
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
                        error_msg = "BMS has warnings 0x";
                        if (bms_resp[WARNINGS_POS] < 0x10) error_msg += "0";
                        error_msg += String(bms_resp[WARNINGS_POS+1],HEX);
                        if (bms_resp[WARNINGS_POS+1] < 0x10) error_msg += "0";
                        error_msg += String(bms_resp[WARNINGS_POS+2],HEX);
                        return false;
                    }
                    // check OVP and UVP setting
                    if ((bms_resp[OVP_POS]<<8|bms_resp[OVP_POS+1]) - ESS_OVP < ESS_BMS_OVP_DIFF) {
                        error_msg = "ESS_OVP less than ";
                        error_msg += ESS_BMS_OVP_DIFF;
                        error_msg += " mV below BMS Cell OVP (";
                        error_msg += (bms_resp[OVP_POS]<<8|bms_resp[OVP_POS+1]);
                        error_msg += " mV)";
                        return false;
                    }
                    bms_uvp = bms_resp[UVP_POS]<<8|bms_resp[UVP_POS+1];
                    if (ESS_UVP - bms_uvp < ESS_BMS_UVP_DIFF) {
                        error_msg = "ESS_UVP less than ";
                        error_msg += ESS_BMS_UVP_DIFF;
                        error_msg += " mV above BMS Cell UVP (";
                        error_msg += bms_uvp;
                        error_msg += " mV)";
                        return false;
                    }
                    // read balancer settings
                    bms_balancer_start = bms_resp[BAL_ST_POS]<<8|bms_resp[BAL_ST_POS+1];
                    if (ESS_UVP < bms_balancer_start) {
                        error_msg = "BMS Balancer Start Voltage higher than ESS_UVP (";
                        error_msg += ESS_UVP;
                        error_msg += " mV)";
                        return false;
                    }
                    bms_balancer_trigger = bms_resp[BAL_TR_POS]<<8|bms_resp[BAL_TR_POS+1];
                    bms_bal_on = bms_resp[BAL_SW_POS];
                    return true;

                }
            }
        } 
    }
    error_msg = "BMS RS485 command 0x";
    if (BMS_command[RS485_COMMAND_POS] < 0x10) error_msg += "0";
    error_msg += String(BMS_command[RS485_COMMAND_POS],HEX);
    error_msg += " 0x";
    if (BMS_command[DATA_ID_POS] < 0x10) error_msg += "0";
    error_msg += String(BMS_command[DATA_ID_POS],HEX);
    error_msg += " failed";
    return false;
}

bool ShellyCommand(IPAddress shelly_addr, const String shelly_command) {

    if (WiFi.status() != WL_CONNECTED) {
        error_msg = "WIFI";
        return false;
    }

    // Prepare Shelly http command
    http_command = "http://";
    http_command += shelly_addr.toString();
    http_command += shelly_command;

    // Send http command to Shelly
    http.setTimeout(HTTP_SHELLY_TIMEOUT*1000);
    http.begin(http_command);
    if (http.GET() == HTTP_CODE_OK) {
        WiFiClient* stream = http.getStreamPtr();
        if (shelly_command == EM_SETTINGS) {
            // read geo coordinates and timezone from Shelly 3EM
            stream->find("lat");
            latitude = stream->parseFloat();
            longitude = stream->parseFloat();
            timezone = round(longitude/15);
        }
        if (shelly_command == EM_STATUS) {
            // read time and grid power from Shelly 3EM
            for (int i=0; i<140; i++) stream->read();  // fast forward to "time"
            stream->find("time");
            min_of_day = stream->parseInt()*60 + stream->parseInt();  // read local time (minutes after midnight)
            long unixtime_utc = stream->parseInt();  // read epoch time (UTC)
            dst = ((min_of_day/60+24-hour(unixtime_utc))%24 > timezone);  // daylight saving time?
            local_unixtime = unixtime_utc + (timezone+dst)*3600;  // adjust local_unixtime to match local time
            for (int i=0; i<550; i++) stream->read();  // fast forward to "total_power"
            stream->find("total_power");
            power_grid = stream->parseFloat();  // read grid power from http response
            if (!starttime || (min_of_day == 210)) {
                Dusk2Dawn ess_location(latitude, longitude, timezone);  // calculate sunrise and sunset of current day
                sunrise = ess_location.sunrise(year(local_unixtime), month(local_unixtime), day(local_unixtime), dst);
                sunset = ess_location.sunset(year(local_unixtime), month(local_unixtime), day(local_unixtime), dst);
            }
            daytime = ((min_of_day > sunrise) && (min_of_day < sunset));
        }
        if ((shelly_command == PM_STATUS[0]) || (shelly_command == PM_STATUS[1])) {
            // read power from Shelly 1PM or 2PM
            stream->find("apower");
            float power_reading = stream->parseFloat();
            if (shelly_addr == PM1_ADDR) power_pv = power_reading;
            else power_ess = power_reading;
        }
        if (shelly_command == PM_ON[0]) {
            // Meanwell turned on
            ts_MW = millis();
            if (!mw_on) mw_counter++;
            mw_on = true;
        }
        if (shelly_command == PM_OFF[0]) {
            // Meanwell turned off
            ts_MW = millis();
            if (mw_on) mw_counter++;
            mw_on = false;
        }
        if (shelly_command == PM_CONFIG) {
            bool eco_mode = stream->findUntil("eco_mode\":true", "eco_mode\":false");
            if (shelly_addr == PM1_ADDR) pm1_eco_mode = eco_mode;
            else pm2_eco_mode = eco_mode;
        }
        http.end();
        return true;
    }
    // Shelly command failed
    http.end();
    if (shelly_addr == EM_ADDR) error_msg = "Shelly 3EM command ";
    if (shelly_addr == PM1_ADDR) error_msg = "Shelly 1PM command ";
    if (shelly_addr == PM2_ADDR) error_msg = "Shelly 2PM command ";
    error_msg += shelly_command;
    error_msg += " failed";
    return false;
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
        error_msg = "Hoymiles RF24 switch command ";
        error_msg += hm_command;
        error_msg += " failed";
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
    error_msg = "Hoymiles RF24 power command failed";
    return false;
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

void FinishCycle() {
    
    // AC power flows
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

    // AC energy counters
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
    en_grid_to_ess = en_to_ess - en_pv_to_ess;
    en_ess_to_cons += hrs_cycle*power_ess_to_cons;
    en_ess_to_grid = en_from_ess - en_ess_to_cons;
    en_pv_consumed = en_pv_to_cons + en_ess_to_cons - en_grid_to_ess*0.8;  // PV energy consumed by household, directly or via ESS
    if (power_ess > 0) en_pv_wasted += hrs_cycle*(1-0.9*pbat/power_ess)*min(power_from_grid, power_pv_to_ess);  // PV energy wasted to ESS, due to AC/DC/AC conversion losses
    
    // DC energy counters
    en_from_batt += hrs_cycle * (pbat < 0) * -pbat;
    en_to_batt += hrs_cycle * (pbat > 0) * pbat;

    // Compile cycle message
    cycle_msg = CLEAR_SCREEN;
    cycle_msg += SHOW_CURSOR;
    cycle_msg += "BinSmart ESS ";
    cycle_msg += SW_VERSION;
    cycle_msg += " ";
    if (error_flag) cycle_msg += ERROR_SYMBOL;
    cycle_msg += WIFI_SYMBOL[(WiFi.RSSI() >= GOOD_WIFI_RSSI)];
    cycle_msg += "\r\n";

    // Time, cycle time, daytime
    sprintf(buf,"%02d:%02d:%02d %+.3f", hour(local_unixtime), minute(local_unixtime), second(local_unixtime), secs_cycle);
    cycle_msg += buf;
    cycle_msg += OPS_SYMBOL[!power_new + (pm1_eco_mode && pm2_eco_mode)];
    cycle_msg += "\r\n";

    // OVP symbol above battery symbol
    cycle_msg += BAT_OVP_SYMBOL[(mw_limit_old < MW_MAX_POWER) + !mw_limit_old];

    // PV power
    sprintf(buf,"\r\n%4d ",int(round(power_pv)));
    cycle_msg += buf;
    cycle_msg += NIGHT_DAY_SYMBOL[daytime];
    cycle_msg += PV_FLOW_SYMBOL[(round(power_pv) > 0) + (round(power_pv) >= PV_MAX_POWER)];
    cycle_msg += CABLE_SYMBOL;
    cycle_msg += PV_CABLE_SYMBOL;

    // ESS power
    cycle_msg += ESS_CABLE_SYMBOL;
    cycle_msg += CABLE_SYMBOL;
    if (!power_old)
        cycle_msg += ESS_FLOW_SYMBOL[!mw_limit_old + 2*(!hm_limit_old)];
    if (power_old > 0)
        cycle_msg += MW_FLOW_SYMBOL[(power_old == mw_limit_old) + ((power_old == mw_limit_old) && (mw_limit_old >= MW_MAX_POWER))][power_grid_to_ess > power_pv_to_ess];
    if (power_old < 0)
        cycle_msg += HM_FLOW_SYMBOL[(power_old == hm_limit_old) + (power_old == HM_MAX_POWER)];
    cycle_msg += BAT_LEVEL_SYMBOL[bat_level];
    sprintf(buf,"%d",int(round(power_ess)));
    cycle_msg += buf;
    if (power_new != power_old) cycle_msg += DIFF_SYMBOL[(power_new < power_old) + !filter_cycles];
    else if (filter_cycles && (filter_cycles < POWER_FILTER_CYCLES)) cycle_msg += POWERFILTER_SYMBOL;
    if (manual_mode) cycle_msg += MANUAL_MODE_SYMBOL;
    if (auto_recharge) cycle_msg += AUTO_RECHARGE_SYMBOL;
    cycle_msg += "\r\n";

    // House symbol
    cycle_msg += HOUSE_SYMBOL;

    // UVP symbol below battery symbol
    cycle_msg += BAT_UVP_SYMBOL[(hm_limit_old > HM_MAX_POWER) + !hm_limit_old];

    // Grid power
    sprintf(buf,"\r\n%4d ",int(round(power_grid)));
    cycle_msg += buf;
    cycle_msg += GRID_SYMBOL;
    cycle_msg += GRID_FLOW_SYMBOL[(int(round(power_grid)) > 0)  + 2*(int(round(power_grid)) < 0)][power_ess_to_grid > power_pv_to_grid];
    cycle_msg += CABLE_SYMBOL;
    cycle_msg += GRID_CABLE_SYMBOL;

    // Power to consumers
    cycle_msg += CONS_CABLE_SYMBOL;
    cycle_msg += CABLE_SYMBOL;
    if (power_grid_to_cons >= max(power_pv_to_cons, power_ess_to_cons)) cycle_msg += CONS_FLOW_SYMBOL[0];
    if (power_ess_to_cons >= max(power_grid_to_cons, power_pv_to_cons)) cycle_msg += CONS_FLOW_SYMBOL[1];
    if (power_pv_to_cons >= max(power_grid_to_cons, power_ess_to_cons)) cycle_msg += CONS_FLOW_SYMBOL[2];
    cycle_msg += CONS_SYMBOL;
    sprintf(buf," %.0f",power_cons);
    cycle_msg += buf;
    cycle_msg += "\r\n\n";

    // Append error info (if error occured)
    if (error_msg != "") {
        cycle_msg += ERROR_SYMBOL;
        cycle_msg += error_msg;
        cycle_msg += "\r\n\n";
    }

    // No ESS power output for at least two cycles: check for new min grid power (min household consumption)
    if (!power_old && !power_pv && (power_grid < power_grid_min)) {
        power_grid_min = power_grid;
        minpower_time = local_unixtime;
    }

    // Keep alive Hoymiles RF24 interface
    if ((millis()-ts_HM)/1000 >= RF24_KEEPALIVE) {
        if (power_new < 0) HoymilesCommand(HM_ON);
        else HoymilesCommand(HM_OFF);
    }

    // Keep alive Meanwell relay (if Meanwell is turned on)
    if (mw_on && ((millis()-ts_MW)/1000 >= MW_TIMER-10*PROCESSING_DELAY)) ShellyCommand(PM2_ADDR, PM_ON[0]);
    // Turn off Meanwell PWM optocoupler (if Meanwell is turned off)
    if (!mw_on && ((millis()-ts_MW)/2000 >= PROCESSING_DELAY)) ledcWrite(PWM_CHANNEL, 0);

    // Set Shelly 1PM eco mode (turn off at sunrise, turn on at sunset)
    if ((min_of_day >= sunrise) && (min_of_day <= sunset) && pm1_eco_mode) pm1_eco_mode = !ShellyCommand(PM1_ADDR, PM_ECO_OFF);
    if (((min_of_day < sunrise) || (min_of_day > sunset)) && !pm1_eco_mode) {
        power_pv = 0;
        pm1_eco_mode = ShellyCommand(PM1_ADDR, PM_ECO_ON);
        if (pm1_eco_mode) ShellyCommand(EM_ADDR, EM_RESET);  // Reset Shelly 3EM energy data (prevents HTTP timeouts during data reorgs)
    }

    // Set Shelly 2PM eco mode (turn off when charging/discharging active or possible, turn on when charging/discharging inactive and impossible)
    if ((power_new || auto_recharge) && pm2_eco_mode) pm2_eco_mode = !ShellyCommand(PM2_ADDR, PM_ECO_OFF);
    if ((power_pv < MW_MIN_POWER) && !power_old && !hm_limit && !auto_recharge && !pm2_eco_mode) pm2_eco_mode = ShellyCommand(PM2_ADDR, PM_ECO_ON);

    // Turn off/on JKBMS balancer (disable/enable bottom balancing), depending on lowest cell voltage 
    if ((vcell_min >= ESS_UVPR) && bms_bal_on) bms_bal_on = !BMSCommand(BAL_OFF);
    if ((vcell_min < ESS_UVP) && !power_old && !bms_bal_on) bms_bal_on = BMSCommand(BAL_ON);

    // Check if public IP address was changed, if yes, update DDNS server entry
    if ((millis()-ts_pubip)/1000 >= DDNS_UPDATE_INTERVAL) UpdateDDNS();
}

void CheckErrors() {

    if (error_msg == "") {
        if (!starttime) return;  // no LED flashes during setup()
        errors_consecutive = 0;
        digitalWrite(LED_PIN, HIGH); delay(20); digitalWrite(LED_PIN, LOW);  // no errors: just flash LED briefly
        return;
    }
    digitalWrite(LED_PIN, HIGH);  // error: keep LED on during next cycle

    if (!starttime) {
        // Error occured during system setup
        telnet.print(ERROR_SYMBOL);
        telnet.println(error_msg);
        telnet.print("\nPress any key to restart: ");
        telnet.print(SHOW_CURSOR);
        while (!telnet.available());
        telnet.print(HIDE_CURSOR);
        telnet.flush();
        telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
        delay(3000);
        telnet.stop();
        ESP.restart();
    }
 
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
        error_flag = true;  // new unread error
        error_msg = "";  // reset error message for next cycle
        return;
    }

    // Error is persistent: Halt the system
    HoymilesCommand(HM_OFF);
    SetMWPower(MW_MIN_POWER);
    ShellyCommand(PM2_ADDR, PM_OFF[0]);
    ShellyCommand(PM2_ADDR, PM_OFF[1]);
    
    while (true) {
        if ((millis()-ts_pubip)/1000 >= DDNS_UPDATE_INTERVAL) UpdateDDNS();  // keep updating DDNS in order to be reachable via Internet
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
        command = 0;
        ts_power = millis();
        ReadCommand();
        if (command) {
            telnet.print(HIDE_CURSOR);
            telnet.flush();
            telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
            delay(3000);
            telnet.stop();
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
            while ((millis()-ts_input)/1000 < READINPUT_TIMEOUT)
                if (telnet.available()) {
                    power_manual = telnet.readString().toInt();
                    manual_mode = true;
                    cmd_resp = "ESS power manually set to ";
                    cmd_resp += power_manual;
                    cmd_resp += " W\r\n\n";
                    return;
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
            while ((millis()-ts_input)/1000 < READINPUT_TIMEOUT)
                if (telnet.available()) {
                    power_target = telnet.readString().toInt();
                    cmd_resp = "Grid power target set to ";
                    cmd_resp += power_target;
                    cmd_resp += " W\r\n\n";
                    return;
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
            if ((vcell_max-vcell_min >= bms_balancer_trigger) && (vcell_max >= bms_balancer_start) && bms_bal_on) cmd_resp += BALANCER_SYMBOL;
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
            cmd_resp += mw_limit;
            cmd_resp += " W";
            cmd_resp += "\r\nHM power limit  : ";
            cmd_resp += hm_limit;
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
            sprintf(buf,"UTC%+d",timezone);
            cmd_resp += buf;
            cmd_resp += "\r\nUTC offset    : ";
            if (dst) sprintf(buf,"%+d (DST)",timezone+dst);
            else sprintf(buf,"%+d (SDT)",timezone);
            cmd_resp += buf;
            cmd_resp += "\r\nESS started   : ";
            sprintf(buf,"%02d/%02d/%04d %02d:%02d:%02d",day(starttime),month(starttime),year(starttime),hour(starttime),minute(starttime),second(starttime));
            cmd_resp += buf;
            cmd_resp += "\r\nESS uptime    : ";
            sprintf(buf,"%03dd %02dh %02dm %02ds",elapsedDays(local_unixtime-starttime),numberOfHours(local_unixtime-starttime),numberOfMinutes(local_unixtime-starttime),numberOfSeconds(local_unixtime-starttime));
            cmd_resp += buf;
            cmd_resp += "\r\nSunrise today : ";
            sprintf(buf,"%02d:%02d",sunrise/60,sunrise%60);
            cmd_resp += buf;
            cmd_resp += "\r\nSunset today  : ";
            sprintf(buf,"%02d:%02d\r\n\n",sunset/60,sunset%60);
            cmd_resp += buf;
            break;
        case 'l':
            cmd_resp = "Lowest consumption since ";
            sprintf(buf,"%02d/%02d/%04d %02d:%02d\r\n\n",day(starttime),month(starttime),year(starttime),hour(starttime),minute(starttime));
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
            sprintf(buf,"%7.3f ",en_from_pv/1000);
            cmd_resp += buf;
            cmd_resp += NIGHT_DAY_SYMBOL[1];
            cmd_resp += PV_FLOW_SYMBOL[1];
            cmd_resp += PV_CABLE_SYMBOL;
            cmd_resp += ESS_CABLE_SYMBOL;
            cmd_resp += MW_FLOW_SYMBOL[0][0];
            cmd_resp += ESS_SYMBOL;
            sprintf(buf," %.1f﹪+%.1f﹪\r\n  ", (en_pv_to_ess-en_pv_wasted)/en_from_pv*100, en_pv_wasted/en_from_pv*100);
            cmd_resp += buf;
            cmd_resp += HOUSE_SYMBOL;
            cmd_resp += "\r\n";
            sprintf(buf,"%6.1f﹪",en_pv_to_grid/en_from_pv*100);
            cmd_resp += buf;
            cmd_resp += GRID_SYMBOL;
            cmd_resp += GRID_FLOW_SYMBOL[2][0];
            cmd_resp += GRID_CABLE_SYMBOL;
            cmd_resp += CONS_CABLE_SYMBOL;
            cmd_resp += CONS_FLOW_SYMBOL[2];
            cmd_resp += CONS_SYMBOL;
            sprintf(buf," %.1f﹪\r\n\n",en_pv_to_cons/en_from_pv*100);
            cmd_resp += buf;

            // Grid energy
            cmd_resp += "                ";
            cmd_resp += ESS_CABLE_SYMBOL;
            cmd_resp += MW_FLOW_SYMBOL[0][1];
            cmd_resp += ESS_SYMBOL;
            sprintf(buf," %.1f﹪\r\n  ",en_grid_to_ess/en_from_grid*100);
            cmd_resp += buf;
            cmd_resp += HOUSE_SYMBOL;
            cmd_resp += "\r\n";
            sprintf(buf,"%7.3f ",en_from_grid/1000);
            cmd_resp += buf;
            cmd_resp += GRID_SYMBOL;
            cmd_resp += GRID_FLOW_SYMBOL[1][0];
            cmd_resp += GRID_CABLE_SYMBOL;
            cmd_resp += CONS_CABLE_SYMBOL;
            cmd_resp += CONS_FLOW_SYMBOL[0];
            cmd_resp += CONS_SYMBOL;
            sprintf(buf," %.1f﹪\r\n\n",en_grid_to_cons/en_from_grid*100);
            cmd_resp += buf;
   
            // ESS energy
            cmd_resp += "                ";
            cmd_resp += ESS_CABLE_SYMBOL;
            cmd_resp += HM_FLOW_SYMBOL[0];
            cmd_resp += ESS_SYMBOL;
            sprintf(buf," %.3f\r\n  ",en_from_ess/1000);
            cmd_resp += buf;
            cmd_resp += HOUSE_SYMBOL;
            cmd_resp += "\r\n";
            sprintf(buf,"%6.1f﹪",en_ess_to_grid/en_from_ess*100);
            cmd_resp += buf;
            cmd_resp += GRID_SYMBOL;
            cmd_resp += GRID_FLOW_SYMBOL[2][1];
            cmd_resp += GRID_CABLE_SYMBOL;
            cmd_resp += CONS_CABLE_SYMBOL;
            cmd_resp += CONS_FLOW_SYMBOL[1];
            cmd_resp += CONS_SYMBOL;
            sprintf(buf," %.1f﹪\r\n\n",en_ess_to_cons/en_from_ess*100);
            cmd_resp += buf;

            cmd_resp += "ESS efficiency:\r\n";
            // Meanwell charging efficiency
            cmd_resp += "MW AC▸DC: ";
            sprintf(buf, "%.1f﹪\r\n", en_to_batt/en_to_ess*100);
            cmd_resp += buf;
            // Hoymiles charging efficiency
            cmd_resp += "HM DC▸AC: ";
            sprintf(buf, "%.1f﹪\r\n", en_from_ess/en_from_batt*100);
            cmd_resp += buf;
            // ESS AC-to-AC total efficiency
            cmd_resp += "AC▸DC▸AC: ";
            sprintf(buf, "%.1f﹪\r\n\n", en_from_ess/en_to_ess*100);
            cmd_resp += buf;
            break;
        case 'e':
            cmd_resp = "Errors since ";
            sprintf(buf,"%02d/%02d/%04d %02d:%02d\r\n\n",day(resettime_errors),month(resettime_errors),year(resettime_errors),hour(resettime_errors),minute(resettime_errors));
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
            sprintf(buf,"%02d/%02d/%04d %02d:%02d\r\n\n",day(starttime),month(starttime),year(starttime),hour(starttime),minute(starttime));
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
            while ((millis()-ts_input)/1000 < READINPUT_TIMEOUT)
                if (telnet.available()) {
                    if (telnet.readString()[0] == 'n') {
                        en_from_pv = en_pv_to_cons = en_pv_to_ess = en_pv_to_grid = en_pv_consumed = en_pv_wasted = 0;  // Reset PV energy counters
                        en_from_grid = en_to_grid = en_grid_to_cons = en_grid_to_ess = 0;  // Reset grid energy counters
                        en_from_ess = en_to_ess = en_ess_to_cons = en_ess_to_grid = 0;  // Reset ESS energy counters
                        en_from_batt = en_to_batt = 0;  // Reset ESS DC energy counters
                        resettime_energy = local_unixtime;
                        cmd_resp = "Energy stats reset to zero\r\n\n";
                        return;
                    }
                    else {
                        for (int i=0; i<ERROR_TYPES; i++) error_counter[i] = errortime[i] = 0;
                        errors_consecutive = 0;
                        last_error_msg = "";
                        error_flag = false;
                        resettime_errors = local_unixtime;
                        cmd_resp = "Error stats reset to zero\r\n\n";
                        return;
                    }
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
            while ((millis()-ts_input)/1000 < READINPUT_TIMEOUT)
                if (telnet.available()) {
                    if (telnet.readString()[0] == 'y') {
                        telnet.print(HIDE_CURSOR);
                        telnet.print("\r\nRestarting in 3 seconds, re-open terminal ...\r\n");
                        delay(3000);
                        telnet.stop();
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
}

void ReadCommand() {
    while ((millis()-ts_power)/1000 < PROCESSING_DELAY) {
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
    // Assumption for next cycle: charging/discharging power equals new power setting
    power_ess = power_new;
}

bool UpdateDDNS() {

    if (WiFi.status() != WL_CONNECTED) {
        error_msg = "WIFI";
        return false;
    }

    // read public IP from server (with shorter timeout)
    http.setTimeout(HTTP_DDNS_TIMEOUT*1000);
    http.begin(PUBLIC_IP_URL);
    if (http.GET() == HTTP_CODE_OK) {
        public_IP = http.getString();
        http.end();
        ts_pubip = millis();
        pubip_time = local_unixtime;
        if (public_IP == DDNS_address) return true;  // public IP unchanged

        http_command = DDNS_SERVER_URL;
        http_command += public_IP;
        http.begin(http_command);  // update DDNS server entry
        if (http.GET() == HTTP_CODE_OK) {
            http.end();
            DDNS_address = public_IP;
            DDNS_time = local_unixtime;
            return true;
        }
        http.end();
        error_msg = "DDNS update command failed";
        return false;
    }
    http.end();
    // Read IP command frequently fails, no big deal, so don't report
    return false;
}


