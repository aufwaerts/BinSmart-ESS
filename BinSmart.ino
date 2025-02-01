const String SW_VERSION = "v1.06";

#include <WiFi.h>  // standard Arduino/ESP32
#include <HTTPClient.h>  // standard Arduino/ESP32 (method connect() must be made public)
#include <ESPAsyncWebServer.h>  // https://github.com/lacamera/ESPAsyncWebServer
#include <ElegantOTA.h>  // https://github.com/ayushsharma82/ElegantOTA
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
    filter_symbol.reserve(10);
    ota_msg.reserve(100);
    error_msg.reserve(200);
    cmd_resp.reserve(700);
    cycle_msg.reserve(1000);
    if (!http_resp.reserve(1800)) error_msg = "String memory allocation failed";
    else error_msg = cmd_resp = "";

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
    ota_msg += "\r\nEnter \"";
    ota_msg += ESP32_ADDR.toString();
    ota_msg += "/update\" for OTA firmware update";
    OTAserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { request->send(200, "text/plain", ota_msg); });
    ElegantOTA.setAutoReboot(true);
    ElegantOTA.begin(&OTAserver);
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
    memcpy(&hm_radio_ID[1], &HM_SERIAL[2], 4);
    memcpy(&hm_turnon[1], &HM_SERIAL[2], 4);
    memcpy(&hm_turnoff[1], &HM_SERIAL[2], 4);
    memcpy(&hm_power[1], &HM_SERIAL[2], 4);
    if (radio.begin()) {
        radio.setPALevel(RF24_PALEVELS[0]);
        radio.setChannel(RF24_CHANNELS[0]);
        radio.setDataRate (RF24_250KBPS);
        radio.setCRCLength(RF24_CRC_16);
        radio.setAddressWidth(sizeof(hm_radio_ID));
        radio.enableDynamicPayloads();
        radio.stopListening();
        radio.openWritingPipe(hm_radio_ID);
        crc8.setPolynome(0x01);
        crc8.setStartXOR(0);
        crc8.setEndXOR(0);
        crc16.setPolynome((uint16_t)0x18005);
        crc16.setStartXOR(0xFFFF);
        crc16.setEndXOR(0x0000);
        crc16.setReverseIn(true);
        crc16.setReverseOut(true);
        // Turn off Hoymiles (DC side)
        if (HoymilesCommand(hm_turnoff, sizeof(hm_turnoff))) telnet.print("RF24 communication with Hoymiles OK\r\n");
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
        telnet.print("\r\n\r\nEnter anything to restart: ");
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
        http_resp = http.getString();
        http.end();
        if (URL == EM_STATUS) {
            ts_EM = millis();  // remember exact time when Shelly response was read (allows precise calculations of current time)
            int index = http_resp.indexOf("time",140);
            memcpy(&current_time[0], &http_resp[index+7], 5);  // current_time considers timezone and DST
            unixtime = atol(&http_resp[index+25]);  // epoch time (UTC timezone)
            UTC_offset = (current_time.toInt()+24-hour(unixtime))%24;  // timezone_offset + DST_offset
            unixtime += UTC_offset*3600;  // adjust unixtime to match current_time
            power_grid = round(http_resp.substring(http_resp.indexOf("total_power",750)+13).toFloat());
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
            power_pv = round(http_resp.substring(http_resp.indexOf("apower",35)+8).toFloat());
            from_pv = http_resp.substring(http_resp.indexOf("total",80)+7).toFloat()/1000;
            return true;
        }
        if (URL == HMPLUG_STATUS) {
            from_ess = http_resp.substring(http_resp.indexOf("total",80)+7).toFloat()/1000;
            return true;
        }
        if (URL == MWPLUG_STATUS) {
            to_ess = http_resp.substring(http_resp.indexOf("total",80)+7).toFloat()/1000;
            return true;
        }
        if (URL == PM_CONFIG) {
            pm_eco_mode = (http_resp.indexOf("eco_mode\":true",750)>0);
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
        if (power_old <= 0) if (!HoymilesCommand(hm_turnoff, sizeof(hm_turnoff))) power_new = power_old;  // repeat command every cycle, just to make sure HM is really turned off
        if (power_old > 0) if (!ShellyCommand(MWPLUG_OFF)) power_new = power_old;
    }
    if (power_n > 0) {  // set new charging power
        if (power_new != power_old) SetMWPower();  // set new PWM value only if power setting has changed
        if (power_old == 0) if (!ShellyCommand(MWPLUG_ON)) power_new = 0;
        if (power_old < 0) {
            if (!HoymilesCommand(hm_turnoff, sizeof(hm_turnoff))) power_new = power_old;
            else if (!ShellyCommand(MWPLUG_ON)) power_new = 0;
        }
        else HoymilesCommand(hm_turnoff, sizeof(hm_turnoff));  // repeat command every cycle, just to make sure HM is really turned off
    }
    if (power_n < 0) {  // set new discharging power
        if (!HoymilesCommand(hm_power, sizeof(hm_power))) power_new = power_old;  // send HM power setting every cycle, even if power remains unchanged
        else {
            if (power_old == 0) {
                delay(RF24_MIN_DELAY);
                if (!HoymilesCommand(hm_turnon, sizeof(hm_turnon))) power_new = 0;
            }
            if (power_old > 0) {
                if (!ShellyCommand(MWPLUG_OFF)) power_new = power_old;  // takes longer than RF24_MIN_DELAY
                else if (!HoymilesCommand(hm_turnon, sizeof(hm_turnon))) power_new = 0;
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

        if (cmd_resp != "") cycle_msg += cmd_resp;

        cycle_msg += "Enter command";
        if (cmd_resp.indexOf("Command options:") == -1) cycle_msg += " or [o] for options";
        cycle_msg += ": ";
        if (telnet.available()) {
            received_command = telnet.read();  // saves keyboard input for later (when ReadCommand() is called)
            if (telnet.available()) telnet.flush();  // clearing rx buffer before printing avoids problems with telnet connection
        }
        telnet.print(cycle_msg);
    }
    else  {  // telnet connection lost: clear command response and command repeat flag
        cmd_resp = "";
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
    HoymilesCommand(hm_turnoff, sizeof(hm_turnoff));
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
    cycle_msg += "\r\n\r\nEnter [r] to restart: ";
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
                        cmd_resp = "ESS power manually set to ";
                        cmd_resp += power_manual;
                        cmd_resp += " W\r\n\r\n";
                        return true;
                    }
                    break;
                }
            if (!manual_mode) cmd_resp = "Automatic mode remains activated\r\n\r\n";
            else {
                cmd_resp = "ESS power remains at ";
                cmd_resp += power_manual;
                cmd_resp += " W\r\n\r\n";
            }
            break;
        case 'a':
            manual_mode = false;
            cmd_resp = "Automatic mode activated\r\n\r\n";
            break;
        case 'g':
            telnet.print("\r\nEnter grid power target: ");
            ts = millis();
            while ((millis()-ts)/1000 < READCOMMAND_TIMEOUT)
                if (telnet.available()) {
                    input = telnet.readString();
                    if ((input[0] == '0') || input.toInt()) {
                        power_target = input.toInt();
                        cmd_resp = "Grid power target set to ";
                        cmd_resp += power_target;
                        cmd_resp += " W\r\n\r\n";
                        return true;
                    }
                    break;
                }
            cmd_resp = "Grid power target remains at ";
            cmd_resp += power_target;
            cmd_resp += " W\r\n\r\n";
            break;
        case 'b':
            repeat_command = 'b';
            cmd_resp = "Batt voltage   : ";
            cmd_resp += String(vbat/1000.0,2);
            cmd_resp += "\r\nCell voltages  : ";
            cmd_resp += String(vcell_min/1000.0,3);
            cmd_resp += " - ";
            cmd_resp += String(vcell_max/1000.0,3);
            cmd_resp += "\r\nVoltage diff   : ";
            cmd_resp += String((vcell_max-vcell_min)/1000.0,3);
            cmd_resp += "\r\nActive balancer: ";
            if ((vcell_max >= BALANCER_THRESHOLD) && (vcell_max-vcell_min >= BALANCER_CELL_DIFF)) cmd_resp += "ON";
            else cmd_resp += "OFF";
            if (hm_power_limit > HM_MAX_POWER) {
                cmd_resp += "\r\nUVP power limit: ";
                cmd_resp += hm_power_limit;
                cmd_resp += " W";
            }
            if (mw_power_limit < MW_MAX_POWER) {
                cmd_resp += "\r\nOVP power limit: ";
                cmd_resp += mw_power_limit;
                cmd_resp += " W";
            }
            cmd_resp += "\r\n\r\n";
            break;
        case 'w':
            repeat_command = 'w';
            cmd_resp = "WiFi RSSI: ";
            cmd_resp += WiFi.RSSI();
            cmd_resp += " dBm\r\n\r\n";
            break;
        case 't':
            repeat_command = 't';
            cmd_resp = "Current time : ";
            ts = unixtime + (millis()- ts_EM + NO_COMMAND_TIMER + 100)/1000 + 1;
            sprintf(&datetime_string[0],"%02d/%02d/%04d %02d:%02d:%02d",day(ts),month(ts),year(ts),hour(ts),minute(ts),second(ts));
            cmd_resp += datetime_string;
            cmd_resp += "\r\nUTC offset   : +";
            cmd_resp += UTC_offset;
            if (UTC_offset == ESS_TIMEZONE+1) cmd_resp += " (DST)";
            cmd_resp += "\r\nESS started  : ";
            sprintf(&datetime_string[0],"%02d/%02d/%04d %02d:%02d:%02d",day(starttime),month(starttime),year(starttime),hour(starttime),minute(starttime),second(starttime));
            cmd_resp += datetime_string;
            cmd_resp += "\r\nESS uptime   : ";
            sprintf(&datetime_string[0],"%03dd %02dh %02dm %02ds   ",elapsedDays(ts-starttime),numberOfHours(ts-starttime),numberOfMinutes(ts-starttime),numberOfSeconds(ts-starttime));
            cmd_resp += datetime_string;
            cmd_resp += "\r\nSunrise today: ";
            cmd_resp += sunrise;
            cmd_resp += "\r\nSunset today : ";
            cmd_resp += sunset;
            cmd_resp += "\r\n\r\n";
            break;
        case 'd':
            cmd_resp = "DDNS address of ESS: ";
            cmd_resp += DDNS_address;
            cmd_resp += "\r\nLast DDNS update: ";
            sprintf(&datetime_string[0],"%02d/%02d/%04d %02d:%02d\r\n",day(DDNS_time),month(DDNS_time),year(DDNS_time),hour(DDNS_time),minute(DDNS_time));
            cmd_resp += datetime_string;
            cmd_resp += "\r\n";
            break;
        case 'l':
            cmd_resp = "Lowest household cons. since ";
            sprintf(&datetime_string[0],"%02d/%02d/%04d:      \r\n",day(starttime),month(starttime),year(starttime));
            cmd_resp += datetime_string;
            if (!minpower_time) cmd_resp += "Not yet measured\r\n\r\n";
            else {
                cmd_resp += power_grid_min;
                cmd_resp += " W (measured ";
                sprintf(&datetime_string[0],"%02d/%02d/%04d %02d:%02d)\r\n",day(minpower_time),month(minpower_time),year(minpower_time),hour(minpower_time),minute(minpower_time));
                cmd_resp += datetime_string;
                cmd_resp += "\r\n";
            }
            break;
        case 'n':
            telnet.print("\r\nReading Shelly energy counters ...");
            ShellyCommand(HMPLUG_STATUS);  // reads Hoymiles energy counter from HM plug
            ShellyCommand(MWPLUG_STATUS);  // reads Meanwell energy counter from MW plug
            ShellyCommand(PM_STATUS);  // reads PV energy counter from Shelly 1PM
            cmd_resp = "Energy [kWh] since ";
            sprintf(&datetime_string[0],"%02d/%02d/%04d:      \r\n",day(resettime_energy),month(resettime_energy),year(resettime_energy));
            cmd_resp += datetime_string;
            cmd_resp += "From PV  :";
            if (!from_pv) cmd_resp += " 0\r\n";
            else {
                cmd_resp += "\r\n";
                sprintf(&energy_string[0],"%7.3f",from_pv);
                cmd_resp += energy_string;
                cmd_resp += PV_SYMBOL[2];
                cmd_resp += FLOW_SYMBOL[2];
                cmd_resp += PV_CABLE_SYMBOL;
                cmd_resp += FLOW_SYMBOL[2];
                cmd_resp += CONS_SYMBOL_SHORT;
                sprintf(&energy_string[0],"%5.1f %%",(from_pv-(to_ess-grid_to_ess)-pv_to_grid)/from_pv*100);
                cmd_resp += energy_string;
                cmd_resp += "\r\n                ";
                cmd_resp += HOUSE_SYMBOL;
                cmd_resp += "\r\n";
                sprintf(&energy_string[0],"%5.1f %%",pv_to_grid/from_pv*100);
                cmd_resp += energy_string;
                cmd_resp += GRID_SYMBOL;
                cmd_resp += FLOW_SYMBOL[1];
                cmd_resp += GRID_CABLE_SYMBOL;
                cmd_resp += FLOW_SYMBOL[2];
                cmd_resp += ESS_SYMBOL;
                cmd_resp += " ";
                sprintf(&energy_string[0],"%5.1f %%",(to_ess-grid_to_ess)/from_pv*100);
                cmd_resp += energy_string;
                cmd_resp += "\r\n";
            }
            cmd_resp += "From Grid:";
            if (!from_grid) cmd_resp += " 0\r\n";
            else {
                cmd_resp += "\r\n       ";
                cmd_resp += PV_SYMBOL[2];
                cmd_resp += FLOW_SYMBOL[7];
                cmd_resp += PV_CABLE_SYMBOL;
                cmd_resp += FLOW_SYMBOL[2];
                cmd_resp += CONS_SYMBOL_SHORT;
                sprintf(&energy_string[0],"%5.1f %%",(from_grid-grid_to_ess)/from_grid*100);
                cmd_resp += energy_string;
                cmd_resp += "\r\n                ";
                cmd_resp += HOUSE_SYMBOL;
                cmd_resp += "\r\n";
                sprintf(&energy_string[0],"%7.3f",from_grid);
                cmd_resp += energy_string;
                cmd_resp += GRID_SYMBOL;
                cmd_resp += FLOW_SYMBOL[2];
                cmd_resp += GRID_CABLE_SYMBOL;
                cmd_resp += FLOW_SYMBOL[2];
                cmd_resp += ESS_SYMBOL;
                cmd_resp += " ";
                sprintf(&energy_string[0],"%5.1f %%",grid_to_ess/from_grid*100);
                cmd_resp += energy_string;
                cmd_resp += "\r\n";
            }
            cmd_resp += "From ESS :";
            if (!from_ess) cmd_resp += " 0\r\n";
            else {
                cmd_resp += "\r\n       ";
                cmd_resp += PV_SYMBOL[2];
                cmd_resp += FLOW_SYMBOL[7];
                cmd_resp += PV_CABLE_SYMBOL;
                cmd_resp += FLOW_SYMBOL[2];
                cmd_resp += CONS_SYMBOL_SHORT;
                sprintf(&energy_string[0],"%5.1f %%",(from_ess-ess_to_grid)/from_ess*100);
                cmd_resp += energy_string;
                cmd_resp += "\r\n                ";
                cmd_resp += HOUSE_SYMBOL;
                cmd_resp += "\r\n";
                sprintf(&energy_string[0],"%5.1f %%",ess_to_grid/from_ess*100);
                cmd_resp += energy_string;
                cmd_resp += GRID_SYMBOL;
                cmd_resp += FLOW_SYMBOL[1];
                cmd_resp += GRID_CABLE_SYMBOL;
                cmd_resp += FLOW_SYMBOL[1];
                cmd_resp += ESS_SYMBOL;
                cmd_resp += " ";
                sprintf(&energy_string[0],"%7.3f",from_ess);
                cmd_resp += energy_string;
                cmd_resp += "\r\n\r\n";
            }
            break;
        case 'e':
            cmd_resp = "Errors since ";
            sprintf(&datetime_string[0],"%02d/%02d/%04d:      \r\n",day(resettime_errors),month(resettime_errors),year(resettime_errors));
            cmd_resp += datetime_string;
            for (int i=0; i<ERROR_TYPES; i++) {
                cmd_resp += ERROR_TYPE[i];
                cmd_resp += "\t";
                cmd_resp += error_counter[i];
                if (error_counter[i]) {
                    cmd_resp += " (last error: ";
                    sprintf(&datetime_string[0],"%02d/%02d/%04d %02d:%02d)  ",day(errortime[i]),month(errortime[i]),year(errortime[i]),hour(errortime[i]),minute(errortime[i]));
                    cmd_resp += datetime_string;
                }
                cmd_resp += "\r\n";
            }
            cmd_resp += "\r\n";
            error_flag = false;
            break;
        case 'z':
            telnet.print("\r\nEnter [e] or [n] to reset error or energy stats: ");
            ts = millis();
            while ((millis()-ts)/1000 < READCOMMAND_TIMEOUT)
                if (telnet.available()) {
                    command = telnet.read();
                    if (command == 'e') {
                        for (int i=0; i<ERROR_TYPES; i++) error_counter[i] = errortime[i] = 0;
                        error_flag = false;
                        resettime_errors = unixtime;
                        cmd_resp = "Error stats reset to zero\r\n\r\n";
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
                        cmd_resp = "Energy stats reset to zero\r\n\r\n";
                        if (telnet.available()) telnet.flush();
                        return true;
                    }
                    break;
                }
            cmd_resp = "No stats reset\r\n\r\n";
            break;
        case 'c':
            cmd_resp = "";
            break;
        case 'r':
            telnet.print("\r\nEnter [y] to confirm system reboot: ");
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
            cmd_resp = "System not rebooted\r\n\r\n";
            break;
        default:
            cmd_resp = "Command options:\r\n";
            cmd_resp += "[m] - Manual power mode\r\n";
            cmd_resp += "[a] - Automatic power mode\r\n";
            cmd_resp += "[g] - Grid power target\r\n";
            cmd_resp += "[b] - Battery info\r\n";
            cmd_resp += "[w] - WiFi RSSI\r\n";
            cmd_resp += "[t] - Time, uptime, astro times\r\n";
            cmd_resp += "[d] - DDNS address of ESS\r\n";
            cmd_resp += "[l] - Lowest household consumption\r\n";
            cmd_resp += "[n] - Energy stats\r\n";
            cmd_resp += "[e] - Error stats\r\n";
            cmd_resp += "[z] - Reset stats to zero\r\n";
            cmd_resp += "[c] - Clear command response\r\n";
            cmd_resp += "[r] - Reboot system\r\n\r\n";
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
        http_resp = http.getString();
        http.end();
        if (http_resp != DDNS_address) {
            http.begin(DDNS_UPDATE + http_resp);
            if (http.GET() == HTTP_CODE_OK) {
                http.end();
                DDNS_address = http_resp;
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
    int mins_after_midnight = ESS_LOCATION.sunrise(year(unixtime), month(unixtime), day(unixtime), (UTC_offset == ESS_TIMEZONE+1));
    sprintf(&sunrise[0], "%02d:%02d", mins_after_midnight/60, mins_after_midnight%60);
    mins_after_midnight = ESS_LOCATION.sunset(year(unixtime), month(unixtime), day(unixtime), (UTC_offset == ESS_TIMEZONE+1));
    sprintf(&sunset[0], "%02d:%02d", mins_after_midnight/60, mins_after_midnight%60);
}

