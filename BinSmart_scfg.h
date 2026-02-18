// ESS configuration (sensitive data)

// Networking
const char WIFI_SSID[] = "***";
const char WIFI_PWD = "***";
const char JKBMS_MAC_ADDR[] = "**:**:**:**:**:**";  // MAC (= BLE) address of JKBMS
IPAddress ESP32_ADDR(***,***,***,***);  // Local IP address of ESS
IPAddress ROUTER_ADDR(***,***,***,***);   // Local WiFi router
IPAddress SUBNET(***,***,***,***);   // WiFi subnet
IPAddress EM_ADDR(***,***,***,***);  // Shelly 3EM
IPAddress PM1_ADDR(***,***,***,***);  // Shelly Plus 1PM, connecting Maxeon solar panel to AC
IPAddress PM2_ADDR(***,***,***,***);  // Shelly 2PM Gen3, connecting Meanwell charger and Hoymiles inverter to AC
const int TELNET_PORT = ***;  // TCP port for communication with terminal

// Astro settings
const float LATITUDE = **.***;  // latitude of ESS
const float LONGITUDE = **.***;  // longitude of ESS
const int TIMEZONE = **;  // timezone (relative to UTC) of ESS

// Hoymiles serial number
const byte HM_SN[] = {0x**, 0x**, 0x**, 0x**, 0x**, 0x**};  // serial number of Hoymiles inverter

// DDNS service
const char DDNS_SERVER_URL[] = "/nic/update?hostname=***&myip=";
const char DDNS_SERVER_CREDS[] = "***";  // base 64 encoded user credentials
