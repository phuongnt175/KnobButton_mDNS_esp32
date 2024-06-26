#include <Mid/api/Mid_api.h>

const char *SSID = " ";
const char *PWD = " ";
const char *IPHC = " ";
const char *MACHC = " ";

char empty[] = " ";

extern WiFiClientSecure  net;
extern PubSubClient client;

const char key_mqtt[] PROGMEM = R"=====(
-----BEGIN CERTIFICATE-----
MIIDqTCCApGgAwIBAgIJAK7m4E783cWuMA0GCSqGSIb3DQEBCwUAMGsxCzAJBgNV
BAYTAlZOMQswCQYDVQQIDAJITjELMAkGA1UEBwwCSE4xDTALBgNVBAoMBExVTUkx
CzAJBgNVBAsMAlJEMQ4wDAYDVQQDDAVsb2NhbDEWMBQGCSqGSIb3DQEJARYHYWJj
QDEyMzAeFw0xOTA5MjMxMDU0NDZaFw0yOTA5MjAxMDU0NDZaMGsxCzAJBgNVBAYT
AlZOMQswCQYDVQQIDAJITjELMAkGA1UEBwwCSE4xDTALBgNVBAoMBExVTUkxCzAJ
BgNVBAsMAlJEMQ4wDAYDVQQDDAVsb2NhbDEWMBQGCSqGSIb3DQEJARYHYWJjQDEy
MzCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALoCzS2fxY3waRvIhewR
BIyBgkFQlagZYV7CGV0xorxHtjJs3+Q4nGxn/Xvl2dF4HE3WstJ+1JcSLYuLgpB3
Z9es68jvlCWX4zIa8Gne27mQSncdTuRV3K038RCKD8Ms00f1xd7cQFNZCHxPSdlV
TXydu4nXwsL8FAzOUXiHB7KT6s2F3JEPqhn9pgNGe7ciQZTfdTSEQPJ5w2wWrnnQ
7FccMxQmyyJNMfM3cwv26yhEFTIIKX9/9JCbqe75QIyeAxoTAUmJWtMvBBjT+HJ0
daGM1N60f6PsBYI4Y5o+NUsJa1ahPNvX44M4q/FxhbAyYOruztMyJFyYpiyxZpkO
8QsCAwEAAaNQME4wHQYDVR0OBBYEFO6WUjahqnRNCyzA54s78gae3LTsMB8GA1Ud
IwQYMBaAFO6WUjahqnRNCyzA54s78gae3LTsMAwGA1UdEwQFMAMBAf8wDQYJKoZI
hvcNAQELBQADggEBAF1ElC5P2hDpnaOiFarHkkVvvwrdry3H/jCckdffkUZFoAqa
AmeY1Zv4czcEVNDdkGh5nBSBlXxySvpR16Y6HM+4DlBlhv1FuBgR+LdHIU1jH86u
GNFX/Fq/jMv4rxBdJP9dgWnMW2vAnucU1DHqSgDD2TDFrvuz5EJADh73FKByYG7a
nVI0Ke+huGvqVv9ynmHBmWE1J4DjGD08IPypgTiS7GkRG3V/KpLpyV2M9FEAbmeP
KENRFSPMPeyRyfzitR98wTtsORlF4I1+fYcPGSh0pQK1mK1X1bI/BWmtnRMqBSXD
Eou01zV/f6o0PDqrnMlYhFi5gTg2bbqLYmLFgyw=
-----END CERTIFICATE-----
)=====";

char ssid[eepromTextVariableSize] = " ";
char pass[eepromTextVariableSize] = " ";
char iphc[eepromTextVariableSize] = " ";
char machc[eepromTextVariableSize] = " ";

IPAddress local_ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

boolean accessPointMode = false; // set true every time started as AP mode
boolean debug = true;

int softApStatus = 0;
int pushDownCounter = 0;

WebServer server(80);
StaticJsonDocument<4096> jsonDocument;

char g_Buffer[4096];
 
void addJsonObjectNet(String dns, String gw, String ssid, String ip, String netmask)
{
  JsonObject obj = jsonDocument.createNestedObject();
  obj["dns"] = dns;
  obj["gateway"] = gw;
  obj["ssid"] = ssid;
  obj["ip"] = ip;
  obj["netmask"] = netmask;
}

void addJsonObjectScan(String bssid, String ssid, int8_t rssi)
{
  JsonObject obj = jsonDocument.createNestedObject();
  obj["bssid"] = bssid;
  obj["ssid"] = ssid;
  obj["rssi"] = rssi;
}

void getNetwork() {
  jsonDocument.clear(); // Clear json buffer
  String dns = WiFi.dnsIP().toString();
  String gw = WiFi.gatewayIP().toString();
  String ipAddress = WiFi.localIP().toString();
  String netmask = WiFi.subnetMask().toString();

  addJsonObjectNet(dns, gw, WiFi.SSID(), ipAddress, netmask);

  serializeJson(jsonDocument, g_Buffer);
  server.send(200, "application/json", g_Buffer);

  ESP_LOGE("main", "Send WiFi network info");
}

void scanNetwork()
{
  jsonDocument.clear(); // Clear json buffer
  JsonObject obj = jsonDocument.createNestedObject();
  int n = WiFi.scanNetworks();
  obj["count"] = n;
  if(n != 0)
  {
    for(int i = 0; i < n; i++)
    {
      addJsonObjectScan( WiFi.BSSIDstr(i), WiFi.SSID(i), WiFi.RSSI(i));
    }
  }
  serializeJson(jsonDocument, g_Buffer);
  server.send(200, "application/json", g_Buffer);
  ESP_LOGE("main", "Scan WiFi");
}

void handlePut()
{
  if(server.hasArg("plain") == false)
  {

  }
  String body = server.arg("plain");
  deserializeJson(jsonDocument, body);

  SSID = jsonDocument["ssid"];
  PWD = jsonDocument["password"];
  IPHC = jsonDocument["ip"];
  MACHC = jsonDocument["mac"];

  server.send(200, "application/json", "{\n status = true \n}");

  saveWiFiToEEPPROM((char*)SSID, (char*)PWD);
  readWiFiFromEEPROM(ssid, pass);
  saveHCInfoToEEPPROM((char*)IPHC, (char*)MACHC);
  readHCInfoFromEEPROM(iphc, machc);

  ESP_LOGE("main", "after api ssid change to : %s", ssid);
  ESP_LOGE("main", "after api pass change to : %s", pass);
  ESP_LOGE("main", "IP HC: %s", iphc);
  ESP_LOGE("main", "Mac HC: %s", machc);
  
  accessPointMode = false;
  softApStatus = 0;
  saveStatusToEeprom(0);
  WiFi.enableAP(false);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA); //disable softAP

  client.setServer(iphc, 38883);

  return setupApi();
}


void setupApi()
{
  int i =0;
  WiFi.disconnect();
  if(accessPointMode == 0)
  {
    WiFi.begin(ssid, pass);
    ESP_LOGE("main", "Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    ESP_LOGE("main", " . ");
    delay(500);
    i++;
    if(i == 10)
    {
      ESP_LOGE("main", "Connect fail, wrong password, enable AP mode");
      accessPointMode = true;
      return accessPoint_init();
    }
  }
  ESP_LOGE("main", "WiFi Connected : %s", WiFi.SSID());
  ESP_LOGE("main", "%s", WiFi.localIP().toString().c_str());
  }
  lv_obj_set_style_bg_color(ui_resetWifi, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(ui_resetWifi, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
}

void accessPoint_init()
{
  byte mac1[6];
  char mac1Address[18];
  WiFi.macAddress(mac1);
  sprintf(mac1Address, "%02X:%02X",mac1[4], mac1[5]);
  char apSSID[33];
  sprintf(apSSID, "SwitchIP_%s", mac1Address);
  WiFi.softAP(apSSID, "ABC123456");
  if(debug)
  {
    ESP_LOGE("main", "AccessPoint IP: %s", WiFi.softAPIP().toString());
  }
  ESP_LOGE("main", "Start AP mode");
  WiFi.softAPConfig(local_ip, gateway, subnet);
  server.on("/api/device/network", getNetwork);
  server.on("/api/device/network/scan", scanNetwork);
  server.on("/api/device/network/config", HTTP_PUT, handlePut);
  // start server
  server.begin();
  //delay(100);
}

void ui_event_resetWifi(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t * target = lv_event_get_target(e);
  while(event_code == LV_EVENT_LONG_PRESSED) //detect button released, run the DemandWiFi function
  {
    switch (softApStatus)
    {
      case 1:
        if (accessPointMode) saveStatusToEeprom(0);
        delay(500);
        softApStatus = 0;
        lv_obj_set_style_bg_color(ui_resetWifi, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(ui_resetWifi, 50, LV_PART_MAIN | LV_STATE_DEFAULT);
        ESP_LOGE("api","off ap");
        return setupAP();
      break;
      
      case 0:
        if (!accessPointMode) saveStatusToEeprom(2);
        delay(500);
        softApStatus = 1;
        lv_obj_set_style_bg_color(ui_resetWifi, lv_color_hex(0x3399FF), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(ui_resetWifi, 150, LV_PART_MAIN | LV_STATE_DEFAULT);
        ESP_LOGE("api","on ap");
        return setupAP();
      break;

      default:
      break;
    }
  }
}

//====================== EEPROM necessary functions ==============
#define eepromBufferSize 200 // have to be > eepromTextVariableSize * (eepromVariables+1) (33 * (5+1))

//========================================== writeDefaultSettingsToEEPPROM
void saveWiFiToEEPPROM(char* ssid_, char* pass_)
{
ESP_LOGE("main", "\n============ saveWiFiToEEPPROM");
writeEEPROM((1 * eepromTextVariableSize) , eepromTextVariableSize , ssid_);
writeEEPROM((2 * eepromTextVariableSize) , eepromTextVariableSize , pass_);
}

void saveHCInfoToEEPPROM(char* iphc_, char* machc_)
{
  ESP_LOGE("main", "\n============ saveHCInfoToEEPPROM");
  writeEEPROM((3 * eepromTextVariableSize) , eepromTextVariableSize , iphc_);
  writeEEPROM((4 * eepromTextVariableSize) , eepromTextVariableSize , machc_);
}
//========================================== readSettingsFromEeprom
void readWiFiFromEEPROM(char* ssid_, char* pass_) {
readEEPROM( (1 * eepromTextVariableSize) , eepromTextVariableSize , ssid_);
readEEPROM( (2 * eepromTextVariableSize) , eepromTextVariableSize , pass_);
ESP_LOGE("main", "\n============ readWiFifromEEPPROM");
}

void readHCInfoFromEEPROM(char* iphc_, char* machc_) {
readEEPROM( (3 * eepromTextVariableSize) , eepromTextVariableSize , iphc_);
readEEPROM( (4 * eepromTextVariableSize) , eepromTextVariableSize , machc_);
ESP_LOGE("main", "\n============ readHCInfofromEEPPROM");
}

//================================================================
void writeEEPROM(int startAdr, int length, char* writeString) {
EEPROM.begin(eepromBufferSize);
yield();
for (int i = 0; i < length; i++) EEPROM.write(startAdr + i, writeString[i]);
EEPROM.commit();
EEPROM.end();
}

//================================================================
void readEEPROM(int startAdr, int maxLength, char* dest) {
EEPROM.begin(eepromBufferSize);
delay(10);
for (int i = 0; i < maxLength; i++) dest[i] = char(EEPROM.read(startAdr + i));
dest[maxLength - 1] = 0;
EEPROM.end();
}

//================================================================ writeEepromSsid
void saveStatusToEeprom(byte value) {
EEPROM.begin(eepromBufferSize);
EEPROM.write(0, value);
EEPROM.commit();
EEPROM.end();
}
//===================================================================
byte getStatusFromEeprom() {
EEPROM.begin(eepromBufferSize);
byte value = 0;
value = EEPROM.read (0);
EEPROM.end();
return value;
}

void eraseEEPROM() {
  EEPROM.begin(eepromBufferSize);
  saveWiFiToEEPPROM(empty, empty);
  saveHCInfoToEEPPROM(empty, empty);
  readWiFiFromEEPROM(ssid, pass);
  readHCInfoFromEEPROM(iphc, machc);
  for (int address = 0; address < eepromBufferSize; address++) {
    EEPROM.write(address, 0);
  }
  EEPROM.commit();
  EEPROM.end();
}


void setupAP(void)
{
  int st = getStatusFromEeprom();
  ESP_LOGE("main", "%d", getStatusFromEeprom());
  if(st == 2){
    accessPointMode = true;
  } 
  else if(st != 0) {
    saveWiFiToEEPPROM(ssid, pass);
    saveHCInfoToEEPPROM(iphc, machc);
  }
  else if(st == 0)
  {
    WiFi.enableAP(false);
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA); //disable softAP
    ESP_LOGE("api", "Disable AP mode");
    accessPointMode = false;
  }

  ESP_LOGE("main", "AP mode = %s", String(accessPointMode));

  readWiFiFromEEPROM(ssid, pass);
  readHCInfoFromEEPROM(iphc, machc);
  ESP_LOGE("main", "ssid: %s", ssid);
  ESP_LOGE("main", "pass: %s", pass);
  ESP_LOGE("main", "IP HC: %s", iphc);
  ESP_LOGE("main", "MAC HC: %s", machc);

  if(accessPointMode)
  {
    accessPoint_init();
    saveStatusToEeprom(0);
  }
}

void handleAP()
{
  server.handleClient();
}