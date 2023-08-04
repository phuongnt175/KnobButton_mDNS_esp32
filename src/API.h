#include <WebServer.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <esp_log.h>
#include <ui.h>

#define eepromTextVariableSize 33

//=======================================================================================
void saveSettingsToEEPPROM(char* ssid_, char* pass_);
void readSettingsFromEEPROM(char* ssid_, char* pass_);
void writeEEPROM(int startAdr, int length, char* writeString);
void readEEPROM(int startAdr, int maxLength, char* dest);
void saveStatusToEeprom(byte value);
byte getStatusFromEeprom();
void setupAP(void);
void handleAP();

//=======================================================================================

void addJsonObjectNet(String dns, String gw, String ssid, String ip, String netmask);
void addJsonObjectScan(String bssid, String ssid, int8_t rssi);
void getNetwork();
void scanNetwork();
void handlePut();
void setupApi();