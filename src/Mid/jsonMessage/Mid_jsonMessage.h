#include <ArduinoJson.h>
#include <ui.h>
#include <FS.h>
#include <LittleFS.h>

void generateJsonCommandPost(const String& bridgeKey, const String& reqId, String ruleConfig, char jsonString[], char *macDevice);
void generateJsonCmdStatus(const String& bridgeKey, const String& reqId, char jsonString[], char *macDevice, const char *ep, boolean flag);
void responseGetStatus(const String& bridgeKey, const String& reqId, char jsonString[], char* macDevice);
void advanceStatusCmd(const String& bridgeKey, const String& reqId, String ruleConfig, char jsonString[], char *macDevice, char *macHC);
void activeRuleCmd(const String& reqId ,char jsonString[], const char ruleId[]);

void writeJsonToFile(const char *path, String output);
void appendJsonToFile(const char *path, String output);
String readJsonFromFile(const char *path);

const char *readRuleIDValue(const char *path, int num);
const char *readIconKeyValue(const char *path, int num);
const char *readNameValue(const char *path, int num);
int readEnableValue(const char *path, int num);
// static int enableStatus;
// static const char *icon;
// static const char *name;