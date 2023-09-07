#include <ArduinoJson.h>
#include <ui.h>

void generateJsonCommandPost(const String& bridgeKey, const String& reqId, JsonArray ruleConfig, char jsonString[4096], char *macDevice);
void generateJsonCmdStatus(const String& bridgeKey, const String& reqId, char jsonString[1024], char *macDevice, const char *ep, boolean flag);
void responseGetStatus(const String& bridgeKey, const String& reqId, char jsonString[], char* macDevice);
void advanceStatusCmd(const String& bridgeKey, const String& reqId, JsonArray ruleConfig, char jsonString[4096], char *macDevice) ;
void activeRuleCmd(const String& reqId ,char jsonString[], const char ruleId[]);