#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <esp_log.h>
#include <Mid/jsonMessage/Mid_jsonMessage.h>

String randomReqId(int length, String& reqId);
void connectBroker();
void jsonPostCmdTask(void* pvParameters);