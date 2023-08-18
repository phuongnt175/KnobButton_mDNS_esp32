#include <App/CmdMessage/cmdStatus.hpp>

// char cmdStatus[] = "{\"cmd\":\"status\",\"objects\":[{\"bridge_key\":\"zigbee\",\"data\":[{\"hash\":\"zigbee-80:4B:50:FF:FE:FA:83:B1-1\","
// "\"states\":{\"OnOff\":{\"on\":false}},\"type\":\"SWITCH\"}],\"type\":\"devices\"}],\"reqid\":\" \",\"source\":\"zigbee\"}";
// DynamicJsonDocument jsonStatus(4096);

// void onStatus()
// {
//     DeserializationError error = deserializeJson(jsonStatus, cmdStatus);
//     jsonStatus["objects"][0]["data"][0]["states"]["OnOff"]["on"] = "false";
//     serializeJson(jsonStatus, cmdStatus);
// }