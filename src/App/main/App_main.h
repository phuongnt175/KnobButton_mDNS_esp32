#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string>
#include <Mid/button/Mid_button.hpp>
#include <Mid/mt8901/Mid_mt8901.hpp>
#include <Mid/jsonMessage/Mid_jsonMessage.h>
#include <Mid/mqtt/Mid_mqtt.h>
#include <Mid/api/Mid_api.h>

#define FORMAT_SPIFFS_IF_FAILED true

enum ButtonStatus 
{
    OFF,
    ON
};

