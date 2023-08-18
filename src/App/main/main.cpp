/*
 * Author: PhuongNT
 * Custom by PhuongNT.
 * Last Change: 21-07-2023
 */
/******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdio.h>
#include <Arduino_GFX_Library.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Mid/button/button.hpp>
#include <Mid/mt8901/mt8901.hpp>
#include <Arduino.h>
#include <ESPmDNS.h>
#include <FastLED.h>
#include <WiFi.h>
#include <lvgl.h>
#include <Mid/api/api.h>
#include <App/main/main.h>
#include <App/CmdMessage/cmdStatus.hpp>
//#include <App/CmdMessage/CmdPost.hpp>

/******************************************************************************/
/*                     PRIVATE TYPES and DEFINITIONS                         */
/******************************************************************************/
#define ECO_O(y) (y > 0) ? -1 : 1
#define ECO_STEP(x) x ? ECO_O(x) : 0
#define GFX_BL 38

#define LED_PIN 4
#define LED_NUM 13
CRGB leds[LED_NUM];

#define SERVICE_NAME "airplay"
#define SERVICE_PROTOCOL "tcp"
#define SERVICE_PORT 5600

#define ON_MSG    "true"
#define OFF_MSG   "false"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
extern boolean accessPointMode;

/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
const char *MQTT_USER = "component"; // leave blank if no credentials used
const char *MQTT_PASS = " "; // leave blank if no credentials used
const char *statusTopic = "component/zigbee/status";
const char *controlTopic = "component/zigbee/control";
const char *configTopic = "component/zigbee/config";

char ep1[] = "zigbee-f4:12:fa:cf:4e:b4-1";
char ep2[] = "zigbee-f4:12:fa:cf:4e:b4-3";
char ep3[] = "zigbee-f4:12:fa:cf:4e:b4-5";
char ep4[] = "zigbee-f4:12:fa:cf:4e:b4-7";

char postMess[] = "{\"cmd\" : \"post\",\"objects\" : [{\"bridge_key\" : \"zigbee\",\"data\" : [{\"attr\" : {\"McuInfo\" : {\"GlassType\" : 2,\"RelayType\" : 0},\"deviceInfo\" : {\"Manufacturer\" : \"Lumi R&D\",\"ModelId\" : \"LM-SZDM4\"},\"sceneConfig\" : {\"led_level\" : 1,"
"\"led_schedule\" : {\"enable\" : false,\"time\" : {\"endtime\" : \"00:00\",\"starttime\" : \"00:00\"}},\"lock_touch\" : false,\"output\" : {\"delay\" : 0,\"mode\" : 0},\"touch_mode\" : 0},\"sceneSetting\" : [{\"name\" : \"lock_touch\"},{\"name\" : \"touch_mode\"},{\"name\" : \"led_level\"},"
"{\"name\" : \"led_schedule\"}]},\"brigde_key\" : \"zigbee\",\"config\" : {},\"deviceInfo\" : {\"ApplicationVer\" : \"\",\"DataCode\" : \"\",\"HardwareVer\" : \"\",\"Manufacturer\" : \"Lumi R&D\",\"ModelId\" : \"LM-SZDM4\",\"ZigbeeProtocolVer\" : \"3.0\",\"ZigbeeStackVer\" : \"\"},"
"\"hash\" : \"zigbee-f4:12:fa:cf:4e:b4-1\",\"mac\" : \"f4:12:fa:cf:4e:b4\",\"macdev\" : \"f4:12:fa:cf:4e:b4\",\"traits\" : [{\"is_main\" : true,\"name\" : \"OnOff\"}],\"type\" : \"SWITCH\"},{\"attr\" : {\"McuInfo\" : {\"GlassType\" : 2,\"RelayType\" : 0},"
"\"deviceInfo\" : {\"Manufacturer\" : \"Lumi R&D\",\"ModelId\" : \"LM-SZDM4\"},\"sceneConfig\" : {\"lock_touch\" : false,\"output\" : {\"delay\" : 0,\"mode\" : 0},\"touch_mode\" : 0},\"sceneSetting\" : [{\"name\" : \"lock_touch\"},{\"name\" : \"touch_mode\"}]},\"brigde_key\" : \"zigbee\","
"\"config\" : {},\"deviceInfo\" : {},\"hash\" : \"zigbee-f4:12:fa:cf:4e:b4-3\",\"mac\" : \"f4:12:fa:cf:4e:b4\",\"macdev\" : \"f4:12:fa:cf:4e:b4\",\"traits\" : [{\"is_main\" : true,\"name\" : \"OnOff\"}],\"type\" : \"SWITCH\"},{\"attr\" : {\"McuInfo\" : {\"GlassType\" : 2,\"RelayType\" : 0},"
"\"deviceInfo\" : {\"Manufacturer\" : \"Lumi R&D\",\"ModelId\" : \"LM-SZDM4\"},\"sceneConfig\" : {\"lock_touch\" : false,\"output\" : {\"delay\" : 0,\"mode\" : 0},\"touch_mode\" : 0},\"sceneSetting\" : [{\"name\" : \"lock_touch\"},{\"name\" : \"touch_mode\"}]},\"brigde_key\" : \"zigbee\",\"config\" : {},"
"\"deviceInfo\" : {},\"hash\" : \"zigbee-f4:12:fa:cf:4e:b4-5\",\"mac\" : \"f4:12:fa:cf:4e:b4\",\"macdev\" : \"f4:12:fa:cf:4e:b4\",\"traits\" : [{\"is_main\" : true,\"name\" : \"OnOff\"}],\"type\" : \"SWITCH\"},{\"attr\" : {\"McuInfo\" : {\"GlassType\" : 2,\"RelayType\" : 0},"
"\"deviceInfo\" : {\"Manufacturer\" : \"Lumi R&D\",\"ModelId\" : \"LM-SZDM4\"},\"sceneConfig\" : {\"lock_touch\" : false,\"output\" : {\"delay\" : 0,\"mode\" : 0},\"touch_mode\" : 0},\"sceneSetting\" : [{\"name\" : \"lock_touch\"},{\"name\" : \"touch_mode\"}]},\"brigde_key\" : \"zigbee\",\"config\" : {},"
"\"deviceInfo\" : {},\"hash\" : \"zigbee-f4:12:fa:cf:4e:b4-7\",\"mac\" : \"f4:12:fa:cf:4e:b4\",\"macdev\" : \"f4:12:fa:cf:4e:b4\",\"traits\" : [{\"is_main\" : true,\"name\" : \"OnOff\"}],\"type\" : \"SWITCH\"}],\"type\" : \"devices_local\"}],\"reqid\" : \"CQ3OqOrTUPImOMV\",\"source\" : \"zigbee\"}";

char cmdStatus[] = "{\"cmd\":\"status\",\"objects\":[{\"bridge_key\":\"zigbee\",\"data\":[{\"hash\":\"zigbee-f4:12:fa:cf:4e:b4-1\",\"states\":{\"OnOff\":{\"on\":true}},\"type\":\"SWITCH\"}],\"type\":\"devices\"}],\"reqid\":\" \",\"source\":\"zigbee\"}";

StaticJsonDocument<4096> jsonStatus;
DeserializationError error = deserializeJson(jsonStatus, cmdStatus);
char output[4096];
//==============================================================================

//==============================================================================
WiFiClientSecure  net;
PubSubClient      client(net);
IPAddress         ADDRESS;

int PORT = 38883;
// bool res;
ButtonStatus btnStatus1 = OFF;
ButtonStatus btnStatus2 = OFF;
ButtonStatus btnStatus3 = OFF;
ButtonStatus btnStatus4 = OFF;

static button_t *g_btn;
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;
static lv_group_t *lv_group;

extern lv_obj_t* ui_resetWifi;

char TAG[] = "Main";

const char local_root_ca[] PROGMEM = R"=====(
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

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/
void mDNSService();
void init_lv_group();
void ui_event_button(lv_event_t *e, ButtonStatus& btn_status, char ep);

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/

/******************************************************************************/
Arduino_DataBus *bus = new Arduino_SWSPI(
  GFX_NOT_DEFINED /* DC */, 21 /* CS */,
  47 /* SCK */, 41 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
  39 /* DE */, 48 /* VSYNC */, 40 /* HSYNC */, 45 /* PCLK */,
  10 /* R0 */, 16 /* R1 */, 9 /* R2 */, 15 /* R3 */, 46 /* R4 */,
  8 /* G0 */, 13 /* G1 */, 18 /* G2 */, 12 /* G3 */, 11 /* G4 */, 17 /* G5 */,
  47 /* B0 */, 41 /* B1 */, 0 /* B2 */, 42 /* B3 */, 14 /* B4 */,
  1 /* hsync_polarity */, 10 /* hsync_front_porch */, 10 /* hsync_pulse_width */, 10 /* hsync_back_porch */,
  1 /* vsync_polarity */, 14 /* vsync_front_porch */, 2 /* vsync_pulse_width */, 12 /* vsync_back_porch */);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
  480 /* width */, 480 /* height */, rgbpanel, 0 /* rotation */, true /* auto_flush */,
  bus, GFX_NOT_DEFINED /* RST */, st7701_type7_init_operations, sizeof(st7701_type7_init_operations));

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void encoder_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  static int16_t cont_last = 0;
  int16_t cont_now = mt8901_get_count();
  data->enc_diff = ECO_STEP(cont_now - cont_last);
  cont_last = cont_now;
  if (button_isPressed(g_btn)) {
    data->state = LV_INDEV_STATE_PR;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void connectBroker()
{
  while(!client.connected())
  {
    ESP_LOGE(TAG, "Attemping MQTT connection...");
    String clientId = "SW-";
    clientId += String(random(0xffff), HEX);
    if(client.connect(clientId.c_str(), MQTT_USER, MQTT_PASS))
    {
      client.subscribe(statusTopic);
      client.subscribe(controlTopic);
      ESP_LOGE(TAG, "Connected");

      client.publish(configTopic,postMess);

    }
    else
    {
      ESP_LOGE(TAG, "Error rc = %d", client.state());
      ESP_LOGE(TAG, "try again in 2 seconds");
      delay(200);
    }
  }
}

void SubCallback(lv_obj_t *ui, char* message, ButtonStatus& btn_status, char *ep)
{
  if(strstr(message, ON_MSG) != NULL)
  {
    //ESP_LOGE(TAG, "ON");
    btn_status = ON;
    if(lv_obj_get_state(ui) == 6 || lv_obj_get_state(ui) == 0)
    {
      _ui_state_modify(ui, LV_STATE_CHECKED, 2);// _UI_STATE_MODIFY_TOGGLE
      jsonStatus["objects"][0]["data"][0]["states"]["OnOff"]["on"] = true;
      jsonStatus["objects"][0]["data"][0]["hash"] = ep;
      serializeJson(jsonStatus, output);
      client.publish(statusTopic, output);
    }
    //ESP_LOGE(TAG, "-------------------------------------------------------------");
  }
  else if(strstr(message, OFF_MSG) != NULL)
  {
    //ESP_LOGE(TAG, "OFF");
    btn_status = OFF;
    _ui_state_modify(ui, LV_STATE_CHECKED, 1);// _UI_STATE_MODIFY_TOGGLE
    jsonStatus["objects"][0]["data"][0]["states"]["OnOff"]["on"] = false;
    jsonStatus["objects"][0]["data"][0]["hash"] = ep;
    serializeJson(jsonStatus, output);
    client.publish(statusTopic, output);
    //ESP_LOGE(TAG, "-------------------------------------------------------------");
  }
}

void Callback(char* topic, byte* payload, unsigned int length) {

  payload[length] = '\0'; //NULL terminator used to terminate the char array
  char* message = (char*)payload;
  if(String(topic) == controlTopic)
  {
    ESP_LOGE(TAG, "%s", message);
    if(strstr(message, ep1) != NULL)
    {
      SubCallback(ui_button1, message, btnStatus1, ep1);
    }
    if(strstr(message, ep2) != NULL)
    {
      SubCallback(ui_button2, message, btnStatus2, ep2);
    }
    if(strstr(message, ep3) != NULL)
    {
      SubCallback(ui_button3, message, btnStatus3, ep3);
    }
    if(strstr(message, ep4) != NULL)
    {
      SubCallback(ui_button4, message, btnStatus4, ep4);
    }
  }
}

void setup() {
  Serial.begin(115200);

  gfx->begin(-1);
  gfx->fillScreen(BLACK);

  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, LED_NUM);
  FastLED.showColor(CHSV(64, 255, 255));

  lv_init();

  // Hardware Button
  g_btn = button_attch(3, 0, 10);

  // Magnetic Encoder
  mt8901_init(5, 6);

  screenWidth = gfx->width();
  screenHeight = gfx->height();

  // Must use PSRAM
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * 32, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  if (!disp_draw_buf) {
    //Serial.println("LVGL disp_draw_buf allocate failed!");
    ESP_LOGE(TAG, "LVGL disp_draw_buf allocate failed!");
  } else {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * 32);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* Initialize the input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = encoder_read;
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    lv_indev_drv_register(&indev_drv);
  }

  setupAP();
  setupApi();

  byte mac[6];
  WiFi.macAddress(mac);

  ESP_LOGE(TAG, "Mac device: %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if(!MDNS.begin("esp32")) {
    //Serial.println("Error starting mDNS");
    ESP_LOGE(TAG, "error starring mDNS!");
    return;
  }

  mDNSService();
  ADDRESS = MDNS.queryHost("HCD84F");
  net.setInsecure();
  net.setCACert(local_root_ca);
  client.setKeepAlive(60);
  client.setBufferSize(4096);
  client.setServer(ADDRESS, PORT);
  client.setCallback(Callback);
  init_lv_group();
  ui_init();
}

void loop() {
  lv_timer_handler();
  handleAP();
  if(!client.connected())
  {
    client.setKeepAlive(60); // setting keep alive to 60 seconds
    if(accessPointMode == false)
    {
      connectBroker();
    }
  }
  else{
    client.loop();
  }
}

void init_lv_group() {
  lv_group = lv_group_create();
  lv_group_set_default(lv_group);

  lv_indev_t *cur_drv = NULL;
  for (;;) {
    cur_drv = lv_indev_get_next(cur_drv);
    if (!cur_drv) {
      break;
    }
    if (cur_drv->driver->type == LV_INDEV_TYPE_ENCODER) {
      lv_indev_set_group(cur_drv, lv_group);
    }
  }
}

void mDNSService()
{
  MDNS.addService(SERVICE_NAME, SERVICE_PROTOCOL, SERVICE_PORT);
  MDNS.addServiceTxt(SERVICE_NAME, SERVICE_PROTOCOL, "manufacturer", "LUMI");
  MDNS.addServiceTxt(SERVICE_NAME, SERVICE_PROTOCOL, "mac", "f4:12:fa:cf:4e:b4");
  
  int nrOfServices = MDNS.queryService("lumismarthome", SERVICE_PROTOCOL);
  if (nrOfServices == 0) {
    ESP_LOGE(TAG, "No services were found.");
  } 
  else {
    ESP_LOGE(TAG, "Number of services found: %d", nrOfServices);
    for (int i = 0; i < nrOfServices; i=i+1) 
    {
      ESP_LOGE(TAG, "---------------");
      ESP_LOGE(TAG, "Hostname: %s", MDNS.hostname(i));
      ESP_LOGE(TAG, "IP address: %s", MDNS.IP(i).toString().c_str());
      ESP_LOGE(TAG, "Port: %d", MDNS.port(i));
      ESP_LOGE(TAG, "---------------");
    }
  }
}

void ui_event_button(lv_event_t *e, ButtonStatus& btn_status, char *ep) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  
  if (event_code == LV_EVENT_VALUE_CHANGED) {
    if (lv_obj_has_state(target, LV_STATE_CHECKED)) {
      if (btn_status == OFF) {
        //client.publish(sw_topic, message1);
        jsonStatus["objects"][0]["data"][0]["states"]["OnOff"]["on"] = true;
        jsonStatus["objects"][0]["data"][0]["hash"] = ep;
        serializeJson(jsonStatus, output);
        client.publish(statusTopic, output);
        btn_status = ON;
      }
    } else {
      if (btn_status == ON) {
        //client.publish(sw_topic, message2);
        jsonStatus["objects"][0]["data"][0]["states"]["OnOff"]["on"] = false;
        jsonStatus["objects"][0]["data"][0]["hash"] = ep;
        serializeJson(jsonStatus, output);
        client.publish(statusTopic, output);
        btn_status = OFF;
      }
    }
  }
}

void ui_event_button1(lv_event_t *e) {
  ui_event_button(e, btnStatus1, ep1);
}

void ui_event_button2(lv_event_t *e) {
  ui_event_button(e, btnStatus2, ep2);
}

void ui_event_button3(lv_event_t *e) {
  ui_event_button(e, btnStatus3, ep3);
}

void ui_event_button4(lv_event_t *e) {
  ui_event_button(e, btnStatus4, ep4);
}