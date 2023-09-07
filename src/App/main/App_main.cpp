/*
 * Author: PhuongNT
 * Custom by PhuongNT.
 * Last Change: 21-08-2023
 */
/******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <stdio.h>
#include <iostream>
#include <Arduino_GFX_Library.h>
#include <Arduino.h>
#include <ESPmDNS.h>
#include <FastLED.h>
#include <lvgl.h>
#include <App/main/App_main.h>

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
extern char iphc[eepromTextVariableSize];
extern char machc[eepromTextVariableSize];

extern String bridgeKey;
extern String reqId;
extern char output[4096];
extern byte mac[6];
extern char macAddress[18];

extern const char *statusTopic;
extern const char *controlTopic;
extern const char *configTopic;
const char *demoTopic = "client/mobile/ivIRldP5KS/config";

extern WiFiClientSecure  net;
extern PubSubClient      client;
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
const char *ep1 = "-1";
const char *ep2 = "-3";
const char *ep3 = "-5";
const char *ep4 = "-7";

const char *ruleId;

long lastTime = 0;
//==============================================================================

//==============================================================================

IPAddress   ADDRESS;

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
void checkIpHCmDNS();
void init_lv_group();
void ui_event_button(lv_event_t *e, ButtonStatus& btn_status, char *ep);
void ui_event_scene(lv_event_t *e);
void spiffsInit();

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

void outlineBorder(lv_obj_t *ui)
{
  static lv_style_t style;
  lv_style_init(&style);

  lv_style_set_outline_width(&style, 5);
  lv_style_set_outline_color(&style, lv_palette_main(LV_PALETTE_BLUE));
  lv_style_set_outline_pad(&style, 8);

  lv_obj_add_style(ui, &style, LV_STATE_FOCUS_KEY);
}

void SubCallback(lv_obj_t *ui, char* message, ButtonStatus& btn_status, const char *ep)
{
  if(strstr(message, ON_MSG) != NULL)
  {
    btn_status = ON;
    if(lv_obj_get_state(ui) == 6 || lv_obj_get_state(ui) == 0)
    {
      generateJsonCmdStatus(bridgeKey, reqId, output, macAddress, ep, true);
      client.publish(statusTopic, output);
      _ui_state_modify(ui, LV_STATE_CHECKED, 2);// _UI_STATE_MODIFY_TOGGLE
    }
  }
  else if(strstr(message, OFF_MSG) != NULL)
  {
    btn_status = OFF;
    generateJsonCmdStatus(bridgeKey, reqId, output, macAddress, ep, false);
    client.publish(statusTopic, output);
    _ui_state_modify(ui, LV_STATE_CHECKED, 1);// _UI_STATE_MODIFY_TOGGLE
  }
}

void Callback(char* topic, byte* payload, unsigned int length) {

  payload[length+1] = '\0'; //NULL terminator used to terminate the char array
  char* message = (char*)payload;
  std::string json = std::string(message);
  char hash1[50];
  char hash3[50];
  char hash5[50];
  char hash7[50];

  sprintf(hash1, "%s-%s%s", bridgeKey.c_str(), macAddress, ep1);
  sprintf(hash3, "%s-%s%s", bridgeKey.c_str(), macAddress, ep2);
  sprintf(hash5, "%s-%s%s", bridgeKey.c_str(), macAddress, ep3);
  sprintf(hash7, "%s-%s%s", bridgeKey.c_str(), macAddress, ep4);

  std::string s_ep1 = std::string(hash1);
  std::string s_ep2 = std::string(hash3);
  std::string s_ep3 = std::string(hash5);
  std::string s_ep4 = std::string(hash7);

  if(String(topic) == controlTopic && strstr(message, "set") != NULL) //
  {
    ESP_LOGE("main", "%s", message);
    if((json.find(s_ep1))!= std::string::npos)
    {
      SubCallback(ui_button1, message, btnStatus1, ep1);
    }
    if((json.find(s_ep2))!= std::string::npos)
    {
      SubCallback(ui_button2, message, btnStatus2, ep2);
    }
    if((json.find(s_ep3))!= std::string::npos)
    {
      SubCallback(ui_button3, message, btnStatus3, ep3);
    }
    if((json.find(s_ep4))!= std::string::npos)
    {
      SubCallback(ui_button4, message, btnStatus4, ep4);
    }
  }

  if(String(topic) == controlTopic && strstr(message, "get") != NULL) //response status when app send get message status
  {
    if(strstr(message, macAddress) != NULL)
    {
      responseGetStatus(bridgeKey, reqId, output, macAddress);
      client.publish(statusTopic, output);
    }
  }

  if(String(topic) == configTopic && strstr(message, "delete") != NULL) //remove device
  {
    if(strstr(message, macAddress) != NULL)
    {
      ESP_LOGE("main", "disconnected");
      eraseEEPROM();
      WiFi.disconnect();
      client.disconnect(); //disconnect from mqtt
      for(int i =0; i <= 10; i++)
      {
        FastLED.showColor(CHSV(160, 255, 255));
        delay(200);
        FastLED.showColor(CHSV(0, 255, 255));
        delay(200);
      }
      return setupApi();
    }
  }

  if(String(topic) == configTopic && strstr(message, "set_scene") != NULL)
  {
    const size_t jsonSize = strlen(message) + 1; // Add 1 for null terminator
    char* json = new char[jsonSize];
    strncpy(json, message, jsonSize);

    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, json);
    if (error) {
      ESP_LOGE("main", "deserializeJson() failed: %s", error.c_str());
    }

    int enable = doc["objects"][0]["data"][0]["params"][0]["ruleconfig"][0]["enable"];
    ruleId = doc["objects"][0]["data"][0]["params"][0]["ruleconfig"][0]["ruleid"];
    JsonArray ruleconfig = doc["objects"][0]["data"][0]["params"][0]["ruleconfig"].as<JsonArray>();

    _ui_flag_modify(ui_scene1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
    _ui_flag_modify(ui_scene1, LV_OBJ_FLAG_CLICKABLE, _UI_MODIFY_FLAG_ADD);
    if(enable == 0) {
      _ui_state_modify(ui_scene1, LV_STATE_DISABLED, _UI_MODIFY_STATE_ADD);
    }
    else {
      _ui_state_modify(ui_scene1, LV_STATE_DISABLED, _UI_MODIFY_STATE_REMOVE);
    }
    advanceStatusCmd(bridgeKey, reqId, ruleconfig, output, macAddress);
    client.publish(configTopic, output);
  }
}

void uiTask(void *pvParameters)
{
    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
  Serial.begin(115200);

  gfx->begin(-1);
  gfx->fillScreen(BLACK);

  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, LED_NUM);
  FastLED.showColor(CHSV(160, 255, 255));

  lv_init();
  xTaskCreate(uiTask, "uiTask", 10000, NULL, 1, NULL);

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
    ESP_LOGE("main", "LVGL disp_draw_buf allocate failed!");
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

  init_lv_group();
  ui_init();
  outlineBorder(ui_button1);
  outlineBorder(ui_button2);
  outlineBorder(ui_button3);
  outlineBorder(ui_button4);
  outlineBorder(ui_resetWifi);
  outlineBorder(ui_sceneSetting);
  outlineBorder(ui_back);
  setupAP();
  setupApi();
  WiFi.macAddress(mac);
  sprintf(macAddress, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if(!MDNS.begin("esp32")) {
    ESP_LOGE("main", "error starring mDNS!");
    return;
  }
  
  mDNSService();
  net.setInsecure();
  net.setCACert(local_root_ca);
  client.setKeepAlive(60);
  client.setBufferSize(4096);
  client.setServer(iphc, PORT);
  client.setCallback(Callback);
}

void loop() {
  long now = millis();
  handleAP();
  if(!client.connected())
  {
    client.setKeepAlive(60); // setting keep alive to 60 seconds
    if(accessPointMode == false)
    {
      FastLED.showColor(CHSV(128, 255, 255)); //led aqua when disconnected mqtt
      connectBroker();
      checkIpHCmDNS();
    }
  }
  else{
    client.loop();
  }

  if(now - lastTime > 600000) // Send status periodically after 10 minutes
  {
    responseGetStatus(bridgeKey, reqId, output, macAddress);
    client.publish(statusTopic, output);
    lastTime = now;
  }
  vTaskDelay(pdMS_TO_TICKS(100));
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
  // MDNS.addService(SERVICE_NAME, SERVICE_PROTOCOL, SERVICE_PORT);
  // MDNS.addServiceTxt(SERVICE_NAME, SERVICE_PROTOCOL, "manufacturer", "LUMI");
  // MDNS.addServiceTxt(SERVICE_NAME, SERVICE_PROTOCOL, "mac", "f4:12:fa:cf:4e:b4");
  
  int nrOfServices = MDNS.queryService("lumismarthome", SERVICE_PROTOCOL);
  if (nrOfServices == 0) {
    ESP_LOGE("main", "No services were found.");
  } 
  else {
    ESP_LOGE("main", "Number of services found: %d", nrOfServices);
    for (int i = 0; i < nrOfServices; i=i+1) 
    {
      ESP_LOGE("main", " ");
      ESP_LOGE("main", "Hostname: %s", MDNS.hostname(i));
      ESP_LOGE("main", "IP address: %s", MDNS.IP(i).toString().c_str());
      ESP_LOGE("main", "Port: %d", MDNS.port(i));
      ESP_LOGE("main", "MAC: %s", MDNS.txt(i, "mac").c_str());
      ESP_LOGE("main", " ");
    }
  }
}

void checkIpHCmDNS()
{
  int nrOfServices = MDNS.queryService("lumismarthome", SERVICE_PROTOCOL);
  if (nrOfServices == 0) {
    return;
  } 
  else {
    ESP_LOGE("main", "Number of services found: %d", nrOfServices);
    for (int i = 0; i < nrOfServices; i++) 
    {
      String macString = MDNS.txt(i, "mac");
      for (int j = 0; j < 18; j++) {
        macString[j] = toupper(macString[j]);
      }
      const char* scanMac = macString.c_str();
      ESP_LOGE("main", "%s", scanMac);
      char scanIP[16];
      sprintf(scanIP, "%s", MDNS.IP(i).toString().c_str());
      ESP_LOGE("main", "%s", scanIP);
      if(strcmp(machc, scanMac) == 0)
      {
        saveHCInfoToEEPPROM(scanIP, machc);
        readHCInfoFromEEPROM(iphc, machc);
        client.setServer(iphc, PORT);
        ESP_LOGE("main", "change iphc");
        return connectBroker();
      }
    }
  }
}

void ui_event_button(lv_event_t *e, ButtonStatus& btn_status, const char *ep) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  
  if (event_code == LV_EVENT_VALUE_CHANGED) {
    if (lv_obj_has_state(target, LV_STATE_CHECKED)) {
      if (btn_status == OFF) {
        //client.publish(sw_topic, message1);
        generateJsonCmdStatus(bridgeKey, reqId, output, macAddress, ep, true);
        client.publish(statusTopic, output);
        btn_status = ON;
      }
    } else {
      if (btn_status == ON) {
        //client.publish(sw_topic, message2);
        generateJsonCmdStatus(bridgeKey, reqId, output, macAddress, ep, false);
        client.publish(statusTopic, output);
        btn_status = OFF;
      }
    }
  }
}

void ui_event_scene(lv_event_t *e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if(event_code == LV_EVENT_CLICKED)
  {
      activeRuleCmd(reqId, output, ruleId);
      client.publish(controlTopic, output);
  }
}

void ui_event_scene1(lv_event_t *e)
{
  ui_event_scene(e);
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

void spiffsInit()
{
  if (!SPIFFS.begin(true)) {
    ESP_LOGE("main", "An Error has occurred while mounting SPIFFS");
    return;
  }

  if (SPIFFS.exists("/data.txt")) {
    ESP_LOGE("main", "File already exists");
  }

  File fileToAppend = SPIFFS.open("/data.txt", FILE_APPEND);

  if (!fileToAppend) {
    ESP_LOGE("main","There was an error opening the file for appending");
    return;
  }

  if (fileToAppend.println("json")) {
    ESP_LOGE("main","File content was appended");
  } else {
    ESP_LOGE("main","File append failed");
  }

  fileToAppend.close();

  File fileToRead = SPIFFS.open("/data.txt", FILE_READ);

  if (!fileToRead) {
    ESP_LOGE("main", "There was an error opening the file for Reading");
    return;
  }

  String readFile = String(fileToRead.read());

  ESP_LOGE("main", "read file: %s", fileToRead.readString().c_str());
 
  fileToRead.close();
}

void addDeleteScene(char *message)
{
  
}