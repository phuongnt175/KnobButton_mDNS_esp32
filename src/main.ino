/*
 * Project ZX2D10GE01R-V4848 for Arduino
 * Description: The base project that can be used in the Arduino environment using the ZX2D10GE01R-V4848 device
 * Author: Eric Nam
 * Date: 03-18-2023
 * Custom by PhuongNT.
 * Last Change: 21-07-2023
 */

// ** Prerequisites **
// ESP32 Arduino 2.0.7 based on ESP-IDF 4.4.4
// https://github.com/espressif/arduino-esp32

// LVGL version 8.3.5
// https://github.com/lvgl/lvgl

// GFX Library for Arduino 1.3.2
// https://github.com/moononournation/Arduino_GFX

// ZX2D10GE01R-V4848 for ESP-IDF
// https://github.com/wireless-tag-com/ZX2D10GE01R-V4848

/******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include "button.hpp"
#include "mt8901.hpp"
#include <ui.h>
#include <WiFiManager.h>
#include <ESPmDNS.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "esp_log.h"
#include <FastLED.h>

/******************************************************************************/
/*                     PRIVATE TYPES and DEFINITIONS                         */
/******************************************************************************/
#define ECO_O(y) (y > 0) ? -1 : 1
#define ECO_STEP(x) x ? ECO_O(x) : 0
#define GFX_BL 38

#define LED_PIN 4
#define LED_NUM 13
CRGB leds[LED_NUM];

#define SERVICE_NAME "lumismarthome"
#define SERVICE_PROTOCOL "tcp"
#define SERVICE_PORT 5600

#define ADDRESS "172.16.100.137"

/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/


/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
const char *MQTT_USER = "component"; // leave blank if no credentials used
const char *MQTT_PASS = " "; // leave blank if no credentials used
const char *sw1topic = "component/sw1";
const char *sw2topic = "component/sw2";
const char *sw3topic = "component/sw3";
const char *sw4topic = "component/sw4";

WiFiManager wm;
WiFiClientSecure  net;
PubSubClient      client(net);

int PORT = 38883;
bool res;
String btnStatus1 = "OFF";
String btnStatus2 = "OFF";
String btnStatus3 = "OFF";
String btnStatus4 = "OFF";

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
void DemandWifi();
void init_lv_group();

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
    String clientId = "esp32-s3-";
    clientId += String(random(0xffff), HEX);
    if(client.connect(clientId.c_str(), MQTT_USER, MQTT_PASS))
    {
      ESP_LOGE(TAG, "Connected");
      client.subscribe(sw1topic);
      client.subscribe(sw2topic);
      client.subscribe(sw3topic);
      client.subscribe(sw4topic);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again in 2 seconds");
      delay(200);
    }
  }
}

void SubCallback(lv_obj_t *ui, String Message, String btnStatus)
{
  if(Message == "ON")
  {
    btnStatus = "ON";
    if(lv_obj_get_state(ui) == 6 || lv_obj_get_state(ui) == 0)
    {
      _ui_state_modify(ui, LV_STATE_CHECKED, 2);// _UI_STATE_MODIFY_TOGGLE
    }
    ESP_LOGE(TAG, "-------------------------------------------------------------");
  }
  else if(Message == "OFF")
  {
    btnStatus = "OFF";
    _ui_state_modify(ui, LV_STATE_CHECKED, 1);// _UI_STATE_MODIFY_TOGGLE
    ESP_LOGE(TAG, "-------------------------------------------------------------");
  }
}

void Callback(char* topic, byte* payload, unsigned int length) {

  payload[length] = '\0'; //NULL terminator used to terminate the char array
  String message = (char*)payload;
  ESP_LOGE(TAG, "%s", message);
  if(String(topic) == sw1topic)
  {
    SubCallback(ui_button1, message, btnStatus1);
  }
  if(String(topic) == sw2topic)
  {
    SubCallback(ui_button2, message, btnStatus2);
  }
  if(String(topic) == sw3topic)
  {
    SubCallback(ui_button3, message, btnStatus3);
  }
  if(String(topic) == sw4topic)
  {
    SubCallback(ui_button4, message, btnStatus4);
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

  init_lv_group();
  ui_init();

  WiFi.mode(WIFI_STA);
  res = wm.autoConnect("KnobButton", "LumiVn@2023");

  ESP_LOGE(TAG, "%d.%d.%d.%d ", WiFi.localIP()&0xFF, (WiFi.localIP()>>8)&0xFF, (WiFi.localIP()>>16)&0xFF, (WiFi.localIP()>>24)&0xFF);

  if(!MDNS.begin("esp32")) {
    //Serial.println("Error starting mDNS");
    ESP_LOGE(TAG, "error starring mDNS!");
    return;
  }

  mDNSService();
  net.setInsecure();
  client.setKeepAlive(60);
  net.setCACert(local_root_ca);
  client.setServer(ADDRESS, PORT);
  client.setCallback(Callback);
  connectBroker();
}

void loop() {
  
  lv_timer_handler(); /* let the GUI do its work */
  if(!client.connected())
  {
    client.setKeepAlive(60); // setting keep alive to 60 seconds
    connectBroker();
  }
  else{
    client.loop();
  }
  delay(5);
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

void ui_event_resetWifi(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t * target = lv_event_get_target(e);
  if(event_code == LV_EVENT_RELEASED) //detect button released, run the DemandWiFi function
  {
    DemandWifi();
  }
}

void DemandWifi()
{
  wm.setConfigPortalTimeout(300); //set timeout 300s
  if (!wm.startConfigPortal("OnDemandAP","LumiVn@2023")) {
    delay(5000);
    MDNS.begin("esp32");
  }
}

void mDNSService()
{
  MDNS.addService(SERVICE_NAME, SERVICE_PROTOCOL, SERVICE_PORT);

  int nrOfServices = MDNS.queryService(SERVICE_NAME, SERVICE_PROTOCOL);
  
  if (nrOfServices == 0) {
    //Serial.println("No services were found.");
    ESP_LOGE(TAG, "No services were found.");
  } 
  else {
    
    //Serial.print("Number of services found: ");
    ESP_LOGE(TAG, "Number of services found: %d", nrOfServices);
      
    for (int i = 0; i < nrOfServices; i=i+1) 
    {
      //Serial.println("---------------");
      ESP_LOGE(TAG, "---------------");
        
      //Serial.print("Hostname: ");
      //Serial.println(MDNS.hostname(i));
      ESP_LOGE(TAG, "Hostname: %s", MDNS.hostname(i));

      //Serial.print("IP address: ");
      //Serial.println(MDNS.IP(i));
      ESP_LOGE(TAG, "IP address: %d.%d.%d.%d", MDNS.IP(i)&0xFF, (MDNS.IP(i)>>8)&0xFF, (MDNS.IP(i)>>16)&0xFF, (MDNS.IP(i)>>24)&0xFF);

      //Serial.print("Port: ");
      //Serial.println(MDNS.port(i));
      ESP_LOGE(TAG, "Port: %d", MDNS.port(i));

      //Serial.println("---------------");
      ESP_LOGE(TAG, "---------------");
    }
  }
}

void ui_event_button1(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t * target = lv_event_get_target(e);
  if(event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target, LV_STATE_CHECKED)) {
    if(btnStatus1 = "OFF")
    {
      client.publish(sw1topic, "ON");
      btnStatus1 = "ON";
      ESP_LOGE(TAG, "button 1 ON");
    }
  }
  if(event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target, LV_STATE_CHECKED)) {
    if(btnStatus1 == "ON")
    {
      client.publish(sw1topic, "OFF");
      btnStatus1 = "OFF";
      ESP_LOGE(TAG, "button 1 OFF");
    }
  }
}

void ui_event_button2(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t * target = lv_event_get_target(e);
  if(event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target, LV_STATE_CHECKED)) {
    if(btnStatus2 = "OFF")
    {
      client.publish(sw2topic, "ON");
      btnStatus2 = "ON";
      ESP_LOGE(TAG, "button 2 ON");
    }
  }
  if(event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target, LV_STATE_CHECKED)) {
    if(btnStatus2 == "ON")
    {
      client.publish(sw2topic, "OFF");
      btnStatus2 = "OFF";
      ESP_LOGE(TAG, "button 2 OFF");
    }
  }
}

void ui_event_button3(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t * target = lv_event_get_target(e);
  if(event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target, LV_STATE_CHECKED)) {
    if(btnStatus3 = "OFF")
    {
      client.publish(sw3topic, "ON");
      btnStatus3 = "ON";
      ESP_LOGE(TAG, "button 3 ON");
    }
  }
  if(event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target, LV_STATE_CHECKED)) {
    if(btnStatus3 == "ON")
    {
      client.publish(sw3topic, "OFF");
      btnStatus3 = "OFF";
      ESP_LOGE(TAG, "button 3 OFF");
    }
  }
}

void ui_event_button4(lv_event_t * e)
{
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t * target = lv_event_get_target(e);
  if(event_code == LV_EVENT_VALUE_CHANGED &&  lv_obj_has_state(target, LV_STATE_CHECKED)) {
    if(btnStatus4 = "OFF")
    {
      client.publish(sw4topic, "ON");
      btnStatus4 = "ON";
      ESP_LOGE(TAG, "button 4 ON");
    }
  }
  if(event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target, LV_STATE_CHECKED)) {
    if(btnStatus4 == "ON")
    {
      client.publish(sw4topic, "OFF");
      btnStatus4 = "OFF";
      ESP_LOGE(TAG, "button 4 OFF");
    }
  }
}