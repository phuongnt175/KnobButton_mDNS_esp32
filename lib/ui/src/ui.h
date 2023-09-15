// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.1
// LVGL version: 8.3.6
// Project name: knobCircle

#ifndef _KNOBCIRCLE_UI_H
#define _KNOBCIRCLE_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
#if __has_include("lvgl.h")
#include "lvgl.h"
#elif __has_include("lvgl/lvgl.h")
#include "lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif
#else
#include "lvgl.h"
#endif

#include "ui_helpers.h"
#include "ui_events.h"
// SCREEN: ui_lumiScreen
void ui_lumiScreen_screen_init(void);
void ui_event_lumiScreen(lv_event_t * e);
extern lv_obj_t * ui_lumiScreen;
extern lv_obj_t * ui_Image1;
// SCREEN: ui_mainScreen
void ui_mainScreen_screen_init(void);
void ui_event_button1(lv_event_t * e);
void ui_event_button2(lv_event_t * e);
void ui_event_button3(lv_event_t * e);
void ui_event_button4(lv_event_t * e);
extern lv_obj_t * ui_mainScreen;
extern lv_obj_t * ui_button1;
extern lv_obj_t * ui_button2;
extern lv_obj_t * ui_button3;
extern lv_obj_t * ui_button4;
void ui_event_resetWifi(lv_event_t * e);
extern lv_obj_t * ui_resetWifi;
void ui_event_sceneSetting(lv_event_t * e);
extern lv_obj_t * ui_sceneSetting;

// SCREEN: ui_SettingScreen
void ui_SettingScreen_screen_init(void);
extern lv_obj_t * ui_SettingScreen;
extern lv_obj_t * ui_scene1;
extern lv_obj_t * ui_canh1;
extern lv_obj_t * ui_scene2;
extern lv_obj_t * ui_canh2;
extern lv_obj_t * ui_scene3;
extern lv_obj_t * ui_canh3;
extern lv_obj_t * ui_scene4;
extern lv_obj_t * ui_canh4;
extern lv_obj_t * ui_scene5;
extern lv_obj_t * ui_canh5;
extern lv_obj_t * ui_panelScroll;
void ui_event_back(lv_event_t * e);
void ui_event_scene1(lv_event_t * e);
void ui_event_scene2(lv_event_t * e);
void ui_event_scene3(lv_event_t * e);
void ui_event_scene4(lv_event_t * e);
void ui_event_scene5(lv_event_t * e);
extern lv_obj_t * ui_back;

extern lv_obj_t * ui____initial_actions0;

void outlineBorder(lv_obj_t *ui);

LV_IMG_DECLARE(ui_img_lumi_png);    // assets\lumi.png
LV_IMG_DECLARE(ui_img_472831459);    // assets\icon_scenes_leave-home (1).png
LV_IMG_DECLARE(ui_img_932150510);    // assets\light-bulb (1).png
LV_IMG_DECLARE(ui_img_554538158);    // assets\curtains (2).png
LV_IMG_DECLARE(ui_img_ac_white_png);    // assets\AC_white.png
LV_IMG_DECLARE(ui_img_1968931049);    // assets\wifi (1).png
LV_IMG_DECLARE(ui_img_1772094659);    // assets\settings (1).png
LV_IMG_DECLARE(ui_img_1592976543);    // assets\left-arrow.png
LV_IMG_DECLARE(ui_img_1704086984);
LV_IMG_DECLARE( ui_img_0_png);   // assets\0.png
LV_IMG_DECLARE( ui_img_1_png);   // assets\1.png
LV_IMG_DECLARE( ui_img_10_png);   // assets\10.png
LV_IMG_DECLARE( ui_img_11_png);   // assets\11.png
LV_IMG_DECLARE( ui_img_12_png);   // assets\12.png
LV_IMG_DECLARE( ui_img_13_png);   // assets\13.png
LV_IMG_DECLARE( ui_img_14_png);   // assets\14.png
LV_IMG_DECLARE( ui_img_15_png);   // assets\15.png
LV_IMG_DECLARE( ui_img_16_png);   // assets\16.png
LV_IMG_DECLARE( ui_img_17_png);   // assets\17.png
LV_IMG_DECLARE( ui_img_18_png);   // assets\18.png
LV_IMG_DECLARE( ui_img_19_png);   // assets\19.png
LV_IMG_DECLARE( ui_img_2_png);   // assets\2.png
LV_IMG_DECLARE( ui_img_20_png);   // assets\20.png
LV_IMG_DECLARE( ui_img_21_png);   // assets\21.png
LV_IMG_DECLARE( ui_img_22_png);   // assets\22.png
LV_IMG_DECLARE( ui_img_23_png);   // assets\23.png
LV_IMG_DECLARE( ui_img_24_png);   // assets\24.png
LV_IMG_DECLARE( ui_img_25_png);   // assets\25.png
LV_IMG_DECLARE( ui_img_26_png);   // assets\26.png
LV_IMG_DECLARE( ui_img_27_png);   // assets\27.png
LV_IMG_DECLARE( ui_img_28_png);   // assets\28.png
LV_IMG_DECLARE( ui_img_29_png);   // assets\29.png
LV_IMG_DECLARE( ui_img_3_png);   // assets\3.png
LV_IMG_DECLARE( ui_img_30_png);   // assets\30.png
LV_IMG_DECLARE( ui_img_31_png);   // assets\31.png
LV_IMG_DECLARE( ui_img_32_png);   // assets\32.png
LV_IMG_DECLARE( ui_img_33_png);   // assets\33.png
LV_IMG_DECLARE( ui_img_34_png);   // assets\34.png
LV_IMG_DECLARE( ui_img_35_png);   // assets\35.png
LV_IMG_DECLARE( ui_img_36_png);   // assets\36.png
LV_IMG_DECLARE( ui_img_37_png);   // assets\37.png
LV_IMG_DECLARE( ui_img_38_png);   // assets\38.png
LV_IMG_DECLARE( ui_img_4_png);   // assets\4.png
LV_IMG_DECLARE( ui_img_5_png);   // assets\5.png
LV_IMG_DECLARE( ui_img_6_png);   // assets\6.png
LV_IMG_DECLARE( ui_img_7_png);   // assets\7.png
LV_IMG_DECLARE( ui_img_8_png);   // assets\8.png
LV_IMG_DECLARE( ui_img_9_png);   // assets\9.png
LV_IMG_DECLARE( ui_img_minus1_png);   // assets\minus1.png

void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
