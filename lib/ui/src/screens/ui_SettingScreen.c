// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: knobCircle

#include "../ui.h"

void ui_SettingScreen_screen_init(void)
{
ui_SettingScreen = lv_obj_create(NULL);
lv_obj_clear_flag( ui_SettingScreen, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM );    /// Flags
lv_obj_set_scrollbar_mode(ui_SettingScreen, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_scroll_dir(ui_SettingScreen, LV_DIR_HOR);
lv_obj_set_style_bg_color(ui_SettingScreen, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_SettingScreen, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_panelScroll = lv_obj_create(ui_SettingScreen);
lv_obj_set_width( ui_panelScroll, 280);
lv_obj_set_height( ui_panelScroll, 480);
lv_obj_set_align( ui_panelScroll, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_panelScroll, LV_OBJ_FLAG_SCROLL_ON_FOCUS | LV_OBJ_FLAG_SCROLL_ONE | LV_OBJ_FLAG_SCROLL_CHAIN_VER);   /// Flags
lv_obj_clear_flag( ui_panelScroll, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM);    /// Flags
lv_obj_set_scrollbar_mode(ui_panelScroll, LV_SCROLLBAR_MODE_ON);
lv_obj_set_scroll_dir(ui_panelScroll, LV_DIR_VER);
lv_obj_set_style_bg_color(ui_panelScroll, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_panelScroll, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_color(ui_panelScroll, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_border_opa(ui_panelScroll, 255, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_scroll_snap_y(ui_panelScroll, LV_SCROLL_SNAP_CENTER);
lv_obj_set_scrollbar_mode(ui_panelScroll, LV_SCROLLBAR_MODE_OFF);

ui_scene1 = lv_btn_create(ui_panelScroll);
lv_obj_set_width( ui_scene1, 250);
lv_obj_set_height( ui_scene1, 250);
lv_obj_set_align( ui_scene1, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_scene1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);   /// Flags
lv_obj_clear_flag( ui_scene1, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_scrollbar_mode(ui_scene1, LV_SCROLLBAR_MODE_OFF);
lv_obj_set_scroll_dir(ui_scene1, LV_DIR_VER);
lv_obj_set_style_radius(ui_scene1, 250, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_scene1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_scene1, 50, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_scene2 = lv_btn_create(ui_panelScroll);
lv_obj_set_width( ui_scene2, 250);
lv_obj_set_height( ui_scene2, 250);
lv_obj_set_x( ui_scene2, 0 );
lv_obj_set_y( ui_scene2, 480 );
lv_obj_set_align( ui_scene2, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_scene2, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_scene2, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_scene2, 200, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_scene2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_scene2, 50, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_scene3 = lv_btn_create(ui_panelScroll);
lv_obj_set_width( ui_scene3, 250);
lv_obj_set_height( ui_scene3, 250);
lv_obj_set_x( ui_scene3, 0 );
lv_obj_set_y( ui_scene3, 960 );
lv_obj_set_align( ui_scene3, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_scene3, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_scene3, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_scene3, 200, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_scene3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_scene3, 50, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_scene4 = lv_btn_create(ui_panelScroll);
lv_obj_set_width( ui_scene4, 250);
lv_obj_set_height( ui_scene4, 250);
lv_obj_set_x( ui_scene4, 0 );
lv_obj_set_y( ui_scene4, 1440 );
lv_obj_set_align( ui_scene4, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_scene4, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_scene4, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_scene4, 200, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_scene4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_scene4, 50, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_scene5 = lv_btn_create(ui_panelScroll);
lv_obj_set_width( ui_scene5, 250);
lv_obj_set_height( ui_scene5, 250);
lv_obj_set_x( ui_scene5, 0 );
lv_obj_set_y( ui_scene5, 1920 );
lv_obj_set_align( ui_scene5, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_scene5, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_scene5, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_scene5, 200, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_scene5, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_scene5, 50, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_scene6 = lv_btn_create(ui_panelScroll);
lv_obj_set_width( ui_scene6, 250);
lv_obj_set_height( ui_scene6, 250);
lv_obj_set_x( ui_scene6, 0 );
lv_obj_set_y( ui_scene6, 5*480 );
lv_obj_set_align( ui_scene6, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_scene6, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_scene6, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_scene6, 200, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_scene6, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_scene6, 50, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_scene7 = lv_btn_create(ui_panelScroll);
lv_obj_set_width( ui_scene7, 250);
lv_obj_set_height( ui_scene7, 250);
lv_obj_set_x( ui_scene7, 0 );
lv_obj_set_y( ui_scene7, 6*480 );
lv_obj_set_align( ui_scene7, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_scene7, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_scene7, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_scene7, 200, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_scene7, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_scene7, 50, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_scene8 = lv_btn_create(ui_panelScroll);
lv_obj_set_width( ui_scene8, 250);
lv_obj_set_height( ui_scene8, 250);
lv_obj_set_x( ui_scene8, 0 );
lv_obj_set_y( ui_scene8, 7*480 );
lv_obj_set_align( ui_scene8, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_scene8, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_scene8, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_scene8, 200, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_scene8, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_scene8, 50, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_scene9 = lv_btn_create(ui_panelScroll);
lv_obj_set_width( ui_scene9, 250);
lv_obj_set_height( ui_scene9, 250);
lv_obj_set_x( ui_scene9, 0 );
lv_obj_set_y( ui_scene9, 8*480 );
lv_obj_set_align( ui_scene9, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_scene9, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_scene9, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_scene9, 200, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_scene9, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_scene9, 50, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_scene10 = lv_btn_create(ui_panelScroll);
lv_obj_set_width( ui_scene10, 250);
lv_obj_set_height( ui_scene10, 250);
lv_obj_set_x( ui_scene10, 0 );
lv_obj_set_y( ui_scene10, 9*480 );
lv_obj_set_align( ui_scene10, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_scene10, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_scene10, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_scene10, 200, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_scene10, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_scene10, 50, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_back = lv_btn_create(ui_panelScroll);
lv_obj_set_width( ui_back, 50);
lv_obj_set_height( ui_back, 50);
lv_obj_set_x( ui_back, 0 );
lv_obj_set_y( ui_back, 10*480 );
lv_obj_set_align( ui_back, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_back, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_back, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_radius(ui_back, 100, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_back, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_back, 50, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_img_src( ui_back, &ui_img_1592976543, LV_PART_MAIN | LV_STATE_DEFAULT );

ui_canh1 = lv_label_create(ui_scene1);
lv_obj_set_width( ui_canh1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_canh1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_canh1, 0 );
lv_obj_set_y( ui_canh1, 60 );
lv_obj_set_align( ui_canh1, LV_ALIGN_CENTER );
lv_label_set_text(ui_canh1,"Canh 1");
lv_obj_clear_flag( ui_canh1, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_canh1, &vn, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_canh2 = lv_label_create(ui_scene2);
lv_obj_set_width( ui_canh2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_canh2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_canh2, 0 );
lv_obj_set_y( ui_canh2, 60 );
lv_obj_set_align( ui_canh2, LV_ALIGN_CENTER );
lv_label_set_text(ui_canh2,"Canh 2");
lv_obj_clear_flag( ui_canh2, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_canh2, &vn, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_canh3 = lv_label_create(ui_scene3);
lv_obj_set_width( ui_canh3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_canh3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_canh3, 0 );
lv_obj_set_y( ui_canh3, 60 );
lv_obj_set_align( ui_canh3, LV_ALIGN_CENTER );
lv_label_set_text(ui_canh3,"Canh 3");
lv_obj_clear_flag( ui_canh3, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_canh3, &vn, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_canh4 = lv_label_create(ui_scene4);
lv_obj_set_width( ui_canh4, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_canh4, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_canh4, 0 );
lv_obj_set_y( ui_canh4, 60 );
lv_obj_set_align( ui_canh4, LV_ALIGN_CENTER );
lv_label_set_text(ui_canh4,"Canh 4");
lv_obj_clear_flag( ui_canh4, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_canh4, &vn, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_canh5 = lv_label_create(ui_scene5);
lv_obj_set_width( ui_canh5, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_canh5, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_canh5, 0 );
lv_obj_set_y( ui_canh5, 60 );
lv_obj_set_align( ui_canh5, LV_ALIGN_CENTER );
lv_label_set_text(ui_canh5,"Canh 5");
lv_obj_clear_flag( ui_canh5, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_canh5, &vn, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_canh6 = lv_label_create(ui_scene6);
lv_obj_set_width( ui_canh6, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_canh6, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_canh6, 0 );
lv_obj_set_y( ui_canh6, 60 );
lv_obj_set_align( ui_canh6, LV_ALIGN_CENTER );
lv_label_set_text(ui_canh6,"Canh 6");
lv_obj_clear_flag( ui_canh6, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_canh6, &vn, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_canh7 = lv_label_create(ui_scene7);
lv_obj_set_width( ui_canh7, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_canh7, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_canh7, 0 );
lv_obj_set_y( ui_canh7, 60 );
lv_obj_set_align( ui_canh7, LV_ALIGN_CENTER );
lv_label_set_text(ui_canh7,"Canh 7");
lv_obj_clear_flag( ui_canh7, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_canh7, &vn, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_canh8 = lv_label_create(ui_scene8);
lv_obj_set_width( ui_canh8, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_canh8, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_canh8, 0 );
lv_obj_set_y( ui_canh8, 60 );
lv_obj_set_align( ui_canh8, LV_ALIGN_CENTER );
lv_label_set_text(ui_canh8,"Canh 8");
lv_obj_clear_flag( ui_canh8, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_canh8, &vn, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_canh9 = lv_label_create(ui_scene9);
lv_obj_set_width( ui_canh9, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_canh9, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_canh9, 0 );
lv_obj_set_y( ui_canh9, 60 );
lv_obj_set_align( ui_canh9, LV_ALIGN_CENTER );
lv_label_set_text(ui_canh9,"Canh 9");
lv_obj_clear_flag( ui_canh9, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_canh9, &vn, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_canh10 = lv_label_create(ui_scene10);
lv_obj_set_width( ui_canh10, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_canh10, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_canh10, 0 );
lv_obj_set_y( ui_canh10, 60 );
lv_obj_set_align( ui_canh10, LV_ALIGN_CENTER );
lv_label_set_text(ui_canh10,"Canh 10");
lv_obj_clear_flag( ui_canh10, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_CLICK_FOCUSABLE | LV_OBJ_FLAG_GESTURE_BUBBLE | LV_OBJ_FLAG_SNAPPABLE | LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN );    /// Flags
lv_obj_set_style_text_font(ui_canh10, &vn, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_add_event_cb(ui_back, ui_event_back, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_scene1, ui_event_scene1, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_scene2, ui_event_scene2, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_scene3, ui_event_scene3, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_scene4, ui_event_scene4, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_scene5, ui_event_scene5, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_scene6, ui_event_scene6, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_scene7, ui_event_scene7, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_scene8, ui_event_scene8, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_scene9, ui_event_scene9, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_scene10, ui_event_scene10, LV_EVENT_ALL, NULL);
}
