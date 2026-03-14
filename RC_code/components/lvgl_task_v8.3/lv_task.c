
#include <lvgl_task_v8.3/include/lv_task.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "./driver/gpio.h"
#include "lvgl.h"
#include "lv_demos.h"
//#include "lv_style_gen.h"
#define SPIFFS_PATH "/spiffs"
#include "lvgl_helpers.h"
#include "xpt2046.h"
#include "tp_spi.h"
#include "touch_driver.h"
#include "lv_examples.h"

#include <stdio.h>//兼容
#include <stdbool.h>//兼容
#include <unistd.h>//兼容

#include "Data_declaration.h"

#include "lvgl/lvgl.h"
#include "lvgl/demos/lv_demos.h"

#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>

//#include "lv_font.h"


lv_indev_t * indev_touchpad;

void LVGL_ALL_driver_init(void);

/*用定时器给LVGL提供时钟*/
static void lv_tick_task(void *arg)
{
	(void)arg;
	lv_tick_inc(10);
	lv_task_handler();
}

char buf[20]; //文本更换的中间变量

lv_obj_t * thrust_bar = NULL;
lv_obj_t * yaw_bar = NULL;
lv_obj_t * roll_bar = NULL;
lv_obj_t * pitch_bar = NULL;

lv_obj_t* thrust_val = NULL;
lv_obj_t* yaw_val = NULL;
lv_obj_t* roll_val = NULL;
lv_obj_t* pitch_val = NULL;
lv_obj_t* rc_vbat = NULL;

lv_obj_t* uav_vbat = NULL;
lv_obj_t* uav_electricity = NULL;
lv_obj_t* uav_alt = NULL;
lv_obj_t* uav_roll = NULL;
lv_obj_t* uav_pitch = NULL;
lv_obj_t* uav_yaw = NULL;
lv_obj_t* uav_lock = NULL;
lv_obj_t* uav_mode = NULL;

int _temp_page_num = 1;
int page_num = 1;
lv_obj_t* home_page = NULL;
lv_obj_t* set_page = NULL;
lv_obj_t* calibration_page = NULL;

bool _temp = false;
bool tr1 = 1;
lv_obj_t * img = NULL;
lv_obj_t * img2 = NULL;
lv_obj_t *pose = NULL;
lv_obj_t *pose2 = NULL;
lv_obj_t *status_bar = NULL;

lv_draw_img_dsc_t img_dsc;
lv_obj_t* canvas = NULL;

lv_obj_t* _temp5 = NULL;

bool button1 = false;

lv_obj_t * gyro_button = NULL;
lv_obj_t * acc_button = NULL;
lv_obj_t * barom_button = NULL;


LV_FONT_DECLARE(Chinese_character_library);//声明中文字库
LV_IMG_DECLARE(uav1); //声明姿态仪图片
LV_IMG_DECLARE(ns); //声明图片
LV_FONT_DECLARE(lv_font_montserrat_14);

enum {
    LV_MENU_ITEM_BUILDER_VARIANT_1,
    LV_MENU_ITEM_BUILDER_VARIANT_2
};
typedef uint8_t lv_menu_builder_variant_t;

static void back_event_handler(lv_event_t * e);
static void switch_handler(lv_event_t * e);
lv_obj_t * root_page;
static lv_obj_t * create_text(lv_obj_t * parent, const char * icon, const char * txt,
                              lv_menu_builder_variant_t builder_variant);
static lv_obj_t * create_slider(lv_obj_t * parent,
                                const char * icon, const char * txt, int32_t min, int32_t max, int32_t val);
static lv_obj_t * create_switch(lv_obj_t * parent,
                                const char * icon, const char * txt, bool chk);

void lv_example_menu_5(void)
{
    lv_obj_t * menu = lv_menu_create(set_page);

	 static lv_style_t style_font ;
	 lv_style_init(&style_font);

	 lv_obj_set_style_text_font(menu, &lv_font_montserrat_14, 0);

    lv_color_t bg_color = lv_obj_get_style_bg_color(menu, 0);
    if(lv_color_brightness(bg_color) > 127) {
        lv_obj_set_style_bg_color(menu, lv_color_darken(lv_obj_get_style_bg_color(menu, 0), 10), 0);
    }
    else {
        lv_obj_set_style_bg_color(menu, lv_color_darken(lv_obj_get_style_bg_color(menu, 0), 50), 0);
    }
    lv_menu_set_mode_root_back_btn(menu, LV_MENU_ROOT_BACK_BTN_ENABLED);
    lv_obj_add_event_cb(menu, back_event_handler, LV_EVENT_CLICKED, menu);
    lv_obj_set_size(menu, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_center(menu);

    lv_obj_t * cont;
    lv_obj_t * section;

    /*Create sub pages*/
    lv_obj_t * sub_mechanics_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_mechanics_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_mechanics_page);
    section = lv_menu_section_create(sub_mechanics_page);
    create_slider(section, LV_SYMBOL_SETTINGS, "Velocity", 0, 150, 120);
    create_slider(section, LV_SYMBOL_SETTINGS, "Acceleration", 0, 150, 50);
    create_slider(section, LV_SYMBOL_SETTINGS, "Weight limit", 0, 150, 80);

    lv_obj_t * sub_sound_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_sound_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_sound_page);
    section = lv_menu_section_create(sub_sound_page);
    create_switch(section, LV_SYMBOL_AUDIO, "Sound", false);

    lv_obj_t * sub_display_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_display_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_display_page);
    section = lv_menu_section_create(sub_display_page);
    create_slider(section, LV_SYMBOL_SETTINGS, "Brightness", 0, 150, 100);

    lv_obj_t * sub_software_info_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_software_info_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    section = lv_menu_section_create(sub_software_info_page);
    create_text(section, NULL, "Version 1.0", LV_MENU_ITEM_BUILDER_VARIANT_1);

    lv_obj_t * sub_legal_info_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_legal_info_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    section = lv_menu_section_create(sub_legal_info_page);
    for(uint32_t i = 0; i < 15; i++) {
        create_text(section, NULL,
                    "This is a long long long long long long long long long text, if it is long enough it may scroll.",
                    LV_MENU_ITEM_BUILDER_VARIANT_1);
    }

    lv_obj_t * sub_about_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_about_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_about_page);
    section = lv_menu_section_create(sub_about_page);
    cont = create_text(section, NULL, "Software information", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_software_info_page);
    cont = create_text(section, NULL, "Legal information", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_legal_info_page);

    lv_obj_t * sub_menu_mode_page = lv_menu_page_create(menu, NULL);
    lv_obj_set_style_pad_hor(sub_menu_mode_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    lv_menu_separator_create(sub_menu_mode_page);
    section = lv_menu_section_create(sub_menu_mode_page);
    cont = create_switch(section, LV_SYMBOL_AUDIO, "Sidebar enable", true);
    lv_obj_add_event_cb(lv_obj_get_child(cont, 2), switch_handler, LV_EVENT_VALUE_CHANGED, menu);

    /*Create a root page*/
    root_page = lv_menu_page_create(menu, "Settings");
    lv_obj_set_style_pad_hor(root_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    section = lv_menu_section_create(root_page);
    cont = create_text(section, LV_SYMBOL_SETTINGS, "Mechanics", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_mechanics_page);
    cont = create_text(section, LV_SYMBOL_AUDIO, "Sound", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_sound_page);
    cont = create_text(section, LV_SYMBOL_SETTINGS, "Display", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_display_page);

    create_text(root_page, NULL, "Others", LV_MENU_ITEM_BUILDER_VARIANT_1);
    section = lv_menu_section_create(root_page);
    cont = create_text(section, NULL, "About", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_about_page);
    cont = create_text(section, LV_SYMBOL_SETTINGS, "Menu mode", LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_menu_mode_page);

    lv_menu_set_sidebar_page(menu, root_page);

    lv_event_send(lv_obj_get_child(lv_obj_get_child(lv_menu_get_cur_sidebar_page(menu), 0), 0), LV_EVENT_CLICKED, NULL);
}

static void back_event_handler(lv_event_t * e)
{
    lv_obj_t * obj = lv_event_get_target(e);
    lv_obj_t * menu = lv_event_get_user_data(e);

    if(lv_menu_back_btn_is_root(menu, obj)) {
        lv_obj_t * mbox1 = lv_msgbox_create(NULL, "Hello", "Root back btn click.", NULL, true);
        lv_obj_center(mbox1);
    }
}

static void switch_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * menu = lv_event_get_user_data(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        if(lv_obj_has_state(obj, LV_STATE_CHECKED)) {
            lv_menu_set_page(menu, NULL);
            lv_menu_set_sidebar_page(menu, root_page);
            lv_event_send(lv_obj_get_child(lv_obj_get_child(lv_menu_get_cur_sidebar_page(menu), 0), 0), LV_EVENT_CLICKED, NULL);
        }
        else {
            lv_menu_set_sidebar_page(menu, NULL);
            lv_menu_clear_history(menu); /* Clear history because we will be showing the root page later */
            lv_menu_set_page(menu, root_page);
        }
    }
}

static lv_obj_t * create_text(lv_obj_t * parent, const char * icon, const char * txt,
                              lv_menu_builder_variant_t builder_variant)
{
    lv_obj_t * obj = lv_menu_cont_create(parent);

    lv_obj_t * img = NULL;
    lv_obj_t * label = NULL;

    if(icon) {
        img = lv_img_create(obj);
        lv_img_set_src(img, icon);
    }

    if(txt) {
        label = lv_label_create(obj);
        lv_label_set_text(label, txt);
        lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_flex_grow(label, 1);
    }

    if(builder_variant == LV_MENU_ITEM_BUILDER_VARIANT_2 && icon && txt) {
        lv_obj_add_flag(img, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);
        lv_obj_swap(img, label);
    }

    return obj;
}

static lv_obj_t * create_slider(lv_obj_t * parent, const char * icon, const char * txt, int32_t min, int32_t max,
                                int32_t val)
{
    lv_obj_t * obj = create_text(parent, icon, txt, LV_MENU_ITEM_BUILDER_VARIANT_2);

    lv_obj_t * slider = lv_slider_create(obj);
    lv_obj_set_flex_grow(slider, 1);
    lv_slider_set_range(slider, min, max);
    lv_slider_set_value(slider, val, LV_ANIM_OFF);

    if(icon == NULL) {
        lv_obj_add_flag(slider, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);
    }

    return obj;
}

static lv_obj_t * create_switch(lv_obj_t * parent, const char * icon, const char * txt, bool chk)
{
    lv_obj_t * obj = create_text(parent, icon, txt, LV_MENU_ITEM_BUILDER_VARIANT_1);

    lv_obj_t * sw = lv_switch_create(obj);
    lv_obj_add_state(sw, chk ? LV_STATE_CHECKED : 0);

    return obj;
}

uint32_t FloatToU32(float dat)
{
	uint8_t buf[4];

	buf[0] = ((uint8_t*)&dat)[0];
	buf[1] = ((uint8_t*)&dat)[1];
	buf[2] = ((uint8_t*)&dat)[2];
	buf[3] = ((uint8_t*)&dat)[3];

	return (buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];
}





void lv_val_refresh_task(void *arg)
{
	TickType_t adp = xTaskGetTickCount();
	const TickType_t adg = 200;//这里的数是指ticks（时间片）的意思，等于1就是每个ticks中断都执行
	while(1){
		vTaskDelayUntil(&adp,adg);
		if(init_ok){

			if(state.isRCLocked == true){
				lv_obj_set_style_text_color(uav_lock, lv_color_hex(0xFFFFFF), 0);//设置颜色
			}else{
				lv_obj_set_style_text_color(uav_lock, lv_color_hex(0xFF00FF), 0);//设置颜色
			}
			vTaskDelay(pdMS_TO_TICKS(1));//延时

			sprintf(buf, "横滚 %0.2f °",state.attitude.roll);
			lv_label_set_text(uav_roll, buf);
			vTaskDelay(pdMS_TO_TICKS(1));//延时
			sprintf(buf, "俯仰 %0.2f °",state.attitude.pitch);
			lv_label_set_text(uav_pitch, buf);
			vTaskDelay(pdMS_TO_TICKS(1));//延时
			sprintf(buf, "航向 %0.2f °",state.attitude.yaw);
			lv_label_set_text(uav_yaw, buf);
			vTaskDelay(pdMS_TO_TICKS(1));//延时
			sprintf(buf, "高度 %0.2f CM ",alt/100.0f);
			lv_label_set_text(uav_alt, buf);
			vTaskDelay(pdMS_TO_TICKS(1));//延时
			sprintf(buf, "电压 %0.2f V ",(float)UAV_VBAT/100);
			lv_label_set_text(uav_vbat, buf);
			vTaskDelay(pdMS_TO_TICKS(1));//延时



			sprintf(buf, "电机 %s ",state.isRCLocked == true ? "解锁" : "锁定");
			lv_label_set_text(uav_lock, buf);
			vTaskDelay(pdMS_TO_TICKS(1));//延时

			if(state.airplane_mode == 1) sprintf(buf, "飞行模式 姿态");
			if(state.airplane_mode == 2) sprintf(buf, "飞行模式 定高");
			if(state.airplane_mode == 3) sprintf(buf, "飞行模式 定点");
			lv_label_set_text(uav_mode, buf);

			vTaskDelay(pdMS_TO_TICKS(1));//延时


			lv_img_set_angle(img, state.attitude.roll*10.59f); //刷新横滚角到图片上
			vTaskDelay(pdMS_TO_TICKS(1));//延时
			lv_obj_set_pos(img, 335, (state.attitude.pitch*1.667f)+15);    //利用图片位置做俯仰角
			vTaskDelay(pdMS_TO_TICKS(1));//延时
			lv_img_set_pivot(img, 151, 474-(state.attitude.pitch*1.667f)); // 根据俯仰角的大小切换旋转中心点位置，确保图片在中央
			vTaskDelay(pdMS_TO_TICKS(1));//延时


			if(VBAT>=410){
				lv_obj_set_style_text_color(rc_vbat, lv_color_hex(0x00FF00), 0);//设置颜色
				lv_label_set_text(rc_vbat, LV_SYMBOL_BATTERY_FULL);		// 设置标签内容
			}else if(VBAT>=400){

				lv_obj_set_style_text_color(rc_vbat, lv_color_hex(0x00FF00), 0);//设置颜色
				lv_label_set_text(rc_vbat, LV_SYMBOL_BATTERY_3);		// 设置标签内容
			}else if(VBAT>=390){

				lv_obj_set_style_text_color(rc_vbat, lv_color_hex(0xFFFF00), 0);//设置颜色
				lv_label_set_text(rc_vbat, LV_SYMBOL_BATTERY_2);		// 设置标签内容
			}else if(VBAT>=380){

				lv_obj_set_style_text_color(rc_vbat, lv_color_hex(0xFF0000), 0);//设置颜色
				lv_label_set_text(rc_vbat, LV_SYMBOL_BATTERY_1);		// 设置标签内容
			}else if(VBAT>=370){

				lv_obj_set_style_text_color(rc_vbat, lv_color_hex(0xFF0000), 0);//设置颜色
				lv_label_set_text(rc_vbat, LV_SYMBOL_BATTERY_EMPTY);		// 设置标签内容
			}
			vTaskDelay(pdMS_TO_TICKS(1));//延时
			sprintf(buf, "%d",VBAT);
			lv_label_set_text(_temp5, buf);		// 设置标签内容
			vTaskDelay(pdMS_TO_TICKS(1));//延时
			printf("1\n");
			vTaskDelay(pdMS_TO_TICKS(1));//延时


		}
	}
}

void lv_page_refresh_task(void *arg)
{
	TickType_t adp = xTaskGetTickCount();
	const TickType_t adg = 10;//这里的数是指ticks（时间片）的意思，等于1就是每个ticks中断都执行
	while(1){

		vTaskDelayUntil(&adp,adg);
		if(init_ok)
		{
			lv_tick_inc(10);
			lv_task_handler();
			/*
			if(setpoint.AUX2 == 0 && _temp == true){
				page_num++;
				if(page_num>3) page_num = 1;
				_temp = false;
			}

			if(setpoint.AUX2 == 1) _temp = true;

			if(_temp_page_num != page_num){
				switch(page_num) {
			    	case 1:    // 主页面
			    		lv_disp_load_scr(home_page);
			    		break;
			    	case 2:    // 校准页面
			    		lv_disp_load_scr(calibration_page);
			    		break;
			    	case 3:    // 设置页面
			    		lv_disp_load_scr(set_page);
			    		break;
				}
				_temp_page_num = page_num;
			}
			vTaskDelay(pdMS_TO_TICKS(1));//延时
			*/
		}
	}
}

static lv_obj_t * spinbox;


static void lv_spinbox_increment_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_SHORT_CLICKED || code  == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(spinbox);
    }
}

static void lv_spinbox_decrement_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(spinbox);
    }
}


void lv_example_spinbox_1(void)
{
    spinbox = lv_spinbox_create(calibration_page);
    lv_spinbox_set_range(spinbox, -1000, 25000);
    lv_spinbox_set_digit_format(spinbox, 5, 2);
    lv_spinbox_step_prev(spinbox);
    lv_obj_set_width(spinbox, 100);
    lv_obj_center(spinbox);

    lv_coord_t h = lv_obj_get_height(spinbox);

    lv_obj_t * btn = lv_btn_create(calibration_page);
    lv_obj_set_size(btn, h, h);
    lv_obj_align_to(btn, spinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    lv_obj_set_style_bg_img_src(btn, LV_SYMBOL_PLUS, 0);
    lv_obj_add_event_cb(btn, lv_spinbox_increment_event_cb, LV_EVENT_ALL,  NULL);

    btn = lv_btn_create(calibration_page);
    lv_obj_set_size(btn, h, h);
    lv_obj_align_to(btn, spinbox, LV_ALIGN_OUT_LEFT_MID, -5, 0);
    lv_obj_set_style_bg_img_src(btn, LV_SYMBOL_MINUS, 0);
    lv_obj_add_event_cb(btn, lv_spinbox_decrement_event_cb, LV_EVENT_ALL, NULL);
}




static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);  // 获取当前部件(对象)触发的事件代码

    if(code == LV_EVENT_PRESSED) button1 = true;  // 按下

    if(code == LV_EVENT_RELEASED) button1 = false;// 松开

}


void LVGL_picture_establish(void){//创建画面






//**************************主页面**********************************//
	home_page = lv_obj_create(NULL);//创建主页面 lv_obj_create()

	//home_page = lv_scr_act();//创建主页面
	lv_obj_set_size(home_page, 320, 240);   //设置页面大小
	lv_obj_set_style_bg_color(home_page, lv_color_hex(0x808080), LV_STATE_DEFAULT); //设置页面默认颜色
	lv_scr_load(home_page); // 加载屏幕

	status_bar = lv_obj_create(home_page);//状态栏
	 lv_obj_set_style_text_font(status_bar, &lv_font_montserrat_14, 0);//状态栏主要显示图标，所以使用系统使用系统自带库
	lv_obj_set_style_bg_color(status_bar, lv_color_hex(0xC0C0C0), LV_STATE_DEFAULT);//设置页面默认颜色
	lv_obj_set_size(status_bar, 320, 25);    //设置大小
	lv_obj_set_pos(status_bar, 0, 0);    //设置位置

	rc_vbat = lv_label_create(status_bar);        	// 创建一个文本
	lv_obj_set_style_text_color(rc_vbat, lv_color_hex(0x00FF00), 0);//设置颜色
	lv_obj_set_pos(rc_vbat, 280, -10);			// 设置标签位置
	lv_label_set_text(rc_vbat, LV_SYMBOL_BATTERY_FULL);		// 设置标签内容

	_temp5 = lv_label_create(status_bar);        	// 创建一个文本
	lv_obj_set_pos(_temp5, 200, -10);			// 设置标签位置


	pose = lv_obj_create(home_page);//创建仪表显示画面 第一层 限制大小
	lv_obj_set_style_bg_color(pose, lv_color_hex(0xFF0000), LV_STATE_DEFAULT);//设置页面默认颜色
	lv_obj_set_size(pose, 150, 200);    //设置大小
	lv_obj_set_pos(pose, 150, 30);    //设置位置

	pose2 = lv_obj_create(pose);//创建仪表显示画面 第二层存放图片
	lv_obj_set_style_bg_color(pose2, lv_color_hex(0xF00000), LV_STATE_DEFAULT);//设置页面默认颜色
	lv_obj_set_size(pose2, 1000, 1000);    //设置大小
	lv_obj_center(pose2); //图片居中于画面

	img = lv_img_create(pose2);//创建图片
	lv_img_set_src(img, &uav1);//显示图片
	lv_obj_set_pos(img, 335, 15);    //设置位置
	lv_img_set_pivot(img, 151, 474); //设置图片旋转点

	uav_roll = lv_label_create(home_page);        	// 创建一个文本
	lv_obj_set_style_text_color(uav_roll, lv_color_hex(0x00FF00), 0);//设置颜色
	lv_obj_set_pos(uav_roll, 5, 25);			// 设置标签位置
	lv_label_set_text(uav_roll, "----");		// 设置标签内容

	uav_pitch = lv_label_create(home_page);        	// 创建一个文本
	lv_obj_set_style_text_color(uav_pitch, lv_color_hex(0xFF0000), 0);//设置颜色
	lv_obj_set_pos(uav_pitch, 5, 50);			// 设置标签位置
	lv_label_set_text(uav_pitch, "----");		// 设置标签内容

	uav_yaw = lv_label_create(home_page);        		// 创建一个文本
	lv_obj_set_style_text_color(uav_yaw, lv_color_hex(0x008000), 0);//设置颜色
	lv_obj_set_pos(uav_yaw, 5, 75);			// 设置标签位置
	lv_label_set_text(uav_yaw, "----");			// 设置标签内容

	uav_alt = lv_label_create(home_page);        		// 创建一个文本
	lv_obj_set_style_text_color(uav_alt, lv_color_hex(0xE87A00), 0);//设置颜色
	lv_obj_set_pos(uav_alt, 5, 100);			// 设置标签位置
	lv_label_set_text(uav_alt, "----");			// 设置标签内容

	uav_vbat = lv_label_create(home_page);        	// 创建一个文本
	lv_obj_set_style_text_color(uav_vbat, lv_color_hex(0x0000FF), 0);//设置颜色
	lv_obj_set_pos(uav_vbat, 5, 125);			// 设置标签位置
	lv_label_set_text(uav_vbat, "----");		// 设置标签内容

	uav_electricity = lv_label_create(home_page);        	// 创建一个文本
	lv_obj_set_style_text_color(uav_electricity, lv_color_hex(0xFFFF00), 0);//设置颜色
	lv_obj_set_pos(uav_electricity, 5, 150);			// 设置标签位置
	lv_label_set_text(uav_electricity, "----");		// 设置标签内容

	uav_lock = lv_label_create(home_page);        	// 创建一个文本
	lv_obj_set_style_text_color(uav_lock, lv_color_hex(0x00FFFF), 0);//设置颜色
	lv_obj_set_pos(uav_lock, 5, 175);			// 设置标签位置
	lv_label_set_text(uav_lock, "----");		// 设置标签内容

	uav_mode = lv_label_create(home_page);        	// 创建一个文本
	lv_obj_set_style_text_color(uav_mode, lv_color_hex(0x00FFFF), 0);//设置颜色
	lv_obj_set_pos(uav_mode, 5, 200);			// 设置标签位置
	lv_label_set_text(uav_mode, "----");		// 设置标签内容

	sprintf(buf, "横滚 %0.2f °",state.attitude.roll);
	lv_label_set_text(uav_roll, buf);
	sprintf(buf, "俯仰 %0.2f °",state.attitude.pitch);
	lv_label_set_text(uav_pitch, buf);
	sprintf(buf, "航向 %0.2f °",state.attitude.yaw);
	lv_label_set_text(uav_yaw, buf);
	sprintf(buf, "高度 %0.2f CM ",alt/100.0f);
	lv_label_set_text(uav_alt, buf);
	sprintf(buf, "电压 %0.2f V ",(float)UAV_VBAT/100);
	lv_label_set_text(uav_vbat, buf);

	sprintf(buf, "电流 00.0 A ");
	lv_label_set_text(uav_electricity, buf);

	sprintf(buf, "电机 %s ",state.isRCLocked == true ? "解锁" : "锁定");
	lv_label_set_text(uav_lock, buf);

	sprintf(buf, "飞行模式 姿态");
	lv_label_set_text(uav_mode, buf);
/*
// 按钮
    lv_obj_t * adjustment_btn1 = lv_btn_create(home_page);
    lv_obj_add_event_cb(adjustment_btn1, event_handler, LV_EVENT_ALL, NULL);
    //lv_obj_align(adjustment_btn1, LV_ALIGN_CENTER, 0, -40);
    lv_obj_set_size(adjustment_btn1, 100, 40);    //设置大小
    lv_obj_set_pos(adjustment_btn1, 5, 185);			// 设置标签位置

    lv_obj_t * label = lv_label_create(adjustment_btn1);
    lv_label_set_text(label, "校准页面");
    lv_obj_center(label);
*/

	//**************************校准页面**********************************//
/*
	calibration_page = lv_obj_create(NULL);        //创建设置页面
	lv_obj_set_size(calibration_page, 320, 240);   //设置页面大小


	lv_obj_t * label = NULL;

    gyro_button = lv_btn_create(calibration_page);
    lv_obj_add_event_cb(gyro_button, event_handler, LV_EVENT_ALL, NULL);
    //lv_obj_align(adjustment_btn1, LV_ALIGN_CENTER, 0, -40);
    lv_obj_set_size(gyro_button, 100, 40);    //设置大小
    lv_obj_set_pos(gyro_button, 5, 25);			// 设置标签位置

    label = lv_label_create(gyro_button);
    lv_label_set_text(label, "校准陀螺仪");
    lv_obj_center(label);

    acc_button = lv_btn_create(calibration_page);
    lv_obj_add_event_cb(acc_button, event_handler, LV_EVENT_ALL, NULL);
    //lv_obj_align(adjustment_btn1, LV_ALIGN_CENTER, 0, -40);
    lv_obj_set_size(acc_button, 100, 40);    //设置大小
    lv_obj_set_pos(acc_button, 5, 75);			// 设置标签位置

    label = lv_label_create(acc_button);
    lv_label_set_text(label, "校准加速度");
    lv_obj_center(label);

    barom_button = lv_btn_create(calibration_page);
    lv_obj_add_event_cb(barom_button, event_handler, LV_EVENT_ALL, NULL);
    //lv_obj_align(adjustment_btn1, LV_ALIGN_CENTER, 0, -40);
    lv_obj_set_size(barom_button, 100, 40);    //设置大小
    lv_obj_set_pos(barom_button, 5, 125);			// 设置标签位置

    label = lv_label_create(barom_button);
    lv_label_set_text(label, "校准气压计");
    lv_obj_center(label);
*/
    //lv_example_spinbox_1();
	//**************************设置页面**********************************//
	/*
	set_page = lv_obj_create(NULL);//创建设置页面
	lv_obj_set_size(set_page, 320, 240);   //设置页面大小

	lv_example_menu_5();

*/
}

void LVGL_ALL_driver_init(void){

	lv_init(); 										//LVGL画面初始化，系统自带
	lvgl_driver_init(); 							//LVGL驱动初始化

//显示屏初始化

	static lv_disp_draw_buf_t draw_buf_dsc_2;       //声明缓冲区结构体参数
	static lv_color_t buf_2_1[DISP_BUF_SIZE];    	//创建第一个10行的显示缓冲区
	static lv_color_t buf_2_2[DISP_BUF_SIZE];    	//创建第二个10行的显示缓冲区
	lv_disp_draw_buf_init(&draw_buf_dsc_2, buf_2_1, buf_2_2, DISP_BUF_SIZE);   //初始化显示缓冲区
	static lv_disp_drv_t disp_drv;             //声明显示驱动参数
	lv_disp_drv_init(&disp_drv);               //初始化显示参数（设置默认参数）

	disp_drv.hor_res = LV_VER_RES_MAX;      //设置横坐标显示分辨率
	disp_drv.ver_res = LV_HOR_RES_MAX;      //设置纵坐标显示分辨率
	disp_drv.flush_cb = disp_driver_flush;  //复制缓冲区内容到显示器（在此启用此功能）
	disp_drv.draw_buf = &draw_buf_dsc_2;    //设置显示缓冲区，刚刚是初始化现在是将缓冲区与显示驱动绑定

	lv_disp_drv_register(&disp_drv);		//注册显示驱动（设定参数）

//触摸屏初始化

	tp_spi_add_device(SPI3_HOST);           //使用SPI3接口通信
	touch_driver_init();                    //初始化触摸屏设备（根据你的cmd设定）

	static lv_indev_drv_t indev_drv;        //注册输入驱动参数结构体
	lv_indev_drv_init(&indev_drv);          //初始化结构体参数（设置为默认参数）
	indev_drv.type = LV_INDEV_TYPE_POINTER; //设置输入类型为触摸屏
	indev_drv.read_cb = xpt2046_read;       //输入回调函数为xpt2046_read
	indev_touchpad = lv_indev_drv_register(&indev_drv); //注册一个输入设备使用刚声明的结构体，并返回指针


	LVGL_picture_establish();//创建画面


	xTaskCreate(lv_page_refresh_task, "lv_page_refresh_task",1024*4,NULL,10,NULL);//创建LVGL处理任务
	xTaskCreate(lv_val_refresh_task, "lv_val_refresh_task",1024*8,NULL,5,NULL);//创建LVGL处理任务

/*
	//定时器初始化

		const esp_timer_create_args_t periodic_timer_args = { //定义参数结构体
		        .callback = &lv_tick_task,                        //设置回调函数
		        .name = "periodic_gui"};                          //设置定时器的名字

	esp_timer_handle_t periodic_timer;                    //声明定时器句柄
	ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));  //创建定时器
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 10 * 1000));      //启动定时器，每隔10*1000us就会中断调用一次lv_tick_task函数
*/






}

