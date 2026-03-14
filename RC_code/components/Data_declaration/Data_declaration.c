/*
 * Data declaration.c
 *  数据声明，结构体，堆栈空间，飞行器姿态和PID各类数据在此声明
 *  Created on: 2023年1月8日
 *      Author: liguanxi
 */

#include "Data_declaration.h"

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_system.h"
#include "esp_log.h"
#include "string.h"
#include "math.h"


#include <sys/time.h>

#include <inttypes.h>
struct timeval tv_now;
int64_t time1_us = 0;
int64_t time_us = 0;

bool init_ok = false;
uint32_t VBAT;
uint32_t UAV_VBAT;
float alt;
int control_mode = 0;


void printf_time1_us(uint8_t set){//发送系统时间

	if(set == 0){
		gettimeofday(&tv_now, NULL);
		time1_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
		//printf("%" PRId64 "\n", time1_us);
	}else{
		gettimeofday(&tv_now, NULL);
		time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
		time_us = time_us-time1_us;
		printf("%" PRId64 "       \n", time_us);
	}

}
