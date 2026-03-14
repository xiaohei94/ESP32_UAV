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

bool init_ok = false;//初始化成功后置1全部任务才会开始运行
bool get_VL53L1X = false; //为1时激光位移传感器获取一次数据
bool LOCK_P = false; //解锁上升沿用于重置各种值
bool iic_lock = false;
int64_t task_run_time_us[10] = {0};  //系统运行时间
int roll_trim;  //横滚角遥控数值补偿
int pitch_trim; //俯仰角遥控数值补偿
int yaw_trim;   //航向角遥控数值补偿
int Acceleration_calibration;
int gyroscope_calibration;
int AirPressure_calibration;
int isRCLocked_p = 0;
uint32_t VBAT;  //电压
float NormAccz = 0;
int control_mode = 0;

