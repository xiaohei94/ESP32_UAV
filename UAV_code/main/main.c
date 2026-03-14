/*
 * liguanxi-UAV
 * 飞行器项目
 * 芯片ESP32S3
 * 创建时间2022.11.13
 */
#include "control.h"
#include "UART.h"
#include "MPU6050.h"
#include "IMU.h"
#include "anotc.h"
#include "LED.h"
#include "driver/gpio.h"
#include "WIFI.h"
#include "UDP_TCP.h"
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "anotc.h"
#include <sys/time.h>
#include <inttypes.h>
#include "Data_declaration.h"
#include "remote_control.h"
#include "PWM.h"
#include "VBAT.h"
#include "SPI.h"
#include "SPL06-001.h"
#include "HMC5883L.h"
#include "GL9306.h"
#include "VL53L1X.h"


void app_main(void)//入口函数一开始就执行一次
{
	printf("最后修改日期 2024.1.20 \n");
	UART_init();//串口通始化
	IIC_init();//IIC初始化
	PWM_init();//PWM初始化

	mpu6050_Init();//MPU6050姿态传感器初始化
	SPL06Init(); //气压计初始化
	HMC5883L_init();//磁力计初始化
	VL53L1X_init(); //激光位移传感器初始化
	GL9306_init(); //光流初始化
	VBAT_init();//电池电压检测初始化

	LED_init();//LED灯初始化

	wifi_init_ap();//初始化WiFi
	UDP_init();//UDP通信初始化
	anotc_Init();//匿名上位机初始化
	remote_control_init();//遥控器初始化

	control_init();//姿态控制初始化

	printf("剩余空闲堆栈: %d 字节\n", xPortGetFreeHeapSize());	//打印剩余堆栈大小

	init_ok = true;//初始化完成置1让所有任务开始运行


}
