/*
 * HMC5883L.c
 *  磁力计
 *  Created on: 2023年1月8日
 *      Author: liguanxi
 */
#include "HMC5883L.h"

#include "UART.h"
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "Data_declaration.h"

#include "anotc.h"

#define HMC5883L_SENSOR_ADDR                 0x1E        //MPU6050传感器地址

INT16_XYZ MAG_OFFSET_RAW; 		//磁力计零偏校准值

int HMC5883L_test(void){ //mpu6050器件检测
	uint8_t test[1] = {0};//接收数据缓冲区
	//读取0x75寄存器
	if(iic_read(I2C_MASTER_NUM,HMC5883L_SENSOR_ADDR,0x0C,test,1) == ESP_OK) printf("寄存器读取成功\n");
	//识别寄存器C的值与传感器地址对比一致则是成功连接，反之就是未连接
	if(test[0] == 51) {
		printf("HMC5883L传感器连接 成功\n");
		return ESP_OK;
	}else{
		printf("HMC5883L传感器连接 失败\n");
		return ESP_FAIL;
	}

}

void HMC5883L_init(void){

	printf("********************HMC5883L初始化开始********************\n");
		if(HMC5883L_test() == ESP_OK){//检测传感器成功连接就进行配置

			if(iic_write(I2C_MASTER_NUM,HMC5883L_SENSOR_ADDR,0x00,(3<<5)|(1<<4)|(0)) == ESP_OK) printf("1.8倍平均值滤波，默认15hz，正常测试\n");
			if(iic_write(I2C_MASTER_NUM,HMC5883L_SENSOR_ADDR,0x01,(1<<5)|(0)) == ESP_OK) printf("2.量程设置±1.3Ga\n");
			if(iic_write(I2C_MASTER_NUM,HMC5883L_SENSOR_ADDR,0x02,0x00) == ESP_OK) printf("3.连续转换模式\n");

		}
		printf("********************HMC5883L初始化完成********************\n\n");


}

int HMC5883L_read_data(sensorData_t* sensorDataHMC5883L){   //HMC5883L读取角度数据

	static uint8_t test[6] = {0};//接收数据缓冲区
	static float HMC5883L_temp = 0;
	//每次从缓冲区读取一个字节，先读高字节，再读低字节，高字节左移八位，低位补0，再与低字节作或运算，即可得到一个寄存器（长度为2个字节）的完整数据
	if(iic_read(I2C_MASTER_NUM,HMC5883L_SENSOR_ADDR,0x03,test,6) == ESP_OK){

		sensorDataHMC5883L->mag_n.X  = (test[0]  << 8 | test[1])  - MAG_OFFSET_RAW.X;
		sensorDataHMC5883L->mag_n.Z  = (test[2]  << 8 | test[3])  - MAG_OFFSET_RAW.Y;
		sensorDataHMC5883L->mag_n.Y  = (test[4]  << 8 | test[5])  - MAG_OFFSET_RAW.Z;

		HMC5883L_temp = atan2((float)sensorDataHMC5883L->mag_n.X,(float)sensorDataHMC5883L->mag_n.Y)*(180/3.14159265); //x和y换算成航向角

		//printf("HMC5883L= %f\n",HMC5883L_temp);

	    ANO_DT_Send_USER_DATA(0xF1, HMC5883L_temp);

		return ESP_OK;
	}else{
		return ESP_FAIL;
	}
}
