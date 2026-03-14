/*
 * HMC5883L.h
 *
 *  Created on: 2023年1月8日
 *      Author: liguanxi
 */

#ifndef HMC5883L_H_
#define HMC5883L_H_

#include "IIC.h"

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "Data_declaration.h"

void HMC5883L_init(void);//磁力计初始化
int HMC5883L_read_data(sensorData_t* sensorDataHMC5883L);   //HMC5883L读取角度数据


#endif /* COMPONENTS_SENSOR_HMC5883L_HMC5883L_H_ */
