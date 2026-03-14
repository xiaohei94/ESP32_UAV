/*
 * IMU.h
 *
 *  Created on: 2022年11月25日
 *      Author: admin
 */
#ifndef _IMU_H_
#define _IMU_H_

#include "Data_declaration.h"

#define G      9.80665f            // m/s^2
#define RadtoDeg    57.324841f        //弧度到角度 (弧度 * 180/3.1415)
#define DegtoRad    0.0174533f        //角度到弧度 (角度 * 3.1415/180)


void Prepare_Data(sensorData_t* sensorDatafilter);
void IMUupdate(sensorData_t* sensorDataimu,state_t* stateimu);
void  SortAver_Filter(float in,float *out,uint8_t n); //窗口滑动滤波函数
void  SortAver_Filter2(float in,float *out,uint8_t n); //窗口滑动滤波函数
void  SortAver_Filter3(float in,float *out,uint8_t n); //窗口滑动滤波函数
void  SortAver_Filter4(float in,float *out,uint8_t n); //窗口滑动滤波函数
void  SortAver_Filter5(float in,float *out,uint8_t n); //窗口滑动滤波函数


#endif
