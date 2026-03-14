/*
 * control.c
 *
 *  Created on: 2023年2月13日
 *      Author: liguanxi
 */
#include "control.h"

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "Data_declaration.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "VL53L1X.h"

#include <sys/time.h>

#include <inttypes.h>

#include "MPU6050.h"
#include "IMU.h"


#include "PID.h"
#include "IMU.h"
#include "PWM.h"

#include "SPL06-001.h"
#include "HMC5883L.h"

#include "anotc.h"

struct timeval tv_now;
int64_t time1_us = 0;
int64_t time_us = 0;



float Pre_THROTTLE,THROTTLE;
float Moto_PWM_1=0.0f,Moto_PWM_2=0.0f,Moto_PWM_3=0.0f,Moto_PWM_4=0.0f;
uint8_t SI24R1_Controlflag = 1,Airplane_Enable;


int mun = 0;

float rc_yaw = 0;
float control_yaw = 0;
bool lock_p = false;
float alt_in = 0;
float pos_x_in = 0;
float pos_y_in = 0;
int mode_p = 0;
float thrust_pid_out = 0;
float roll_pid_out = 0;
float pitch_pid_out = 0;
float atl_last = 0;
float atl_acc = 0;

void Control(state_t *att_in,sensorData_t *gyr_in, setpoint_t *rc_in)
{
	attitude_t Measure_Angle,Target_Angle;
	Measure_Angle.roll = att_in->attitude.roll;
	Measure_Angle.pitch = att_in->attitude.pitch;
	Measure_Angle.yaw = att_in->attitude.yaw - control_yaw;
	Target_Angle.roll = (float)((rc_in->attitude.roll-1500)/35.0f);
	Target_Angle.pitch = (float)((rc_in->attitude.pitch-1500)/35.0f);
	Target_Angle.yaw = (float)((1500-rc_in->attitude.yaw)/35.0f);
	if(Target_Angle.yaw >1 || Target_Angle.yaw <-1) rc_yaw -= Target_Angle.yaw/100;



	if(state.airplane_mode >= 2){ //定高
		if(mode_p!=state.airplane_mode) {
			alt_in = 1600;
		}
		rc_in->thrust = 500;
		PID_Postion_Cal(&PID_ALT,alt_in,state.position.Z);//ROLL角度环PID （输入角度 输出角速度）
		PID_Postion_Cal(&PID_ALT_Rate,PID_ALT.OutPut,state.velocity.Z); //ROLL角速度环PID （输入角度环的输出，输出电机控制量）
		thrust_pid_out = PID_ALT_Rate.OutPut;
		rc_in->thrust = rc_in->thrust + thrust_pid_out;

		//printf("%f %f  %f\n",thrust_pid_out,PID_ALT.OutPut,state.velocity.Z);
	}

	if(state.airplane_mode == 3){ //定点
		if(mode_p!=state.airplane_mode) {
			pos_x_in = state.position.X;
			pos_y_in = state.position.Y;
		}else{
			if(Target_Angle.pitch > 3 || Target_Angle.pitch < -3) pos_y_in += Target_Angle.pitch/10;
			if(Target_Angle.roll > 3 || Target_Angle.roll < -3) pos_x_in += Target_Angle.roll/10;
		}

		Target_Angle.roll = 0;
		Target_Angle.pitch = 0;
		PID_Postion_Cal(&PID_POSITION_X,pos_x_in,state.position.X);//x轴位置环PID （输入位置 输出速度）
		PID_Postion_Cal(&PID_POSITION_X_Rate,PID_POSITION_X.OutPut,state.velocity.X); //x轴速度环PID （输入位置环的输出，输出电机控制量）
		roll_pid_out = PID_POSITION_X_Rate.OutPut;
		Target_Angle.roll =(Target_Angle.roll+roll_pid_out)/5;

		PID_Postion_Cal(&PID_POSITION_Y,pos_y_in,state.position.Y);//y轴位置环PID （输入位置 输出速度）
		PID_Postion_Cal(&PID_POSITION_Y_Rate,PID_POSITION_Y.OutPut,state.velocity.Y); //y轴速度环PID （输入位置环的输出，输出电机控制量）
		pitch_pid_out = PID_POSITION_Y_Rate.OutPut;
		Target_Angle.pitch = (Target_Angle.pitch+pitch_pid_out)/5;



		//printf("%f %f  \n",Target_Angle.roll,Target_Angle.pitch);
		printf("%f %f  \n",pos_x_in,pos_y_in);
		//printf("%f %f  \n",roll_pid_out,pitch_pid_out);
	}
	mode_p = state.airplane_mode;

	//printf(" in %0.2f out %0.2f \n",rc_yaw,Measure_Angle.yaw);
	//角度环
	PID_Postion_Cal(&PID_ROL_Angle,Target_Angle.roll,Measure_Angle.roll);//ROLL角度环PID （输入角度 输出角速度）
	PID_Postion_Cal(&PID_PIT_Angle,Target_Angle.pitch,Measure_Angle.pitch);//PITH角度环PID （输入角度 输出角速度）
    PID_Postion_Cal(&PID_YAW_Angle,Target_Angle.yaw,Measure_Angle.yaw);//YAW角度环PID  （输入角度 输出角速度）

	//角速度环
	PID_Postion_Cal(&PID_ROL_Rate,PID_ROL_Angle.OutPut,(gyr_in->gyro_f.Y*RadtoDeg)); //ROLL角速度环PID （输入角度环的输出，输出电机控制量）
	PID_Postion_Cal(&PID_PIT_Rate,PID_PIT_Angle.OutPut,(gyr_in->gyro_f.X*RadtoDeg)); //PITH角速度环PID （输入角度环的输出，输出电机控制量）
	PID_Postion_Cal(&PID_YAW_Rate,PID_YAW_Angle.OutPut,(gyr_in->gyro_f.Z*RadtoDeg)); //YAW角速度环PID （输入角度，输出电机控制量）


	//动力分配（自己DIY时动力分配一定要好好研究，动力分配搞错飞机肯定飞不起来!!!）
	if(att_in->isRCLocked)//当油门大于150时和飞机解锁时动力分配才生效
	{
		if(lock_p == false) {
			control_yaw = att_in->attitude.yaw ; //解锁时清空yaw值
			rc_yaw = 0;
		}

		if(rc_in->thrust>180){
			Moto_PWM_1 = rc_in->thrust + PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;
			Moto_PWM_2 = rc_in->thrust - PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
			Moto_PWM_3 = rc_in->thrust - PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;
			Moto_PWM_4 = rc_in->thrust + PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
		}else{
			  mun++;
		      if (mun >= 1)Moto_PWM_1 = 5;
		      if (mun >= 10)Moto_PWM_2 = 5;
		      if (mun >= 20)Moto_PWM_3 = 5;
		      if (mun >= 30)Moto_PWM_4 = 5;
		      if (mun >= 40)mun = 40;
		}
	}
	else
	{
		Moto_PWM_1 = 0;
		Moto_PWM_2 = 0;
		Moto_PWM_3 = 0;
		Moto_PWM_4 = 0;
		mun = 0;
	}

   //保护程序当飞机角度过大时将电机锁上避免乱转打坏桨叶
	if((fabs(att_in->attitude.roll)>45.0f||fabs(att_in->attitude.pitch)>45.0f) || (fabs(sensorData.acc_f.X)>9.0f||fabs(sensorData.acc_f.Y)>9.0f)){

		state.isRCLocked = false;
		Moto_PWM_1 = 0;
		Moto_PWM_2 = 0;
		Moto_PWM_3 = 0;
		Moto_PWM_4 = 0;
	}

   Moto_Pwm(Moto_PWM_1,Moto_PWM_2,Moto_PWM_3,Moto_PWM_4); //将此数值分配到定时器，输出对应占空比的PWM波

}

void printf_time1_us(uint8_t set){//发送系统时间

	if(set == 0){
		gettimeofday(&tv_now, NULL);
		time1_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
		//printf("%" PRId64 "\n", time1_us);
	}else{
		gettimeofday(&tv_now, NULL);
		time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
		time_us = time_us-time1_us;
		printf("%" PRId64 "       ", time_us);
	}

}


void angle_control_Task(void *pvParameter)
{

	const TickType_t adg = 10;//这里的数是指ticks（时间片）的意思，等于1就是每个ticks中断都执行
	TickType_t adp = xTaskGetTickCount();
	while(1){
		vTaskDelayUntil(&adp,adg);
		if(init_ok)
		{
			//printf_time1_us(1);
			//printf_time1_us(0);
			mpu6050_read_data(0x6B,&sensorData);   //获取mpu6050原始数值
			HMC5883L_read_data(&sensorData);     //获取HMC5883L原始数值，
			Prepare_Data(&sensorData);             //将mpu6050原始数值滤波并转换单位
			IMUupdate(&sensorData,&state);         //将姿态数据通过四元数转为欧拉角
			Height_Get();                        //获取气压计的数据并进行滤波换算
			Control(&state,&sensorData,&setpoint); //PID控制并输出到PWM

			//printf_time1_us(1);
			//printf("\n");
			//printf("roll:=%0.2f pitch:=%0.2f yaw:=%0.2f \n",state.attitude.roll,state.attitude.pitch,state.attitude.yaw);


		}
	}
}
void control_init(void){
	xTaskCreate(angle_control_Task, "angle_control_Task", 1024 * 8, NULL, 24, NULL);
}


