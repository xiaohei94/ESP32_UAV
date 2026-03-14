/*
 * SPL06-001.c
 *
 *  Created on: 2023年6月4日
 *      Author: 30383
 */

#include "IIC.h"
#include "SPL06-001.h"
#include "Data_declaration.h"


#include <math.h>
#include "driver/i2c.h"
#include "stdio.h"
#include "string.h"
#include "mpu6050.h"
#include "anotc.h"

#include "phy_init_data.h"
#include "IMU.h"

#define ABS(x) ((x) > 0 ? (x) : -(x))

float Drv_Spl0601_Read(void);
void High_Data_Reset(void);
void Moving_Average(float moavarray[],uint16_t len ,uint16_t *fil_cnt,float in,float *out);

/*

#define P_MEASURE_RATE SPL06_MWASURE_16   // 每秒测量次数
#define P_OVERSAMP_RATE SPL06_OVERSAMP_64 // 过采样率
#define SPL06_PRESSURE_CFG (P_MEASURE_RATE << 4 | P_OVERSAMP_RATE)

#define T_MEASURE_RATE SPL06_MWASURE_16  // 每秒测量次数
#define T_OVERSAMP_RATE SPL06_OVERSAMP_8 // 过采样率
#define SPL06_TEMPERATURE_CFG (TEMPERATURE_EXTERNAL_SENSOR << 7 | T_MEASURE_RATE << 4 | T_OVERSAMP_RATE)

#define SPL06_MODE (SPL06_CONTINUOUS_MODE)
*/
const uint32_t scaleFactor[8] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};
const uint8_t num_map[8] = {1,2,4,8,16,32,64,128};

float alt_3, height;

float asl_offset = 0;

static uint8_t devAddr;

int32_t kp = 0;
int32_t kt = 0;
int32_t SPL06RawPressure = 0;
int32_t SPL06RawTemperature = 0;
float pressure, temperature, asl;





void spl0601_get_calib_param(void)  //读取传感器内置校准值
{
    uint8_t buffer[SPL06_CALIB_COEFFICIENT_LENGTH] = {0};

    iic_read(I2C_MASTER_NUM,devAddr, SPL06_COEFFICIENT_CALIB_REG, buffer,SPL06_CALIB_COEFFICIENT_LENGTH);

    spl06Calib.c0 = (int16_t)buffer[0] << 4 | buffer[1] >> 4;
    spl06Calib.c0 = (spl06Calib.c0 & 0x0800) ? (spl06Calib.c0 | 0xF000) : spl06Calib.c0;

    spl06Calib.c1 = (int16_t)(buffer[1] & 0x0F) << 8 | buffer[2];
    spl06Calib.c1 = (spl06Calib.c1 & 0x0800) ? (spl06Calib.c1 | 0xF000) : spl06Calib.c1;

    spl06Calib.c00 = (int32_t)buffer[3] << 12 | (int32_t)buffer[4] << 4 | (int32_t)buffer[5] >> 4;
    spl06Calib.c00 = (spl06Calib.c00 & 0x080000) ? (spl06Calib.c00 | 0xFFF00000) : spl06Calib.c00;

    spl06Calib.c10 = (int32_t)(buffer[5] & 0x0F) << 16 | (int32_t)buffer[6] << 8 | (int32_t)buffer[7];
    spl06Calib.c10 = (spl06Calib.c10 & 0x080000) ? (spl06Calib.c10 | 0xFFF00000) : spl06Calib.c10;

    spl06Calib.c01 = (int16_t)buffer[8] << 8 | buffer[9];
    spl06Calib.c11 = (int16_t)buffer[10] << 8 | buffer[11];
    spl06Calib.c20 = (int16_t)buffer[12] << 8 | buffer[13];
    spl06Calib.c21 = (int16_t)buffer[14] << 8 | buffer[15];
    spl06Calib.c30 = (int16_t)buffer[16] << 8 | buffer[17];
}

bool SPL06_test(void){//测试模块是否连接

	uint8_t SPL06ID[2] = {0};
	iic_read(I2C_MASTER_NUM,devAddr, SPL06_CHIP_ID, SPL06ID,1); // 读取SPL06 ID
	if(SPL06ID[0] == 0x10) {
		printf("SPL06-001气压计连接 成功\n");
		return ESP_OK;
	}
	printf("SPL06-001气压计连接 失败\n");
	return ESP_FAIL;
}

void SPL06Init(void){ //初始化模块
	printf("********************SPL06-001初始化开始********************\n");

	devAddr = SPL06_I2C_ADDR;

	if(SPL06_test() == ESP_OK){

		 // 读取校准数据
		    spl0601_get_calib_param();

		    printf("pressure    %d \n",Pressure_measurement_rate << 5 | Pressure_oversampling_rate);
		    printf("temperatuer %d \n", Temperature_measurement_rate << 4 | Temperature_oversampling_rate| 0x80);

		    if(iic_write(I2C_MASTER_NUM,devAddr,0x06, 0 << 7 | Pressure_measurement_rate << 4 | Pressure_oversampling_rate) == ESP_OK) printf("1.配置压力寄存器，每秒刷新%d次，过采样率%d次（高精度）\n",num_map[Pressure_measurement_rate],num_map[Pressure_oversampling_rate]);
		    kp = scaleFactor[Pressure_oversampling_rate];
		    if(iic_write(I2C_MASTER_NUM,devAddr,0x07,1 << 7 | Temperature_measurement_rate << 4 | Temperature_oversampling_rate) == ESP_OK) printf("2.配置温度寄存器，使用外部传感器，3.6ms测量一次，采样1次（默认）\n");
		    kt = scaleFactor[Temperature_oversampling_rate];
			if(iic_write(I2C_MASTER_NUM,devAddr,0x09,0x0C) == ESP_OK) printf("3.温度和压力结果移位\n");

			if(iic_write(I2C_MASTER_NUM,devAddr,0x08,0x07) == ESP_OK) printf("4.开始连续测量压力和温度\n");

		    if(iic_write(I2C_MASTER_NUM,devAddr,0x08,0x07) == ESP_OK) printf("4.开始连续测量压力和温度\n");

		    AirPressure_calibration = 1;
		    while(!get_offset_start()); //校准气压计零偏

	}

    printf("********************SPL06-001初始化完成********************\n");
}

void SPL06GetPressure(void)
{
    uint8_t data[SPL06_DATA_FRAME_SIZE];

    iic_read(I2C_MASTER_NUM,devAddr, SPL06_PRESSURE_MSB_REG, data,SPL06_DATA_FRAME_SIZE); // 读取SPL06 ID

    SPL06RawPressure = (int32_t)data[0] << 16 | (int32_t)data[1] << 8 | (int32_t)data[2];
    SPL06RawPressure = (SPL06RawPressure & 0x800000) ? (0xFF000000 | SPL06RawPressure) : SPL06RawPressure;

    SPL06RawTemperature = (int32_t)data[3] << 16 | (int32_t)data[4] << 8 | (int32_t)data[5];
    SPL06RawTemperature = (SPL06RawTemperature & 0x800000) ? (0xFF000000 | SPL06RawTemperature) : SPL06RawTemperature;
}

float spl0601_get_temperature(int32_t rawTemperature)
{
    float fTCompensate;
    float fTsc;

    fTsc = rawTemperature / (float)kt;
    fTCompensate = spl06Calib.c0 * 0.5 + spl06Calib.c1 * fTsc;
    return fTCompensate;
}

float spl0601_get_pressure(int32_t rawPressure, int32_t rawTemperature)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = rawTemperature / (float)kt;
    fPsc = rawPressure / (float)kp;
    qua2 = spl06Calib.c10 + fPsc * (spl06Calib.c20 + fPsc * spl06Calib.c30);
    qua3 = fTsc * fPsc * (spl06Calib.c11 + fPsc * spl06Calib.c21);
    // qua3 = 0.9f *fTsc * fPsc * (spl06Calib.c11 + fPsc * spl06Calib.c21);

    fPCompensate = spl06Calib.c00 + fPsc * qua2 + fTsc * spl06Calib.c01 + qua3;
    // fPCompensate = spl06Calib.c00 + fPsc * qua2 + 0.9f *fTsc  * spl06Calib.c01 + qua3;
    return fPCompensate;
}

//压力转海拔高度
float SPL06PressureToAltitude(float pressure /*, float* groundPressure, float* groundTemp*/)
{
    alt_3 = (101000 - pressure) / 1000.0f;
    height = 0.82f * alt_3 * alt_3 * alt_3 + 0.09f * (101000 - pressure) * 100.0f;
    return height;
}

// 参数：com 为采样的原始数值
// 返回值：iData 经过一阶滤波后的采样值
float lowV(float com)
{
    static float iLastData;                            // 上一次值
    float iData;                                       // 本次计算值
    float dPower = 0.1;                                // 滤波系数
    iData = (com * dPower) + (1 - dPower) * iLastData; // 计算
    iLastData = iData;                                 // 存贮本次数据
    return iData;                                      // 返回数据
}

void SPL06GetData(float *pressure, float *temperature, float *asl)
{
    static float t;
    static float p;

    SPL06GetPressure();

    t = spl0601_get_temperature(SPL06RawTemperature);
    p = spl0601_get_pressure(SPL06RawPressure, SPL06RawTemperature);

    //	pressureFilter(&p,pressure);
    *temperature = (float)t; /*单位度*/
    *pressure = (float)p;    /*单位hPa*/

    *pressure = lowV(*pressure);
    *asl = SPL06PressureToAltitude(*pressure); /*转换成海拔*/
}

/**
 * Converts pressure to altitude above sea level (ASL) in meters
 */





//气压校准零偏
int get_offset_start(void)
{
    static uint16_t cnt_a=0;
    float pressure, temperature, asl;

    if(cnt_a==0)
    {
    	for (uint16_t i = 0; i < 200; i++)
    	{
    		SPL06GetData(&pressure, &temperature, &asl);
    	}

    }

    SPL06GetData(&pressure, &temperature, &asl);
    asl_offset += asl;

    if(cnt_a==200)
    {
        asl_offset = asl_offset / 200.0f;
        cnt_a = 0;
        return 1;
    }
      cnt_a++;
      return 0;
}
/*
float acc_error = 0;
float acc_offset = 0;
float acc_out = 0;
float asl_last = 0;
float alt_temp = 0;
float alt_out = 0;
float air_acc = 0;
float air_acc_temp = 0;

float pos_x_temp = 0;
*/

void  VAL_LIMIT(float val,float min , float max){
    if(val > max) val = max;
    if(val < min) val = min;
}

FLOAT_XYZ fusion_acc,acc_temp,acc_offset,pos_last,pos_temp;

void Height_Get(void) // 取得高度原始值
{

    // 读取气压计高度
    //SPL06GetData(&pressure, &temperature, &asl);
    //ALT_BRO.h = asl - asl_offset;
    //printf("asl:%f\n",asl);
    //printf("asl_offset:%f\n",asl_offset);

    SortAver_Filter(NormAccz,&acc_temp.Z,20);
    //SortAver_Filter2(sensorData.acc_f.X,&acc_temp.X,10);
    //SortAver_Filter3(sensorData.acc_f.Y,&acc_temp.Y,10);
    acc_temp.X = sensorData.acc_f.X;
    acc_temp.Y = sensorData.acc_f.Y;
    //ANO_DT_Send_USER_DATA(0xF1, state.position.Y);
    //ANO_DT_Send_USER_DATA(0xF2, state.position.X);

    VAL_LIMIT(acc_temp.Z, -5 , 5);
    VAL_LIMIT(acc_temp.X, -5 , 5);
	VAL_LIMIT(acc_temp.Y, -5 , 5);

    if(AirPressure_calibration == 1){
    	if(get_offset_start() == 1)  {
    		AirPressure_calibration = 2;
    		acc_offset.X = acc_temp.X;
    		acc_offset.Y = acc_temp.Y;
    		acc_offset.Z = acc_temp.Z;
    	}
    }//校准零偏记录零偏值

    if(LOCK_P){
    	sensorData.Optic_flow_n.X = 0;
    	sensorData.Optic_flow_n.Y = 0;
    }
    // 校准零偏
    acc_temp.X = acc_temp.X - acc_offset.X;
    acc_temp.Y = acc_temp.Y - acc_offset.Y;
    acc_temp.Z = acc_temp.Z - acc_offset.Z;

    if(AirPressure_calibration != 1) { //数据融合
    	//alt_temp = (alt_temp + acc_error)*0.98f + 0.02f*(asl - asl_offset)/10; //气压互补滤波速度环永远是主调，所以互补滤波关键就抓在速度环这里
    	pos_temp.Z = (pos_temp.Z + acc_temp.Z)*0.8f + 0.5f*ALT_BRO.h; //激光互补滤波速度环永远是主调，所以互补滤波关键就抓在速度环这里
    	pos_temp.X = (pos_temp.X + acc_temp.X)*0.8f + 0.3f*sensorData.Optic_flow_n.X; //激光互补滤波速度环永远是主调，所以互补滤波关键就抓在速度环这里
    	pos_temp.Y = (pos_temp.Y + acc_temp.Y)*0.8f + 0.3f*sensorData.Optic_flow_n.Y; //激光互补滤波速度环永远是主调，所以互补滤波关键就抓在速度环这里
    }

    fusion_acc.Z = (pos_temp.Z - pos_last.Z)*10;
    fusion_acc.X = (pos_temp.X - pos_last.X)*10;
    fusion_acc.Y = (pos_temp.Y - pos_last.Y)*10;

    pos_last.Z = pos_temp.Z;
    pos_last.X = pos_temp.X;
    pos_last.Y = pos_temp.Y;

    state.position.Z = (pos_temp.Z-80)/2.3;
    state.position.X = pos_temp.X;
    state.position.Y = pos_temp.Y;

    SortAver_Filter2(fusion_acc.Z,&state.velocity.Z,12); //加速度滤波
    SortAver_Filter3(fusion_acc.X,&state.velocity.X,20); //加速度滤波
    SortAver_Filter4(fusion_acc.Y,&state.velocity.Y,20); //加速度滤波


    //ANO_DT_Send_USER_DATA(0xF1, state.velocity.Y);
    //ANO_DT_Send_USER_DATA(0xF2, state.position.Y);
    //ANO_DT_Send_USER_DATA(0xF3, state.velocity.X);
    //ANO_DT_Send_USER_DATA(0xF4, state.position.X);


    //ANO_DT_Send_USER_DATA(0xF3, fusion_acc.Y);


    //SortAver_Filter3(ALT_BRO.h,&state.position.Z,20); //加速度滤波
    //SortAver_Filter2(alt_temp,&alt_out,20); //加速度滤波

    //printf("%f\n",state.position.X);
    //printf("%f\n",state.position.Y);
    //高度波形查看
    //ANO_DT_Send_USER_DATA(0xF1, state.position.Z);
    //z轴加速度波形查看
    //ANO_DT_Send_USER_DATA(0xF2, air_acc);
    //ANO_DT_Send_USER_DATA(0xF3, state.velocity.Z);

    //printf("%f\n",air_acc);
    //ANO_DT_Send_USER_DATA(0xF2, air_acc*100);
	//printf("GZ:%f\n",asl - asl_offset);
    //printf("height:%f \n",alt_temp);


}

