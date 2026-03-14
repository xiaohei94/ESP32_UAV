/*
 * HMC5883L.c
 *  磁力计
 *  Created on: 2023年1月8日
 *      Author: liguanxi
 */
#include "include/VL53L1X.h"

#include "UART.h"

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "Data_declaration.h"


int VL53L1X_test(void){ //VL53L1X器件检测
	uint8_t re_Buf[50] = {0};
	const int rxBytes = uart_read_bytes(UART_NUM_2, re_Buf, 50, 1000 / portTICK_RATE_MS);
	printf(" 接收到 %d 字节  \n",rxBytes);
		if(re_Buf[0] == 0x5A && re_Buf[1] == 0x5A) {//检查帧头
			printf("VL53L1X传感器连接 成功\n");
			return ESP_OK;
		}else{
			printf("VL53L1X传感器连接 失败\n");
			return ESP_FAIL;
		}

}

void VL53L1X_data_Task(void *pvParameter)
{
	int sum = 0;
    uint16_t _temp = 0;
	static const char *RX_TASK_TAG = "RX_TASK";
	    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
	    uint8_t* re_Buf = (uint8_t*) malloc(RX_BUF_SIZE+1);
	    while (1) {
			if(init_ok)
			{
	        const int rxBytes = uart_read_bytes(UART_NUM_2, re_Buf, RX_BUF_SIZE, 0 / portTICK_RATE_MS);

	        if (rxBytes > 0) {
	        	//printf(" \n bus%d    ",rxBytes);
	        	for(int i = 0 ; i < 8 ; i++)  {
	        	//	printf("  %d    ",re_Buf[i]);
	        	}
	        	if(re_Buf[0] == 0x5A && re_Buf[1] == 0x5A){//检查帧头
	        		sum = 0;
		        	for(int i = 0 ; i < 7 ; i++)  sum += re_Buf[i]; //叠加校验值
		        	//printf("\n sum_in %d   sum_out  %d ",re_Buf[7],sum);
		        	//if(sum == re_Buf[7]) {//检查校验值
		        		_temp = re_Buf[4]<<8|re_Buf[5];
		        		ALT_BRO.h = _temp;
		        		//printf(" %f   \n ",ALT_BRO.h);
		        	//}
	        	}
	        }
	    }

	vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

void VL53L1X_init(void){

	printf("********************VL53L1X初始化开始********************\n");

	if(VL53L1X_test() == ESP_OK)  //检测传感器成功连接就进行配置



		xTaskCreate(VL53L1X_data_Task, "VL53L1X_data_Task", 1024 * 8, NULL, 24, NULL);


	printf("********************VL53L1X初始化完成********************\n\n");


}

