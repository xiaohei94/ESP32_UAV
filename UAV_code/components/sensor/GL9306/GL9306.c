/*
 * HMC5883L.c
 *  磁力计
 *  Created on: 2023年1月8日
 *      Author: liguanxi
 */
#include "GL9306.h"

#include "UART.h"
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "Data_declaration.h"

//#include "anotc.h"


int gl9306_test(void){ //VL53L1X器件检测
	uint8_t re_Buf[50] = {0};
	const int rxBytes = uart_read_bytes(UART_NUM_1, re_Buf, 50, 1000 / portTICK_RATE_MS);

	printf(" 接收到 %d 字节  \n",rxBytes);

		if(re_Buf[0] ==0xfe) {//检查帧头
			printf("gl9306传感器连接 成功\n");
			return ESP_OK;
		}else{
			printf("gl9306传感器连接 失败\n");
			return ESP_FAIL;
		}

}


void gl9306_data_Task(void *pvParameter)
{

	static const char *RX_TASK_TAG = "RX_TASK";
	    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
	    uint8_t* UartRxOpticalFlow = (uint8_t*) malloc(RX_BUF_SIZE+1);
	    while (1) {
			if(init_ok)
			{
	        const int rxBytes = uart_read_bytes(1, UartRxOpticalFlow, RX_BUF_SIZE, 0);
	        if (rxBytes > 0) {

	        	uint8_t Check_sum = 0;
	        	static int16_t flow_x,flow_y;
	        	if(UartRxOpticalFlow[0]==0xfe)//校验头
	        	{
	        	Check_sum=(uint8_t)(UartRxOpticalFlow[2]+UartRxOpticalFlow[3]
	        	+UartRxOpticalFlow[4]+UartRxOpticalFlow[5]);
	        	if(Check_sum == UartRxOpticalFlow[6])//校验和正确
	        	{
	        	flow_x = UartRxOpticalFlow[2] + (UartRxOpticalFlow[3] << 8);
	        	flow_y = UartRxOpticalFlow[4] + (UartRxOpticalFlow[5] << 8);
	        	sensorData.Optic_flow_n.X += flow_x;
	        	sensorData.Optic_flow_n.Y += flow_y;
	        	//printf("X=%f Y=%f\n",state.position.X,state.position.Y);
	        	}
	        	}



	        }
	    }

	vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

void GL9306_init(void){

	printf("********************GL9306初始化开始********************\n");
	if(gl9306_test() == ESP_OK){
	xTaskCreate(gl9306_data_Task, "gl9306_data_Task", 1024 * 8, NULL, 24, NULL);
	}

	printf("********************GL9306初始化完成********************\n\n");


}

