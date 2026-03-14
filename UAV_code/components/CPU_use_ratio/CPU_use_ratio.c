/*
 * CPU_use_ratio.c
 *
 *  Created on: 2023年2月8日
 *      Author: liguanxi
 */

#include "CPU_use_ratio.h"

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/event_groups.h"
#include <string.h>

#include "Data_declaration.h"


void CPU_Task(void* parameter)
{
  uint8_t CPU_RunInfo[400];		//保存任务运行时间信息

  while (1)
  {
	  if(init_ok)
	  {
		  memset(CPU_RunInfo,0,400);				//信息缓冲区清零

		  vTaskList((char *)&CPU_RunInfo);  //获取任务运行时间信息

		  printf("---------------------------------------------\r\n");
		  printf("任务名  任务状态  优先级  剩余栈  任务序号\r\n");
		  printf("%s", CPU_RunInfo);
		  printf("---------------------------------------------\r\n");

		  memset(CPU_RunInfo,0,400);				//信息缓冲区清零

		  vTaskGetRunTimeStats((char *)&CPU_RunInfo);

		  printf("任务名     运行计数     使用率\r\n");
		  printf("%s", CPU_RunInfo);
		  printf("---------------------------------------------\r\n\n");
		  vTaskDelay(500 / portTICK_PERIOD_MS);   /* 延时500个tick */
	  }
  }
}
