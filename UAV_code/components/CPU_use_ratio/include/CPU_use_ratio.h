/*
 * CPU_use_ratio.h
 *
 *  Created on: 2023年2月8日
 *      Author: liguanxi
 */

#ifndef COMPONENTS_CPU_USE_RATIO_INCLUDE_CPU_USE_RATIO_H_
#define COMPONENTS_CPU_USE_RATIO_INCLUDE_CPU_USE_RATIO_H_



#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/event_groups.h"
#include <string.h>


void CPU_Task(void* parameter);

#endif /* COMPONENTS_CPU_USE_RATIO_INCLUDE_CPU_USE_RATIO_H_ */
