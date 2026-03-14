/*
 * SI24R1.c
 *
 *  Created on: 2023年2月6日
 *      Author: liguanxi
 */

#include "SI24R1.h"

#include "driver/gpio.h"

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "SPI.h"
#include "Data_declaration.h"

//NRF24L01寄存器操作命令

#define R_REGISTER 0x00
#define W_REGISTER 0x02
#define DUMMY      0x00

#define RX_PAYLOAD	0X61
#define TX_PAYLOAD	0XA0
#define FLUSH_TX 0XE1
#define FLUSH_RX 0XE2
#define RX_WID 0X60
#define NOP 0XFF
#define CONFIG 0X00
#define EN_AA 0X01
#define EN_RXADDR 0X02
#define SETUP_AW 0X03
#define SETUP_RETR 0X04
#define RF_CH 0X05
#define RF_SETUP 0X06
#define STATUS 0X07
#define OBSERVE_TX 0X08
#define RX_ADDR_P0 0X0A
#define RX_PW_P0 0X11
#define TX_ADDR 0X10
#define FIFO_STATUS 0X17
#define DYNPD 0X1C
#define FEATURE 0X1D



#define PIN_NUM_IRQ     45
#define PIN_NUM_CE      11

void SI24R1_Read_Task(void * pvParameters);

int SI24R1_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
	SI24R1_Write_Buf(0x30,buf,5);//写入5个字节的地址.
	SI24R1_Read_Buf(0x10,buf,5); //读出写入的地址
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;
	if(i!=5){
		printf("SI24R1连接 失败\n");
		return ESP_FAIL;//检测24L01错误
	}else{
		printf("SI24R1连接 成功\n");
		return ESP_OK;		 //检测到24L01
	}

}



uint8_t DATAWIDTH=32;
uint8_t TX_ADDRESS[5]={0X3E,0x7E,0x7E,0x7E,0x7E};

//发送模式配置，ACK
void NRF24_RX_MODE(void)
{
	gpio_set_level(PIN_NUM_CE, 0);

	SI24R1_Write_Buf(W_REGISTER+RX_ADDR_P0,TX_ADDRESS,5);//写入接收通道0地址，5字节

	SI24R1_Write_Reg(W_REGISTER+FLUSH_RX,0xFF);//清空RX_FIFO
	SI24R1_Write_Reg(W_REGISTER+EN_AA,0X01);//使能自动确认
	SI24R1_Write_Reg(W_REGISTER+EN_RXADDR,0X01);//使能接收通道0
	SI24R1_Write_Reg(W_REGISTER+RF_CH,40);//射频通道40
	SI24R1_Write_Reg(W_REGISTER+RX_PW_P0,DATAWIDTH);//设置接收通道0，与发送字节相同
	SI24R1_Write_Reg(W_REGISTER+RF_SETUP,0X0F);//速率2Mbps，功率7dBm
	SI24R1_Write_Reg(W_REGISTER+CONFIG,0X0F);//接收送模式，2 Byte CRC

	gpio_set_level(PIN_NUM_CE, 1);
}

//接收数据
void RXPACKET(uint8_t *rx)
{
	uint8_t status,rdata[32];

	if(gpio_get_level(PIN_NUM_IRQ)){
		SI24R1_Read_Reg(STATUS,&status);
			SI24R1_Write_Reg(W_REGISTER+STATUS,status);//清除标志位
			printf("status=0x%x\n",status);

			if(status&0x40)
			{
				SI24R1_Read_Buf(RX_PAYLOAD,rdata,32);
				SI24R1_Write_Reg(W_REGISTER+FLUSH_RX,0xFF);
				printf("接收的数据为：0x%x\n",rdata[0]);
				//return 1;
			}else{
				printf("数据接收失败！\n");
				//return 0;
			}
	}
}




void si24r1_init(void){

	printf("*******************SI24R1初始化开始*******************\n");
	if(SI24R1_Check() == ESP_OK) //判断SI24R1是否连接
	{
		gpio_config_t led_conf;

		led_conf.mode = GPIO_MODE_OUTPUT;		     // 配置gpio的模式
		led_conf.intr_type = GPIO_PIN_INTR_DISABLE;  // 失能中断
		led_conf.pin_bit_mask = 1ULL << PIN_NUM_CE;// 配置GPIO_IN寄存器，选择初始化的GPIO3口为led控制
		led_conf.pull_down_en = 0;                   // 下拉失能
		led_conf.pull_up_en = 0;                     // 上拉失能

		gpio_config(&led_conf);                       // 配置gpio参数，并使能

		led_conf.mode = GPIO_MODE_INPUT;		     // 配置gpio的模式
		led_conf.intr_type = GPIO_PIN_INTR_DISABLE;  // 失能中断
		led_conf.pin_bit_mask = 1ULL << PIN_NUM_IRQ;// 配置GPIO_IN寄存器，选择初始化的GPIO3口为led控制
		led_conf.pull_down_en = 0;                   // 下拉失能
		led_conf.pull_up_en = 1;                     // 上拉失能

		gpio_set_level(PIN_NUM_CE, 1);//CE为高,进入接收模式
		NRF24_RX_MODE();

	}
	xTaskCreate(SI24R1_Read_Task,"SI24R1_Read_Task",1024*5,NULL,5,NULL);//循环读取

	printf("*******************SI24R1初始化结束*******************\n");
}

uint8_t in_data[50];

void SI24R1_Read_Task(void * pvParameters){//自动接收解析数据
	TickType_t adp = xTaskGetTickCount();
	const TickType_t adg = 1000;//这里的1是指10ms运行一次100hz的意思
	while(1){
		vTaskDelayUntil(&adp,adg);
		if(init_ok)
			{
			RXPACKET(in_data);
			printf("%d\n",gpio_get_level(PIN_NUM_IRQ));
			}
	}
}


