/*
 * UDP_TCP.c
 *
 *  Created on: 2023年2月7日
 *      Author: liguanxi
 */

#include "UDP_TCP.h"

#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include "Data_declaration.h"

#include <sys/param.h>
#include "esp_netif.h"

#include "remote_control.h"
/*
static const char *TAG = "wifi station";

#define PORT 5555 //端口号
int sock;

#define HOST_IP_ADDR "192.168.43.42"

struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
socklen_t socklen = sizeof(source_addr);

char rx_buffer[128];//接收缓冲区
char addr_str[128];
int addr_family = 2;
int ip_protocol = 0;
struct sockaddr_in6 dest_addr;

uint8_t tt[50];



void socket_init(void);

static void udp_server_task(void *pvParameters) //接送数据
{
    while (1) {
    	 if(init_ok){
    		 while(1){
    			 int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
    			 if (len < 0) break;//数小于0就是接收错误
    		 }
    		 if (sock != -1) {
    		             printf("关闭套接字并重新启动\n");
    		             shutdown(sock, 0);
    		             close(sock);
    		         }
            }
        }
    }


static void udp_task(void *pvParameters)//循环发送数据
{
	TickType_t adp = xTaskGetTickCount();
		const TickType_t adg = 1000;//这里的1是指10ms运行一次100hz的意思
	while (1) {
		vTaskDelayUntil(&adp,adg);
		if(init_ok){
			tt[0] = 0x50;
			UDP_wehir(tt,2);
			printf("发送数据\n");
			//vTaskDelay(pdMS_TO_TICKS(1000));
	   }
	}

}




void socket_init(void){//套接字初始化

	struct sockaddr_in dest_addr;
	        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);//ip地址
	        dest_addr.sin_family = AF_INET;//套接字要使用的协议簇 AF_INET = （TCP/IP – IPv4）
	        dest_addr.sin_port = htons(PORT);//端口号
	        addr_family = AF_INET;
	        ip_protocol = IPPROTO_IP;


	        //创建套接字，创建成功后返回套接字，创建失败返回-1，错误代码则写入“errno”中
	        sock = socket(addr_family,  //套接字要使用的协议簇 AF_INET = （TCP/IP – IPv4）
	        				SOCK_DGRAM, //套接字类型
							ip_protocol //确定套接字的协议簇和类型时这个参数为0
							);//创建套接字
	        if (sock < 0) printf("UDP套接字创建 失败\n");
	        else printf("UDP套接字创建 成功  主机IP地址%s  主机端口%d\n", HOST_IP_ADDR, PORT);

}

void UDP_wehir(uint8_t *data,uint8_t len2){


		int err = sendto(sock, data, len2, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));


}
*/

#include "UDP_TCP.h"

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#define HOST_IP_ADDR "192.168.43.42"

#define PORT 5555

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

char rx_buffer[128];
char host_ip[] = HOST_IP_ADDR;
int addr_family = 0;
int ip_protocol = 0;
int sock;
struct sockaddr_in dest_addr;
socklen_t socklen_rc = sizeof(dest_addr);//计算遥控器存储变量大小

float U32ToFloat(uint32_t dat)
{
	uint8_t buf[4];

	buf[0] = dat >> 24;
	buf[1] = dat >> 16;
	buf[2] = dat >> 8;
	buf[3] = dat & 0xff;

	return *((float*)buf);
}


void rc_data_decode (char *data){
	uint8_t sum = 0; //校验
	uint32_t _temp; //临时变量

	rc_Send_data();

	if(data[0] != 0xBB && data[1] != 0xBB)  return; //当帧头不等于4个B就退出舍弃这段数据
	for(int i = 0;i < data[3] + 4 ;i++) sum+=(uint8_t)data[i];
	if(sum != data[data[3]+4]) return; //当校验不等时就退出舍弃这段数据
	//printf("数据校验正确\n");

	_temp = ((uint8_t)data[4]<< 24) + ((uint8_t)data[5] << 16) + ((uint8_t)data[6]  << 8) + (uint8_t)data[7];
	state.attitude.roll = U32ToFloat(_temp);

	_temp = ((uint8_t)data[8]<< 24) + ((uint8_t)data[9] << 16) + ((uint8_t)data[10]  << 8) + (uint8_t)data[11];
	state.attitude.pitch = U32ToFloat(_temp);

	_temp = ((uint8_t)data[12]<< 24) + ((uint8_t)data[13] << 16) + ((uint8_t)data[14]  << 8) + (uint8_t)data[15];
	state.attitude.yaw = U32ToFloat(_temp);

	_temp = ((uint8_t)data[16]<< 24) + ((uint8_t)data[17] << 16) + ((uint8_t)data[18]  << 8) + (uint8_t)data[19];
	alt = U32ToFloat(_temp);

	UAV_VBAT = ((uint8_t)data[20]<< 24) + ((uint8_t)data[21] << 16) + ((uint8_t)data[22]  << 8) + (uint8_t)data[23];

	 //printf("ROLL=%d  ",state.attitude.roll);
	 //printf("PITCH=%d  ",state.attitude.pitch);
	 //printf("YAW=%d  ",state.attitude.yaw);
	 //printf("alt=%d  ",alt);
	 //printf("vbat=%d  ",UAV_VBAT);



}


static void udp_client_task(void *pvParameters)
{
    while (1) {
        while (1) {
        	struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        	            socklen_t socklen = sizeof(source_addr);
        	            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&dest_addr, &socklen_rc);
        	            if (len < 0) {
        	                printf("接收失败\n");
        	                break;
        	            }
        	            // Data received
        	            else {
        	            	rc_data_decode(rx_buffer);

        	                  }

        }
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}




void udp_rc_send(uint8_t *data,uint8_t len)
{

    int err = sendto(sock, data, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

}


void udp_client_init(void){


    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }
    ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

    xTaskCreate(udp_client_task, "udp_client", 4096*2, NULL, 4, NULL);
    rc_Send_data();

}


