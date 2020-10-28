/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "driver/gpio.h"


#define PORT CONFIG_EXAMPLE_PORT
#define GPIO_IO_32      32
#define GPIO_OUTPUT_SEL ((1ULL << GPIO_IO_32))

static const char *TAG = "example";


static void tcpServer(void *pvParameters)
{
	int addr_family = AF_INET;
	int ip_protocol = 0;
	struct sockaddr_in dest_addr;

	struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
	dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
	dest_addr_ip4->sin_family = AF_INET;
	dest_addr_ip4->sin_port = htons(PORT);
	ip_protocol = IPPROTO_IP;

	//Create a socket
	int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
	if (listen_sock < 0) {
		ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
		vTaskDelete(NULL);
		return;
	}
	int opt = 1;
	setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

	ESP_LOGI(TAG, "Socket created");

	//Bind the socket to an address
	int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
	if (err != 0) {
		ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
		goto CLEAN_UP;
	}
	ESP_LOGI(TAG, "Socket bound, port %d", PORT);

	//Listen for connections
	err = listen(listen_sock, 1);
	if (err != 0) {
		ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
		goto CLEAN_UP;
	}


	ESP_LOGI(TAG, "Socket listening");

	struct sockaddr_in source_addr; 
	uint addr_len = sizeof(source_addr);
	//Accept a connection, This call typically blocks until a client connects with the server.
	int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
	if (sock < 0) {
		ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
		return;
	}

	while (1) {
		
		//Send and receive data
		int len,to_write;
		char *rx_buffer = NULL,rx;
		char rx_buffer0[] = "Led is Off";
		char rx_buffer1[] = "Led is On";

		len = recv(sock, &rx,1, 0);
		printf("Rx = %c\n",rx);

		if (len < 0) {
			ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
			break;
		} 

		switch (rx)
		{
			case '0':
				{
					rx_buffer = rx_buffer0;
					len = strlen(rx_buffer0);
					gpio_set_level(GPIO_IO_32,0);
					break;
				}
			case '1':
				{
					rx_buffer = rx_buffer1;
					len = strlen(rx_buffer1);
					gpio_set_level(GPIO_IO_32,1);
					break;
				}

		}
		to_write = len;
		while (to_write > 0) {
			int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
			if (written < 0) {
				ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
				break;
			}
			to_write -= written;
		}
	}
CLEAN_UP:
	close(listen_sock);
	vTaskDelete(NULL);
}

void app_main(void)
{
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	/* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
	 * Read "Establishing Wi-Fi or Ethernet Connection" section in
	 * examples/protocols/README.md for more information about this function.
	 */
	ESP_ERROR_CHECK(example_connect());


	gpio_config_t io_config;

        io_config.pin_bit_mask = GPIO_OUTPUT_SEL;
        io_config.mode = GPIO_MODE_OUTPUT;
        io_config.pull_up_en = 0;
        io_config.pull_down_en = 0;
        io_config.intr_type = 0;

        gpio_config(&io_config);

	xTaskCreate(tcpServer, "tcp_server", 4096,NULL, 5, NULL);
}
