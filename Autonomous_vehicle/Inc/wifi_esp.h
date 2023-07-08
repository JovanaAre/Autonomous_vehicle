/** @file wifi_esp.h
* @brief This file contains API for WiFi ESP Click Driver.
*/ 

#ifndef INC_WIFI_ESP_H_
#define INC_WIFI_ESP_H_

#include <string.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"

#define UART_TIMEOUT		2000
#define TIME_DELAY			200
#define BUFFER_SIZE			1024

/**
 * @addtogroup wifi_set
 * @{
 */

/**
 * @brief WiFi ESP setting.
 * Settings of WiFi ESP Click driver.
 */

#define CMD_RST				"AT+RST\r\n"
#define CMD_ECHO			"ATE%d\r\n"
#define CMD_AT_MODE			"AT+CWMODE=%d\r\n"
#define CMD_AT_CWJAP		"AT+CWJAP=\"%s\",\"%s\"\r\n"
#define CMD_LIST_AP			"AT+CWLAP\r\n"
#define CMD_GET_STATE		"AT+CWSTATE?\r\n"
#define CMD_TCP_CONNECT		"AT+CIPSTART=\"TCP\",\"%s\",%d\r\n"
#define CMD_VER_INFO		"AT+GMR\r\n"
#define CMD_IP_ADDR			"AT+CIPSTA?\r\n"
#define CMD_MULT_CONN		"AT+CIPMUX=%d\r\n"
#define CMD_SEND_DATA		"AT+CIPSEND=%d\r\n"

#define MESSAGE_OK			"OK"
#define MESSAGE_ERROR		"ERROR"
#define MESSAGE_BUSY 		"busy p..."
#define MESSAGE_CONNECT		"CONNECT"

#define MODE_STATION		1
#define MODE_AP				2
#define MODE_AP_STATION		3

#define SYSLOG_DISABLE		0
#define SYSLOG_ENABLE		1

#define ECHO_OFF			0
#define ECHO_ON				1

#define MUX_DISABLED		0
#define MUX_ENABLED			1

typedef struct
{
	// UART for communication with WiFi board
	// 
	UART_HandleTypeDef *uart_wifi;

	// UART for printing on console
	// 
	UART_HandleTypeDef *uart_prnt;

	char *ssid;
	char *password;

	uint8_t mode;
	uint8_t echo;
	uint8_t mux_conn;

} wifi_esp_t;

// Public API functions
//

/**
 * @brief WiFi ESP initialization function.
 * @details This function executes initialization of WiFi ESP Click board.
 * @param[in] ctx : Click context object.
 * See #wifi_esp_t object definition for detailed explanation.
 *
 * See #HAL_StatusTypeDef definition for detailed explanation.
 * @note None.
 *
 * @endcode
 */
HAL_StatusTypeDef wifi_init(wifi_esp_t * ctx);

/**
 * @brief WiFi ESP connection function.
 * @details This function sets mode, send SSID and password
 * and enables multiple connections.
 * @param[in] ctx : Click context object.
 * See #wifi_esp_t object definition for detailed explanation.
 *
 * See #HAL_StatusTypeDef definition for detailed explanation.
 * @note None.
 *
 * @endcode
 */
HAL_StatusTypeDef wifi_connect(wifi_esp_t * ctx);

/**
 * @brief WiFi ESP hardware version function.
 * @details This function writes hardware version of WiFi ESP Click board.
 * @param[in] ctx : Click context object.
 * See #wifi_esp_t object definition for detailed explanation.
 *
 * See #HAL_StatusTypeDef definition for detailed explanation.
 * @note None.
 *
 * @endcode
 */
HAL_StatusTypeDef wifi_hw_version(wifi_esp_t * ctx);

/**
 * @brief WiFi ESP getting IP function.
 * @details This function retrieves IP address.
 * @param[in] ctx : Click context object.
 * See #wifi_esp_t object definition for detailed explanation.
 *
 * See #HAL_StatusTypeDef definition for detailed explanation.
 * @note None.
 *
 * @endcode
 */
HAL_StatusTypeDef wifi_get_ip(wifi_esp_t * ctx);

/**
 * @brief WiFi ESP reset function.
 * @details This function resets WiFi ESP.
 * @param[in] ctx : Click context object.
 * See #wifi_esp_t object definition for detailed explanation.
 *
 * See #HAL_StatusTypeDef definition for detailed explanation.
 * @note None.
 *
 * @endcode
 */
HAL_StatusTypeDef wifi_rst(wifi_esp_t * ctx);

/**
 * @brief WiFi ESP listing AP function.
 * @details This function lists AP(Access Points).
 * @param[in] ctx : Click context object.
 * See #wifi_esp_t object definition for detailed explanation.
 *
 * See #HAL_StatusTypeDef definition for detailed explanation.
 * @note None.
 *
 * @endcode
 */
HAL_StatusTypeDef wifi_list_ap(wifi_esp_t * ctx);

/**
 * @brief WiFi ESP TCP connection function.
 * @details This function implements TCP connection of WiFi ESP with host.
 * @param[in] ctx : Click context object.
 * @param[in] host : Host IP address.
 * @param[in] port : Port number.
 * See #wifi_esp_t object definition for detailed explanation.
 *
 * See #HAL_StatusTypeDef definition for detailed explanation.
 * @note None.
 *
 * @endcode
 */
HAL_StatusTypeDef wifi_tcp_connect(wifi_esp_t * ctx, char * host, uint16_t port);

/**
 * @brief WiFi ESP send data function.
 * @details This function sends data to WiFi ESP.
 * @param[in] ctx : Click context object.
 * @param[in] data : Data array.
 * @param[in] size : Size of data array.
 * See #wifi_esp_t object definition for detailed explanation.
 *
 * See #HAL_StatusTypeDef definition for detailed explanation.
 * @note None.
 *
 * @endcode
 */
HAL_StatusTypeDef wifi_send_data(wifi_esp_t * ctx, char * data, uint16_t size);

/**
 * @brief WiFi ESP send data function.
 * @details This function gets data from WiFi ESP.
 * @param[in] ctx : Click context object.
 * @param[in] data : Data array.
 * @param[in] size : Size of data array.
 * See #wifi_esp_t object definition for detailed explanation.
 *
 * See #HAL_StatusTypeDef definition for detailed explanation.
 * @note None.
 *
 * @endcode
 */
HAL_StatusTypeDef wifi_get_data(wifi_esp_t * ctx, char * data, uint16_t size);

#endif /* INC_WIFI_ESP_H_ */
/*** end of file ***/
