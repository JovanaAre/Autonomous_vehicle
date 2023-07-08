/*!
 * @file wifi_esp.c
 * @brief WiFi ESP Click Driver.
 */

#include "wifi_esp.h"

// ---------------------------------------------- PRIVATE FUNCTION DECLARATION
// 
static HAL_StatusTypeDef send_cmd(wifi_esp_t * ctx, char * msg, char * cmd);

// ------------------------------------------------ PUBLIC FUNCTION DEFINITIONS
// 
HAL_StatusTypeDef wifi_init(wifi_esp_t * ctx)
{
	char msg[BUFFER_SIZE];
	char cmd[BUFFER_SIZE];

	sprintf(msg, "Set echo\r\n");
	sprintf(cmd, CMD_ECHO, ctx->echo);

	return send_cmd(ctx, msg, cmd);
}

HAL_StatusTypeDef wifi_connect(wifi_esp_t * ctx)
{
	HAL_StatusTypeDef status;
	char msg[BUFFER_SIZE];
	char cmd[BUFFER_SIZE];

	memset(msg, 0, BUFFER_SIZE);
	memset(cmd, 0, BUFFER_SIZE);

	// Set mode
	// 
	sprintf(msg, "Set mode\r\n");
	sprintf(cmd, CMD_AT_MODE, ctx->mode);
	status = send_cmd(ctx, msg, cmd);
	if (status != HAL_OK)
		return status;

	memset(msg, 0, BUFFER_SIZE);
	memset(cmd, 0, BUFFER_SIZE);

	// Send SSID and password
	// 
	sprintf(msg, "Send SSID\r\n");
	sprintf(cmd, CMD_AT_CWJAP, ctx->ssid, ctx->password);
	status = send_cmd(ctx, msg, cmd);
	if (status != HAL_OK)
		return status;

	memset(msg, 0, BUFFER_SIZE);
	memset(cmd, 0, BUFFER_SIZE);

	// Enable multiple connections
	// 
	sprintf(msg, "Enable multiple connections\r\n");
	sprintf(cmd, CMD_MULT_CONN, ctx->mux_conn);
	status = send_cmd(ctx, msg, cmd);
	if (status != HAL_OK)
		return status;

	return HAL_OK;
}

HAL_StatusTypeDef wifi_hw_version(wifi_esp_t * ctx)
{
	char *msg = "HW version\r\n";
	char cmd[BUFFER_SIZE];

	sprintf(cmd, CMD_VER_INFO);

	return send_cmd(ctx, msg, cmd);
}

HAL_StatusTypeDef wifi_get_ip(wifi_esp_t * ctx)
{
	char *msg = "Get IP\r\n";
	char cmd[BUFFER_SIZE];

	sprintf(cmd, CMD_IP_ADDR);

	return send_cmd(ctx, msg, cmd);
}

HAL_StatusTypeDef wifi_rst(wifi_esp_t * ctx)
{
	char *msg = "Reset\r\n";
	char cmd[BUFFER_SIZE];

	sprintf(cmd, CMD_RST);

	return send_cmd(ctx, msg, cmd);
}

HAL_StatusTypeDef wifi_list_ap(wifi_esp_t * ctx)
{
	char *msg = "List AP\r\n";
	char cmd[BUFFER_SIZE];

	sprintf(cmd, CMD_LIST_AP);

	return send_cmd(ctx, msg, cmd);
}

HAL_StatusTypeDef wifi_tcp_connect(wifi_esp_t * ctx, char * host, uint16_t port)
{
	char *msg = "TCP connect\r\n";
	char cmd[BUFFER_SIZE];

	sprintf(cmd, CMD_TCP_CONNECT, host, port);

	return send_cmd(ctx, msg, cmd);
}

HAL_StatusTypeDef wifi_send_data(wifi_esp_t * ctx, char * data, uint16_t size)
{
	char cmd[BUFFER_SIZE];

	sprintf(cmd, CMD_SEND_DATA, size);

	HAL_StatusTypeDef status = send_cmd(ctx, "", cmd);

	if (status != HAL_OK)
		return status;

	HAL_UART_Transmit(ctx->uart_wifi, (uint8_t *) data, size, UART_TIMEOUT);
	HAL_UART_Transmit(ctx->uart_prnt, (uint8_t *) data, size, UART_TIMEOUT);

	return HAL_OK;
}

HAL_StatusTypeDef wifi_get_data(wifi_esp_t * ctx, char * data, uint16_t size)
{
	HAL_UART_Receive(ctx->uart_wifi, (uint8_t *) data, size, UART_TIMEOUT);
	HAL_UART_Transmit(ctx->uart_prnt, (uint8_t *) data, size, UART_TIMEOUT);

	return HAL_OK;
}

static HAL_StatusTypeDef send_cmd(wifi_esp_t * ctx, char * msg, char * cmd)
{
	HAL_StatusTypeDef status;
	char *res = NULL;
	uint8_t data[BUFFER_SIZE];

	do
	{
		HAL_UART_Transmit(ctx->uart_prnt, (uint8_t *) msg, strlen((char *) msg), UART_TIMEOUT);

		status = HAL_UART_Transmit(ctx->uart_wifi, (uint8_t *) cmd, strlen((char *) cmd), UART_TIMEOUT);
		if (status != HAL_OK)
			return status;

		memset(data, 0, BUFFER_SIZE);
		HAL_UART_Receive(ctx->uart_wifi, (uint8_t *) data, BUFFER_SIZE, UART_TIMEOUT);
		HAL_UART_Transmit(ctx->uart_prnt, data, strlen((char *) data), UART_TIMEOUT);

		res = strstr((char *) data, MESSAGE_OK);

		if (res == NULL)
		{
			res = strstr((char *) data, MESSAGE_ERROR);
			if (res != NULL)
				return HAL_ERROR;

			do
			{
				memset(data, 0, BUFFER_SIZE);
				HAL_UART_Receive(ctx->uart_wifi, (uint8_t *) data, BUFFER_SIZE, UART_TIMEOUT);
				HAL_UART_Transmit(ctx->uart_prnt, data, strlen((char *) data), UART_TIMEOUT);

				res = strstr((char *) data, MESSAGE_ERROR);
				if (res != NULL)
					return HAL_ERROR;

				res = strstr((char *) data, MESSAGE_BUSY);
				if (res != NULL)
				{
					HAL_Delay(100);
					continue;
				}

				res = strstr((char *) data, MESSAGE_OK);

				HAL_Delay(100);

			} while (res == NULL);

		}

	} while (res == NULL);

	return HAL_OK;
}

