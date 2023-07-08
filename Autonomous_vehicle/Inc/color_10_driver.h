/*
 * color_10_driver.h
 */

#ifndef INC_COLOR_10_DRIVER_H_
#define INC_COLOR_10_DRIVER_H_

#include "stm32f4xx_hal.h"

/*!
 * \file
 *
 * \brief This file contains API for Color 10 Click driver.
 *
 * \addtogroup color10 Color 10 Click Driver
 * @{
 */
// ----------------------------------------------------------------------------

// -------------------------------------------------------------- PUBLIC MACROS
/**
 * \defgroup macros Macros
 * \{
 */

/**
 * \defgroup map_mikrobus MikroBUS
 * \{
 */
#if 0
#define COLOR10_MAP_MIKROBUS( cfg, mikrobus ) \
  cfg.scl  = MIKROBUS( mikrobus, MIKROBUS_SCL ); \
  cfg.sda  = MIKROBUS( mikrobus, MIKROBUS_SDA )
#endif
/** \} */

/**
 * \defgroup error_code Error Code
 * \{
 */
#define COLOR10_RETVAL  uint8_t

#define COLOR10_OK           	0x00
#define COLOR10_INIT_ERROR   	0xFF
#define COLOR10_I2C_SPEED      	100000
/** \} */

#define       COLOR10_SLAVE_ADDR                              0x10

#define       COLOR10_CMD_REG_CFG                             0x00
#define       COLOR10_CMD_REG_C                               0x04
#define       COLOR10_CMD_REG_R                               0x05
#define       COLOR10_CMD_REG_G                               0x06
#define       COLOR10_CMD_REG_B                               0x07
#define       COLOR10_CMD_REG_IR                              0x08
#define       COLOR10_CMD_REG_ID                              0x0C

#define       COLOR10_DEVICE_ID                               0x28

#define       COLOR10_COLOR_ORANGE                            0x11
#define       COLOR10_COLOR_RED                               0x22
#define       COLOR10_COLOR_PINK                              0x33
#define       COLOR10_COLOR_PURPLE                            0x44
#define       COLOR10_COLOR_BLUE                              0x55
#define       COLOR10_COLOR_CYAN                              0x66
#define       COLOR10_COLOR_GREEN                             0x77
#define       COLOR10_COLOR_YELLOW                            0x88
#define       COLOR10_COLOR_OTHER                             0x99

#define       COLOR10_CFG_HIGH_DYNAMIC_RANGE_1_3            0x0040
#define       COLOR10_CFG_HIGH_DYNAMIC_RANGE_1              0x0000
#define       COLOR10_CFG_INTEGRATION_TIME_SETT_50_MS       0x0000
#define       COLOR10_CFG_INTEGRATION_TIME_SETT_100_MS      0x0010
#define       COLOR10_CFG_INTEGRATION_TIME_SETT_200_MS      0x0020
#define       COLOR10_CFG_INTEGRATION_TIME_SETT_400_MS      0x0030
#define       COLOR10_CFG_AUTO_MODE                         0x0000
#define       COLOR10_CFG_FORCE_MODE                        0x0008
#define       COLOR10_CFG_TRIGGER_NO                        0x0000
#define       COLOR10_CFG_TRIGGER_ONE_TIME                  0x0004
#define       COLOR10_CFG_POWER_ON                          0x0000
#define       COLOR10_CFG_SHUT_DOWN                         0x8001
#define       COLOR10_CFG_POWER_ON_G_C_IR                   0x4000
#define       COLOR10_CFG_GAIN_1_X1                         0x0000
#define       COLOR10_CFG_GAIN_1_X2                         0x1000
#define       COLOR10_CFG_GAIN_1_X4                         0x2000
#define       COLOR10_CFG_GAIN_2_X1_2                       0x0C00
#define       COLOR10_CFG_GAIN_2_X1                         0x0000
#define       COLOR10_CFG_GAIN_2_X2                         0x0400
#define       COLOR10_CFG_GAIN_2_X4                         0x0800

/** \} */ // End group macro
// --------------------------------------------------------------- PUBLIC TYPES
/**
 * \defgroup type Types
 * \{
 */


/**
 * @brief Click configuration structure definition.
 */
typedef struct
{
    // Communication GPIO pins
	I2C_HandleTypeDef *i2c_handle;
	UART_HandleTypeDef *uart_handle;

	GPIO_TypeDef *scl_pin_base;
	GPIO_TypeDef *sda_pin_base;

    uint32_t scl;
    uint32_t sda;

    uint32_t i2c_speed;
    uint8_t i2c_address;

} color_10_t;

// static uint8_t data_sent_flag = 0;

/** \} */ // End types group
// ----------------------------------------------- PUBLIC FUNCTION DECLARATIONS

/**
 * \defgroup public_function Public function
 * \{
 */

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @brief Generic write function.
 *
 * @param ctx          Click object.
 * @param reg          Register address.
 * @param data_buf     Data buf to be written.
 * @param len          Number of the bytes in data buf.
 *
 * @description This function writes data to the desired register.
 */
void color10_generic_write(color_10_t *cfg, uint8_t reg, uint8_t *data_buf, uint8_t len);

/**
 * @brief Generic read function.
 *
 * @param ctx          Click object.
 * @param cmd_addr     Command address.
 *
 * @returns            Data read from given address.
 *
 * @description This function reads data from the desired register.
 */
uint16_t color10_generic_read(color_10_t *cfg, uint8_t cmd_addr);

/**
 * @brief Configuration function
 *
 * @param ctx          Click object.
 * @param cfg_data     Configuration data.
 *
 * @description        This function configures the click according to the config data.
 */
void color10_config(color_10_t *cfg, uint16_t cfg_data);

/**
 * @brief ID retrieval function
 *
 * @param ctx          Click object.
 *
 * @returns            ID
 *
 * @description        This function returns the ID.
 */
uint8_t color10_get_id(color_10_t *cfg);

/**
 * @brief Color ratio function
 *
 * @param ctx           Click object.
 * @param color_cmd_reg Color command register.
 *
 * @returns            Color ratio
 *
 * @description        This function reads the color ratio from given register.
 */
float color10_read_color_ratio(color_10_t *cfg, uint8_t color_cmd_reg);

/**
 * @brief Color value function
 *
 * @param ctx           Click object.
 *
 * @returns             Color value.
 *
 * @description        This function calculates the color value.
 */
float color10_get_color_value(color_10_t *cfg);

/**
 * @brief Color retrieval function
 *
 * @param color_value          Color value
 *
 * @returns                    Color ID macro
 *
 * @description        This function identifies the color using the color value.
 */
uint8_t color10_get_color(float color_value);

/**
 *
 */
//void WS2812_send(TIM_HandleTypeDef * htim, int red, int green, int blue);


#ifdef __cplusplus
}
#endif

#endif /* INC_COLOR_10_DRIVER_H_ */
