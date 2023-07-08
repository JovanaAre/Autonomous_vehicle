/*!
 * @file color_10_driver.c
 * @brief Color 10 Click Driver.
 */

#include "color_10_driver.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_it.h"
#include <stdio.h>
#include <string.h>
// ------------------------------------------------------------- PRIVATE MACROS

#define COLOR10_MIN_ORANGE_VAL                              0.096
#define COLOR10_MAX_ORANGE_VAL                              0.154

#define COLOR10_MIN_RED_VAL                                 0.045
#define COLOR10_MAX_RED_VAL                                 0.095

#define COLOR10_MIN_PINK_VAL                                0.920
#define COLOR10_MAX_PINK_VAL                                0.98

#define COLOR10_MIN_PURPLE_VAL                              0.6201
#define COLOR10_MAX_PURPLE_VAL                              0.919

#define COLOR10_MIN_BLUE_VAL                                0.521
#define COLOR10_MAX_BLUE_VAL                                0.6200

#define COLOR10_MIN_CYAN_VAL                                0.430
#define COLOR10_MAX_CYAN_VAL                                0.520

#define COLOR10_MIN_GREEN_VAL                               0.300
#define COLOR10_MAX_GREEN_VAL                               0.429

#define COLOR10_MIN_YELLOW_VAL                              0.155
#define COLOR10_MAX_YELLOW_VAL                              0.299

#define BUFF_SIZE                                           256
#define MIN_BUFF_SIZE                                       2

// ---------------------------------------------- PRIVATE FUNCTION DECLARATIONS

static float drv_max_color(float color_a, float color_b);
static float drv_min_color(float color_a, float color_b);
static float drv_rgb_to_hsl(float red, float green, float blue) ;

// ------------------------------------------------ PUBLIC FUNCTION DEFINITIONS

#if 0
COLOR10_RETVAL color10_init ( color10_cfg_t *cfg )
{
    i2c_master_config_t i2c_cfg;

    i2c_master_configure_default( &i2c_cfg );
    i2c_cfg.speed  = cfg->i2c_speed;
    i2c_cfg.scl    = cfg->scl;
    i2c_cfg.sda    = cfg->sda;

    ctx->slave_address = cfg->i2c_address;

    if (  i2c_master_open( &ctx->i2c, &i2c_cfg ) == I2C_MASTER_ERROR )
    {
        return COLOR10_INIT_ERROR;
    }

    i2c_master_set_slave_address( &ctx->i2c, ctx->slave_address );
    i2c_master_set_speed( &ctx->i2c, cfg->i2c_speed );

    return COLOR10_OK;
}
#endif

void color10_generic_write(color_10_t *cfg, uint8_t reg, uint8_t *data_buf, uint8_t len)
{
    uint8_t tx_buff[BUFF_SIZE];

    for (uint8_t count = 0; count <= len-1; count++ )
    {
        tx_buff[count] = data_buf[count];
    }

    HAL_I2C_Mem_Write(cfg->i2c_handle, cfg->i2c_address << 1, reg, sizeof(reg), tx_buff, len, HAL_MAX_DELAY );
}

uint16_t color10_generic_read(color_10_t *cfg, uint8_t cmd_addr)
{
    uint8_t rx_buff[MIN_BUFF_SIZE];
    uint16_t temp_data;
    uint8_t r_addr = (cfg->i2c_address << 1) + 1;
    HAL_I2C_Mem_Read(cfg->i2c_handle, r_addr, cmd_addr, sizeof(cmd_addr), rx_buff, sizeof(rx_buff), HAL_MAX_DELAY);
    temp_data = rx_buff[1];
    temp_data = (temp_data << 8);
    temp_data = temp_data | rx_buff[0];

    return temp_data;
}

void color10_config(color_10_t *cfg, uint16_t cfg_data)
{
    uint8_t tx_buff[MIN_BUFF_SIZE];

    uint8_t reg_id = COLOR10_CMD_REG_CFG;
    tx_buff[0] = cfg_data;
    tx_buff[1] = (cfg_data >> 8);

    uint8_t message[50] = {0};

    sprintf((char *) message, "Control data is: %d and %d and register\n", tx_buff[0], tx_buff[1]);
    HAL_UART_Transmit(cfg->uart_handle, message, strlen((char *) message), HAL_MAX_DELAY);
    HAL_Delay(100);
    HAL_I2C_Mem_Write(cfg->i2c_handle, cfg->i2c_address << 1, reg_id, sizeof(reg_id), tx_buff, sizeof(tx_buff), HAL_MAX_DELAY);
    HAL_Delay(100);
}

uint8_t color10_get_id(color_10_t *cfg)
{
    uint8_t rx_buff[MIN_BUFF_SIZE];
    uint8_t tx_tmp;
    uint8_t r_addr = (cfg->i2c_address << 1) + 1;
    tx_tmp = COLOR10_CMD_REG_ID;


    HAL_I2C_Mem_Read(cfg->i2c_handle, r_addr, tx_tmp, sizeof(tx_tmp), rx_buff, sizeof(rx_buff), HAL_MAX_DELAY);


    return rx_buff[0];
}

float color10_read_color_ratio(color_10_t *cfg, uint8_t color_cmd_reg)
{
    uint16_t color_data;
    uint16_t clear_data;
    float color_ratio;

    color_data = color10_generic_read(cfg, color_cmd_reg);
    clear_data = color10_generic_read(cfg, COLOR10_CMD_REG_C);

    color_ratio = ((float) color_data / (float) clear_data);

    return color_ratio;
}

float color10_get_color_value(color_10_t *cfg)
{
    float red_ratio;
    float green_ratio;
    float blue_ratio;

    float color_data;
    float sum_color;

   for (uint8_t count = 0; count < 16; count++ )
    {
        red_ratio = color10_read_color_ratio(cfg, COLOR10_CMD_REG_R);
        green_ratio = color10_read_color_ratio(cfg, COLOR10_CMD_REG_G);
        blue_ratio = color10_read_color_ratio(cfg, COLOR10_CMD_REG_B);

        color_data = drv_rgb_to_hsl(red_ratio, green_ratio, blue_ratio);
        sum_color = sum_color + color_data;
    }

    sum_color = sum_color / 16.0;

    return sum_color;
}

uint8_t color10_get_color(float color_value)
{
    if ((color_value >= COLOR10_MIN_ORANGE_VAL) && (color_value <= COLOR10_MAX_ORANGE_VAL))
     {
         return COLOR10_COLOR_ORANGE;
     }
     else if ((color_value >= COLOR10_MIN_RED_VAL) && (color_value <= COLOR10_MAX_RED_VAL))
     {
         return COLOR10_COLOR_RED;
     }
     else if ((color_value >= COLOR10_MIN_PINK_VAL) && (color_value <= COLOR10_MAX_PINK_VAL))
     {
         return COLOR10_COLOR_PINK;
     }
     else if ((color_value >= COLOR10_MIN_PURPLE_VAL) && (color_value <= COLOR10_MAX_PURPLE_VAL))
     {
         return COLOR10_COLOR_PURPLE;
     }
     else if ((color_value >= COLOR10_MIN_BLUE_VAL) && (color_value <= COLOR10_MAX_BLUE_VAL))
     {
         return COLOR10_COLOR_BLUE;
     }
     else if ((color_value >= COLOR10_MIN_CYAN_VAL) && (color_value < COLOR10_MAX_CYAN_VAL))
     {
         return COLOR10_COLOR_CYAN;
     }
     else if ((color_value >= COLOR10_MIN_GREEN_VAL) && (color_value <= COLOR10_MAX_GREEN_VAL))
     {
         return COLOR10_COLOR_GREEN;
     }
     else if ((color_value >= COLOR10_MIN_YELLOW_VAL) && (color_value <= COLOR10_MAX_YELLOW_VAL))
     {
         return COLOR10_COLOR_YELLOW;
     }

     return COLOR10_COLOR_OTHER;
}

// ----------------------------------------------- PRIVATE FUNCTION DEFINITIONS

static float drv_max_color(float color_a, float color_b)
{
    if (color_a > color_b)
    {
        return color_a;
    }
    else
    {
        return color_b;
    }
}

static float drv_min_color(float color_a, float color_b)
{
    if (color_a < color_b)
    {
        return color_a;
    }
    else
    {
        return color_b;
    }
}

static float drv_rgb_to_hsl(float red, float green, float blue)
{
    float fmax;
    float fmin;
    float hue_value;
    float saturation_value;

    fmax = drv_max_color(drv_max_color(red, green), blue);
    fmin = drv_min_color(drv_min_color(red, green), blue);


    if (fmax > 0)
    {
        saturation_value = (fmax - fmin) / fmax;
    }
    else
    {
        saturation_value = 0;
    }

    if (saturation_value == 0)
    {
        hue_value = 0;
    }
    else
    {
        if (fmax == red)
        {
             hue_value = (green - blue) / (fmax - fmin);
        }
        else if (fmax == green)
        {
             hue_value = 2 + (blue - red) / (fmax - fmin);
        }
        else
        {
             hue_value = 4 + (red - green) / (fmax - fmin);
        }
        hue_value = hue_value / 6;
        if (hue_value < 0)
        {
             hue_value = hue_value + 1;
        }
    }
    return hue_value;
}

// ------------------------------------------------------------------------- END
