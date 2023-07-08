/*!
 * @file lightranger8.c
 * @brief LightRanger 8 Click Driver.
 */

#include "light_ranger.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_it.h"

// ------------------------------------------------------------- PRIVATE MACROS

#define DEV_IDENTIFICATION_MODEL_ID                                     0xEA
#define DEV_REGISTER_READ_LSB                                           0x01
#define DEV_REGISTER_BITMASK_LSB                                        0xFE

#define DEV_SOFT_RESET_CMD_SET                                          0x00
#define DEV_SOFT_RESET_CMD_CLEAR                                        0x01

#define DEV_SYS_INT_CLEAR_BIT                                           0x01

#define DEV_CMD_RANGING_ENABLE                                          0x40
#define DEV_CMD_RANGING_DISABLE                                         0x00

#define DEV_DATA_READY_BIT                                              0x03

#define DEV_REGISTER_BITMASK_10_BIT                                     0x3FF
#define DEV_REGISTER_BITMASK_30_BIT_SHIFT                               0x40000000
#define DEV_REGISTER_BITMASK_12_BIT_SHIFT                               0x800
#define DEV_REGISTER_CLEAR_LSBYTE                                       0xFFFFFF00
#define DEV_REGISTER_BITMASK_LSBYTE                                     0xFF
#define DEV_REGISTER_BITMASK_U16                                        0x00FF
#define DEV_REGISTER_BITMASK_U32                                        0x000000FF

#define DEV_REGISTER_STATUS_BITMASK                                     0x1F

#define DEV_REGISTER_ROI_GLO_XY_SIZE_NIBBLE_L                           0x0F
#define DEV_REGISTER_ROI_GLO_XY_SIZE_NIBBLE_H                           0xF0

#define DEV_REGISTER_ROI_NIBBLE_H_OFFSET                                4

#define DEV_PART_TO_PART_RANGE_OFFSET_MULT                              4

#define DEV_MM_CONF_INNER_OFFSET_RESET                                  0x0000
#define DEV_MM_CONF_OUTER_OFFSET_RESET                                  0x0000
#define DEV_PART_TO_PART_RANGE_OFFSET_RESET                             0x0000

#define DEV_FIRMWARE_READ_TIMEOUT                                       100

#define DEV_ROI_DEFAULT_CENTER_VAL                                      199
#define DEV_ROI_MAX_VAL                                                 16
#define DEV_ROI_CENTER_EDGE_VAL                                         10

#define DEV_MEASUREMENT_BUDGET_MIN                                      4528
#define DEV_MEASUREMENT_BUDGET_MAX                                      1100000
#define DEV_MEASUREMENT_PHASECAL_TIMEOUT                                1000
#define DEV_MEASUREMENT_PHASECAL_TIMEOUT_MAX                            0xFF

#define DEV_CLOCK_COEFF                                                 1.065
#define DEV_MACRO_PERIOD_COEFF                                          2304

// Default config macros
#define DEV_DEFAULT_INT_POL                                             0x02
#define DEV_DEFAULT_PULSE_WIDTH_NS                                      0x08
#define DEV_DEFAULT_AMBIENT_WIDTH_NS                                    0x10
#define DEV_DEFAULT_CC_VALID_HEIGHT_MM                                  0x01
#define DEV_DEFAULT_RAN_IGN_VAL_HEIGHT_MM                               0xFF
#define DEV_DEFAULT_RAN_MIN_CLIP                                        0x00
#define DEV_DEFAULT_CONSISTENCY_CHECK_TOL                               0x02
#define DEV_DEFAULT_THRESH_RATE_HIGH                                    0x0000
#define DEV_DEFAULT_THRESH_RATE_LOW                                     0x0000
#define DEV_DEFAULT_DSS_CONF_AP_ATT                                     0x38
#define DEV_DEFAULT_RAN_CONF_SIGMA_THRESH                               0x0168
#define DEV_DEFAULT_RAN_CONF_MIN_CNT_RATE_LIM_MCPS                      0x00C0
#define DEV_DEFAULT_GROUPED_PARAM_HOLD_0                                0x01
#define DEV_DEFAULT_GROUPED_PARAM_HOLD_1                                0x01
#define DEV_DEFAULT_SD_CONF_QUANTIFIER                                  0x02
#define DEV_DEFAULT_GROUPED_PARAM_HOLD                                  0x00
#define DEV_DEFAULT_SEED_CONF                                           0x01
#define DEV_DEFAULT_SEQUENCE_CONF                                       0x8B
#define DEV_DEFAULT_DSS_CONF_MAN_EFF_SPADS_SELECT                       0xC800
#define DEV_DEFAULT_DSS_CONF_ROI_MODE_CONTROL                           0x02

// Distance mode config macros
#define DEV_SHORT_MODE_RAN_CONF_PERIOD_A                                0x07
#define DEV_SHORT_MODE_RAN_CONF_PERIOD_B                                0x05
#define DEV_SHORT_MODE_RAN_CONF_VAL_PH_H                                0x38
#define DEV_SHORT_MODE_SD_CONFIG_WOI_SD0                                0x07
#define DEV_SHORT_MODE_SD_CONFIG_WOI_SD1                                0x05
#define DEV_SHORT_MODE_SD_CONF_IN_PH_SD0                                0x06
#define DEV_SHORT_MODE_SD_CONF_IN_PH_SD1                                0x06

#define DEV_MEDIUM_MODE_RAN_CONF_PERIOD_A                               0x0B
#define DEV_MEDIUM_MODE_RAN_CONF_PERIOD_B                               0x09
#define DEV_MEDIUM_MODE_RAN_CONF_VAL_PH_H                               0x78
#define DEV_MEDIUM_MODE_SD_CONFIG_WOI_SD0                               0x0B
#define DEV_MEDIUM_MODE_SD_CONFIG_WOI_SD1                               0x09
#define DEV_MEDIUM_MODE_SD_CONF_IN_PH_SD0                               0x0A
#define DEV_MEDIUM_MODE_SD_CONF_IN_PH_SD1                               0x0A

#define DEV_LONG_MODE_RAN_CONF_PERIOD_A                                 0x0F
#define DEV_LONG_MODE_RAN_CONF_PERIOD_B                                 0x0D
#define DEV_LONG_MODE_RAN_CONF_VAL_PH_H                                 0xB8
#define DEV_LONG_MODE_SD_CONFIG_WOI_SD0                                 0x0F
#define DEV_LONG_MODE_SD_CONFIG_WOI_SD1                                 0x0D
#define DEV_LONG_MODE_SD_CONF_IN_PH_SD0                                 0x0E
#define DEV_LONG_MODE_SD_CONF_IN_PH_SD1                                 0x0E

// ---------------------------------------------- PRIVATE FUNCTION DECLARATIONS

// Calculate time functions
static uint32_t calc_macro_period(light_ranger_8_t *cfg, uint8_t vcsel_period);
static uint32_t timeout_microseconds_to_mclks(uint32_t timeout_us, uint32_t macro_period_us);
static uint16_t encode_timeout(uint32_t timeout_mclks);
static uint32_t timeout_mclks_to_microseconds(uint32_t timeout_mclks, uint32_t macro_period_us);
static uint32_t decode_timeout(uint16_t reg_val);

// Variety of functions for read/write data
static void write_data_u8(light_ranger_8_t *cfg, uint16_t addr, uint8_t _data);
static void write_data_u16(light_ranger_8_t *cfg, uint16_t addr, uint16_t _data);
static void write_data_u32(light_ranger_8_t *cfg, uint16_t addr, uint32_t _data);
static uint8_t read_data_u8(light_ranger_8_t *cfg, uint16_t addr);
static uint16_t read_data_u16(light_ranger_8_t *cfg, uint16_t addr);

// Driver delays
static void delay_power_full_on(light_ranger_8_t *cfg);
static void delay_power_up(light_ranger_8_t *cfg);
static void delay_config_set(light_ranger_8_t *cfg);
static void delay_soft_reset(light_ranger_8_t *cfg);
static void delay_data_ready(light_ranger_8_t *cfg);

// ------------------------------------------------ PUBLIC FUNCTION DEFINITIONS

LIGHTRANGER8_RETVAL lightranger8_default_cfg(light_ranger_8_t *cfg)
{
    uint16_t model_id;
    uint16_t result;
    // uint8_t reg_val;
    uint8_t status;
    uint8_t cnt = 0;

    // Click default configuration

    HAL_GPIO_WritePin (cfg->en_pin_base, cfg->en, GPIO_PIN_SET);

    delay_power_up(cfg);
    delay_power_up(cfg);

    model_id = read_data_u8(cfg, LIGHTRANGER8_IDENTIFICATION_MODEL_ID);
    uint8_t test1[50] = {0};
    sprintf((char *) test1, "LR: Model_ID value = %x \n", model_id);
    HAL_UART_Transmit(cfg->uart_handle, test1, sizeof(test1), HAL_MAX_DELAY);

    if (model_id != DEV_IDENTIFICATION_MODEL_ID)
    {
        uint8_t error[] = "LR: Device ID error \n";
        HAL_UART_Transmit(cfg->uart_handle, error, sizeof(error), HAL_MAX_DELAY);
        return LIGHTRANGER8_RESP_DEVICE_ERROR_ID_IS_NOT_VALID;
    }

    delay_power_up(cfg);

    lightranger8_software_reset(cfg);

    delay_power_up(cfg);
    delay_power_up(cfg);
    delay_power_up(cfg);

    status = read_data_u8(cfg, LIGHTRANGER8_FIRMWARE_SYSTEM_STATUS);

    while((status & DEV_REGISTER_READ_LSB) == 0)
    {
        status = read_data_u8(cfg, LIGHTRANGER8_FIRMWARE_SYSTEM_STATUS);
        delay_config_set(cfg);
        delay_config_set(cfg);

        if (cnt++ == DEV_FIRMWARE_READ_TIMEOUT)
        {
            uint8_t error1[] = "LR: DEV_FIRMWARE error \n ";
            HAL_UART_Transmit(cfg->uart_handle, error1, sizeof(error1), HAL_MAX_DELAY);
            return LIGHTRANGER8_RESP_FIRMWARE_TIMEOUT_ERROR;
        }
    }

    result = read_data_u16(cfg, LIGHTRANGER8_PAD_I2C_HV_EXTSUP_CONFIG);
    result = (result & DEV_REGISTER_BITMASK_LSB ) | DEV_REGISTER_READ_LSB;

    write_data_u16(cfg, LIGHTRANGER8_PAD_I2C_HV_EXTSUP_CONFIG, result);
    delay_config_set(cfg);
    cfg->fast_osc_frequency = read_data_u16(cfg, LIGHTRANGER8_OSC_MEASURED_FAST_OSC_FREQUENCY);
    cfg->osc_calibrate_val = read_data_u16(cfg, LIGHTRANGER8_RESULT_OSC_CALIBRATE_VAL);

    write_data_u8(cfg, LIGHTRANGER8_GPIO_TIO_HV_STATUS, DEV_DEFAULT_INT_POL);
    delay_config_set(cfg);
    write_data_u8(cfg, LIGHTRANGER8_SIGMA_ESTIMATOR_EFFECTIVE_PULSE_WIDTH_NS, DEV_DEFAULT_PULSE_WIDTH_NS);
    delay_config_set(cfg);
    write_data_u8(cfg, LIGHTRANGER8_SIGMA_ESTIMATOR_EFFECTIVE_AMBIENT_WIDTH_NS, DEV_DEFAULT_AMBIENT_WIDTH_NS);
    delay_config_set(cfg);
    write_data_u8(cfg, LIGHTRANGER8_ALGO_CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, DEV_DEFAULT_CC_VALID_HEIGHT_MM);
    delay_config_set(cfg);
    write_data_u8(cfg, LIGHTRANGER8_ALGO_RANGE_IGNORE_VALID_HEIGHT_MM, DEV_DEFAULT_RAN_IGN_VAL_HEIGHT_MM);
    delay_config_set(cfg);
    write_data_u8(cfg, LIGHTRANGER8_ALGO_RANGE_MIN_CLIP, DEV_DEFAULT_RAN_MIN_CLIP);
    delay_config_set(cfg);
    write_data_u8(cfg, LIGHTRANGER8_ALGO_CONSISTENCY_CHECK_TOLERANCE, DEV_DEFAULT_CONSISTENCY_CHECK_TOL);
    delay_config_set(cfg);

    // General config.
    write_data_u16(cfg, LIGHTRANGER8_SYSTEM_THRESH_RATE_HIGH, DEV_DEFAULT_THRESH_RATE_HIGH);
    delay_config_set(cfg);
    write_data_u16(cfg, LIGHTRANGER8_SYSTEM_THRESH_RATE_LOW, DEV_DEFAULT_THRESH_RATE_LOW);
    delay_config_set(cfg);
    write_data_u8(cfg, LIGHTRANGER8_DSS_CONFIG_APERTURE_ATTENUATION, DEV_DEFAULT_DSS_CONF_AP_ATT);

    // Timing config.
    delay_config_set(cfg);
    write_data_u16(cfg, LIGHTRANGER8_RANGE_CONFIG_SIGMA_THRESH, DEV_DEFAULT_RAN_CONF_SIGMA_THRESH);
    delay_config_set(cfg);
    write_data_u16(cfg, LIGHTRANGER8_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT_MCPS, DEV_DEFAULT_RAN_CONF_MIN_CNT_RATE_LIM_MCPS);

    // Dynamic config.
    delay_config_set(cfg);
    write_data_u8(cfg, LIGHTRANGER8_SYSTEM_GROUPED_PARAMETER_HOLD_0, DEV_DEFAULT_GROUPED_PARAM_HOLD_0);
    delay_config_set(cfg);
    write_data_u8(cfg, LIGHTRANGER8_SYSTEM_GROUPED_PARAMETER_HOLD_1, DEV_DEFAULT_GROUPED_PARAM_HOLD_1);
    delay_config_set(cfg);
    write_data_u8(cfg, LIGHTRANGER8_SD_CONFIG_QUANTIFIER, DEV_DEFAULT_SD_CONF_QUANTIFIER);
    delay_config_set(cfg);
    write_data_u8(cfg, LIGHTRANGER8_SYSTEM_GROUPED_PARAMETER_HOLD, DEV_DEFAULT_GROUPED_PARAM_HOLD);
    delay_config_set(cfg);
    write_data_u8(cfg, LIGHTRANGER8_SYSTEM_SEED_CONFIG, DEV_DEFAULT_SEED_CONF);
    delay_config_set(cfg);
    write_data_u8(cfg, LIGHTRANGER8_SYSTEM_SEQUENCE_CONFIG, DEV_DEFAULT_SEQUENCE_CONF);
    delay_config_set(cfg);
    write_data_u16(cfg, LIGHTRANGER8_DSS_CONFIG_MANUAL_EFFECTIVE_SPADS_SELECT, DEV_DEFAULT_DSS_CONF_MAN_EFF_SPADS_SELECT);
    delay_config_set(cfg);
    write_data_u8(cfg, LIGHTRANGER8_DSS_CONFIG_ROI_MODE_CONTROL, DEV_DEFAULT_DSS_CONF_ROI_MODE_CONTROL);
    delay_config_set(cfg);

    return LIGHTRANGER8_RESP_INIT_IS_SUCCESSFUL;
}

HAL_StatusTypeDef lightranger8_generic_write(light_ranger_8_t *cfg, uint16_t reg, uint8_t *tx_buff, uint8_t tx_len)
{
    uint8_t data_buff[257];
    uint8_t count;
    uint8_t address;

    address = cfg->i2c_address ;
    data_buff[0] = reg >> 8;
    data_buff[1] = reg & DEV_REGISTER_BITMASK_U16;

    for(count = 2; count <= tx_len + 1; count++)
    {
        data_buff[count] = tx_buff[count - 2];
    }

    return HAL_I2C_Mem_Write(cfg->i2c_handle, address, reg, sizeof(reg), tx_buff, tx_len, HAL_MAX_DELAY);

}

HAL_StatusTypeDef lightranger8_generic_read(light_ranger_8_t *cfg, uint16_t reg, uint8_t *rx_buff, uint8_t rx_len )
{
    uint8_t address;
    address = cfg->i2c_address + 1;

    return HAL_I2C_Mem_Read(cfg->i2c_handle, address, reg , sizeof(reg) ,rx_buff, rx_len, HAL_MAX_DELAY);
}

void lightranger8_software_reset(light_ranger_8_t *cfg )
{
    write_data_u8(cfg, LIGHTRANGER8_SOFT_RESET, DEV_SOFT_RESET_CMD_SET);
    delay_soft_reset(cfg);
    write_data_u8(cfg, LIGHTRANGER8_SOFT_RESET, DEV_SOFT_RESET_CMD_CLEAR);
    delay_soft_reset(cfg);
}

void lightranger8_system_interrupt_clear(light_ranger_8_t *cfg)
{
    write_data_u8(cfg, LIGHTRANGER8_SYSTEM_INTERRUPT_CLEAR, DEV_SYS_INT_CLEAR_BIT);
}

void lightranger8_start_ranging_cmd(light_ranger_8_t *cfg, uint8_t ranging_mode)
{
    if(ranging_mode == LIGHTRANGER8_RANGING_ENABLE)
    {
        write_data_u8(cfg, LIGHTRANGER8_SYSTEM_MODE_START, DEV_CMD_RANGING_ENABLE);
    }
    else
    {
        write_data_u8(cfg, LIGHTRANGER8_SYSTEM_MODE_START, DEV_CMD_RANGING_DISABLE);
    }
}

void lightranger8_start_measurement(light_ranger_8_t *cfg, uint32_t period_ms)
{
    write_data_u32(cfg, LIGHTRANGER8_SYSTEM_INTERMEASUREMENT_PERIOD, period_ms * cfg->osc_calibrate_val);
    lightranger8_system_interrupt_clear(cfg);
    lightranger8_start_ranging_cmd(cfg, LIGHTRANGER8_RANGING_ENABLE);
}

void lightranger8_stop_measurement ( light_ranger_8_t *cfg )
{
    lightranger8_start_ranging_cmd(cfg, LIGHTRANGER8_RANGING_DISABLE);
}

uint8_t lightranger8_new_data_ready(light_ranger_8_t *cfg)
{
    uint8_t tmp;

    tmp = read_data_u8(cfg, LIGHTRANGER8_GPIO_TIO_HV_STATUS);

    if (tmp != DEV_DATA_READY_BIT)
        return 0;

    return 1;
}

uint16_t lightranger8_get_distance(light_ranger_8_t *cfg)
{
    uint16_t distance;

    distance = read_data_u16(cfg, LIGHTRANGER8_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0);

    return distance;
}

uint16_t lightranger8_get_signal_rate(light_ranger_8_t *cfg)
{
    uint16_t reading;

    reading = read_data_u16(cfg, LIGHTRANGER8_PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0);

    return reading;
}

uint8_t lightranger8_set_distance_mode(light_ranger_8_t *cfg, uint8_t distance_mode)
{

    switch (distance_mode)
    {
        case LIGHTRANGER8_DISTANCE_MODE_SHORT:
        {
            write_data_u8(cfg, LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_A, DEV_SHORT_MODE_RAN_CONF_PERIOD_A);
            write_data_u8(cfg, LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_B, DEV_SHORT_MODE_RAN_CONF_PERIOD_B);
            write_data_u8(cfg, LIGHTRANGER8_RANGE_CONFIG_VALID_PHASE_HIGH, DEV_SHORT_MODE_RAN_CONF_VAL_PH_H);
            write_data_u8(cfg, LIGHTRANGER8_SD_CONFIG_WOI_SD0, DEV_SHORT_MODE_SD_CONFIG_WOI_SD0);
            write_data_u8(cfg, LIGHTRANGER8_SD_CONFIG_WOI_SD1, DEV_SHORT_MODE_SD_CONFIG_WOI_SD1);
            write_data_u8(cfg, LIGHTRANGER8_SD_CONFIG_INITIAL_PHASE_SD0, DEV_SHORT_MODE_SD_CONF_IN_PH_SD0);
            write_data_u8(cfg, LIGHTRANGER8_SD_CONFIG_INITIAL_PHASE_SD1, DEV_SHORT_MODE_SD_CONF_IN_PH_SD1);
            break;
        }
        case LIGHTRANGER8_DISTANCE_MODE_MEDIUM:
        {
            write_data_u8(cfg, LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_A, DEV_MEDIUM_MODE_RAN_CONF_PERIOD_A);
            write_data_u8(cfg, LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_B, DEV_MEDIUM_MODE_RAN_CONF_PERIOD_B);
            write_data_u8(cfg, LIGHTRANGER8_RANGE_CONFIG_VALID_PHASE_HIGH, DEV_MEDIUM_MODE_RAN_CONF_VAL_PH_H);
            write_data_u8(cfg, LIGHTRANGER8_SD_CONFIG_WOI_SD0, DEV_MEDIUM_MODE_SD_CONFIG_WOI_SD0);
            write_data_u8(cfg, LIGHTRANGER8_SD_CONFIG_WOI_SD1, DEV_MEDIUM_MODE_SD_CONFIG_WOI_SD1);
            write_data_u8(cfg, LIGHTRANGER8_SD_CONFIG_INITIAL_PHASE_SD0, DEV_MEDIUM_MODE_SD_CONF_IN_PH_SD0);
            write_data_u8(cfg, LIGHTRANGER8_SD_CONFIG_INITIAL_PHASE_SD1, DEV_MEDIUM_MODE_SD_CONF_IN_PH_SD1);
            break;
        }
        case LIGHTRANGER8_DISTANCE_MODE_LONG:
        {
            write_data_u8(cfg, LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_A, DEV_LONG_MODE_RAN_CONF_PERIOD_A);
            write_data_u8(cfg, LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_B, DEV_LONG_MODE_RAN_CONF_PERIOD_B);
            write_data_u8(cfg, LIGHTRANGER8_RANGE_CONFIG_VALID_PHASE_HIGH, DEV_LONG_MODE_RAN_CONF_VAL_PH_H);
            write_data_u8(cfg, LIGHTRANGER8_SD_CONFIG_WOI_SD0, DEV_LONG_MODE_SD_CONFIG_WOI_SD0);
            write_data_u8(cfg, LIGHTRANGER8_SD_CONFIG_WOI_SD1, DEV_LONG_MODE_SD_CONFIG_WOI_SD1);
            write_data_u8(cfg, LIGHTRANGER8_SD_CONFIG_INITIAL_PHASE_SD0, DEV_LONG_MODE_SD_CONF_IN_PH_SD0);
            write_data_u8(cfg, LIGHTRANGER8_SD_CONFIG_INITIAL_PHASE_SD1, DEV_LONG_MODE_SD_CONF_IN_PH_SD1);
            break;
        }
        default:
        {
            return LIGHTRANGER8_RESP_WRONG_MODE;
        }
    }
    return LIGHTRANGER8_RESP_MODE_SUCCESSFULLY_SET;
}

uint8_t lightranger8_get_range_status(light_ranger_8_t *cfg )
{
    uint8_t m_status;

    m_status = read_data_u8(cfg, LIGHTRANGER8_RESULT_RANGE_STATUS) & DEV_REGISTER_STATUS_BITMASK;

    return m_status;
}

uint8_t lightranger8_get_roi_center(light_ranger_8_t *cfg)
{
    uint8_t ROICenter;

    ROICenter = read_data_u8(cfg, LIGHTRANGER8_ROI_CONFIG_USER_ROI_CENTRE_SPAD);

    return ROICenter;
}

void lightranger8_set_roi_center(light_ranger_8_t *cfg, uint8_t roi_center)
{
    write_data_u8(cfg, LIGHTRANGER8_ROI_CONFIG_USER_ROI_CENTRE_SPAD, roi_center);
}

void lightranger8_set_roi(light_ranger_8_t *cfg, uint16_t x, uint16_t y)
{
    uint8_t optical_center;

    optical_center = read_data_u8(cfg, LIGHTRANGER8_ROI_CONFIG_MODE_ROI_CENTRE_SPAD);

    if (x > DEV_ROI_MAX_VAL)
    {
        x = DEV_ROI_MAX_VAL ;
    }
    if (y > DEV_ROI_MAX_VAL)
    {
        y = DEV_ROI_MAX_VAL;
    }
    if (x > DEV_ROI_CENTER_EDGE_VAL || y > DEV_ROI_CENTER_EDGE_VAL)
    {
        optical_center = DEV_ROI_DEFAULT_CENTER_VAL;
    }
    write_data_u8(cfg, LIGHTRANGER8_ROI_CONFIG_USER_ROI_CENTRE_SPAD, optical_center);
    write_data_u8(cfg, LIGHTRANGER8_ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE, (y - 1) << DEV_REGISTER_ROI_NIBBLE_H_OFFSET | (x - 1));
}

void lightranger8_get_roi(light_ranger_8_t *cfg, uint16_t *roi_x, uint16_t *roi_y)
{
    uint8_t tmp;

    tmp = read_data_u8(cfg, LIGHTRANGER8_ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE);
    *roi_x = ((uint16_t)tmp & DEV_REGISTER_ROI_GLO_XY_SIZE_NIBBLE_L) + 1;
    *roi_y = (((uint16_t)tmp & DEV_REGISTER_ROI_GLO_XY_SIZE_NIBBLE_H) >> DEV_REGISTER_ROI_NIBBLE_H_OFFSET) + 1;
}

void lightranger8_set_offset(light_ranger_8_t *cfg, int16_t offset_value_mm)
{
    int16_t temp;

    temp = offset_value_mm * DEV_PART_TO_PART_RANGE_OFFSET_MULT;

    write_data_u16(cfg, LIGHTRANGER8_ALGO_PART_TO_PART_RANGE_OFFSET_MM, (uint16_t)temp);
    write_data_u16(cfg, LIGHTRANGER8_MM_CONFIG_INNER_OFFSET_MM, DEV_MM_CONF_INNER_OFFSET_RESET);
    write_data_u16(cfg, LIGHTRANGER8_MM_CONFIG_OUTER_OFFSET_MM, DEV_MM_CONF_OUTER_OFFSET_RESET);
}

void lightranger8_calibrate_offset(light_ranger_8_t *cfg, uint16_t target_distance_mm, uint32_t period_ms, int16_t *offset)
{
    uint8_t i;
    int16_t average_distance = 0;
    uint16_t distance;
    uint8_t averaging_number = 50;

    write_data_u16(cfg, LIGHTRANGER8_ALGO_PART_TO_PART_RANGE_OFFSET_MM, DEV_PART_TO_PART_RANGE_OFFSET_RESET);
    write_data_u16(cfg, LIGHTRANGER8_MM_CONFIG_INNER_OFFSET_MM, DEV_MM_CONF_INNER_OFFSET_RESET);
    write_data_u16(cfg, LIGHTRANGER8_MM_CONFIG_OUTER_OFFSET_MM, DEV_MM_CONF_OUTER_OFFSET_RESET);

    lightranger8_start_measurement(cfg, period_ms);

    for(i = 0; i < averaging_number; i++)
    {
        while(lightranger8_new_data_ready(cfg) != 0)
        {
            delay_data_ready(cfg);
        }

        distance = lightranger8_get_distance(cfg);
        average_distance = average_distance + distance;
    }

    lightranger8_stop_measurement(cfg);

    average_distance = average_distance / averaging_number;
    *offset = target_distance_mm - average_distance;

    write_data_u16(cfg, LIGHTRANGER8_ALGO_PART_TO_PART_RANGE_OFFSET_MM, *offset * DEV_PART_TO_PART_RANGE_OFFSET_MULT);
}

LIGHTRANGER8_RETVAL lightranger8_set_measurement_timing_budget(light_ranger_8_t *cfg, uint32_t budget_us)
{
    uint32_t range_config_timeout_us;
    uint32_t macro_period_us;
    uint32_t phasecal_timeout_mclks;
    uint16_t _data16;
    uint8_t _data8;

    if(budget_us <= DEV_MEASUREMENT_BUDGET_MIN)
    {
        return LIGHTRANGER8_RESP_INSUFFICIENT_BUDGET;
    }
    budget_us -= DEV_MEASUREMENT_BUDGET_MIN;
    range_config_timeout_us = budget_us;

    if(range_config_timeout_us > DEV_MEASUREMENT_BUDGET_MAX)
    {
        return LIGHTRANGER8_RESP_TOO_HIGH_BUDGET;
    }
    range_config_timeout_us /= 2;

    _data8 = read_data_u8(cfg, LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_A);
    macro_period_us = calc_macro_period(cfg, _data8);

    phasecal_timeout_mclks = timeout_microseconds_to_mclks(DEV_MEASUREMENT_PHASECAL_TIMEOUT, macro_period_us);

    if(phasecal_timeout_mclks > DEV_MEASUREMENT_PHASECAL_TIMEOUT_MAX)
    {
        phasecal_timeout_mclks = DEV_MEASUREMENT_PHASECAL_TIMEOUT_MAX;
    }

    write_data_u8(cfg, LIGHTRANGER8_PHASECAL_CONFIG_TIMEOUT_MACROP, phasecal_timeout_mclks);

    _data16 = timeout_microseconds_to_mclks(1, macro_period_us);
    _data16 = encode_timeout(_data16);
    write_data_u16(cfg, LIGHTRANGER8_MM_CONFIG_TIMEOUT_MACROP_A_HI, _data16);

    _data16 = timeout_microseconds_to_mclks(range_config_timeout_us, macro_period_us);
    _data16 = encode_timeout(_data16);
    write_data_u16(cfg, LIGHTRANGER8_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, _data16);

    _data8 = read_data_u8(cfg, LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_B);
    macro_period_us = calc_macro_period(cfg, _data8);

    _data16 = timeout_microseconds_to_mclks(1, macro_period_us);
    _data16 = encode_timeout(_data16);
    write_data_u16(cfg, LIGHTRANGER8_MM_CONFIG_TIMEOUT_MACROP_B_HI, _data16);

    _data16 = timeout_microseconds_to_mclks(range_config_timeout_us, macro_period_us);
    _data16 = encode_timeout(_data16);
    write_data_u16(cfg, LIGHTRANGER8_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, _data16);

    return LIGHTRANGER8_RESP_BUDGET_IS_SUCCESSFULLY_SET;
}

uint8_t lightranger8_get_interrupt_state(light_ranger_8_t *cfg)
{
    return HAL_GPIO_ReadPin(cfg->int_pin_base, cfg->int_pin);
}

void lightranger8_set_xshut_pin(light_ranger_8_t *cfg, GPIO_PinState state, GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, state);
}

void lightranger8_power_on(light_ranger_8_t *cfg)
{
	lightranger8_set_xshut_pin(cfg, GPIO_PIN_RESET, cfg->en_pin_base, cfg->en);
	delay_power_up(cfg);
	lightranger8_set_xshut_pin(cfg, GPIO_PIN_SET, cfg->en_pin_base, cfg->en);
	delay_power_full_on(cfg);
}

uint32_t lightranger8_get_timing_budget(light_ranger_8_t *cfg)
{
    uint32_t macro_period_us;
    uint32_t range_config_timeout_us;
    uint16_t _data16;
    uint8_t _data8;

    _data8 = read_data_u8(cfg, LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_A);
    macro_period_us = calc_macro_period(cfg, _data8);

    _data16 = read_data_u16(cfg, LIGHTRANGER8_RANGE_CONFIG_TIMEOUT_MACROP_A_HI);
    _data16 = decode_timeout(_data16);
    range_config_timeout_us = timeout_mclks_to_microseconds(_data16, macro_period_us);

    return 2 * range_config_timeout_us + DEV_MEASUREMENT_BUDGET_MIN;
}

uint16_t lightranger8_get_measurement_period(light_ranger_8_t *cfg)
{
    uint16_t temp;
    uint16_t ClockPLL;

    temp = read_data_u16(cfg, LIGHTRANGER8_SYSTEM_INTERMEASUREMENT_PERIOD_1);
    ClockPLL = read_data_u16(cfg, LIGHTRANGER8_RESULT_OSC_CALIBRATE_VAL);
    ClockPLL = ClockPLL & DEV_REGISTER_BITMASK_10_BIT;
    temp = (uint16_t)((float)temp / (float)ClockPLL * DEV_CLOCK_COEFF);

    return temp;
}

// ----------------------------------------------- PRIVATE FUNCTION DEFINITIONS

static uint32_t calc_macro_period(light_ranger_8_t *cfg, uint8_t vcsel_period)
{
    uint32_t pll_period_us;
    uint8_t vcsel_period_pclks;
    uint32_t macro_period_us;

    pll_period_us = DEV_REGISTER_BITMASK_30_BIT_SHIFT / cfg->fast_osc_frequency;
    vcsel_period_pclks = (vcsel_period + 1) << 1;
    macro_period_us = DEV_MACRO_PERIOD_COEFF * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;

    return macro_period_us;
}

static uint32_t timeout_microseconds_to_mclks(uint32_t timeout_us, uint32_t macro_period_us)
{
    return ((uint32_t)(timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

static uint16_t encode_timeout(uint32_t timeout_mclks)
{
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if(timeout_mclks > 0)
    {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & DEV_REGISTER_CLEAR_LSBYTE) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }
        return (ms_byte << 8) | (uint16_t)(ls_byte & DEV_REGISTER_BITMASK_LSBYTE);
    }
    return 0;
}

static uint32_t timeout_mclks_to_microseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
  return (timeout_mclks * macro_period_us + DEV_REGISTER_BITMASK_12_BIT_SHIFT) >> 12;
}

static uint32_t decode_timeout(uint16_t reg_val)
{
  return ((uint32_t)(reg_val & DEV_REGISTER_BITMASK_LSBYTE) << (reg_val >> 8)) + 1;
}

static void write_data_u8(light_ranger_8_t *cfg, uint16_t addr, uint8_t _data)
{
    lightranger8_generic_write(cfg, addr, &_data, 1);
}

static void write_data_u16(light_ranger_8_t *cfg, uint16_t addr, uint16_t _data)
{
    uint8_t tx_buff[2];

    tx_buff[0] = _data >> 8;
    tx_buff[1] = _data & DEV_REGISTER_BITMASK_U16;

    lightranger8_generic_write(cfg, addr, tx_buff, 2);
}

static void write_data_u32(light_ranger_8_t *cfg, uint16_t addr, uint32_t _data)
{
    uint8_t tx_buff[4];

    tx_buff[0] = _data >> 24;
    tx_buff[1] = _data >> 16;
    tx_buff[2] = _data >> 8;
    tx_buff[3] = _data & DEV_REGISTER_BITMASK_U32;

    lightranger8_generic_write(cfg, addr, tx_buff, 4);
}

static uint8_t read_data_u8(light_ranger_8_t *cfg, uint16_t addr)
{
    uint8_t rx_data;

    lightranger8_generic_read(cfg, addr, &rx_data, 1);

    return rx_data;
}

static uint16_t read_data_u16(light_ranger_8_t *cfg, uint16_t addr)
{
    uint8_t rx_buff[2];
    uint16_t read_data;

    lightranger8_generic_read(cfg, addr, rx_buff, 2);

    read_data = rx_buff[0];
    read_data <<= 8;
    read_data |= rx_buff[1];

    return read_data;
}

static void delay_power_full_on(light_ranger_8_t *cfg)
{
    HAL_Delay(1000);
}

static void delay_power_up(light_ranger_8_t *cfg)
{
    HAL_Delay(100);
}

static void delay_config_set(light_ranger_8_t *cfg)
{
    HAL_Delay(10);
}

static void delay_soft_reset(light_ranger_8_t *cfg)
{
   HAL_Delay(1);
}

static void delay_data_ready(light_ranger_8_t *cfg)
{
    HAL_Delay(1);
}

// ----------------------------------------------- PRIVATE FUNCTION DEFINITIONS

// ------------------------------------------------------------------------- END

