#include "gd32f1x0_usart.h"
#include "string.h"
#include "stdint.h"

void com_init0(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
    usart_enable(USART0);
}

void com_init1(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART1);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_15);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_15);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_15);
    usart_deinit(USART1);
    usart_baudrate_set(USART1, 115200U);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_interrupt_enable(USART1, USART_INT_RBNE);
    usart_enable(USART1);
}

#define ADC_IN0_VCC24_MIN              3217
#define ADC_IN0_VCC24_MAX              3574
#define ADC_IN1_VCC3V3_ULP_MIN         2247
#define ADC_IN1_VCC3V3_ULP_MAX         2491
#define ADC_IN4_VBAT_MIN               2518
#define ADC_IN4_VBAT_MAX               3438
#define ADC_IN5_VCC12_SYS_MIN          3084
#define ADC_IN5_VCC12_SYS_MAX          3484
#define ADC_IN6_VCC5V0_CORE_MIN        3231
#define ADC_IN6_VCC5V0_CORE_MAX        3588
#define ADC_IN7_VCC5V0_SYS_MIN         3231
#define ADC_IN7_VCC5V0_SYS_MAX         3588
#define MODIFY_FULL_CAP                10

void delay(int time);
extern uint8_t adc_value[20];

void power_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);

    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_0);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_0);
    gpio_bit_reset(GPIOA, GPIO_PIN_0);

    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_1);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_1);
    gpio_bit_reset(GPIOA, GPIO_PIN_1);

    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_4);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_4);
    gpio_bit_reset(GPIOA, GPIO_PIN_4);

    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_5);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_5);
    gpio_bit_reset(GPIOA, GPIO_PIN_5);

    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_6);
    gpio_bit_reset(GPIOA, GPIO_PIN_6);

    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_7);
    gpio_bit_reset(GPIOA, GPIO_PIN_7);

    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_6);
    gpio_bit_reset(GPIOB, GPIO_PIN_6);

    gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_7);

    gpio_bit_set(GPIOA, GPIO_PIN_0);
    delay(10);
    gpio_bit_set(GPIOA, GPIO_PIN_7);
    delay(10);
    gpio_bit_set(GPIOA, GPIO_PIN_1);
    delay(25);
    gpio_bit_set(GPIOA, GPIO_PIN_6);
    delay(10);
    gpio_bit_set(GPIOA, GPIO_PIN_5);
    delay(100);

    gpio_bit_set(GPIOA, GPIO_PIN_4);
}

void usart_transmit(uint32_t usart_periph, uint8_t *data, uint8_t length)
{
    uint8_t i;

    for (i = 0; i < length; i++) {
        usart_data_transmit(usart_periph, *(data + i));
        while (RESET == usart_flag_get(usart_periph, USART_FLAG_TBE));
    }
}

#define HOST_RECEIVE_COUNT                 6
#define HOST_COMMAND_HEART_BEAT            0x00
#define HOST_COMMAND_GET_POWER_STATUS      0x01
#define HOST_COMMAND_SET_POWER_STATUS      0x02
#define HOST_COMMAND_GET_ADC_VALUE         0x03
#define HOST_COMMAND_GET_BAT_CAPACITY      0x04
#define HOST_COMMAND_GET_BAT_TEMPEARATURE  0x05
#define HOST_COMMAND_GET_BAT_VOLTAGE       0x06
#define HOST_COMMAND_SHUTDOWN              0x07
#define HOST_COMMAND_BOOTUP_SUCCESS        0x10

extern uint8_t adc_value[];
extern uint8_t host_receiver_buffer[HOST_RECEIVE_COUNT];
extern uint8_t host_bootup_ok;

extern volatile uint32_t shutdown_counter;
extern void shutdown_execute(void);

int battery_temp_data = -1;

uint8_t host_reply_heart_beat_buffer[6]          = {0x55, 0xaa, HOST_COMMAND_HEART_BEAT,           0x00, 0x00, 0xff};
uint8_t host_reply_get_power_status_buffer[6]    = {0x55, 0xaa, HOST_COMMAND_GET_POWER_STATUS,     0x00, 0x00, 0xff};
uint8_t host_reply_set_power_status_buffer[6]    = {0x55, 0xaa, HOST_COMMAND_SET_POWER_STATUS,     0x00, 0x00, 0xff};
uint8_t host_reply_get_adc_value_buffer[24]      = {0x55, 0xaa, HOST_COMMAND_GET_ADC_VALUE,
                                                    0x00, 0x00, 0x00, 0x00,
                                                    0x00, 0x00, 0x00, 0x00,
                                                    0x00, 0x00, 0x00, 0x00,
                                                    0x00, 0x00, 0x00, 0x00,
                                                    0x00, 0x00, 0x00, 0x00,
                                                    0xff};
uint8_t host_reply_get_bat_capacity_buffer[6]    = {0x55, 0xaa, HOST_COMMAND_GET_BAT_CAPACITY,     0x00, 0x00, 0xff};
uint8_t host_reply_get_bat_temperature_buffer[6] = {0x55, 0xaa, HOST_COMMAND_GET_BAT_TEMPEARATURE, 0x00, 0x00, 0xff};
uint8_t host_reply_get_bat_voltage_buffer[6]     = {0x55, 0xaa, HOST_COMMAND_GET_BAT_VOLTAGE,      0x00, 0x00, 0xff};
uint8_t host_reply_shutdown_buffer[6]            = {0x55, 0xaa, HOST_COMMAND_SHUTDOWN,             0x00, 0x00, 0xff};
uint8_t host_reply_bootup_success_buffer[6]      = {0x55, 0xaa, HOST_COMMAND_BOOTUP_SUCCESS,       0x00, 0x00, 0xff};
uint8_t host_reply_error_status_buffer[6]        = {0x55, 0xaa, 0xff, 0xff, 0xff, 0xff};

uint32_t pin_power_map[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6};

void process_command(void)
{
    if ((host_receiver_buffer[0] == 0xaa) &&
        (host_receiver_buffer[1] == 0x55) &&
        (host_receiver_buffer[5] == 0xff)) {

        switch (host_receiver_buffer[2]) {

        case HOST_COMMAND_HEART_BEAT:
            usart_transmit(USART0,
                           host_reply_heart_beat_buffer,
                           sizeof(host_reply_heart_beat_buffer));
            break;

        case HOST_COMMAND_GET_POWER_STATUS:
            if (host_receiver_buffer[3] < 5) {
                host_reply_get_power_status_buffer[3] =
                    host_receiver_buffer[3];
                host_reply_get_power_status_buffer[4] =
                    gpio_output_bit_get(GPIOA,
                                        pin_power_map[host_receiver_buffer[3]]);
                usart_transmit(USART0,
                               host_reply_get_power_status_buffer,
                               sizeof(host_reply_get_power_status_buffer));
            } else {
                usart_transmit(USART0,
                               host_reply_error_status_buffer,
                               sizeof(host_reply_error_status_buffer));
            }
            break;

        case HOST_COMMAND_SET_POWER_STATUS:
            if (host_receiver_buffer[3] < 5) {
                gpio_bit_write(GPIOA,
                               pin_power_map[host_receiver_buffer[3]],
                               (host_receiver_buffer[4] > 0) ? SET : RESET);
                host_reply_set_power_status_buffer[3] =
                    host_receiver_buffer[3];
                host_reply_set_power_status_buffer[4] =
                    gpio_output_bit_get(GPIOA,
                                        pin_power_map[host_receiver_buffer[3]]);
                usart_transmit(USART0,
                               host_reply_set_power_status_buffer,
                               sizeof(host_reply_set_power_status_buffer));
            } else {
                usart_transmit(USART0,
                               host_reply_error_status_buffer,
                               sizeof(host_reply_error_status_buffer));
            }
            break;

        case HOST_COMMAND_GET_ADC_VALUE:
            memcpy(&host_reply_get_adc_value_buffer[3], adc_value, 20);
            usart_transmit(USART0,
                           host_reply_get_adc_value_buffer,
                           sizeof(host_reply_get_adc_value_buffer));
            break;

        case HOST_COMMAND_GET_BAT_CAPACITY:
            battery_temp_data = BQ40Z50_Read_SOC();
            host_reply_get_bat_capacity_buffer[3] =
                (battery_temp_data >> 8) & 0xff;
            host_reply_get_bat_capacity_buffer[4] =
                battery_temp_data & 0xff;
            usart_transmit(USART0,
                           host_reply_get_bat_capacity_buffer,
                           sizeof(host_reply_get_bat_capacity_buffer));
            break;

        case HOST_COMMAND_GET_BAT_TEMPEARATURE:
            battery_temp_data = BQ40Z50_Read_Temp();
            host_reply_get_bat_temperature_buffer[3] =
                (battery_temp_data >> 8) & 0xff;
            host_reply_get_bat_temperature_buffer[4] =
                battery_temp_data & 0xff;
            usart_transmit(USART0,
                           host_reply_get_bat_temperature_buffer,
                           sizeof(host_reply_get_bat_temperature_buffer));
            break;

        case HOST_COMMAND_GET_BAT_VOLTAGE:
            battery_temp_data = BQ40Z50_Read_Vol();
            host_reply_get_bat_voltage_buffer[3] =
                (battery_temp_data >> 8) & 0xff;
            host_reply_get_bat_voltage_buffer[4] =
                battery_temp_data & 0xff;
            usart_transmit(USART0,
                           host_reply_get_bat_voltage_buffer,
                           sizeof(host_reply_get_bat_voltage_buffer));
            break;
        
        case HOST_COMMAND_SHUTDOWN:
        {
            uint16_t shutdown_time =
                (uint16_t)(((uint16_t)host_receiver_buffer[3] << 8) |
                           (uint16_t)host_receiver_buffer[4]);

            host_reply_shutdown_buffer[3] = host_receiver_buffer[3];
            host_reply_shutdown_buffer[4] = host_receiver_buffer[4];
            usart_transmit(USART0,
                           host_reply_shutdown_buffer,
                           sizeof(host_reply_shutdown_buffer));

            if (shutdown_time == 0) {
                shutdown_counter = 0;
                shutdown_execute();
            } else {
                shutdown_counter = shutdown_time;
            }
            break;
        }

        case HOST_COMMAND_BOOTUP_SUCCESS:
            host_bootup_ok = 1;
            usart_transmit(USART0,
                           host_reply_bootup_success_buffer,
                           sizeof(host_reply_bootup_success_buffer));
            break;

        default:
            usart_transmit(USART0,
                           host_reply_error_status_buffer,
                           sizeof(host_reply_error_status_buffer));
            break;
        }
    }
}

#define PERIODIC_REPORT_ADC_VALUE 0x22

uint8_t periodic_report_adc_value_buffer[24] = {
    0x55, 0xaa, PERIODIC_REPORT_ADC_VALUE,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0xff
};

void periodic_report(void)
{
    memcpy(&periodic_report_adc_value_buffer[3], adc_value, 20);
    usart_transmit(USART0,
                   periodic_report_adc_value_buffer,
                   sizeof(periodic_report_adc_value_buffer));
}

uint8_t crc8(uint8_t *data, int length)
{
    uint8_t crc  = 0x00;
    uint8_t poly = 0x07;
    int i, j;

    for (i = 0; i < length; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1) ^ poly);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

static void Delay(int n)
{
    int target_time = 1 * n;
    while (target_time--);
}

void rcu_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_I2C0);
}

#define I2C_SCL_GPIO_PIN   GPIO_PIN_8
#define I2C_SDA_GPIO_PIN   GPIO_PIN_9

void gpio_config(void)
{
    gpio_af_set(GPIOB, GPIO_AF_1, I2C_SCL_GPIO_PIN);
    gpio_af_set(GPIOB, GPIO_AF_1, I2C_SDA_GPIO_PIN);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_SCL_GPIO_PIN);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SCL_GPIO_PIN);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_SDA_GPIO_PIN);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_SDA_GPIO_PIN);
}

void i2c_config(void)
{
    i2c_deinit(I2C0);
    i2c_clock_config(I2C0, 100000, I2C_DTCY_2);
    i2c_enable(I2C0);
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
}

void i2c_scan(uint32_t i2c_bus)
{
    uint8_t addr;
    int timeout;

    printf("Scanning I2C bus...\r\n");

    for (addr = 0; addr < 0x80; addr++) {
        timeout = 1000;

        while (i2c_flag_get(i2c_bus, I2C_FLAG_I2CBSY));

        i2c_start_on_bus(i2c_bus);
        while (!i2c_flag_get(i2c_bus, I2C_FLAG_SBSEND));

        i2c_master_addressing(i2c_bus, addr, I2C_TRANSMITTER);

        while (!i2c_flag_get(i2c_bus, I2C_FLAG_ADDSEND) && --timeout);

        if (timeout > 0) {
            printf("Found device at 0x%02X\r\n", addr);
            i2c_flag_clear(i2c_bus, I2C_STAT0_ADDSEND);
        } else {
            printf("not Found device at 0x%02X\r\n", addr);
        }

        i2c_stop_on_bus(i2c_bus);
        while (I2C_CTL0(i2c_bus) & I2C_CTL0_STOP);
    }

    printf("Scan complete\r\n");
}

void BQ40Z50_Init(void)
{
    rcu_config();
    gpio_config();
    i2c_config();
}

int i2c_read_register2(uint32_t i2c_bus, uint8_t i2c_addr, uint8_t reg_addr)
{
    uint8_t msb = 0, lsb = 0;
    int timeout;
    uint16_t result;

    timeout = 10000;
    while (i2c_flag_get(i2c_bus, I2C_FLAG_I2CBSY) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] BUS BUSY timeout\r\n");
        goto error;
    }

    i2c_start_on_bus(i2c_bus);
    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_SBSEND) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] START condition timeout\r\n");
        goto error;
    }

    i2c_master_addressing(i2c_bus, i2c_addr, I2C_TRANSMITTER);
    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_ADDSEND) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] WRITE ADDR NACK (0x%02X)\r\n", i2c_addr);
        goto error;
    }
    i2c_flag_clear(i2c_bus, I2C_STAT0_ADDSEND);

    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_TBE) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] TBE timeout before sending reg\r\n");
        goto error;
    }

    i2c_data_transmit(i2c_bus, reg_addr);

    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_TBE) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] TBE timeout after sending reg\r\n");
        goto error;
    }

    i2c_start_on_bus(i2c_bus);
    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_SBSEND) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] Repeated START timeout\r\n");
        goto error;
    }

    i2c_master_addressing(i2c_bus, i2c_addr, I2C_RECEIVER);
    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_ADDSEND) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] READ ADDR NACK (0x%02X)\r\n", i2c_addr);
        goto error;
    }
    i2c_flag_clear(i2c_bus, I2C_STAT0_ADDSEND);

    i2c_ack_config(i2c_bus, I2C_ACK_ENABLE);

    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_RBNE) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] RBNE timeout (1st byte)\r\n");
        goto error;
    }
    lsb = i2c_data_receive(i2c_bus);

    i2c_ack_config(i2c_bus, I2C_ACK_DISABLE);
    i2c_stop_on_bus(i2c_bus);

    timeout = 10000;
    while (!i2c_flag_get(i2c_bus, I2C_FLAG_RBNE) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] RBNE timeout (2nd byte)\r\n");
        goto error;
    }
    msb = i2c_data_receive(i2c_bus);

    timeout = 10000;
    while ((I2C_CTL0(i2c_bus) & I2C_CTL0_STOP) && --timeout);
    if (timeout <= 0) {
        printf("[I2C] STOP timeout\r\n");
        goto error;
    }

    i2c_ack_config(i2c_bus, I2C_ACK_ENABLE);
    i2c_ackpos_config(i2c_bus, I2C_ACKPOS_CURRENT);

    result = (uint16_t)((msb << 8) | lsb);
    return (int)result;

error:
    i2c_stop_on_bus(i2c_bus);
    timeout = 10000;
    while ((I2C_CTL0(i2c_bus) & I2C_CTL0_STOP) && --timeout);
    i2c_ack_config(i2c_bus, I2C_ACK_ENABLE);
    i2c_ackpos_config(i2c_bus, I2C_ACKPOS_CURRENT);
    printf("[I2C] READ FAILED at reg 0x%02X\r\n", reg_addr);
    return -1;
}

#define BQ40Z50_I2C_BUS      I2C0
#define BQ40Z50_I2C_ADDR     0x16

int BQ40Z50_Read_SOC(void)
{
    int attempt;
    int soc_raw = -1;

    for (attempt = 1; attempt <= 10; attempt++) {
        soc_raw = i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x0D);

        if (soc_raw >= 0 && soc_raw <= 100) {
            return soc_raw;
        } else {
            printf("[SOC] Invalid: %d, retrying...\r\n", soc_raw);
        }
    }

    printf("[SOC] Failed after 10 attempts\r\n");
    return -1;
}

int BQ40Z50_Read_Temp(void)
{
    int attempt;
    int temp_raw = -1;

    for (attempt = 1; attempt <= 10; attempt++) {
        temp_raw = i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x08);

        if (temp_raw > 0) {
            return temp_raw;
        } else {
            printf("[TEMP] Invalid: %d, retrying...\r\n", temp_raw);
        }
    }

    printf("[TEMP] Failed after 10 attempts\r\n");
    return -1;
}

int BQ40Z50_Read_Vol(void)
{
    int attempt;
    int vol_raw = -1;

    for (attempt = 1; attempt <= 10; attempt++) {
        vol_raw = i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x09);

        if (vol_raw > 0 && vol_raw < 65536) {
            return vol_raw;
        } else {
            printf("[VOL] Invalid: %d, retrying...\r\n", vol_raw);
        }
    }

    printf("[VOL] Failed after 10 attempts\r\n");
    return -1;
}

int BQ40Z50_Read_CycleCount(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x17);
}

int BQ40Z50_Read_FullChargeCapacity(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x10);
}

int BQ40Z50_Read_DesignCapacity(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x18);
}

int BQ40Z50_Read_RemainingCapacity(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x0F);
}

int BQ40Z50_Read_ChargingStatus(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x16);
}

int BQ40Z50_Read_TimeToFull(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x13);
}

int BQ40Z50_Read_Current(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x0A);
}

int BQ40Z50_Read_ChargingCurrent(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x14);
}

int BQ40Z50_Read_ChargingVoltage(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x15);
}

int BQ40Z50_Read_BatteryMode(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x03);
}

int BQ40Z50_Read_AverageCurrent(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x0B);
}

int BQ40Z50_Read_RelativeStateOfCharge(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x0D);
}

int BQ40Z50_Read_AbsoluteStateOfCharge(void)
{
    return i2c_read_register2(BQ40Z50_I2C_BUS, BQ40Z50_I2C_ADDR, 0x0E);
}

extern uint8_t adc_value[20];

static uint8_t calc_add_crc(uint8_t *data, uint16_t length)
{
    uint32_t sum = 0;
    uint16_t i;

    for (i = 0; i < length; i++) {
        sum += data[i];
    }

    return (uint8_t)(sum & 0xFF);
}

void periodic_adc_report(void)
{
    uint8_t buf[64];
    uint16_t idx = 0;
    uint16_t len_field;
    uint8_t  len_hi, len_lo;
    uint8_t  crc;
    uint16_t crc_start_index;
    uint16_t crc_len;
    uint8_t  i;

    const uint16_t payload_len = 1 + 20;
    len_field = 1 + payload_len ;
    len_hi    = (uint8_t)((len_field >> 8) & 0xFF);
    len_lo    = (uint8_t)(len_field & 0xFF);

    buf[idx++] = 0xAA;
    buf[idx++] = 0xAA;
    buf[idx++] = 0xAA;

    buf[idx++] = len_hi;
    buf[idx++] = len_lo;

    buf[idx++] = 0xFE;

    buf[idx++] = 0x80;

    for (i = 0; i < 20; i++) {
        buf[idx++] = adc_value[i];
    }

    crc_start_index = 3;
    crc_len         = idx - crc_start_index;
    crc             = calc_add_crc(&buf[crc_start_index], crc_len);

    buf[idx++] = crc;

    buf[idx++] = 0xFF;
    buf[idx++] = 0xFF;
    buf[idx++] = 0xFF;

    usart_transmit(USART0, buf, (uint8_t)idx);
}

static uint8_t s_bat_led_on = 0;
static uint8_t s_charging_active = 0;
static uint8_t s_pb7_low_seconds = 0;

static void monitor_pb7_shutdown(void)
{
    uint8_t pb7_state;
    pb7_state = (uint8_t)gpio_input_bit_get(GPIOB, GPIO_PIN_7);

    if (pb7_state == RESET) {
        if (s_pb7_low_seconds < 8U) {
            s_pb7_low_seconds++;
        }
        if (s_pb7_low_seconds >= 8U) {
            s_pb7_low_seconds = 0U;
            shutdown_counter = 0U;
            shutdown_execute();
        }
    } else {
        s_pb7_low_seconds = 0U;
    }
}

void periodic_battery_report(void)
{
    uint8_t  buf[96];
    uint16_t idx;
    uint16_t len_field;
    uint8_t  len_hi;
    uint8_t  len_lo;
    uint8_t  crc;
    uint16_t crc_start_index;
    uint16_t crc_len;
    uint16_t chg_raw;
    int      val_cycle;
    int      val_full_cap;
    int      val_design_cap;
    int      val_remaining;
    int      val_chg_status;
    int      val_time_to_full;
    int      val_temp;
    int      val_vol;
    int      val_curr;
    int      val_chg_curr;
    int      val_chg_volt;
    int      val_bat_mode;
    int      val_avg_curr;
    int      val_rel_soc;
    int      val_abs_soc;
    const uint16_t DSG_MASK = (1U << 6);
    uint8_t  dsg;
    const uint16_t payload_len = (uint16_t)(1 + 30 + 1);
    int      percent;
    int16_t  avg_curr_signed = 0;
    uint8_t  ac_on = 0U;
    uint8_t  battery_present = 0U;

    idx     = 0;
    chg_raw = 0;
    dsg     = 0;
    percent = 0;

    monitor_pb7_shutdown();

    val_cycle = BQ40Z50_Read_CycleCount();
    
    if (val_cycle >= 0) {
        battery_present = 1U;
        
        val_full_cap     = BQ40Z50_Read_FullChargeCapacity();
        val_design_cap   = BQ40Z50_Read_DesignCapacity();
        val_remaining    = BQ40Z50_Read_RemainingCapacity();
        val_chg_status   = BQ40Z50_Read_ChargingStatus();
        val_time_to_full = BQ40Z50_Read_TimeToFull();
        val_temp         = BQ40Z50_Read_Temp();
        val_vol          = BQ40Z50_Read_Vol();
        val_curr         = BQ40Z50_Read_Current();
        val_chg_curr     = BQ40Z50_Read_ChargingCurrent();
        val_chg_volt     = BQ40Z50_Read_ChargingVoltage();
        val_bat_mode     = BQ40Z50_Read_BatteryMode();
        val_avg_curr     = BQ40Z50_Read_AverageCurrent();
        val_rel_soc      = BQ40Z50_Read_RelativeStateOfCharge();
        val_abs_soc      = BQ40Z50_Read_AbsoluteStateOfCharge();

        if ((val_full_cap > MODIFY_FULL_CAP) && (val_remaining >= 0)) {
            percent = (int)((long long)val_remaining * 100LL / 
                           ((long long)val_full_cap - MODIFY_FULL_CAP));
            
            if (percent > 100) {
                percent = 100;
            }
        } else if (val_full_cap > 0 && val_remaining >= 0) {
            percent = (int)((long long)val_remaining * 100LL / (long long)val_full_cap);
            if (percent > 100) {
                percent = 100;
            }
        } else {
            percent = 0;
        }

        if (val_chg_volt > 15000) {
            ac_on = 1U;
        } else if (val_avg_curr >= 0) {
            avg_curr_signed = (int16_t)((uint16_t)val_avg_curr);
            
            if (val_chg_curr > 100) {
                ac_on = 1U;
            } else if (avg_curr_signed >= 0) {
                ac_on = 1U;
            }
        }
    } else {
        battery_present  = 0U;
        val_cycle        = -1;
        val_full_cap     = -1;
        val_design_cap   = -1;
        val_remaining    = -1;
        val_chg_status   = -1;
        val_time_to_full = -1;
        val_temp         = -1;
        val_vol          = -1;
        val_curr         = -1;
        val_chg_curr     = -1;
        val_chg_volt     = -1;
        val_bat_mode     = -1;
        val_avg_curr     = -1;
        val_rel_soc      = -1;
        val_abs_soc      = -1;
        percent          = -1;
    }
        
    len_field = (uint16_t)(1 + payload_len);
    len_hi    = (uint8_t)((len_field >> 8) & 0xFF);
    len_lo    = (uint8_t)(len_field & 0xFF);

    buf[idx++] = 0xAA;
    buf[idx++] = 0xAA;
    buf[idx++] = 0xAA;

    buf[idx++] = len_hi;
    buf[idx++] = len_lo;

    buf[idx++] = 0xFE;

    buf[idx++] = 0x81;
        
#define PUT_16BE_FROM_VAL(v)                          \
    do {                                              \
        uint16_t _u16;                                \
        if ((v) < 0) {                                \
            _u16 = 0xFFFF;                            \
        } else {                                      \
            _u16 = (uint16_t)(v);                     \
        }                                             \
        buf[idx++] = (uint8_t)((_u16 >> 8) & 0xFF);   \
        buf[idx++] = (uint8_t)(_u16 & 0xFF);          \
    } while (0)

    PUT_16BE_FROM_VAL(val_cycle);
    PUT_16BE_FROM_VAL(val_full_cap);
    PUT_16BE_FROM_VAL(val_design_cap);
    PUT_16BE_FROM_VAL(val_remaining);
    PUT_16BE_FROM_VAL(val_chg_status);
    PUT_16BE_FROM_VAL(val_time_to_full);
    PUT_16BE_FROM_VAL(val_temp);
    PUT_16BE_FROM_VAL(val_vol);
    PUT_16BE_FROM_VAL(val_curr);
    PUT_16BE_FROM_VAL(val_chg_curr);
    PUT_16BE_FROM_VAL(val_chg_volt);
    PUT_16BE_FROM_VAL(val_bat_mode);
    PUT_16BE_FROM_VAL(val_avg_curr);
    PUT_16BE_FROM_VAL(val_rel_soc);
    PUT_16BE_FROM_VAL(val_abs_soc);

#undef PUT_16BE_FROM_VAL

    buf[idx++] = (uint8_t)((percent < 0) ? 0xFF : (percent & 0xFF));

    crc_start_index = 3;
    crc_len         = (uint16_t)(idx - crc_start_index);
    crc             = calc_add_crc(&buf[crc_start_index], crc_len);

    buf[idx++] = crc;

    buf[idx++] = 0xFF;
    buf[idx++] = 0xFF;
    buf[idx++] = 0xFF;

    usart_transmit(USART0, buf, (uint8_t)idx);

    if (!battery_present) {
        gpio_bit_reset(GPIOB, GPIO_PIN_6);
        s_bat_led_on = 0U;
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        return;
    }

    s_charging_active = 0U;

    if (ac_on) {
        if (percent < 100) {
            s_charging_active = 1U;
        }
    }

    if (val_chg_status >= 0) {
        chg_raw = (uint16_t)val_chg_status;
        dsg = (chg_raw & DSG_MASK) ? 1U : 0U;
    }

    if (s_charging_active) {
        gpio_bit_set(GPIOB, GPIO_PIN_6);

        s_bat_led_on = (uint8_t)!s_bat_led_on;
        if (s_bat_led_on) {
            gpio_bit_set(GPIOC, GPIO_PIN_13);
        } else {
            gpio_bit_reset(GPIOC, GPIO_PIN_13);
        }
    } else {
        gpio_bit_reset(GPIOB, GPIO_PIN_6);

        s_bat_led_on = 0U;
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
    }
}

uint8_t is_battery_low_capacity(void)
{
    int val_full_cap;
    int val_remaining;
    int percent;

    val_full_cap = BQ40Z50_Read_FullChargeCapacity();
    if (val_full_cap <= 0) {
        return 0;
    }

    val_remaining = BQ40Z50_Read_RemainingCapacity();
    if (val_remaining < 0) {
        return 0;
    }

    if (val_full_cap > MODIFY_FULL_CAP) {
        percent = (int)((long long)val_remaining * 100LL /
                       ((long long)val_full_cap - MODIFY_FULL_CAP));
    } else {
        percent = (int)((long long)val_remaining * 100LL / (long long)val_full_cap);
    }

    if (percent < 0) {
        percent = 0;
    } else if (percent > 100) {
        percent = 100;
    }

    if (percent >= 0 && percent <= 3) {
        return 1;
    } else {
        return 0;
    }
}