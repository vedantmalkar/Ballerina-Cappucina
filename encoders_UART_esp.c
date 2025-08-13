#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TXD_PIN GPIO_NUM_17
#define RXD_PIN GPIO_NUM_16
#define UART_NUM UART_NUM_2
#define BUF_SIZE 1024

float linear_x_val = 0.0;
float angular_z_val = 0.0;

#define MOTOR_PWM_FRONT_LEFT GPIO_NUM_18
// #define MOTOR_PWM_FRONT_RIGHT GPIO_NUM_19
// #define MOTOR_PWM_REAR_LEFT GPIO_NUM_21
// #define MOTOR_PWM_REAR_RIGHT GPIO_NUM_22

#define MOTOR_DIR_FRONT_LEFT GPIO_NUM_25
// #define MOTOR_DIR_FRONT_RIGHT GPIO_NUM_26
// #define MOTOR_DIR_REAR_LEFT GPIO_NUM_27
// #define MOTOR_DIR_REAR_RIGHT GPIO_NUM_32

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_FREQUENCY 5000
#define LEDC_DUTY_RESOLUTION LEDC_TIMER_10_BIT

#define LEDC_CHANNEL_FRONT_LEFT LEDC_CHANNEL_0
// #define LEDC_CHANNEL_FRONT_RIGHT LEDC_CHANNEL_1
// #define LEDC_CHANNEL_REAR_LEFT LEDC_CHANNEL_2
// #define LEDC_CHANNEL_REAR_RIGHT LEDC_CHANNEL_3
    
#define ENCODER_M1_A GPIO_NUM_34 
#define ENCODER_M1_B GPIO_NUM_35 
#define PCNT_UNIT_M1 PCNT_UNIT_0
#define ENCODER_TICKS_PER_REVOLUTION 3960.0
#define TIMER_INTERVAL_MS 100
static volatile int16_t motor1_count_change = 0;

void uart_init()
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void ledc_setup()
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RESOLUTION,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel_front_left = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_FRONT_LEFT,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_PWM_FRONT_LEFT,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel_front_left);

    // ledc_channel_config_t ledc_channel_front_right = {
    //     .speed_mode = LEDC_MODE,
    //     .channel = LEDC_CHANNEL_FRONT_RIGHT,
    //     .timer_sel = LEDC_TIMER,
    //     .intr_type = LEDC_INTR_DISABLE,
    //     .gpio_num = MOTOR_PWM_FRONT_RIGHT,
    //     .duty = 0,
    //     .hpoint = 0,
    // };
    // ledc_channel_config(&ledc_channel_front_right);

    // ledc_channel_config_t ledc_channel_rear_left = {
    //     .speed_mode = LEDC_MODE,
    //     .channel = LEDC_CHANNEL_REAR_LEFT,
    //     .timer_sel = LEDC_TIMER,
    //     .intr_type = LEDC_INTR_DISABLE,
    //     .gpio_num = MOTOR_PWM_REAR_LEFT,
    //     .duty = 0,
    //     .hpoint = 0,
    // };
    // ledc_channel_config(&ledc_channel_rear_left);

    // ledc_channel_config_t ledc_channel_rear_right = {
    //     .speed_mode = LEDC_MODE,
    //     .channel = LEDC_CHANNEL_REAR_RIGHT,
    //     .timer_sel = LEDC_TIMER,
    //     .intr_type = LEDC_INTR_DISABLE,
    //     .gpio_num = MOTOR_PWM_REAR_RIGHT,
    //     .duty = 0,
    //     .hpoint = 0,
    // };
    // ledc_channel_config(&ledc_channel_rear_right);
}

void motor_driver_setup()
{
    ledc_setup();

    gpio_reset_pin(MOTOR_DIR_FRONT_LEFT);
    gpio_set_direction(MOTOR_DIR_FRONT_LEFT, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_DIR_FRONT_LEFT, 0);

    // gpio_reset_pin(MOTOR_DIR_FRONT_RIGHT);
    // gpio_set_direction(MOTOR_DIR_FRONT_RIGHT, GPIO_MODE_OUTPUT);
    // gpio_set_level(MOTOR_DIR_FRONT_RIGHT, 0);

    // gpio_reset_pin(MOTOR_DIR_REAR_LEFT);
    // gpio_set_direction(MOTOR_DIR_REAR_LEFT, GPIO_MODE_OUTPUT);
    // gpio_set_level(MOTOR_DIR_REAR_LEFT, 0);

    // gpio_reset_pin(MOTOR_DIR_REAR_RIGHT);
    // gpio_set_direction(MOTOR_DIR_REAR_RIGHT, GPIO_MODE_OUTPUT);
    // gpio_set_level(MOTOR_DIR_REAR_RIGHT, 0);
}

void set_motor_speed_and_direction(ledc_channel_t channel, gpio_num_t dir_pin, float speed_val, int direction)
{
    if (speed_val < 0.0)
        speed_val = 0.0;
    if (speed_val > 1.0)
        speed_val = 1.0;

    gpio_set_level(dir_pin, direction);

    int duty_cycle = (int)(speed_val * ((1 << LEDC_DUTY_RESOLUTION) - 1));

    ledc_set_duty(LEDC_MODE, channel, duty_cycle);
    ledc_update_duty(LEDC_MODE, channel);
}


void encoder_setup()
{
    pcnt_config_t pcnt_config_m1 = {
        .pulse_gpio_num = ENCODER_M1_A,
        .ctrl_gpio_num = ENCODER_M1_B,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_M1,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 0,
        .counter_l_lim = 0,
    };
    pcnt_unit_config(&pcnt_config_m1);
    pcnt_counter_pause(PCNT_UNIT_M1);
    pcnt_counter_clear(PCNT_UNIT_M1);
    pcnt_counter_resume(PCNT_UNIT_M1);
}

static void IRAM_ATTR read_encoder_counts(void *arg)
{
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_M1, &count);
    pcnt_counter_clear(PCNT_UNIT_M1);
    motor1_count_change = count;
}

void app_main(void)
{
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE + 1);

    uart_init();
    motor_driver_setup();
    encoder_setup(); 

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &read_encoder_counts,
        .name = "encoder_timer"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMER_INTERVAL_MS * 1000));

    char rx_char;
    char rx_buffer[128];
    int buffer_index = 0;

    while (1)   
    {

        int len = uart_read_bytes(UART_NUM, &rx_char, 1, 10 / portTICK_PERIOD_MS);

        if (len > 0)
        {
            
            if (rx_char == '\n')
            {
                rx_buffer[buffer_index] = '\0'; 
                printf("Received data: %s\n", rx_buffer);
                
                float temp_linear_x, temp_angular_z;
                int result = sscanf(rx_buffer, "%f %f", &temp_linear_x, &temp_angular_z);

                if (result == 2)
                {
                    linear_x_val = temp_linear_x;
                    angular_z_val = temp_angular_z;
                    printf("recieved %f %f \n", linear_x_val, angular_z_val);
                }
                else
                {
                    printf("Failed to identify data\n");
                }
                
                
                buffer_index = 0;
            }
            else
            {
                
                if (buffer_index < sizeof(rx_buffer) - 1)
                {
                    rx_buffer[buffer_index++] = rx_char;
                }
                else
                {
                    
                    buffer_index = 0;
                }
            }
        }

        float speed = linear_x_val;

        if (speed > 0.4)
        {
            if (angular_z_val < -0.1)
            {
                set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_LEFT, MOTOR_DIR_FRONT_LEFT, speed * 0, 1);
                // set_motor_speed_and_direction(LEDC_CHANNEL_REAR_LEFT, MOTOR_DIR_REAR_LEFT, speed, 1);
                // set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_RIGHT, MOTOR_DIR_FRONT_RIGHT, speed, 1);
                // set_motor_speed_and_direction(LEDC_CHANNEL_REAR_RIGHT, MOTOR_DIR_REAR_RIGHT, speed * 0, 1);
                printf("Turning Right Forward\n");
            }
            else if (angular_z_val > 0.1)
            {
                set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_LEFT, MOTOR_DIR_FRONT_LEFT, speed, 1);
                // set_motor_speed_and_direction(LEDC_CHANNEL_REAR_LEFT, MOTOR_DIR_REAR_LEFT, speed * 0, 1);
                // set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_RIGHT, MOTOR_DIR_FRONT_RIGHT, speed * 0, 1);
                // set_motor_speed_and_direction(LEDC_CHANNEL_REAR_RIGHT, MOTOR_DIR_REAR_RIGHT, speed, 1);
                printf("Turning Left Forward\n");
            }
            else
            {
                set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_LEFT, MOTOR_DIR_FRONT_LEFT, speed, 1);
                // set_motor_speed_and_direction(LEDC_CHANNEL_REAR_LEFT, MOTOR_DIR_REAR_LEFT, speed, 1);
                // set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_RIGHT, MOTOR_DIR_FRONT_RIGHT, speed, 1);
                // set_motor_speed_and_direction(LEDC_CHANNEL_REAR_RIGHT, MOTOR_DIR_REAR_RIGHT, speed, 1);
                printf("Moving Straight Forward\n");
            }
        }
        else if (speed > 0.01 && speed <= 0.4)
        {
            set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_LEFT, MOTOR_DIR_FRONT_LEFT, 0.0, 1);
            // set_motor_speed_and_direction(LEDC_CHANNEL_REAR_LEFT, MOTOR_DIR_REAR_LEFT, 0.0, 1);
            // set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_RIGHT, MOTOR_DIR_FRONT_RIGHT, 0.0, 1);
            // set_motor_speed_and_direction(LEDC_CHANNEL_REAR_RIGHT, MOTOR_DIR_REAR_RIGHT, 0.0, 1);
            printf("Ball close - stopped\n");
        }
        else
        {
            set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_LEFT, MOTOR_DIR_FRONT_LEFT, 0.0, 1);
            // set_motor_speed_and_direction(LEDC_CHANNEL_REAR_LEFT, MOTOR_DIR_REAR_LEFT, 0.0, 1);
            // set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_RIGHT, MOTOR_DIR_FRONT_RIGHT, 0.0, 1);
            // set_motor_speed_and_direction(LEDC_CHANNEL_REAR_RIGHT, MOTOR_DIR_REAR_RIGHT, 0.0, 1);
            printf("Motors Stopped\n");
        }

        float motor1_velocity_rpm = (float)motor1_count_change / ENCODER_TICKS_PER_REVOLUTION * (60.0 / (TIMER_INTERVAL_MS / 1000.0));
        printf("Motor 1 Encoder Count: %d, Velocity: %.2f RPM\n", motor1_count_change, motor1_velocity_rpm);    

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
