#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTOR_PWM_FRONT_LEFT  GPIO_NUM_25
#define MOTOR_PWM_FRONT_RIGHT GPIO_NUM_0
#define MOTOR_PWM_REAR_LEFT   GPIO_NUM_5
#define MOTOR_PWM_REAR_RIGHT  GPIO_NUM_22

#define MOTOR_DIR_FRONT_LEFT  GPIO_NUM_26
#define MOTOR_DIR_FRONT_RIGHT GPIO_NUM_4
#define MOTOR_DIR_REAR_LEFT   GPIO_NUM_18
#define MOTOR_DIR_REAR_RIGHT  GPIO_NUM_23

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_FREQUENCY      5000
#define LEDC_DUTY_RESOLUTION LEDC_TIMER_10_BIT

#define LEDC_CHANNEL_FRONT_LEFT  LEDC_CHANNEL_0
#define LEDC_CHANNEL_FRONT_RIGHT LEDC_CHANNEL_1
#define LEDC_CHANNEL_REAR_LEFT   LEDC_CHANNEL_2
#define LEDC_CHANNEL_REAR_RIGHT  LEDC_CHANNEL_3

#define ENCODER_M1_A GPIO_NUM_32
#define ENCODER_M1_B GPIO_NUM_33
#define ENCODER_M2_A GPIO_NUM_15
#define ENCODER_M2_B GPIO_NUM_2
#define ENCODER_M3_A GPIO_NUM_14
#define ENCODER_M3_B GPIO_NUM_12
#define ENCODER_M4_A GPIO_NUM_19
#define ENCODER_M4_B GPIO_NUM_21

#define PCNT_UNIT_M1 PCNT_UNIT_0
#define PCNT_UNIT_M2 PCNT_UNIT_1
#define PCNT_UNIT_M3 PCNT_UNIT_2
#define PCNT_UNIT_M4 PCNT_UNIT_3

#define ENCODER_TICKS_PER_REVOLUTION 3960.0
#define TIMER_INTERVAL_MS 100

static volatile int16_t motor1_count_change = 0;
static volatile int16_t motor2_count_change = 0;
static volatile int16_t motor3_count_change = 0;
static volatile int16_t motor4_count_change = 0;

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

    ledc_channel_config_t ledc_channels[] = {
        {.speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_FRONT_LEFT, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = MOTOR_PWM_FRONT_LEFT, .duty = 0, .hpoint = 0},
        {.speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_FRONT_RIGHT, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = MOTOR_PWM_FRONT_RIGHT, .duty = 0, .hpoint = 0},
        {.speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_REAR_LEFT, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = MOTOR_PWM_REAR_LEFT, .duty = 0, .hpoint = 0},
        {.speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_REAR_RIGHT, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = MOTOR_PWM_REAR_RIGHT, .duty = 0, .hpoint = 0},
    };

    for (int i = 0; i < sizeof(ledc_channels) / sizeof(ledc_channels[0]); i++)
    {
        ledc_channel_config(&ledc_channels[i]);
    }
}

void motor_driver_setup()
{
    ledc_setup();

    gpio_set_direction(MOTOR_DIR_FRONT_LEFT, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_DIR_FRONT_RIGHT, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_DIR_REAR_LEFT, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_DIR_REAR_RIGHT, GPIO_MODE_OUTPUT);
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
    pcnt_config_t pcnt_config_template = {
        .pulse_gpio_num = 0,
        .ctrl_gpio_num = 0,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 0,
        .counter_l_lim = 0,
    };
    
    pcnt_config_t pcnt_config_m1 = pcnt_config_template;
    pcnt_config_m1.pulse_gpio_num = ENCODER_M1_A;
    pcnt_config_m1.ctrl_gpio_num = ENCODER_M1_B;
    pcnt_config_m1.unit = PCNT_UNIT_M1;
    pcnt_unit_config(&pcnt_config_m1);

    pcnt_config_t pcnt_config_m2 = pcnt_config_template;
    pcnt_config_m2.pulse_gpio_num = ENCODER_M2_A;
    pcnt_config_m2.ctrl_gpio_num = ENCODER_M2_B;
    pcnt_config_m2.unit = PCNT_UNIT_M2;
    pcnt_unit_config(&pcnt_config_m2);

    pcnt_config_t pcnt_config_m3 = pcnt_config_template;
    pcnt_config_m3.pulse_gpio_num = ENCODER_M3_A;
    pcnt_config_m3.ctrl_gpio_num = ENCODER_M3_B;
    pcnt_config_m3.unit = PCNT_UNIT_M3;
    pcnt_unit_config(&pcnt_config_m3);

    pcnt_config_t pcnt_config_m4 = pcnt_config_template;
    pcnt_config_m4.pulse_gpio_num = ENCODER_M4_A;
    pcnt_config_m4.ctrl_gpio_num = ENCODER_M4_B;
    pcnt_config_m4.unit = PCNT_UNIT_M4;
    pcnt_unit_config(&pcnt_config_m4);

    pcnt_counter_pause(PCNT_UNIT_M1);
    pcnt_counter_clear(PCNT_UNIT_M1);
    pcnt_counter_resume(PCNT_UNIT_M1);

    pcnt_counter_pause(PCNT_UNIT_M2);
    pcnt_counter_clear(PCNT_UNIT_M2);
    pcnt_counter_resume(PCNT_UNIT_M2);

    pcnt_counter_pause(PCNT_UNIT_M3);
    pcnt_counter_clear(PCNT_UNIT_M3);
    pcnt_counter_resume(PCNT_UNIT_M3);

    pcnt_counter_pause(PCNT_UNIT_M4);
    pcnt_counter_clear(PCNT_UNIT_M4);
    pcnt_counter_resume(PCNT_UNIT_M4);
}

static void IRAM_ATTR read_encoder_counts(void *arg)
{
    int16_t count;
    pcnt_get_counter_value(PCNT_UNIT_M1, &count);
    pcnt_counter_clear(PCNT_UNIT_M1);
    motor1_count_change = count;

    pcnt_get_counter_value(PCNT_UNIT_M2, &count);
    pcnt_counter_clear(PCNT_UNIT_M2);
    motor2_count_change = count;

    pcnt_get_counter_value(PCNT_UNIT_M3, &count);
    pcnt_counter_clear(PCNT_UNIT_M3);
    motor3_count_change = count;

    pcnt_get_counter_value(PCNT_UNIT_M4, &count);
    pcnt_counter_clear(PCNT_UNIT_M4);
    motor4_count_change = count;
}

void app_main(void)
{
    motor_driver_setup();
    encoder_setup();

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &read_encoder_counts,
        .name = "encoder_timer"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, TIMER_INTERVAL_MS * 1000));

    float fixed_speed = 0.8;
    set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_LEFT, MOTOR_DIR_FRONT_LEFT, fixed_speed, 1);
    set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_RIGHT, MOTOR_DIR_FRONT_RIGHT, fixed_speed, 1);
    set_motor_speed_and_direction(LEDC_CHANNEL_REAR_LEFT, MOTOR_DIR_REAR_LEFT, fixed_speed, 1);
    set_motor_speed_and_direction(LEDC_CHANNEL_REAR_RIGHT, MOTOR_DIR_REAR_RIGHT, fixed_speed, 1);
    printf("All motors started at speed: %.2f\n", fixed_speed);

    while (1)
    {
        float motor1_velocity_rpm = (float)motor1_count_change / ENCODER_TICKS_PER_REVOLUTION * (60.0 / (TIMER_INTERVAL_MS / 1000.0));
        float motor2_velocity_rpm = (float)motor2_count_change / ENCODER_TICKS_PER_REVOLUTION * (60.0 / (TIMER_INTERVAL_MS / 1000.0));
        float motor3_velocity_rpm = (float)motor3_count_change / ENCODER_TICKS_PER_REVOLUTION * (60.0 / (TIMER_INTERVAL_MS / 1000.0));
        float motor4_velocity_rpm = (float)motor4_count_change / ENCODER_TICKS_PER_REVOLUTION * (60.0 / (TIMER_INTERVAL_MS / 1000.0));
        
        printf("M1: Count=%d, Vel=%.2f RPM | M2: Count=%d, Vel=%.2f RPM | M3: Count=%d, Vel=%.2f RPM | M4: Count=%d, Vel=%.2f RPM\n",
               motor1_count_change, motor1_velocity_rpm,
               motor2_count_change, motor2_velocity_rpm,
               motor3_count_change, motor3_velocity_rpm,
               motor4_count_change, motor4_velocity_rpm);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}