#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// UART Configuration
#define TXD_PIN GPIO_NUM_17
#define RXD_PIN GPIO_NUM_16
#define UART_NUM UART_NUM_2
#define BUF_SIZE 1024

// Forward sequence configuration (left turn + forward)
#define LEFT_PHASE_DURATION_MS 1500      // Left turn for ~1.1s
#define FORWARD_PHASE_DURATION_MS 3000   // Forward drive for ~3s
#define FORWARD_TOTAL_DURATION_MS (LEFT_PHASE_DURATION_MS + FORWARD_PHASE_DURATION_MS)

static TickType_t forward_start_time = 0;
static bool forward_mode = false;

// Servo motor at GPIO 13 configuration
#define SERVO_PIN GPIO_NUM_13
#define SERVO_ROTATION_DURATION_MS 1000
static TickType_t servo_start_time = 0;
static bool servo_rotating = false;
static bool servo_triggered = false;
static int current_servo_angle = 0;

// Global variables for motor commands
float linear_x_val = 0.0;
float angular_z_val = 0.0;

// Motor PWM and Direction Pin Definitions
#define MOTOR_PWM_FRONT_LEFT GPIO_NUM_25
#define MOTOR_PWM_FRONT_RIGHT GPIO_NUM_0
#define MOTOR_PWM_REAR_LEFT GPIO_NUM_5
#define MOTOR_PWM_REAR_RIGHT GPIO_NUM_22

#define MOTOR_DIR_FRONT_LEFT GPIO_NUM_26
#define MOTOR_DIR_FRONT_RIGHT GPIO_NUM_4
#define MOTOR_DIR_REAR_LEFT GPIO_NUM_18
#define MOTOR_DIR_REAR_RIGHT GPIO_NUM_23

// LEDC (PWM) Configuration
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_FREQUENCY 5000
#define LEDC_DUTY_RESOLUTION LEDC_TIMER_10_BIT

// Servo PWM Configuration
#define SERVO_TIMER LEDC_TIMER_1
#define SERVO_FREQUENCY 50
#define SERVO_DUTY_RESOLUTION LEDC_TIMER_10_BIT

#define LEDC_CHANNEL_FRONT_LEFT LEDC_CHANNEL_0
#define LEDC_CHANNEL_FRONT_RIGHT LEDC_CHANNEL_1
#define LEDC_CHANNEL_REAR_LEFT LEDC_CHANNEL_2
#define LEDC_CHANNEL_REAR_RIGHT LEDC_CHANNEL_3
#define LEDC_CHANNEL_SERVO LEDC_CHANNEL_4

// Servo angle to duty cycle conversion
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define SERVO_PERIOD_US 20000

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

int servo_angle_to_duty(int angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    int pulse_width_us = SERVO_MIN_PULSE_US + (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;
    int duty = (pulse_width_us * ((1 << SERVO_DUTY_RESOLUTION) - 1)) / SERVO_PERIOD_US;
    return duty;
}

void servo_set_angle(int angle)
{
    int duty = servo_angle_to_duty(angle);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_SERVO, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_SERVO);
}

void ledc_setup()
{
    // Motor timer configuration
    ledc_timer_config_t motor_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RESOLUTION,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&motor_timer);

    // Servo timer configuration
    ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = SERVO_DUTY_RESOLUTION,
        .timer_num = SERVO_TIMER,
        .freq_hz = SERVO_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&servo_timer);

    // Motor channels
    ledc_channel_config_t ledc_channels[] = {
        {.speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_FRONT_LEFT, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = MOTOR_PWM_FRONT_LEFT, .duty = 0, .hpoint = 0},
        {.speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_FRONT_RIGHT, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = MOTOR_PWM_FRONT_RIGHT, .duty = 0, .hpoint = 0},
        {.speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_REAR_LEFT, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = MOTOR_PWM_REAR_LEFT, .duty = 0, .hpoint = 0},
        {.speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_REAR_RIGHT, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = MOTOR_PWM_REAR_RIGHT, .duty = 0, .hpoint = 0},
    };

    // Servo channel
    ledc_channel_config_t servo_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_SERVO,
        .timer_sel = SERVO_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_PIN,
        .duty = servo_angle_to_duty(0),
        .hpoint = 0
    };

    for (int i = 0; i < sizeof(ledc_channels) / sizeof(ledc_channels[0]); i++)
        ledc_channel_config(&ledc_channels[i]);

    ledc_channel_config(&servo_channel);
    printf("Servo initialized at 0 degrees\n");
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
    if (speed_val < 0.0) speed_val = 0.0;
    if (speed_val > 1.0) speed_val = 1.0;
    gpio_set_level(dir_pin, direction);
    int duty_cycle = (int)(speed_val * ((1 << LEDC_DUTY_RESOLUTION) - 1));
    ledc_set_duty(LEDC_MODE, channel, duty_cycle);
    ledc_update_duty(LEDC_MODE, channel);
}

void update_detection_sequence(float linear_x, float angular_z)
{
    TickType_t current_time = xTaskGetTickCount();

    // Trigger left+forward sequence on ball detection
    if (linear_x > 0.1) {
        if (!forward_mode) {
            printf("BALL DETECTED! Starting left+forward sequence\n");
            forward_mode = true;
            forward_start_time = current_time;
            servo_triggered = false;
        }
    }

    // Handle end of forward sequence
    if (forward_mode) {
        TickType_t elapsed_time = (current_time - forward_start_time) * portTICK_PERIOD_MS;
        if (elapsed_time >= FORWARD_TOTAL_DURATION_MS) {
            printf("SEQUENCE COMPLETE - Starting servo rotation\n");
            forward_mode = false;
            if (!servo_triggered) {
                current_servo_angle += 60;
                if (current_servo_angle > 180) current_servo_angle = 60;
                servo_rotating = true;
                servo_start_time = current_time;
                servo_triggered = true;
                servo_set_angle(current_servo_angle);
            }
        }
    }
}

void update_servo()
{
    if (servo_rotating) {
        TickType_t current_time = xTaskGetTickCount();
        TickType_t elapsed_time = (current_time - servo_start_time) * portTICK_PERIOD_MS;
        if (elapsed_time >= SERVO_ROTATION_DURATION_MS) {
            printf("SERVO ROTATION COMPLETE - Staying at %d degrees\n", current_servo_angle);
            servo_rotating = false;
        }
    }
}

void app_main(void)
{
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE + 1);
    uart_init();
    motor_driver_setup();

    char rx_char;
    char rx_buffer[128];
    int buffer_index = 0;

    while (1) {
        int len = uart_read_bytes(UART_NUM, &rx_char, 1, 10 / portTICK_PERIOD_MS);

        if (len > 0) {
            if (rx_char == '\n') {
                rx_buffer[buffer_index] = '\0';
                printf("Received data: %s\n", rx_buffer);

                float temp_linear_x, temp_angular_z;
                int result = sscanf(rx_buffer, "%f %f", &temp_linear_x, &temp_angular_z);

                if (result == 2) {
                    linear_x_val = temp_linear_x;
                    angular_z_val = temp_angular_z;
                    update_detection_sequence(linear_x_val, angular_z_val);
                }
                buffer_index = 0;
            } else {
                if (buffer_index < sizeof(rx_buffer) - 1)
                    rx_buffer[buffer_index++] = rx_char;
                else
                    buffer_index = 0;
            }
        }

        TickType_t current_time = xTaskGetTickCount();
        update_detection_sequence(linear_x_val, angular_z_val);
        update_servo();

        // MOTOR CONTROL LOGIC
        if (servo_rotating) {
            printf("SERVO ACTIVE: All wheels stopped, rotating servo\n");
            set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_LEFT, MOTOR_DIR_FRONT_LEFT, 0, 1);
            set_motor_speed_and_direction(LEDC_CHANNEL_REAR_LEFT, MOTOR_DIR_REAR_LEFT, 0, 1);
            set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_RIGHT, MOTOR_DIR_FRONT_RIGHT, 0, 1);
            set_motor_speed_and_direction(LEDC_CHANNEL_REAR_RIGHT, MOTOR_DIR_REAR_RIGHT, 0, 1);
        }
        else if (forward_mode) {
            TickType_t elapsed = (current_time - forward_start_time) * portTICK_PERIOD_MS;

            if (elapsed < LEFT_PHASE_DURATION_MS) {
                // LEFT TURN
                printf("FORWARD MODE: LEFT TURN PHASE\n");
                set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_LEFT, MOTOR_DIR_FRONT_LEFT, 0, 0);
                set_motor_speed_and_direction(LEDC_CHANNEL_REAR_LEFT, MOTOR_DIR_REAR_LEFT, 0.5, 0);
                set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_RIGHT, MOTOR_DIR_FRONT_RIGHT, 0.5, 0);
                set_motor_speed_and_direction(LEDC_CHANNEL_REAR_RIGHT, MOTOR_DIR_REAR_RIGHT, 0, 0);
            }
            else if (elapsed < FORWARD_TOTAL_DURATION_MS) {
                // STRAIGHT DRIVE
                printf("FORWARD MODE: STRAIGHT DRIVE PHASE\n");
                set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_LEFT, MOTOR_DIR_FRONT_LEFT, 0.5, 1);
                set_motor_speed_and_direction(LEDC_CHANNEL_REAR_LEFT, MOTOR_DIR_REAR_LEFT, 0.5, 1);
                set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_RIGHT, MOTOR_DIR_FRONT_RIGHT, 0.5, 0);
                set_motor_speed_and_direction(LEDC_CHANNEL_REAR_RIGHT, MOTOR_DIR_REAR_RIGHT, 0.5, 0);
            }
        }
        else {
            // SEARCH MODE
            if (angular_z_val > 0) {
                printf("SEARCH MODE: Turning Left\n");
                float speed = (angular_z_val > 0) ? angular_z_val : -angular_z_val;
                set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_LEFT, MOTOR_DIR_FRONT_LEFT, speed, 0);
                set_motor_speed_and_direction(LEDC_CHANNEL_REAR_LEFT, MOTOR_DIR_REAR_LEFT, speed, 0);
                set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_RIGHT, MOTOR_DIR_FRONT_RIGHT, speed, 1);
                set_motor_speed_and_direction(LEDC_CHANNEL_REAR_RIGHT, MOTOR_DIR_REAR_RIGHT, speed, 1);
            }
            else if (angular_z_val < 0) {
                printf("SEARCH MODE: Turning Right\n");
                set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_LEFT, MOTOR_DIR_FRONT_LEFT, 0, 1);
                set_motor_speed_and_direction(LEDC_CHANNEL_REAR_LEFT, MOTOR_DIR_REAR_LEFT, 0.5, 1);
                set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_RIGHT, MOTOR_DIR_FRONT_RIGHT, 0.5, 1);
                set_motor_speed_and_direction(LEDC_CHANNEL_REAR_RIGHT, MOTOR_DIR_REAR_RIGHT, 0, 1);
            }
            else {
                printf("SEARCH MODE: Stopped\n");
                set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_LEFT, MOTOR_DIR_FRONT_LEFT, 0, 1);
                set_motor_speed_and_direction(LEDC_CHANNEL_REAR_LEFT, MOTOR_DIR_REAR_LEFT, 0, 1);
                set_motor_speed_and_direction(LEDC_CHANNEL_FRONT_RIGHT, MOTOR_DIR_FRONT_RIGHT, 0, 1);
                set_motor_speed_and_direction(LEDC_CHANNEL_REAR_RIGHT, MOTOR_DIR_REAR_RIGHT, 0, 1);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
