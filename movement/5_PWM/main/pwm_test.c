//C Headers
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdlib.h>
#define BUF_SIZE 1024
// Shared global variables for motor control
volatile float pos_x = 0.0;
volatile float pos_z = 0.0;
#include "sra_board.h"
// uart 1 nai use kar rhe coz woh usb serial ko conflict karega during idf.py monitor
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
#define UART_NUM UART_NUM_2

void uart_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';  // Null-terminate
            printf("Received: %s\n", data);

            float linear_x, angular_z;
            int items_read = sscanf((char *)data, "%f %f", &linear_x, &angular_z);

            if (items_read == 2) {
                printf("Parsed values: linear_x = %f, angular_z = %f\n", linear_x, angular_z);
                pos_x = linear_x;
                pos_z = angular_z;
            } else {
                printf("Failed to parse UART message.\n");
            }
        }
    }

    free(data);
}
void pwm_task(void *arg)
{
	motor_handle_t motor_r_0, motor_r_1, motor_l_0, motor_l_1;
	ESP_ERROR_CHECK(enable_motor_driver(&motor_r_0, MOTOR_A_0)); // Enable motor driver A0
	ESP_ERROR_CHECK(enable_motor_driver(&motor_r_1, MOTOR_A_1)); // Enable motor driver A1
    ESP_ERROR_CHECK(enable_motor_driver(&motor_l_0, MOTOR_B_0)); // Enable motor driver B1
    ESP_ERROR_CHECK(enable_motor_driver(&motor_l_1, MOTOR_B_1)); // Enable motor driver B2

	// FORWARD MOTION
	if(-0.4<pos_x<0.4)
	{
		for (int duty_cycle = 60; duty_cycle <= 100; duty_cycle++)
		{
			// setting motor speed of MOTOR A0 in forward direction with duty cycle
			set_motor_speed(motor_r_0, MOTOR_FORWARD, duty_cycle);

			// setting motor speed of MOTOR A1 in forward direction with duty cycle
			set_motor_speed(motor_r_1, MOTOR_FORWARD, duty_cycle);

            // setting motor speed of MOTOR B0 in forward direction with duty cycle
			set_motor_speed(motor_l_0, MOTOR_FORWARD, duty_cycle);

			// setting motor speed of MOTOR B1 in forward direction with duty cycle
			set_motor_speed(motor_l_1, MOTOR_FORWARD, duty_cycle);

			// adding delay of 100ms
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
        
	}
    else
    {
        for (int duty_cycle = 60; duty_cycle <= 100; duty_cycle++)
		{
            set_motor_speed(motor_r_0, MOTOR_BACKWARD, duty_cycle);
            set_motor_speed(motor_r_1, MOTOR_BACKWARD, duty_cycle);
            set_motor_speed(motor_l_1, MOTOR_FORWARD, duty_cycle);
            set_motor_speed(motor_l_1, MOTOR_FORWARD, duty_cycle);
        }

    }
    /*
    // BACKWARD MOTION 
    while()
    {
       for (int duty_cycle = 60; duty_cycle <= 100; duty_cycle++)
		{
			// setting motor speed of MOTOR A0 in backward direction with duty cycle
			set_motor_speed(motor_r_0, MOTOR_BACKWARD, duty_cycle);

			// setting motor speed of MOTOR A1 in backward direction with duty cycle
			set_motor_speed(motor_r_1, MOTOR_BACKWARD, duty_cycle);

            // setting motor speed of MOTOR B0 in backward direction with duty cycle
			set_motor_speed(motor_l_0, MOTOR_BACKWARD, duty_cycle);

			// setting motor speed of MOTOR B1 in backward direction with duty cycle
			set_motor_speed(motor_l_1, MOTOR_BACKWARD, duty_cycle);

			// adding delay of 100ms
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
 
    }
    //LEFT MOTION
    
    while()
    {
        for (int duty_cycle = 60; duty_cycle <= 100; duty_cycle++)
        {
            set_motor_speed(motor_r_0, MOTOR_FORWARD, duty_cycle);
            set_motor_speed(motor_r_1, MOTOR_BACKWARD, duty_cycle);
            set_motor_speed(motor_l_0, MOTOR_BACKWARD, duty_cycle);
            set_motor_speed(motor_l_1, MOTOR_FORWARD, duty_cycle);
            // adding delay of 100ms
			vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
    }
    //RIGHT MOTION
    while()
    {
        for (int duty_cycle = 60; duty_cycle <= 100; duty_cycle++)
        {
            set_motor_speed(motor_r_1, MOTOR_FORWARD, duty_cycle);
            set_motor_speed(motor_r_0, MOTOR_BACKWARD, duty_cycle);
            set_motor_speed(motor_l_1, MOTOR_BACKWARD, duty_cycle);
            set_motor_speed(motor_l_0, MOTOR_FORWARD, duty_cycle);
            // adding delay of 100ms
			vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
    }
    // FRONT LEFT MOTION
    while()
    {
        for (int duty_cycle = 60; duty_cycle <= 100; duty_cycle++)
        {
            set_motor_speed(motor_r_0, MOTOR_FORWARD, duty_cycle);
            // stopping the MOTOR A1
		    set_motor_speed(motor_r_1, MOTOR_STOP, 0);
            set_motor_speed(motor_l_0, MOTOR_STOP, 0);
            set_motor_speed(motor_l_1, MOTOR_FORWARD, duty_cycle);
            // adding delay of 100ms
			vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
    }
    //RIGHT BACKWARD MOTION
    while()
    {
        for (int duty_cycle = 60; duty_cycle <= 100; duty_cycle++)
        {
            set_motor_speed(motor_r_0, MOTOR_BACKWARD, duty_cycle);
		    set_motor_speed(motor_r_1, MOTOR_STOP, 0);
            set_motor_speed(motor_l_0, MOTOR_STOP, 0);
            set_motor_speed(motor_l_1, MOTOR_BACKWARD, duty_cycle);
            // adding delay of 100ms
			vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
    }
    // RIGHT FORWARD MOTION
    while()
    {
        for (int duty_cycle = 60; duty_cycle <= 100; duty_cycle++)
        {
            set_motor_speed(motor_r_1, MOTOR_FORWARD, duty_cycle);
		    set_motor_speed(motor_r_0, MOTOR_STOP, 0);
            set_motor_speed(motor_l_1, MOTOR_STOP, 0);
            set_motor_speed(motor_l_0, MOTOR_FORWARD, duty_cycle);
            // adding delay of 100ms
			vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
    }
    // LEFT BACKWARD MOTION
    while()
    {
        for (int duty_cycle = 60; duty_cycle <= 100; duty_cycle++)
        {
            set_motor_speed(motor_r_1, MOTOR_BACKWARD, duty_cycle);
		    set_motor_speed(motor_r_0, MOTOR_STOP, 0);
            set_motor_speed(motor_l_1, MOTOR_STOP, 0);
            set_motor_speed(motor_l_0, MOTOR_BACKWARD, duty_cycle);
            // adding delay of 100ms
			vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
    }
     */   
}

void app_main()
{
    // Setup UART with baud rate 115200
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Start tasks
    xTaskCreate(&uart_task, "uart_task", 2048, NULL, 10, NULL);
    xTaskCreate(&pwm_task, "pwm_task", 4096, NULL, 1, NULL);
}
