#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <string.h>

#define TXD_PIN GPIO_NUM_17
#define RXD_PIN GPIO_NUM_16
#define UART_NUM UART_NUM_2
#define BUFF_SIZE 1024

void uart_init(){

    const uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
};

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUFF_SIZE * 2, 0, 0, NULL, 0);
}

#define MOTOR_PIN GPIO_NUM_18

void motor_init(){
    gpio_set_direction(MOTOR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_PIN, 0);
}

void motor_run() {
    gpio_set_level(MOTOR_PIN, 1);
}

void motor_stop() {
    gpio_set_level(MOTOR_PIN, 0);
}

#include <stdlib.h>

void app_main(void) {
    uart_init();
    motor_init();
    
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE + 1);

    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        
        if (len > 0) {
            data[len] = '\0';
            printf("Recived data ( %s )\n" , (char*)data);
            
            float linear_x, angular_z;
            int result = sscanf((char *)data, "%f %f", &linear_x, &angular_z);

            if (result == 2) {
                printf("Values: linear_x=%f, angular_z=%f \n", linear_x, angular_z);

                if (linear_x > 0.4) {
                    motor_run();
                    printf("Motor Running \n");
                } else {
                    motor_stop();
                    printf("Motor Stopped\n");
                }
            } else {
                printf("failed to identify data\n");
            }
        }
    }

    free(data);
}

