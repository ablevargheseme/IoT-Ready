
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_sleep.h"

static const char *TAG = "IoTready";

#define EX_UART_NUM UART_NUM_0
#define TEST_RTS (UART_PIN_NO_CHANGE)  //replace with rts pin number
#define TEST_CTS (UART_PIN_NO_CHANGE)  //replace with cts pin number

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;


uint8_t* ranBuffer;

static void uart_event_sub_task(void *pvParameters)
{  

ESP_LOGI(TAG, "uart event sub task");
uart_write_bytes(EX_UART_NUM, (const char*) ranBuffer,128);
uart_wait_tx_idle_polling(EX_UART_NUM);
/* Enter sleep mode */
 esp_light_sleep_start();
 vTaskDelete(NULL);
}
static void uart_event_task(void *pvParameters)
{   


     uart_event_t event;
    size_t buffered_size;

    uint8_t* rxBuffer = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(rxBuffer, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
         
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, rxBuffer, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[DATA EVT]:");
                    //echo rx data
                     uart_write_bytes(EX_UART_NUM, (const char*) rxBuffer, event.size);
                     break;
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                   //Event of UART TX break detected
                case UART_DATA_BREAK:
                    ESP_LOGI(TAG, "uart Tx break");
                    ESP_LOGI(TAG, "ACTIVATING  TX task");
                    xTaskCreate(uart_event_sub_task, "uart_event_sub_task", 2048, NULL,11, NULL);
                    break;

                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(rxBuffer);
    rxBuffer = NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{  
     //initialzing buffer of 128 bytes 
    ranBuffer=(uint8_t*)malloc(128);
    //filling buffer with random chars
    esp_fill_random(ranBuffer,128);
    esp_log_level_set(TAG, ESP_LOG_INFO);


    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
       // .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
         .flow_ctrl  =UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
      //  .rx_flow_ctrl_thresh = 122,
    };
    //Installing uart driver
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    

  
    uart_set_pin(EX_UART_NUM,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,TEST_RTS,TEST_CTS);

    //wake up on uart0 (external module unsolicited)
    uart_set_wakeup_threshold(UART_NUM_0, 3);
    esp_sleep_enable_uart_wakeup(UART_NUM_0);

    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}
  