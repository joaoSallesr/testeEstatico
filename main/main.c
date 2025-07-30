#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hx711.h>
#include <max31855.h>
#include <esp_idf_lib_helpers.h>
#include <inttypes.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <driver/uart.h>
#include <sdmmc_cmd.h>
#include <driver/sdspi_host.h>
#include <esp_vfs_fat.h>

#define hxData 4
#define hxClock 2
#define maxClock 14
#define maxCS1 25
#define maxCS2 26
#define maxCS3 33
#define NUM_SENSORS 3
#define loraRX 17
#define loraTX 16
#define loraUART 1
#define loraBAUD 9600
#define loraBUF (1024)
#define sdCS   34
#define sdMOSI 23
#define sdCLK  18
#define sdMISO 19
#define HOST HELPER_SPI_HOST_DEFAULT
//#define DONT_USE_MOCK_DATA 1

const gpio_num_t cs_pins[NUM_SENSORS] = { maxCS1, maxCS2, maxCS3 };
//QueueHandle_t uart_queue;


void dataLog_task()
{
    // HX711 initializer
    hx711_t hx = {
        .dout = hxData,
        .pd_sck = hxClock,
        .gain = HX711_GAIN_A_64
    };
    ESP_ERROR_CHECK(hx711_init(&hx));


    // MAX31855 initializer
    max31855_t devs[NUM_SENSORS] = { 0 };
    
    spi_bus_config_t max = {
       .mosi_io_num = -1,
       .miso_io_num = -1,
       .sclk_io_num = maxClock,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0,
       .flags = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HOST, &max, 1));
    
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        ESP_ERROR_CHECK(max31855_init_desc(&devs[i], HOST, MAX31855_MAX_CLOCK_SPEED_HZ, cs_pins[i]));
    }


    // UART initializer
    ESP_ERROR_CHECK(uart_driver_install(1, loraBUF, loraBUF, 10, 0, 0));

    uart_config_t uart_config = {
    .baud_rate = loraBAUD,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Configure UART parameters
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(uart_param_config(loraUART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(loraUART, loraTX, loraRX, -1, -1));
    

    // Print to terminal
    printf ("TEST STARTING!\n");
    const char *header = "timestamp_ms,thrust,temp1,temp2,temp3\n";
    printf("%s",header); 

    // Send over UART
    uart_write_bytes(loraUART, header, strlen(header)); 


    while (1)
    {
        // HX711
        bool valid = true;
        esp_err_t hxR = hx711_wait(&hx, 500);
        int32_t thrust = 0;
        if (hxR == ESP_OK)
        {
            esp_err_t errH = hx711_read_average(&hx, 5, &thrust);
            if (errH != ESP_OK) {
                valid = false;
                break;
            }
        }else
        {
            ESP_LOGE("HX711", "Device not ready: %d (%s)\n", hxR, esp_err_to_name(hxR));
        }

        // MAX31855
        float temps[NUM_SENSORS] = { 0 };

        for (int i = 0; i < NUM_SENSORS; i++)
        {
            float temp;
            bool scv, scg, oc;
            esp_err_t errM = max31855_get_temperature(&devs[i], &temp, NULL, &scv, &scg, &oc);
            if (errM != ESP_OK || scv || scg || oc) {
                valid = false;
                break;
            }
            temps[i] = temp;
        }

        int64_t timestamp_ms = esp_timer_get_time() / 1000;
        char dataLog[128];
        if (valid)
        {
            snprintf(dataLog, sizeof(dataLog), "%lld,%" PRIi32 ",%.2f,%.2f,%.2f\n", timestamp_ms, thrust, temps[0], temps[1], temps[2] );
            printf("%s", dataLog);
            uart_write_bytes(loraUART, dataLog, strlen(dataLog)); // Ignore ESP_ERROR_CHECK
            //ESP_ERROR_CHECK(uart_write_bytes(UART_NUM, dataLog, strlen(dataLog))); // ESP_ERROR_CHECK -> abort(), if E220 not connected
        }else
        {
            ESP_LOGW("DATALOG", "Sensor error");
        }

        //vTaskDelay(pdMS_TO_TICKS(10)); // 1 sample per 10ms
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 sample per 1000ms 
    }
}

void sd_init(){} //

void app_main()
{
    xTaskCreate(dataLog_task, "Data Log", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL); // Data Log Task
}

///TAREFAS

// Postar no github
// Implementar o adaptrador para Micro SD
// Implementação com arquivo .CSV
// Implementar a Tara para o HX711 -> Newtons?
// Fazer o ESP32 parar a coleta após 10 segundos (10?)


///PERGUNTAS

// É necessário usar ESP_ERROR_CHECK antes de toda função?
// Configuração da frequência dos loras
// Arquivo .CSV
// void dataLog_task(void *pvParameters)