#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hx711.h>
#include <max31855.h>
#include <esp_idf_lib_helpers.h>
#include <inttypes.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_random.h>
#include <driver/uart.h>
#include <sdmmc_cmd.h>
#include <driver/sdmmc_host.h>
#include <driver/sdmmc_defs.h>
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
#define sdCS 0
#define sdMOSI 23
#define sdCLK  18
#define sdMISO 19
#define bfLINES 650
#define bfLENGTH 128
#define sdMOUNT "/sdcard"
#define sHOST HSPI_HOST


const gpio_num_t cs_pins[NUM_SENSORS] = { maxCS1, maxCS2, maxCS3 };
char dataBF[bfLINES][bfLENGTH];
int bfINDEX = 0;

void dataLog_task(void *pvParameters)
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
    ESP_ERROR_CHECK(spi_bus_initialize(sHOST, &max, 1));
    
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        ESP_ERROR_CHECK(max31855_init_desc(&devs[i], sHOST, MAX31855_MAX_CLOCK_SPEED_HZ, cs_pins[i]));
    }


    // UART initializer
    ESP_ERROR_CHECK(uart_driver_install(loraUART, loraBUF, loraBUF, 10, 0, 0));

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
    ESP_LOGI("TEST","TEST STARTING!\n");
    const char *header = "timestamp_ms,thrust,temp1,temp2,temp3\n";
    ESP_LOGI("TEST","%s",header); 

    // Send over UART
    uart_write_bytes(loraUART, header, strlen(header)); 

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    while (esp_timer_get_time()/1000<=6300)
    {
        // Start collecting
        int64_t timestamp_ms = esp_timer_get_time() / 1000;

        // HX711
        bool valid = true;
        esp_err_t hxR = hx711_wait(&hx, 10);
        int32_t thrust = 0;
        if (hxR == ESP_OK)
        {
            esp_err_t errH = hx711_read_average(&hx, 5, &thrust);
            if (errH != ESP_OK) {
                valid = false;
                //break;
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
                //break;
            }
            temps[i] = temp;
        }

        //char dataLog[128];
        //if (valid)
        //{
        //    snprintf(dataLog, sizeof(dataLog), "%lld,%" PRIi32 ",%.2f,%.2f,%.2f\n", timestamp_ms, thrust, temps[0], temps[1], temps[2] );
        //    ESP_LOGI("TEST","%s", dataLog);
        //    uart_write_bytes(loraUART, dataLog, strlen(dataLog)); // Ignore ESP_ERROR_CHECK
            //ESP_ERROR_CHECK(uart_write_bytes(UART_NUM, dataLog, strlen(dataLog))); // ESP_ERROR_CHECK -> abort(), if E220 not connected
        if (valid && bfINDEX < bfLINES)
        {
            snprintf(dataBF[bfINDEX], bfLENGTH, "%lld,%" PRIi32 ",%.2f,%.2f,%.2f\n", timestamp_ms, thrust, temps[0], temps[1], temps[2]);
            ESP_LOGI("TEST","%s", dataBF[bfINDEX]);
            uart_write_bytes(loraUART, dataBF[bfINDEX], strlen(dataBF[bfINDEX])); // Ignore ESP_ERROR_CHECK
            //ESP_ERROR_CHECK(uart_write_bytes(loraUART, dataBF[bfINDEX], strlen(dataBF[bfINDEX])); // ESP_ERROR_CHECK -> abort(), if E220 not connected

            bfINDEX++;        
        }else
        {
            ESP_LOGW("DATALOG", "Sensor error");
        }
        
        ESP_ERROR_CHECK(esp_task_wdt_reset());  // Reset watchdog timer
        vTaskDelay(pdMS_TO_TICKS(10)); // 1 sample per 10ms
        
        //vTaskDelay(pdMS_TO_TICKS(1000)); // 1 sample per 1000ms 
    }
    // Free devices
    for (int i = 0; i < NUM_SENSORS; i++) {
        max31855_free_desc(&devs[i]);
    }
    // Free bus
    spi_bus_free(sHOST);

    ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
    fflush(stdout);
    vTaskDelete(NULL);
}

void sd_init()
{
    esp_err_t errSD;
    sdmmc_card_t *card;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = VSPI_HOST;
    
    // SD config
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = sdMOSI,
        .miso_io_num = sdMISO,
        .sclk_io_num = sdCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ESP_LOGI("SD", "Initializing SD card");
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = sdCS;
    slot_config.host_id = host.slot;
    printf("%lld\n", esp_timer_get_time()/1000);
    // SPI initializer
    ESP_LOGI("SD", "Initializing SPI bus");
    errSD = spi_bus_initialize(VSPI_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (errSD != ESP_OK) {
        ESP_LOGE("SD", "Failed to initialize SPI bus: %s", esp_err_to_name(errSD));
        return;
    }
    ESP_LOGI("SD", "SPI bus initialized");

    // System mount
    ESP_LOGI("SD", "Mounting system");
    errSD = esp_vfs_fat_sdspi_mount(sdMOUNT, &host, &slot_config, &mount_cfg, &card);
    if (errSD != ESP_OK)
    {
        ESP_LOGE("SD", "Failed to mount system.");
        return;
    }
    ESP_LOGI("SD", "System mounted");

    // Create log file
    int64_t rdID = esp_random() % 10000; // Random id
    char filename[64];
    snprintf(filename, sizeof(filename), sdMOUNT "/log_%lld.csv",  rdID);

    FILE* f = fopen(filename, "w");
    if (f == NULL) {
        ESP_LOGE("SD", "Failed to open file: %s", filename);
        esp_vfs_fat_sdcard_unmount(sdMOUNT, card);
        return;
    }

    //Write data
    fprintf(f, "timestamp_ms,thrust,temp1,temp2,temp3\n");
    for (int i = 0; i < bfINDEX; i++) {
        fprintf(f, "%s", dataBF[i]);
    }
    fclose(f);
    esp_vfs_fat_sdcard_unmount(sdMOUNT , card);
    ESP_LOGI("SD", "Saved log to: %s", filename);
    ESP_LOGI("SD", "Saved %d samples to: %s", bfINDEX, filename);
    spi_bus_free(VSPI_HOST);
}

void app_main()
{
 
    TaskHandle_t dataLogHandle = NULL;
    xTaskCreate(dataLog_task, "Data Log", configMINIMAL_STACK_SIZE * 5, NULL, 5, &dataLogHandle);

    // Wait for the task to complete
    while (eTaskGetState(dataLogHandle) != eDeleted) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    sd_init();
}

