// josette vigil 2024


#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include <driver/gpio.h>
#include <driver/timer.h>
#include <driver/i2c.h>
#include <esp_timer.h>


#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "sdkconfig.h"

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "10.0.0.57"
#define WEB_PORT "1234"
#define WEB_PATH "/"

// preprocessor for shtc3
#define I2C_MASTER_SCL_IO           8       // GPIO number for I2C SCL
#define I2C_MASTER_SDA_IO           10      // GPIO number for I2C SDA
#define I2C_MASTER_FREQ_HZ          50000  // I2C master clock frequency
#define I2C_MASTER_PORT_NUM         I2C_NUM_0
#define SHTC3_I2C_ADDRESS           0x70    // SHTC3 I2C address

#define WAKEUP_CMD                  0x3517
#define MEASUREMENT_CMD             0x7CA2
#define SLEEP_CMD                   0xB098


// shtc3 function declarations
static esp_err_t init_i2c(void);
void shtc3_wakeup(void);
void shtc3_sleep(void);
bool shtc3_read_measurement(uint16_t* temperature, uint16_t* humidity);
uint8_t calculate_crc(uint16_t value);
void get_temp_and_hum(float *temp_fl, float *hum_fl);

static const char *TAG = "lab7_1";

static const char *PAYLOAD = "Temperature: %.2f C\nHumidity: %.2f%\n";
static const char *REQUEST = "POST " WEB_PATH " HTTP/1.0\r\n"
    "Host: "WEB_SERVER":"WEB_PORT"\r\n"
    "User-Agent: esp-idf/1.0 esp32\r\n"
    "Content-Type: text/plain\r\n"
    "Content-Length: %d\r\n"
    "\r\n"
    "%s";

// wrapper for shtc3 temp reading
void get_temp_and_hum(float *temp_fl, float *hum_fl)
{
        shtc3_wakeup();
        vTaskDelay(5 / portTICK_PERIOD_MS);
        uint16_t raw_temp = 0, raw_humidity = 0;
        if (shtc3_read_measurement(&raw_temp, &raw_humidity)) {
            *temp_fl = -45.0 + 175.0 * ((float)raw_temp / 65535.0);
            *hum_fl = 100.0 * ((float)raw_humidity / 65535.0);
        } else {
            printf("Measurement failed\n");
        }
        
        shtc3_sleep();
}

static void http_post_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];

    while(1) {
        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);
	
	float temp, hum;
	get_temp_and_hum(&temp, &hum);
	char payload[64];
	int payload_len = snprintf(payload, sizeof(payload), PAYLOAD, temp, hum);
	char request[512];
	int request_len = snprintf(request, sizeof(request), REQUEST, payload_len, payload);
//	ESP_LOGI(TAG, "%s\n", request);

        if (write(s, request, request_len) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        close(s);
        for(int countdown = 10; countdown >= 0; countdown--) {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Starting again!");
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(init_i2c());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(&http_post_task, "http_post_task", 4096, NULL, 5, NULL);
}

// Initialize the I2C master
static esp_err_t init_i2c(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    return i2c_param_config(I2C_MASTER_PORT_NUM, &conf) | i2c_driver_install(I2C_MASTER_PORT_NUM, conf.mode, 0, 0, 0);
}

// Wakeup the SHTC3 sensor
void shtc3_wakeup(void) {
    uint8_t cmd[2] = { WAKEUP_CMD >> 8, WAKEUP_CMD & 0xFF };
//    printf("wakeup: %x%x\n", cmd[0], cmd[1]);
    vTaskDelay(1/portTICK_PERIOD_MS);
    i2c_master_write_to_device(I2C_MASTER_PORT_NUM, SHTC3_I2C_ADDRESS, cmd, 2, 1000 / portTICK_PERIOD_MS);
}

// Put the SHTC3 sensor into sleep mode
void shtc3_sleep(void) {
    uint8_t cmd[2] = { SLEEP_CMD >> 8, SLEEP_CMD & 0xFF };
    printf("sleep: %x%x\n", cmd[0], cmd[1]);
    i2c_master_write_to_device(I2C_MASTER_PORT_NUM, SHTC3_I2C_ADDRESS, cmd, 2, 1000 / portTICK_PERIOD_MS);
}

// Function to read both temperature and humidity data from the SHTC3 sensor
bool shtc3_read_measurement(uint16_t* temperature, uint16_t* humidity) {
    uint8_t cmd[2] = { MEASUREMENT_CMD >> 8, MEASUREMENT_CMD & 0xFF };
    printf("measure: %x%x\n", cmd[0], cmd[1]);
    i2c_master_write_to_device(I2C_MASTER_PORT_NUM, SHTC3_I2C_ADDRESS, cmd, 2, 1000 / portTICK_PERIOD_MS);
    
    // Wait for the measurement to complete (typically takes 10-15ms)
    vTaskDelay(35 / portTICK_PERIOD_MS);
    
    uint8_t data[6];
    i2c_master_read_from_device(I2C_MASTER_PORT_NUM, SHTC3_I2C_ADDRESS, data, 6, 1000 / portTICK_PERIOD_MS);
   
    // DEBUG
//    for (int i = 0; i < 6; i++){
//   	printf("byte %d: %x\n", i, data[i]);
//    }

    //vTaskDelay(20 / portTICK_PERIOD_MS);

    // Read and validate temperature
    *temperature = (data[0] << 8) | data[1];
    uint8_t temp_crc = data[2];
    if (calculate_crc(*temperature) != temp_crc) {
        printf("Temperature CRC check failed: calculated CRC = %02X, received CRC = %02X\n", calculate_crc(*temperature), temp_crc);
        return false;
    }
    
    // Read and validate humidity
    *humidity = (data[3] << 8) | data[4];
    uint8_t humidity_crc = data[5];
    if (calculate_crc(*humidity) != humidity_crc) {
        printf("Humidity CRC check failed: calculated CRC = %02X, received CRC = %02X\n", calculate_crc(*humidity), humidity_crc);
        return false;
    }
    
    return true;
}

// CRC calculation based on SHTC3 datasheet
uint8_t calculate_crc(uint16_t value) {
    uint8_t crc = 0xFF; // Start with a CRC value of 0xFF
    uint8_t data[2] = { value >> 8, value & 0xFF }; // Split the value into two bytes

    for (int i = 0; i < 2; i++) {
        crc ^= data[i]; // XOR the data byte with the CRC
        for (int j = 0; j < 8; j++) { // Process each bit
            if (crc & 0x80) { // If the highest bit is set
                crc = (crc << 1) ^ 0x31; // Shift left and XOR with polynomial
            } else {
                crc <<= 1; // Just shift left
            }
        }
    }
    
    return crc; // Return the calculated CRC
}

