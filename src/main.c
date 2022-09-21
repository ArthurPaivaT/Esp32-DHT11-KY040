#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "wifi.h"
#include "http_client.h"
#include "mqtt.h"
#include "dht11.h"
#include "rotary_encoder.h"

#define TAG "MAIN"

#define ROT_ENC_A_GPIO 15
#define ROT_ENC_B_GPIO 14

rotary_encoder_info_t info = {0};

#define ENABLE_HALF_STEPS false // Set to true to enable tracking of rotary encoder at half step resolution
#define RESET_AT 0              // Set to a positive non-zero number to reset the position if this value is exceeded
#define FLIP_DIRECTION false    // Set to true to reverse the clockwise/counterclockwise sense

xSemaphoreHandle conexaoWifiSemaphore;
xSemaphoreHandle conexaoMQTTSemaphore;
int ledValue;

float roomTemperature;
float referenceTemperature;

int rotaryPosition;

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define TRUE "true"
#define FALSE "false"

void conectadoWifi(void *params)
{
    while (true)
    {
        if (xSemaphoreTake(conexaoWifiSemaphore, portMAX_DELAY))
        {
            // Processamento Internet
            mqtt_start();
        }
    }
}

void trataComunicacaoComServidor(void *params)
{
    char mensagem[50];
    char exceededTemperature[6];
    char jsonAtributos[100];
    if (xSemaphoreTake(conexaoMQTTSemaphore, portMAX_DELAY))
    {
        while (true)
        {
            sprintf(mensagem, "{\"temperature\": %.2f, \"reference\": %.2f}", roomTemperature, referenceTemperature);
            printf("sending %s\n", mensagem);
            mqtt_envia_mensagem("v1/devices/me/telemetry", mensagem);

            if (roomTemperature > referenceTemperature)
            {
                strcpy(exceededTemperature, TRUE);
            }
            else
            {
                strcpy(exceededTemperature, FALSE);
            }

            printf("ledValue %d\n", ledValue);

            sprintf(jsonAtributos, "{\"rotaryPosition\": %d, \"exceededTemperature\": %s}", rotaryPosition, exceededTemperature);
            printf("sending %s\n", jsonAtributos);
            mqtt_envia_mensagem("v1/devices/me/attributes", jsonAtributos);
            vTaskDelay(3000 / portTICK_PERIOD_MS);
        }
    }
}

void temperatureReader(void *params)
{
    struct dht11_reading dht11_info;
    while (true)
    {
        dht11_info = DHT11_read();
        printf("Status code is %d\n", dht11_info.status);
        if (dht11_info.status == 0)
        {
            roomTemperature = dht11_info.temperature;
            printf("temperature is %.2f\n", roomTemperature);
            vTaskDelay(3000 / portTICK_PERIOD_MS);
        }
        else
        {
            printf("Failed to read temperature\n");
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }
}

void rotaryReader(void *params)
{
    QueueHandle_t event_queue = rotary_encoder_create_queue();
    ESP_ERROR_CHECK(rotary_encoder_set_queue(&info, event_queue));
    while (true)
    {
        // Wait for incoming events on the event queue.
        rotary_encoder_event_t event = {0};
        if (xQueueReceive(event_queue, &event, 1000 / portTICK_PERIOD_MS) == pdTRUE)
        {
            rotaryPosition = event.state.position;
            referenceTemperature = MAX(0, rotaryPosition * 5);
            ESP_LOGI(TAG, "Event: position %d, direction %s", event.state.position,
                     event.state.direction ? (event.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET");
        }
        else
        {
            // Poll current position and direction
            rotary_encoder_state_t state = {0};
            ESP_ERROR_CHECK(rotary_encoder_get_state(&info, &state));
            ESP_LOGI(TAG, "Poll: position %d, direction %s", state.position,
                     state.direction ? (state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET");

            // Reset the device
            if (RESET_AT && (state.position >= RESET_AT || state.position <= -RESET_AT))
            {
                ESP_LOGI(TAG, "Reset");
                ESP_ERROR_CHECK(rotary_encoder_reset(&info));
            }
        }
    }
}

void app_main(void)
{

    // esp32-rotary-encoder requires that the GPIO ISR service is installed before calling rotary_encoder_register()
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Initialise the rotary encoder device with the GPIOs for A and B signals
    ESP_ERROR_CHECK(rotary_encoder_init(&info, ROT_ENC_A_GPIO, ROT_ENC_B_GPIO));
    ESP_ERROR_CHECK(rotary_encoder_enable_half_steps(&info, ENABLE_HALF_STEPS));
#ifdef FLIP_DIRECTION
    ESP_ERROR_CHECK(rotary_encoder_flip_direction(&info));
#endif

    // Create a queue for events from the rotary encoder driver.
    // Tasks can read from this queue to receive up to date position information.
    QueueHandle_t event_queue = rotary_encoder_create_queue();
    ESP_ERROR_CHECK(rotary_encoder_set_queue(&info, event_queue));

    DHT11_init(GPIO_NUM_4);

    // Inicializa o NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    conexaoWifiSemaphore = xSemaphoreCreateBinary();
    conexaoMQTTSemaphore = xSemaphoreCreateBinary();
    wifi_start();
    printf("wifi_started\n");

    xTaskCreate(&rotaryReader, "Leitura do codificador rotativo", 4096, NULL, 1, NULL);
    printf("rotaryReader created\n");
    xTaskCreate(&temperatureReader, "Leitura de temperatura", 4096, NULL, 1, NULL);
    printf("temperatureReader created\n");
    xTaskCreate(&conectadoWifi, "Conexão ao MQTT", 4096, NULL, 1, NULL);
    printf("conectadoWifi created\n");
    xTaskCreate(&trataComunicacaoComServidor, "Comunicação com Broker", 4096, NULL, 1, NULL);
    printf("trataComunicacaoComServidor created\n");
}
