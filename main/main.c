#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include <sys/socket.h>
#include <esp_http_server.h>

TaskHandle_t task_1;
TaskHandle_t task_2;
QueueHandle_t xQueue1;
EventGroupHandle_t sensor_event;
int listen_sock;

void blink_led(void*)
{
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    xEventGroupWaitBits(sensor_event, (1 << 2), pdTRUE, pdFALSE, pdMS_TO_TICKS(0xffffffff));
    while (1) {
        char rx_buffer[32] = {0};
        int len = recv(listen_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        printf("[data recv]: %s\n", rx_buffer);
        if(strstr(rx_buffer, "led on") != 0)
        {
            gpio_set_level(GPIO_NUM_2, 1);
            
        }
        else if(strstr(rx_buffer, "led off") != 0)
        {
            gpio_set_level(GPIO_NUM_2, 0);
        }
    }
}

int global_var;
int wait_reading_data_done_flag = 0;
void read_sensor_data(void* param)
{
    int ss_data = 0;
    int cnt = 0;
    while(1)
    {
        ss_data ++;
        // global_var = ss_data;   //write data to global variable
        xQueueSend(xQueue1, &ss_data, pdMS_TO_TICKS(10000));
        cnt++;
        if(cnt == 10) {
            // wait_reading_data_done_flag = 1;    // set flag (event)
            xEventGroupSetBits(sensor_event, 1 << 0);
            cnt = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

    }
}

static esp_err_t ledon_get_handler(httpd_req_t *req)
{
    const char* resp_str = (const char*) req->user_ctx;
    printf("LED ONNNNN!");
    gpio_set_level(GPIO_NUM_2, 1);
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t ledoff_get_handler(httpd_req_t *req)
{
    const char* resp_str = (const char*) req->user_ctx;
    printf("LED OFFFFFF!");
    gpio_set_level(GPIO_NUM_2, 0);
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t home_get_handler(httpd_req_t *req)
{
    const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if(event_base == WIFI_EVENT) {
        switch (event_id)
        {
            case WIFI_EVENT_WIFI_READY:
                printf("WiFi Ready\n");
                break;
            case WIFI_EVENT_STA_START:
                printf("WiFi station start \n");
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_CONNECTED:
             uint8_t mac[6] = { 0, };
                if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
                    ESP_LOGW("MAC", "sta mac: " MACSTR "", MAC2STR(mac));
                }
                printf("WiFi connect successfully \n");
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                printf("WiFi disconnected \n");
                break;
            default:
                printf("event id: %ld\n", event_id);
                break;
        }
        
    }
    else if(event_base == IP_EVENT)
    {
        switch (event_id)
        {
        case IP_EVENT_STA_GOT_IP:
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            printf("ESP32 got ip: %ld\n", event->ip_info.ip.addr);
            xEventGroupSetBits(sensor_event, 1 << 1);
            break;
        
        default:
            break;
        }
    }
}

 const char* html =           "<!DOCTYPE html>"\
                        "<html lang=\"vi\">"\
                        "<head>"\
                        "  <meta charset=\"UTF-8\">"\
                        "  <title>Hai nút đơn giản</title>"\
                        "  <style>"\
                        "    button {"\
                        "      padding: 10px 20px;"\
                        "      margin: 10px;"\
                        "      font-size: 16px;"\
                        "      cursor: pointer;"\
                        "    }"\
                        "  </style>"\
                        "</head>"\
                        "<body>"\
                        "  <button onclick=\"window.location.href='http://192.168.1.13/ledon'\">Nút 1</button>"\
                        "  <button onclick=\"window.location.href='http://192.168.1.13/ledoff'\">Nút 2</button>"\
                        "</body>"\
                        "</html>";

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }

    esp_netif_init();
    

    ESP_ERROR_CHECK(ret);
    uint8_t mac[6] = {0xA2, 0x08, 0x6D, 0x3B, 0x62, 0xAE};
    esp_wifi_set_mac(WIFI_IF_STA, mac);
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    // create flag (event)
    sensor_event = xEventGroupCreate();

    xTaskCreate(
        blink_led,
        "bink_led",
        2048,
        NULL,
        1,
        &task_1
    );

    xTaskCreate(
        read_sensor_data,
        "read_sensor",
        2048,
        NULL,
        1,
        &task_2);
    
    xQueue1 = xQueueCreate(100, sizeof(int));

    int ss_data;
    esp_event_loop_create_default();    

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&wifi_init_cfg) == ESP_OK) {
        printf("Initialize wifi success\n");
    }
    else {
        printf("Initialize wifi failed\n");
        return;
    }
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register( WIFI_EVENT, 
        ESP_EVENT_ANY_ID,
        &event_handler,
        NULL,
        &instance_any_id
    );

    esp_event_handler_instance_register( IP_EVENT, 
        IP_EVENT_STA_GOT_IP, 
        &event_handler,
        NULL,
        &instance_got_ip
    );

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "IMIC Technology 2.4G",
            .password = "123123123@",
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();


    xEventGroupWaitBits(sensor_event, (1 << 1), pdTRUE, pdFALSE, pdMS_TO_TICKS(1000000));

    static const httpd_uri_t uri_home = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = home_get_handler,
        /* Let's pass response string in user
         * context to demonstrate it's usage */
        .user_ctx  = (void*)html
    };
    
    static const httpd_uri_t uri_ledon = {
        .uri       = "/ledon",
        .method    = HTTP_GET,
        .handler   = ledon_get_handler,
        /* Let's pass response string in user
         * context to demonstrate it's usage */
        .user_ctx  = "LED ON!"
    };

    static const httpd_uri_t uri_ledoff = {
        .uri       = "/ledoff",
        .method    = HTTP_GET,
        .handler   = ledoff_get_handler,
        /* Let's pass response string in user
         * context to demonstrate it's usage */
        .user_ctx  = "LED OFF!"
    };

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_home);
        httpd_register_uri_handler(server, &uri_ledon);
        httpd_register_uri_handler(server, &uri_ledoff);
    }
    

   while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
