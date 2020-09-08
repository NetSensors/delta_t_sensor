/*
 *
 *
 * Copyright (c) 2019 <NETSENSORS (UK) LTD>
 *
 *
 */

#include "task_ota.h"

static const char *O_TAG = "TASK_OTA";

// #define SOFTWARE_VERSION "1"
// #define CONFIG_FIRMWARE_UPGRADE_URL "http://dev.netsensors.co.uk/static-2/ota_files/boiler/firmware.bin"
// #define METADATA_LOCATION_PATH "http://dev.netsensors.co.uk/static-2/ota_files/boiler/metadata.txt"



#define METADATA_URL_SIZE 200

char metadata_url[METADATA_URL_SIZE];

// extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
// extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

// ///TODO move this into a utils area
// void ota_set_pixel(pixel_task_action_t p_action){

//     pixel_task_action_t  pixel_action = p_action;
//     if ( xQueueSendToBack( xQueue_pixel_task, ( void * ) &pixel_action, ( TickType_t ) 10 ) != pdPASS){
//         /* Failed to post the message, even after 10 ticks. */
//        // ESP_LOGW(TAG, "Unable to add command to pixel queue");
//     }
// }


esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(O_TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(O_TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(O_TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(O_TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(O_TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(O_TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(O_TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

 esp_http_client_config_t ota_config = {
        .url = CONFIG_FIRMWARE_UPGRADE_URL,
        .event_handler = _http_event_handler,
    };


void task_ota::ota_task(void * pvParameter)
{
    char* file_url = METADATA_LOCATION_PATH;
    //strncpy(download_params,file_url, sizeof(download_params));

    strncpy(metadata_url, file_url, METADATA_URL_SIZE);

    printf("Starting download %s\n",metadata_url);
    ESP_LOGI(O_TAG, "url:  %s \n", metadata_url);

    readMetadata();

    // esp_err_t ret = esp_https_ota(&ota_config);
    // if (ret == ESP_OK) {
    //     esp_restart();
    // } else {
    //     ota_set_pixel(COLOUR_OTA_FAILED);
    //     ESP_LOGE(O_TAG, "Firmware Upgrades Failed");
    // }
    vTaskDelete(NULL);
}

void task_ota::readMetadata(){

    esp_http_client_config_t config_metadata = {
        .url = metadata_url
    };

    esp_http_client_handle_t esp_http_client_handle_metadata = esp_http_client_init(&config_metadata);
    esp_err_t err = esp_http_client_open(esp_http_client_handle_metadata, 0);

    int content_length;
    int status_code;
    char downloaded_software_version[100] = {'\0'};
    long file_size;
    

    // // check remote connection to file has opened
    if (err == ESP_OK) {
        content_length = esp_http_client_fetch_headers(esp_http_client_handle_metadata);
        status_code = esp_http_client_get_status_code(esp_http_client_handle_metadata);

        
        if (status_code == 200){
                ESP_LOGE(O_TAG, "Report status code: %d \n", status_code);
                    char charbuffer[2000] = {'\0'};                    
                    esp_http_client_read(esp_http_client_handle_metadata, charbuffer,esp_http_client_get_content_length(esp_http_client_handle_metadata));
                    
                    //ESP_LOGI(O_TAG, "Bytes: %i \n",esp_http_client_get_content_length(esp_http_client_handle_metadata));

                    //strcpy(downloaded_software_version, charbuffer);

                    ESP_LOGI(O_TAG, "Software Version %s \n", charbuffer);
        }
        else {
            ESP_LOGE(O_TAG, "Error in retrieving file from server, code: %d \n", status_code);
        }
    } else {
        ESP_LOGE(O_TAG, "Error creating http handel %d \n", err);
    }
    esp_http_client_close(esp_http_client_handle_metadata);
    esp_http_client_cleanup(esp_http_client_handle_metadata);

    if (strcmp(downloaded_software_version, SOFTWARE_VERSION) == 0){
        ESP_LOGI(O_TAG, "VERSION UP TO DATE");
    }
    else {
            ESP_LOGI(O_TAG, "VERSION NEEDS UPDATING");

            esp_err_t ret = esp_https_ota(&ota_config);
            if (ret == ESP_OK) {
                esp_restart();
            }
            else {
                
                ESP_LOGE(O_TAG, "Firmware Upgrades Failed");
            }
    }
    ESP_LOGI(O_TAG, "Done");

}
