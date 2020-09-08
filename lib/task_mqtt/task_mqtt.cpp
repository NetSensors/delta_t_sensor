/*
 *
 *
 * Copyright (c) 2019 <NETSENSORS (UK) LTD>
 *
 *
 */

#include "task_mqtt.h"


static const char *TAG = "TASK_MQTT";


CallbackFunction MqttRecieve;
   
int connected = 0;

char task_mqtt_chip_id[16];

esp_mqtt_client_handle_t g_client;
esp_mqtt_event_handle_t g_event;

esp_err_t task_mqtt::mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
   esp_mqtt_client_handle_t client = event->client;
    int ms_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

            // ms_id = esp_mqtt_client_publish(client, "test", "connected", 0, 1, 0);
            // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", ms_id);

            // printout the chip id
            char chip_id[50];
            sprintf(chip_id, "%s/management/#", task_mqtt_chip_id);
            ESP_LOGI(TAG, "Chip ID :  %s", chip_id);
           
            esp_mqtt_client_subscribe(client, chip_id ,0);
            
            connected = 1;
            
            // msg_id = esp_mqtt_client_subscribe(client, "test", 0);
            // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            // msg_id = esp_mqtt_client_subscribe(client, "test", 1);
            // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            // msg_id = esp_mqtt_client_unsubscribe(client, "test");
            // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            connected = 0;
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            // msg_id = esp_mqtt_client_publish(client, "test", "data", 0, 0, 0);
            // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            ESP_LOGI(TAG, "TOPIC=%.*s\r", event->topic_len, event->topic);
            ESP_LOGI(TAG, "DATA=%.*s\r\n", event->data_len, event->data);
            recieved_message_handler(event);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

void task_mqtt::mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb((esp_mqtt_event_handle_t)event_data);
}

void task_mqtt::mqtt_app_start(CallbackFunction cbFunctionMqttRecieve)
{
    getMacAddress();
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = CONFIG_BROKER_URL,
        .port = 1883,
        .username = "netsensors",
        .password = "voobgr33n",
        .task_prio = 5
        
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
    g_client = client;

    MqttRecieve = cbFunctionMqttRecieve;

 
}

esp_mqtt_event_handle_t task_mqtt::get_event(){
    return g_event;
} 

void task_mqtt::recieved_message_handler(esp_mqtt_event_handle_t event) {
    g_event = event;
    MqttRecieve();
}

void task_mqtt::publish(char* path, char* value){
    esp_mqtt_client_publish(g_client, path, value, 0, 1, 0);
}

int task_mqtt::is_connected(void){
    return connected;
}

void task_mqtt::getMacAddress() {
    uint8_t baseMac[6];
    // Get MAC address for WiFi station
    esp_efuse_mac_get_default(baseMac);
    char baseMacChr[18] = {0};
    sprintf(task_mqtt_chip_id, "%02x%02x%02x%02x%02x%02x", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    ESP_LOGI(TAG, "MAC ID ............................ :  %s", task_mqtt_chip_id);
}


