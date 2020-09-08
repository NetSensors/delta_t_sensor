/*
 * 
 *
 * Copyright (c) 2019 <NETSENSORS (UK) LTD>
 *
 *
 */


// only include once
#ifndef _TASK_MQTT_H_
#define _TASK_MQTT_H_

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "ESP.h"
#include "esp_system.h"
#include "esp_log.h"
#include "heat_exchange_config.h"
#include "mqtt_client.h"


typedef void (*CallbackFunction)(void);

class task_mqtt
{
    public:
    
        static void mqtt_app_start(CallbackFunction cbFunctionMqttRecieve);
        static int is_connected(void);
        static void publish(char* path, char* value);
        static esp_mqtt_event_handle_t get_event();
        static void getMacAddress();

        
    private:

        static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event);
        static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
        static void recieved_message_handler(esp_mqtt_event_handle_t event);
        
};





#endif