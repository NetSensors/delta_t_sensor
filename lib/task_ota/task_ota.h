/*
 * 
 *
 * Copyright (c) 2019 <NETSENSORS (UK) LTD>
 *
 *
 */


// only include once
#ifndef _TASK_OTA_H_
#define _TASK_OTA_H_

#include "heat_exchange_config.h"
#include <string.h>
#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"

class task_ota
{
    public:
        static void ota_task(void * pvParameter);
        
    
    private:
        static void readMetadata();
        esp_err_t _http_event_handler(esp_http_client_event_t *evt);


};








#endif