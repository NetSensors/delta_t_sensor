/*
 * 
 *
 * Copyright (c) 2019 <NETSENSORS (UK) LTD>
 *
 *
 */


// only include once
#ifndef _TASK_SUPERVISOR_H_
#define _TASK_SUPERVISOR_H_


#include <WiFi.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "mqtt_client.h"

#include "pid_control.h"



class task_supervisor
{
    public:
    
        static void  run(void * parameter);
        static void keep_warm_flow_threashold(float threashold);
       
    private:

        static void write_setpoint(float setpoint, int pid_index);
        static void write_enable(int pid_index, int enable);
        static void get_sensor_data();
        
        
};





#endif