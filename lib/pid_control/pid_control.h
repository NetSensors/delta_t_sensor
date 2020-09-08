/*
 *
 *
 * Copyright (c) 2019 <NETSENSORS (UK) LTD>
 *
 *
 */
// only include once
#ifndef _PID_CONTROL_H_
#define _PID_CONTROL_H_

#include "heat_exchange_config.h"
#include "task_mqtt.h"
#include "heat_exchanger_sensor_task.h"
#include "actuator_control.h"
#include "PID_v1.h"




typedef struct pid_setting {

        double Setpoint;
        double Input;
        double Output;
        int enabled;
        double consKp; 
        double consKi;
        double consKd;
        
} pid_setting_t;

typedef struct pid_settings {

    pid_setting pid[3];
           
} pid_settings_t;



class pid_control {

    public:
        // static QueueHandle_t getQueueHandle(void);
        static SemaphoreHandle_t getSemaphoreHandle(void);
        static void run(void * parameter);
        static void get_sensor_data();
        static void pid_0_begin();
        static void pid_0_run();
        static void pid_1_begin();
        static void pid_1_run();
        static void pid_2_begin();
        static void pid_2_run();
        static pid_settings_t* getPidDataHandle();
        static void send(pid_settings_t ps_to_send);

        static int scale_input(float start_range, float end_range, float current_input);
        
        
  
    private:
        

};































#endif