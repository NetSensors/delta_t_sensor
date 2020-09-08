/*
 *
 *
 * Copyright (c) 2019 <NETSENSORS (UK) LTD>
 *
 *
 */
// only include once
#ifndef _HEAT_EXCHANGER_SENSOR_TASK_H_
#define _HEAT_EXCHANGER_SENSOR_TASK_H_

#include <Wire.h>
#include "Adafruit_ADS1015.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "task_mqtt.h"

#define FAILOVER_DETECT  34
#define ADC_CALIBRATION_VALUE 1.05

typedef struct sensors{
        float pressure_1_pressure;
        float pressure_1_temp;
        float pressure_2_pressure;
        float pressure_2_temp;
        float flow_1_flow;
        float flow_1_temperature;
        float flow_2_flow;
        float flow_2_temperature;
        float pt1;
        float pt2;
        float pt3;
        float pt4;
        float onewire_0_temperature;
        float onewire_1_temperature;
        float onewire_2_temperature;
        float onewire_3_temperature;
        float delta_t;
        float battery_voltage;
} sensors_t;

typedef struct onewire_address{
    byte addr[8];
} onewire_address_t;



    class heat_exchanger_sensor_task
    {
        public:

            static SemaphoreHandle_t getSemaphoreHandle(void);
            // static QueueHandle_t getQueueHandle(void);
            static sensors_t * getSensorDataHandle(void);
            
            static void begin(void);
            // static void send(sensors_t hest_data);
            //static int  read_sensor_data();
            //static sensors_t get_sensor_data();
            static void run(void * parameter);
            
  
        private:

            static float adc2_one_moving_average();
            static float adc1_one_moving_average();
            static int  find_one_wire_addresses();
            static void getTemperature();

    };















































#endif