/*
 *
 *
 * Copyright (c) 2019 <NETSENSORS (UK) LTD>
 *
 *
 */
// only include once
#ifndef _ACTUATOR_CONTROL_H_
#define _ACTUATOR_CONTROL_H_

#include <Arduino.h>


typedef struct actuator_settings {
        
        int current_duty_cycle_valve_1;
        int current_duty_cycle_valve_2;
        int current_duty_cycle_pump;
        
} actuator_settings_t;

class actuator_control {

    public:
        
        static void set_duty_cycle(int channel, int duty_cycle);
        static void set_duty_cycle_valve(int channel, float duty_cycle);
        static void pwm_begin();
        static actuator_settings_t* get_pActuator_settings();

    private:

};


#endif
