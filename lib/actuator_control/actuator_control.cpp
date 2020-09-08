#include "heat_exchange_config.h"
#include "actuator_control.h"


actuator_settings_t as = {0};


void actuator_control::set_duty_cycle_valve(int channel, float duty_cycle){

    // constrain to less than 100
    if (duty_cycle > 230) duty_cycle = 230;
    if (duty_cycle < 0) duty_cycle = 0;
    
    //convert from percentage to range
    // float range_start = 10;
    // float range_end = 220;
    // float range = (((range_end - range_start)*duty_cycle)/100) + range_start;
    // int cy = (int)round(range);
    set_duty_cycle(channel, duty_cycle);

}

void actuator_control::set_duty_cycle(int channel, int duty_cycle){
  
  switch (channel) {
        case PWM1_CHANNEL_VALVE_1:
            as.current_duty_cycle_valve_1 = duty_cycle;
            break;
        
        case PWM2_CHANNEL_VALVE_2:
            as.current_duty_cycle_valve_2 = duty_cycle;
            break;

        case PWM3_CHANNEL_PUMP:
            // full speed is duty cycle of 0 so reverse this to make more logical
            as.current_duty_cycle_pump = 254 - duty_cycle;
            break;
    
    }
    
    // write a value to the pwm
    ledcWrite(channel, duty_cycle);

}

void actuator_control::pwm_begin()
{

        as.current_duty_cycle_valve_1 = 0;
        as.current_duty_cycle_valve_2 = 0;
        as.current_duty_cycle_pump = 100;

        pinMode(PWM1_PIN_VALVE_1,OUTPUT);
        pinMode(PWM2_PIN_VALVE_2,OUTPUT);
        pinMode(PWM3_PIN_PUMP,OUTPUT);

        // configure LED PWM functionalitites
        ledcSetup(PWM1_CHANNEL_VALVE_1, PWM_FREQUENCY, PWM_RESOUTION);
        ledcSetup(PWM2_CHANNEL_VALVE_2, PWM_FREQUENCY, PWM_RESOUTION);
        ledcSetup(PWM3_CHANNEL_PUMP, PWM_FREQUENCY, PWM_RESOUTION);

        // attach the channel to the GPIO2 to be controlled
        ledcAttachPin(PWM1_PIN_VALVE_1, PWM1_CHANNEL_VALVE_1);
        ledcAttachPin(PWM2_PIN_VALVE_2, PWM2_CHANNEL_VALVE_2);
        ledcAttachPin(PWM3_PIN_PUMP, PWM3_CHANNEL_PUMP);

}

actuator_settings_t* actuator_control::get_pActuator_settings(){

        return &as;

}
