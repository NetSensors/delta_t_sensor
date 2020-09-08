#include "pid_control.h"

static const char *TAG = "TASK_PID_CONTROL";

pid_settings_t pid_data;
pid_settings_t *p_pid_data = &pid_data;

sensors_t pid_sensor_data;


//Specify the links and initial tuning parameters
PID pid_0(&pid_data.pid[0].Input, &pid_data.pid[0].Output, &pid_data.pid[0].Setpoint, pid_data.pid[0].consKp, pid_data.pid[0].consKi, pid_data.pid[0].consKd, DIRECT);
PID pid_1(&pid_data.pid[1].Input, &pid_data.pid[1].Output, &pid_data.pid[1].Setpoint, pid_data.pid[1].consKp, pid_data.pid[1].consKi, pid_data.pid[1].consKd, DIRECT);
PID pid_2(&pid_data.pid[2].Input, &pid_data.pid[2].Output, &pid_data.pid[2].Setpoint, pid_data.pid[2].consKp, pid_data.pid[2].consKi, pid_data.pid[2].consKd, DIRECT);

SemaphoreHandle_t pid_task_sensorSemaphore = NULL;
SemaphoreHandle_t pid_task_pidSemaphore = NULL;

SemaphoreHandle_t pid_control::getSemaphoreHandle(void){
     return pid_task_pidSemaphore;
}

void pid_control::pid_0_begin(){

    //   p.pid[0].consKp=0;
    //   p.pid[0].consKi=0;
    //   p.pid[0].consKd=0;
 
    //initialize the variables we're linked to
    pid_data.pid[0].Input = scale_input(TEMP_LOWER_RANGE, TEMP_UPPER_RANGE, pid_sensor_data.pt3);
    pid_data.pid[0].Setpoint = 400;
    pid_0.SetOutputLimits(0, 240);

    //turn the PID on
    pid_0.SetMode(AUTOMATIC);
    pid_data.pid[0].enabled = 1;

}

void pid_control::pid_0_run(){

  // pid control part
  // sensor will be flow 1 temp
  // control will be primary valve heating

    if(pid_data.pid[0].enabled != 0){

        pid_data.pid[0].Input = scale_input(TEMP_LOWER_RANGE, TEMP_UPPER_RANGE, pid_sensor_data.pt3);
        pid_0.SetTunings(pid_data.pid[0].consKp, pid_data.pid[0].consKi, pid_data.pid[0].consKd);
        pid_0.Compute();
        actuator_control::set_duty_cycle_valve(PWM1_CHANNEL_VALVE_1, (int)pid_data.pid[0].Output);

    }
 
}

void pid_control::pid_1_begin(){

  pid_data.pid[1].consKp=0.7;
  pid_data.pid[1].consKi=0.08;
  pid_data.pid[1].consKd=0;

  ///initialize the variables we're linked to
  pid_data.pid[1].Input = scale_input(TEMP_LOWER_RANGE, TEMP_UPPER_RANGE, pid_sensor_data.flow_2_temperature);
  pid_data.pid[1].Setpoint = 400;
  pid_1.SetOutputLimits(0, 240);
  
  //turn the PID on
  pid_1.SetMode(AUTOMATIC);
  pid_data.pid[1].enabled = 1;
  
}

void pid_control::pid_1_run(){

  // pid control part
  // sensor will be flow 1 temp
  // control will be primary valve heating

  if(pid_data.pid[1].enabled != 0){

        //ESP_LOGI(TAG, "flow_2_temperature :: %f" ,pid_sensor_data.flow_2_temperature);

        pid_data.pid[1].Input = scale_input(TEMP_LOWER_RANGE, TEMP_UPPER_RANGE, pid_sensor_data.flow_2_temperature);
        pid_1.SetTunings(pid_data.pid[1].consKp, pid_data.pid[1].consKi, pid_data.pid[1].consKd);
        pid_1.Compute();

        actuator_control::set_duty_cycle_valve(PWM2_CHANNEL_VALVE_2, (int)pid_data.pid[1].Output);
       
  }
 

}

void pid_control::pid_2_begin(){

  pid_data.pid[2].consKp=0.7;
  pid_data.pid[2].consKi=0.08;
  pid_data.pid[2].consKd=0;

  ///initialize the variables we're linked to
  pid_data.pid[2].Input = scale_input(TEMP_LOWER_RANGE, TEMP_UPPER_RANGE, pid_sensor_data.flow_2_temperature);
  pid_data.pid[2].Setpoint = 400;
  pid_2.SetOutputLimits(0, 240);
  
  //turn the PID on
  pid_2.SetMode(AUTOMATIC);
  pid_data.pid[2].enabled = 0;
  
}

void pid_control::pid_2_run(){

  // pid control part
  // sensor will be flow 1 temp
  // control will be primary valve heating

  if(pid_data.pid[2].enabled != 0){

        //ESP_LOGI(TAG, "flow_2_temperature :: %f" ,pid_sensor_data.flow_2_temperature);

        pid_data.pid[2].Input = scale_input(TEMP_LOWER_RANGE, TEMP_UPPER_RANGE, pid_sensor_data.pt2);
        pid_2.SetTunings(pid_data.pid[2].consKp, pid_data.pid[2].consKi, pid_data.pid[2].consKd);
        pid_2.Compute();

        actuator_control::set_duty_cycle_valve(PWM2_CHANNEL_VALVE_2, (int)pid_data.pid[2].Output);
       
  }
 

}

pid_settings_t* pid_control::getPidDataHandle(){
     return p_pid_data;
}

void pid_control::run(void * parameter){


    while(1) { // setup
        
        pid_task_pidSemaphore = xSemaphoreCreateMutex();
        pid_task_sensorSemaphore = heat_exchanger_sensor_task::getSemaphoreHandle();


        if( pid_task_pidSemaphore != NULL )
        {
            /* See if we can obtain the semaphore.  If the semaphore is not
            available wait 10 ticks to see if it becomes free. */
            if( xSemaphoreTake( pid_task_pidSemaphore, ( TickType_t ) 20 ) == pdTRUE )
            {
                /* We were able to obtain the semaphore and can now access the
                shared resource. */

                
                get_sensor_data();
                pid_0_begin();
                pid_1_begin();
                pid_2_begin();
            
                /* We have finished accessing the shared resource.  Release the
                semaphore. */
                xSemaphoreGive( pid_task_pidSemaphore );
            }
            else
            {
                /* We could not obtain the semaphore and can therefore not access
                the shared resource safely. */

            }
        }
              

        int err = 1;

        while(err > 0){ //loop

            get_sensor_data();

            if( pid_task_pidSemaphore != NULL )
                {
                    /* See if we can obtain the semaphore.  If the semaphore is not
                    available wait 10 ticks to see if it becomes free. */
                    if( xSemaphoreTake( pid_task_pidSemaphore, ( TickType_t ) 20 ) == pdTRUE )
                    {
                        /* We were able to obtain the semaphore and can now access the
                        shared resource. */

                        
                        //vTaskDelay(100 / portTICK_RATE_MS);
                        pid_control::pid_0_run();
                        vTaskDelay(100 / portTICK_RATE_MS);
                        pid_control::pid_1_run();
                        vTaskDelay(100 / portTICK_RATE_MS);
                        pid_control::pid_2_run();
                        vTaskDelay(100 / portTICK_RATE_MS);
                        //send(p);
                        //ESP_LOGI(TAG, "\tpid compute has run...");
    
                        /* We have finished accessing the shared resource.  Release the
                        semaphore. */
                        xSemaphoreGive( pid_task_pidSemaphore );
                    }
                    else
                    {
                        /* We could not obtain the semaphore and can therefore not access
                        the shared resource safely. */
                        ESP_LOGW(TAG, "\tUnable to obtain semaphore...");

                    }
                }
                vTaskDelay(200 / portTICK_RATE_MS);

        }
        
       
    }
}

void  pid_control::get_sensor_data(){
     
    if( pid_task_sensorSemaphore != NULL )
        {
            /* See if we can obtain the semaphore.  If the semaphore is not
            available wait 10 ticks to see if it becomes free. */
            if( xSemaphoreTake( pid_task_sensorSemaphore, ( TickType_t ) 30 ) == pdTRUE )
            {

                pid_sensor_data = *heat_exchanger_sensor_task::getSensorDataHandle();
                /* We have finished accessing the shared resource.  Release the
                semaphore. */
                xSemaphoreGive( pid_task_sensorSemaphore );
            }
            else
            {
                /* We could not obtain the semaphore and can therefore not access
                the shared resource safely. */
                    ESP_EARLY_LOGW(TAG, "Failed to get pid_task_sensorSemaphore");
            }
        }

}

int pid_control::scale_input(float start_range, float end_range, float current_input){

        return (int)((current_input/(end_range-start_range)) * 1000);

}