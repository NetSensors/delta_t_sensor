#include "task_supervisor.h"

static const char *TAG = "TASK_SUPERVISOR";

pid_settings_t task_supervisor_pid_data;
pid_settings_t *p_task_supervisor_pid_data;

float kw_flow_threashold = 0;

sensors_t task_supervisor_sensor_data;

SemaphoreHandle_t task_supervisor_pidSemaphore = NULL;
SemaphoreHandle_t task_supervisor_sensorSemaphore = NULL;

int kw_status_high = 0;
int kw_status_low = 0;

void task_supervisor::run(void * parameter){

    while(1) { // setup

        
        task_supervisor_sensorSemaphore = heat_exchanger_sensor_task::getSemaphoreHandle();

        int err = 1;

        while(err > 0){ //loop

                
                ESP_LOGI(TAG, "\tSupervisor task has just run...");
                
                task_supervisor::get_sensor_data();

                if ((task_supervisor_sensor_data.flow_2_flow >= kw_flow_threashold) && (kw_status_high == 0)){
                        task_supervisor::write_enable(1,0);
                        task_supervisor::write_enable(2,1);
                        kw_status_high = 1;
                        kw_status_low = 0;
                }
                else if ((task_supervisor_sensor_data.flow_2_flow < kw_flow_threashold) && (kw_status_low == 0)){
                        task_supervisor::write_enable(1,1);
                        task_supervisor::write_enable(2,0);
                        kw_status_high = 0;
                        kw_status_low = 1;
                }




                vTaskDelay(2000 / portTICK_RATE_MS);

        }
        
       
    }
}

void task_supervisor::write_setpoint(float setpoint, int pid_index){

    task_supervisor_pidSemaphore = pid_control::getSemaphoreHandle();

      // get the data
    if( task_supervisor_pidSemaphore != NULL )
    {
            /* See if we can obtain the semaphore.  If the semaphore is not
            available wait 10 ticks to see if it becomes free. */
    
            if( xSemaphoreTake( task_supervisor_pidSemaphore, ( TickType_t ) 30 ) == pdTRUE )
            {

                p_task_supervisor_pid_data = pid_control::getPidDataHandle();
                
                p_task_supervisor_pid_data->pid[pid_index].Setpoint = setpoint;

                // /* We have finished accessing the shared resource.  Release the
                // semaphore. */
                xSemaphoreGive( task_supervisor_pidSemaphore );

                ESP_LOGI(TAG, "Setpoint for pid index %i changed to :  %f", pid_index, setpoint );

            }
            else
            {
                /* We could not obtain the semaphore and can therefore not access
                the shared resource safely. */
                 ESP_EARLY_LOGW(TAG, "Failed to get pidSemaphore");
            }
    } 


}

void task_supervisor::write_enable(int pid_index, int enable){

    task_supervisor_pidSemaphore = pid_control::getSemaphoreHandle();

      // get the data
    if( task_supervisor_pidSemaphore != NULL )
    {
            /* See if we can obtain the semaphore.  If the semaphore is not
            available wait 10 ticks to see if it becomes free. */
    
            if( xSemaphoreTake( task_supervisor_pidSemaphore, ( TickType_t ) 30 ) == pdTRUE )
            {

                p_task_supervisor_pid_data = pid_control::getPidDataHandle();
                
                p_task_supervisor_pid_data->pid[pid_index].enabled = enable;

                // /* We have finished accessing the shared resource.  Release the
                // semaphore. */
                xSemaphoreGive( task_supervisor_pidSemaphore );

                ESP_LOGI(TAG, "Enabled state for pid index %i changed to :  %i", pid_index, enable );

            }
            else
            {
                /* We could not obtain the semaphore and can therefore not access
                the shared resource safely. */
                 ESP_EARLY_LOGW(TAG, "Failed to get pidSemaphore");
            }
    } 


}

void task_supervisor::get_sensor_data(){
     
    if( task_supervisor_sensorSemaphore != NULL )
        {
            /* See if we can obtain the semaphore.  If the semaphore is not
            available wait 10 ticks to see if it becomes free. */
            if( xSemaphoreTake( task_supervisor_sensorSemaphore, ( TickType_t ) 30 ) == pdTRUE )
            {

                task_supervisor_sensor_data = *heat_exchanger_sensor_task::getSensorDataHandle();
                /* We have finished accessing the shared resource.  Release the
                semaphore. */
                xSemaphoreGive( task_supervisor_sensorSemaphore );
            }
            else
            {
                /* We could not obtain the semaphore and can therefore not access
                the shared resource safely. */
                    ESP_LOGI(TAG, "Failed to get task_supervisor_sensorSemaphore");
            }
        }

}

void task_supervisor::keep_warm_flow_threashold(float threashold){

    kw_flow_threashold = threashold;
    ESP_LOGI(TAG, "Keep warm threashold chnaged to: %f", threashold);

}