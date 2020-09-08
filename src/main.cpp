#include "sdkconfig.h"

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>


#include "esp_log.h"
#include "esp_wifi.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "tcpip_adapter.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include <Arduino.h>
#include <WiFi.h>
#include <TelnetStream.h>

// #include "lwip/sockets.h"
// #include "lwip/dns.h"
// #include "lwip/netdb.h"

#include "heat_exchange_config.h"

#include "task_ota.h"
#include "heat_exchanger_sensor_task.h"
#include "task_mqtt.h"
#include "pid_control.h"
#include "task_supervisor.h"

#include "nvs.h"

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

static const char *TAG = "MAIN";

//QueueHandle_t main_qh;
SemaphoreHandle_t handle_sensorSemaphore = NULL;
SemaphoreHandle_t handle_pidSemaphore = NULL;

sensors_t main_sensor_data;
pid_settings_t main_pid_data;
pid_settings_t *p_main_pid_data;

int loop_interval = 7000;

int _log_vprintf(const char *fmt, va_list args);
void keepWiFiAlive(void * parameter);

void mqtt_msg_recieved_callback();
void task_pid_get_pid_settings();
void check_pid_topic(esp_mqtt_event_handle_t event);
void pid_publish();
void sensor_data_publish();
void nvs_write_value(char* name, int value);
int32_t nvs_read_value(char* name);
void nvs_save_values();
void nvs_read_values();
void getMacAddress();

static char log_print_buffer[512];

// it wil set the static IP address to 192, 168, 10, 47
IPAddress local_IP(LOCAL_IP);
IPAddress gateway(GATEWAY);
IPAddress subnet(SUBNET);
IPAddress primaryDNS(DNS);   //optional
IPAddress secondaryDNS(DNS); //optional

TelnetStreamClass telnetstream(TELNET_PORT);

int free_mem = 0;

char main_chip_id[23];

#if !CONFIG_AUTOSTART_ARDUINO
void arduinoTask(void *pvParameter) {
   
}

void app_main()
{
    // initialize arduino library before we start the tasks
    initArduino();

    xTaskCreate(&arduinoTask, "arduino_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}
#else

  void setup() {

  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  
  // set for debug and logging purposes
  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set("MAIN", ESP_LOG_VERBOSE);
  esp_log_level_set("TASK_MQTT", ESP_LOG_WARN);
  esp_log_level_set("MQTT_CLIENT", ESP_LOG_WARN);
  esp_log_level_set("TRANSPORT_TCP", ESP_LOG_WARN);
  esp_log_level_set("TRANSPORT_SSL", ESP_LOG_WARN);
  esp_log_level_set("TRANSPORT", ESP_LOG_WARN);
  esp_log_level_set("OUTBOX", ESP_LOG_WARN);
  esp_log_level_set("sensor_task", ESP_LOG_VERBOSE);
  
  Serial.begin(115200);
  Serial.println("Booting");

  // Initialize NVS.
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // 1.OTA app partition table has a smaller NVS partition size than the non-OTA
      // partition table. This size mismatch may cause NVS initialization to fail.
      // 2.NVS partition contains data in new format and cannot be recognized by this version of code.
      // If this happens, we erase NVS partition and initialize NVS again.
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  // setup gpios // turn on solenoid
  // pinMode(RELAY_PIN_SOLENOID,OUTPUT);
  // digitalWrite(RELAY_PIN_SOLENOID,HIGH); 

  //enable watchdog done pin
  pinMode(WATCHDOG_DONE,OUTPUT);

  //enable watchdog done pin
  pinMode(TIMER_INPUT,INPUT);

  // printout the chip id
  
  getMacAddress();

  xTaskCreatePinnedToCore(
    keepWiFiAlive,
    "keepWiFiAlive",  // Task name
    5000,             // Stack size (bytes)
    NULL,             // Parameter
    1,                // Task priority
    NULL,             // Task handle
    1
  );


  
  // WiFi.mode(WIFI_STA);
  // WiFi.config(local_IP,gateway,subnet,primaryDNS,secondaryDNS);
  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // while (WiFi.waitForConnectResult() != WL_CONNECTED) {
  //   Serial.println("Connection Failed");
  //   delay(5000);
  //   break;
  //   Serial.println(" rebooting ...");
  //   // ESP.restart();
  // }

  int cpuf = 0;
  cpuf = getCpuFrequencyMhz();
  ESP_LOGI(TAG, "CPU Frequency %i", cpuf);
  
  //heat_exchanger_sensor_task::begin();
  vTaskDelay(100 / portTICK_RATE_MS);
  xTaskCreatePinnedToCore(
    heat_exchanger_sensor_task::run,
    "heat_exchanger_sensor_task",  // Task name
    4096,             // Stack size (bytes)
    NULL,             // Parameter
    5,                // Task priority
    NULL,             // Task handle
    0                 // core
  );
  // allow time for task to start before running pid's
  vTaskDelay(1000 / portTICK_RATE_MS);
  
  // actuator_control::pwm_begin();

  // // set valves to shut and turn off pump
  // actuator_control::set_duty_cycle(PWM1_CHANNEL_VALVE_1, 0);
  // actuator_control::set_duty_cycle(PWM2_CHANNEL_VALVE_2, 0);
  // actuator_control::set_duty_cycle(PWM3_CHANNEL_PUMP, 125);

  handle_sensorSemaphore = heat_exchanger_sensor_task::getSemaphoreHandle();
  handle_pidSemaphore = pid_control::getSemaphoreHandle();


  task_mqtt::mqtt_app_start(mqtt_msg_recieved_callback);

  char topic_startup[50];
  sprintf(topic_startup, "%s/startup", main_chip_id);
  task_mqtt::publish("management", topic_startup);

  // vTaskDelay(100 / portTICK_RATE_MS);
  // xTaskCreatePinnedToCore(
  //   pid_control::run,
  //   "pid_control_task",  // Task name
  //   4096,             // Stack size (bytes)
  //   NULL,             // Parameter
  //   5,                // Task priority
  //   NULL,             // Task handle
  //   0                 // core
  // );
  // // allow time for task to start before running loop
  // vTaskDelay(100 / portTICK_RATE_MS);

  // nvs_read_values();

  // vTaskDelay(1000 / portTICK_RATE_MS);



  if (WiFi.status() == WL_CONNECTED){
    telnetstream.begin();
    ESP_LOGI(TAG, "***Redirecting log output to telnet");
    esp_log_set_vprintf(&_log_vprintf);
  }


  vTaskDelay(100 / portTICK_RATE_MS);
  
  // xTaskCreatePinnedToCore(
  //   task_supervisor::run,
  //   "task_supervisor",  // Task name
  //   4096,             // Stack size (bytes)
  //   NULL,             // Parameter
  //   5,                // Task priority
  //   NULL,             // Task handle
  //   0                 // core
  // );


}

void loop() {

          // only undertake these tasks if wifi is connected
        if (WiFi.status() == WL_CONNECTED){
            sensor_data_publish();
            // pid_publish();
        }
        else{
            ESP_LOGE(TAG, "WiFi connection currently unavaliable");
        }

        free_mem =  esp_get_free_heap_size();
        ESP_LOGI(TAG, "Free memory: %d bytes\r\n", free_mem);

        digitalWrite(WATCHDOG_DONE,HIGH);
        vTaskDelay(10 / portTICK_RATE_MS);
        digitalWrite(WATCHDOG_DONE,LOW); 

        vTaskDelay(loop_interval / portTICK_RATE_MS);

}


#endif

/**
 * Task: monitor the WiFi connection and keep it alive!
 * 
 * When a WiFi connection is established, this task will check it every 10 seconds 
 * to make sure it's still alive.
 * 
 * If not, a reconnect is attempted. If this fails to finish within the timeout,
 * the ESP32 will wait for it to recover and try again.
 */
void keepWiFiAlive(void * parameter){


  // WiFi.mode(WIFI_STA);
  // WiFi.config(local_IP,gateway,subnet,primaryDNS,secondaryDNS);
  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  // while (WiFi.waitForConnectResult() != WL_CONNECTED) {
  //   Serial.println("Connection Failed");
  //   delay(5000);
  //   break;
  //   Serial.println(" rebooting ...");
  //   // ESP.restart();
  // }





    for(;;){
        
        
        if(WiFi.status() == WL_CONNECTED){
            vTaskDelay(10000 / portTICK_PERIOD_MS);

            ESP_LOGI(TAG, "WIFI Is connected");

            continue;
        }

        Serial.println("[WIFI] Connecting");
        WiFi.mode(WIFI_STA);
        //WiFi.config(local_IP,gateway,subnet,primaryDNS,secondaryDNS);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        unsigned long startAttemptTime = millis();

        // Keep looping while we're not connected and haven't reached the timeout
        while ((WiFi.waitForConnectResult() != WL_CONNECTED) && 
                (millis() - startAttemptTime < WIFI_TIMEOUT_MS)){
                     vTaskDelay(10 / portTICK_PERIOD_MS);
                }

        // When we couldn't make a WiFi connection (or the timeout expired)
		    // sleep for a while and then retry.
        if(WiFi.status() != WL_CONNECTED){
            
            Serial.println("[WIFI] FAILED");
            vTaskDelay(WIFI_RECOVER_TIME_MS / portTICK_PERIOD_MS);
			      continue;
        
        }

        ESP_LOGI(TAG,"[WIFI] Connected: %s ", WIFI_SSID);

          
        vTaskDelay(1000 / portTICK_RATE_MS);
          
       

    }
}

// This function will be called by the ESP log library every time ESP_LOG needs to be performed.
//      @important Do NOT use the ESP_LOG* macro's in this function ELSE recursive loop and stack overflow! So use printf() instead for debug messages.
int _log_vprintf(const char *fmt, va_list args) {

    //write evaluated format string into buffer
    vsnprintf (log_print_buffer, sizeof(log_print_buffer), fmt, args);
    telnetstream.printf("%s\r\n", log_print_buffer);    
    return vprintf(fmt, args);

}

void check_pid_topic(esp_mqtt_event_handle_t event){


           handle_pidSemaphore = pid_control::getSemaphoreHandle();

           if( handle_pidSemaphore != NULL )
                {
                    /* See if we can obtain the semaphore.  If the semaphore is not
                    available wait 10 ticks to see if it becomes free. */
                    ESP_LOGI(TAG, "Checking handle is not null \r\n");
                  
                  if( xSemaphoreTake( handle_pidSemaphore, ( TickType_t ) 30 ) == pdTRUE )
                  {
                        /* We were able to obtain the semaphore and can now access the
                        shared resource. */

                           ESP_LOGI(TAG, "Checking semaphore taken \r\n");

                                char ed[20];

                                p_main_pid_data = pid_control::getPidDataHandle();

                                // pid 0

                                if (strncmp(event->topic,"pid/pid_0/proportional",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  char* pEnd;
                                  float pid_0_proportional = strtof(ed,&pEnd);
                                  ESP_LOGI(TAG, "Setting pid_0 proportional to %f\r\n", pid_0_proportional);
                                  //pid_control::pid_0_consKp = pid_0_proportional;
                                  p_main_pid_data->pid[0].consKp = pid_0_proportional;
                                }

                                else if (strncmp(event->topic,"pid/pid_0/integral",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  char* pEnd;
                                  float pid_0_integral = strtof(ed,&pEnd);
                                  ESP_LOGI(TAG, "Setting pid_0 integral to %f\r\n", pid_0_integral);
                                  p_main_pid_data->pid[0].consKi = pid_0_integral;

                                }

                                else if (strncmp(event->topic,"pid/pid_0/derivative",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  char* pEnd;
                                  float pid_0_derivative = strtof(ed,&pEnd);
                                  ESP_LOGI(TAG, "Setting pid_0 derivative to %f\r\n", pid_0_derivative);
                                  p_main_pid_data->pid[0].consKd = pid_0_derivative;

                                }

                                else if (strncmp(event->topic,"pid/pid_0/setpoint",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  int pid_0_setpoint = atoi(ed);
                                  ESP_LOGI(TAG, "Setting pid_0 setpoint to %i\r\n", pid_0_setpoint);
                                  p_main_pid_data->pid[0].Setpoint = pid_0_setpoint;

                                }

                                else if (strncmp(event->topic,"pid/pid_0/enable",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  p_main_pid_data->pid[0].enabled = atoi(ed);
                                  ESP_LOGI(TAG, "Setting pid_0 enable to %i\r\n", p_main_pid_data->pid[0].enabled);

                                }


                                // pid 1

                                else if (strncmp(event->topic,"pid/pid_1/proportional",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  char* pEnd;
                                  float pid_1_proportional = strtof(ed,&pEnd);
                                  ESP_LOGI(TAG, "Setting pid_1 proportional to %f\r\n", pid_1_proportional);
                                  p_main_pid_data->pid[1].consKp = pid_1_proportional;
                                }

                                else if (strncmp(event->topic,"pid/pid_1/integral",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  char* pEnd;
                                  float pid_1_integral = strtof(ed,&pEnd);
                                  ESP_LOGI(TAG, "Setting pid_1 integral to %f\r\n", pid_1_integral);
                                  p_main_pid_data->pid[1].consKi = pid_1_integral;

                                }

                                else if (strncmp(event->topic,"pid/pid_1/derivative",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  char* pEnd;
                                  float pid_1_derivative = strtof(ed,&pEnd);
                                  ESP_LOGI(TAG, "Setting pid_1 derivative to %f\r\n", pid_1_derivative);
                                  p_main_pid_data->pid[1].consKd = pid_1_derivative;

                                }

                                else if (strncmp(event->topic,"pid/pid_1/setpoint",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  int pid_1_setpoint = atoi(ed);
                                  ESP_LOGI(TAG, "Setting pid_1 setpoint to %i\r\n", pid_1_setpoint);
                                  p_main_pid_data->pid[1].Setpoint = pid_1_setpoint;

                                }

                                else if (strncmp(event->topic,"pid/pid_1/enable",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  p_main_pid_data->pid[1].enabled = atoi(ed);
                                  ESP_LOGI(TAG, "Setting pid_1 enable to %i\r\n", p_main_pid_data->pid[1].enabled);

                                }

                                // pid 2

                                 else if (strncmp(event->topic,"pid/pid_2/proportional",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  char* pEnd;
                                  float pid_2_proportional = strtof(ed,&pEnd);
                                  ESP_LOGI(TAG, "Setting pid_2 proportional to %f\r\n", pid_2_proportional);
                                  p_main_pid_data->pid[2].consKp = pid_2_proportional;
                                }

                                else if (strncmp(event->topic,"pid/pid_2/integral",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  char* pEnd;
                                  float pid_2_integral = strtof(ed,&pEnd);
                                  ESP_LOGI(TAG, "Setting pid_2 integral to %f\r\n", pid_2_integral);
                                  p_main_pid_data->pid[2].consKi = pid_2_integral;

                                }

                                else if (strncmp(event->topic,"pid/pid_2/derivative",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  char* pEnd;
                                  float pid_2_derivative = strtof(ed,&pEnd);
                                  ESP_LOGI(TAG, "Setting pid_2 derivative to %f\r\n", pid_2_derivative);
                                  p_main_pid_data->pid[2].consKd = pid_2_derivative;

                                }

                                else if (strncmp(event->topic,"pid/pid_2/setpoint",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  int pid_2_setpoint = atoi(ed);
                                  ESP_LOGI(TAG, "Setting pid_2 setpoint to %i\r\n", pid_2_setpoint);
                                  p_main_pid_data->pid[2].Setpoint = pid_2_setpoint;

                                }

                                else if (strncmp(event->topic,"pid/pid_2/enable",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  p_main_pid_data->pid[2].enabled = atoi(ed);
                                  ESP_LOGI(TAG, "Setting pid_2 enable to %i\r\n", p_main_pid_data->pid[2].enabled);

                                }


                                else if (strncmp(event->topic,"pid/save",event->topic_len)==0) {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  ESP_LOGI(TAG, "Saving pid settings\r\n");
                                  nvs_save_values();
                                }
                                else {
                                  sprintf(ed, "%.*s\r\n",event->data_len, event->data);
                                  ESP_LOGI(TAG, "topic not recognised: %s", ed);
            
                                }

                        /* We have finished accessing the shared resource.  Release the
                        semaphore. */
                        xSemaphoreGive( handle_pidSemaphore );
                    }
                    else
                    {
                        /* We could not obtain the semaphore and can therefore not access
                        the shared resource safely. */
                         ESP_LOGI(TAG, "Cannot get semaphore handle_pidSemaphore \r\n");
                    }
                }

}         

void mqtt_msg_recieved_callback(){

   
    esp_mqtt_event_handle_t event = task_mqtt::get_event();
    
    char ed[20];

    char topic_firmware[50];
    sprintf(topic_firmware, "%s/management/firmware", main_chip_id);


    char topic_reboot[50];
    sprintf(topic_reboot, "%s/management/reboot", main_chip_id);



    if (strncmp(event->topic, topic_firmware, event->topic_len)==0) {
      
      ESP_LOGI(TAG, "Upgrading Firmware\r\n");
      xTaskCreate(task_ota::ota_task, "ota_task", 16384, NULL, 3, NULL);

    }

    else if (strncmp(event->topic,topic_reboot,event->topic_len)==0) {
      
      ESP_LOGI(TAG, "Restarting device .....");
      esp_restart();
        
    }

   
    // else if (strncmp(event->topic, root_topic,  event->topic_len)==0) {
      
    //   sprintf(ed, "%.*s\r\n",event->data_len, event->data);
    //   char* pEnd;
    //   float test_data = strtof(ed,&pEnd);
    //   ESP_LOGI(TAG, "Test recieve mqtt %f\r\n", test_data);
    //   // actuator_control::set_duty_cycle(PWM1_CHANNEL_VALVE_1, valve_1_duty_cycle);
    
    // }
    
    
    
    
    // else if  (strncmp(event->topic,"valve_1",event->topic_len)==0) {
    
    //   sprintf(ed, "%.*s\r\n",event->data_len, event->data);
    //   int valve_2_duty_cycle = atoi(ed);
    //   ESP_LOGI(TAG, "Setting valve 1 duty cycle to %i\r\n", valve_2_duty_cycle);
    //   actuator_control::set_duty_cycle(PWM2_CHANNEL_VALVE_2, valve_2_duty_cycle);
    
    // }
    

    else{

      sprintf(ed, "%.*s\r\n",event->data_len, event->data);
      ESP_LOGI(TAG, "topic not recognised: %s", ed);

    }


  

}
    
void  sensor_data_publish(){

    // get the data
    if( handle_sensorSemaphore != NULL )
    {
            /* See if we can obtain the semaphore.  If the semaphore is not
            available wait 10 ticks to see if it becomes free. */
            if( xSemaphoreTake( handle_sensorSemaphore, ( TickType_t ) 30 ) == pdTRUE )
            {

                main_sensor_data = *heat_exchanger_sensor_task::getSensorDataHandle();
                /* We have finished accessing the shared resource.  Release the
                semaphore. */
                xSemaphoreGive( handle_sensorSemaphore );


                char publish_topic[50];
                sprintf(publish_topic, "%s", main_chip_id);

        ESP_LOGI(TAG,"\r\n \
        sensor_id \t\"%s\"\r\n \
        onewire_0_temperature \t%f\r\n \
        onewire_1_temperature \t%f\r\n \
        delta_temperature \t%f\r\n",
        publish_topic,
        main_sensor_data.onewire_0_temperature,
        main_sensor_data.onewire_1_temperature,
        main_sensor_data.delta_t

        );

            if (task_mqtt::is_connected()){

               
                
                char sensors_combined[1000];
sprintf(
sensors_combined,
"{\
\"sensor_id\":\"%s\",\
\"onewire_0_temperature\":%f,\
\"onewire_1_temperature\":%f,\
\"delta_temperature\":%f,\
\"free_mem\":%i\
}",
publish_topic, 
main_sensor_data.onewire_0_temperature,
main_sensor_data.onewire_1_temperature,
main_sensor_data.delta_t,
free_mem
);

               

                #ifdef DEVELOPMENT
                    task_mqtt::publish(publish_topic,&sensors_combined[0]);
                #else
                    task_mqtt::publish(publish_topic,&sensors_combined[0]);
                #endif
    
              }

            }
            else
            {
                /* We could not obtain the semaphore and can therefore not access
                the shared resource safely. */
                    ESP_EARLY_LOGW(TAG, "Failed to get sensorSemaphore");
            }
      }

     
    
        
      
    
      
}

void nvs_write_value(char* name, int value){

  nvs_handle nvs_handle;
  esp_err_t nvs_err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
  if (nvs_err != ESP_OK) {
      printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
  } else {
      printf("Open done\n");
  }

  nvs_err = nvs_set_i32(nvs_handle, name, value);
  printf((nvs_err != ESP_OK) ? "Failed!\n" : "Write done\n");
  nvs_err = nvs_commit(nvs_handle);
  printf((nvs_err != ESP_OK) ? "Failed!\n" : "Commit done\n");

  // Close
  nvs_close(nvs_handle);

}

int32_t nvs_read_value(char* name){

  nvs_handle nvs_handle;
  esp_err_t nvs_err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
  if (nvs_err != ESP_OK) {
      printf("Error (%s) opening NVS handle!\n", esp_err_to_name(nvs_err));
  } else {
      printf("Open done\n");
  }

    // Read
    printf("Reading from NVS ... ");
    int32_t value = 0; // value will default to 0, if not set yet in NVS
    nvs_err = nvs_get_i32(nvs_handle, name, &value);
    switch (nvs_err) {
        case ESP_OK:
            printf("Done\n");
            printf("Read value = %d\n", value);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("The value is not initialized yet!\n");
            break;
        default :
            printf("Error (%s) reading!\n", esp_err_to_name(nvs_err));
    }
    return value;

}

void nvs_save_values(){
  
  // semaphore has already been taken in calling function so safe to access pid data; 
  // main_pid_data = *pid_control::getPidDataHandle();


  // nvs_write_value("kp_0", (int32_t)(main_pid_data.pid[0].consKp * 1000000));
  // nvs_write_value("ki_0", (int32_t)(main_pid_data.pid[0].consKi * 1000000));
  // nvs_write_value("kd_0", (int32_t)(main_pid_data.pid[0].consKd * 1000000));
  // nvs_write_value("kp_1", (int32_t)(main_pid_data.pid[1].consKp * 1000000));
  // nvs_write_value("ki_1", (int32_t)(main_pid_data.pid[1].consKi * 1000000));
  // nvs_write_value("kd_1", (int32_t)(main_pid_data.pid[1].consKd * 1000000));
  // nvs_write_value("kp_2", (int32_t)(main_pid_data.pid[2].consKp * 1000000));
  // nvs_write_value("ki_2", (int32_t)(main_pid_data.pid[2].consKi * 1000000));
  // nvs_write_value("kd_2", (int32_t)(main_pid_data.pid[2].consKd * 1000000));
   
}

void nvs_read_values(){

    // handle_pidSemaphore = pid_control::getSemaphoreHandle();

    //   // get the data
    // if( handle_pidSemaphore != NULL )
    // {
    //         /* See if we can obtain the semaphore.  If the semaphore is not
    //         available wait 10 ticks to see if it becomes free. */
    
    //         //handle_pidSemaphore = pid_control::getSemaphoreHandle();

    //         if( xSemaphoreTake( handle_pidSemaphore, ( TickType_t ) 30 ) == pdTRUE )
    //         {

    //             p_main_pid_data = pid_control::getPidDataHandle();
                
    //             p_main_pid_data->pid[0].consKp = (double)(nvs_read_value("kp_0")/1000000.00);
    //             p_main_pid_data->pid[0].consKi = (double)(nvs_read_value("ki_0")/1000000.00);
    //             p_main_pid_data->pid[0].consKd = (double)(nvs_read_value("kd_0")/1000000.00);

    //             p_main_pid_data->pid[1].consKp = (double)(nvs_read_value("kp_1")/1000000.00);
    //             p_main_pid_data->pid[1].consKi = (double)(nvs_read_value("ki_1")/1000000.00);
    //             p_main_pid_data->pid[1].consKd = (double)(nvs_read_value("kd_1")/1000000.00);

    //             p_main_pid_data->pid[2].consKp = (double)(nvs_read_value("kp_2")/1000000.00);
    //             p_main_pid_data->pid[2].consKi = (double)(nvs_read_value("ki_2")/1000000.00);
    //             p_main_pid_data->pid[2].consKd = (double)(nvs_read_value("kd_2")/1000000.00);
                
    //             // /* We have finished accessing the shared resource.  Release the
    //             // semaphore. */
    //             xSemaphoreGive( handle_pidSemaphore );

    //         }
    //         else
    //         {
    //             /* We could not obtain the semaphore and can therefore not access
    //             the shared resource safely. */
    //              ESP_EARLY_LOGW(TAG, "Failed to get pidSemaphore");
    //         }
    // } 

}

void getMacAddress() {
    uint8_t baseMac[6];
    // Get MAC address for WiFi station
    esp_efuse_mac_get_default(baseMac);
    char baseMacChr[18] = {0};
    sprintf(main_chip_id, "%02x%02x%02x%02x%02x%02x", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    
    ESP_LOGI(TAG, "MAC ID ............................ :  %s", main_chip_id);
}

