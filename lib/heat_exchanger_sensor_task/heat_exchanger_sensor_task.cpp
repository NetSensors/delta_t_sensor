#include "heat_exchanger_sensor_task.h"

#define ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION 0.0001875

#define FIVE_VOLT_TO_CENTIGRADE_SCALAR 0.03

#define MOVING_AVERAGE_STEPS 5

// 0 degrees  = 13024
// 100 degrees = 3938
// 1 deree = 39.38

#define PTC_OFFSET 13000
#define PTC_BITS_PER_CENTIGRADE 39.38


OneWire ds(26);  //data wire connected to GPIO15
DallasTemperature sensors(&ds);

onewire_address onewire_address_array[2];

static const char *TAG = "sensor_task";

Adafruit_ADS1115 adc1(0x48);
Adafruit_ADS1115 adc2(0x4B);
Adafruit_ADS1115 adc3(0x49);

sensors_t sensor_data;
sensors_t *p_sensor_data = &sensor_data;

sensors_t ss;

SemaphoreHandle_t sensorSemaphore = NULL;

SemaphoreHandle_t heat_exchanger_sensor_task::getSemaphoreHandle(void){
     return sensorSemaphore;
}

sensors_t * heat_exchanger_sensor_task::getSensorDataHandle(void){
     return p_sensor_data;
}

void heat_exchanger_sensor_task::begin(){

    sensors.begin();

    vTaskDelay(1000 / portTICK_RATE_MS);

    unsigned long startAttemptTime = millis();
    int address_count = 0;
    while(1)
    {
       address_count = heat_exchanger_sensor_task::find_one_wire_addresses();
       if ((address_count!=2) && (millis() - startAttemptTime < 10000)){
           
       }
       else {
           break;
       }
       vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    if (address_count != 2){


        // char topic_startup[50];
        // sprintf(topic_startup, "%s/startup", main_chip_id);
        // task_mqtt::publish("management", topic_startup);

        ESP_LOGE(TAG, "Required Addresses not found rebooting... ");

        esp_restart();
        // TODO send management message here to indicate issue with sensor
    }




    //     Wire.begin(21,22);
    //     Wire.setClock(400000); // I2C frequency at 400 kHz 
        
    //     adc1.setGain(GAIN_TWOTHIRDS);      // +/- 6.144V  1 bit = 0.1875mV (default)
    //     //adc1.setGain(GAIN_ONE);               // +/- 4.096V  1 bit = 0.125mV
    //     //adc1.setGain(GAIN_TWO);            // +/- 2.048V  1 bit = 0.0625mV
    //    // adc1.setGain(GAIN_FOUR);           // +/- 1.024V  1 bit = 0.03125mV
    //     //adc1.setGain(GAIN_EIGHT);            // +/- 0.512V  1 bit = 0.015625mV
    //     //adc1.setGain(GAIN_SIXTEEN);        // +/- 0.256V  1 bit = 0.0078125mV 

    //     adc2.setGain(GAIN_TWOTHIRDS);      // +/- 6.144V  1 bit = 0.1875mV (default)
    //     //adc2.setGain(GAIN_ONE);               // +/- 4.096V  1 bit = 0.125mV
    //     //adc2.setGain(GAIN_TWO);            // +/- 2.048V  1 bit = 0.0625mV
    //    // adc2.setGain(GAIN_FOUR);           // +/- 1.024V  1 bit = 0.03125mV
    //     // adc2.setGain(GAIN_EIGHT);            // +/- 0.512V  1 bit = 0.015625mV
    //     //adc2.setGain(GAIN_SIXTEEN);        // +/- 0.256V  1 bit = 0.0078125mV

    //     //adc3.setGain(GAIN_TWOTHIRDS);      // +/- 6.144V  1 bit = 0.1875mV (default)
    //     //adc3.setGain(GAIN_ONE);               // +/- 4.096V  1 bit = 0.125mV
    //     adc3.setGain(GAIN_TWO);            // +/- 2.048V  1 bit = 0.0625mV
    //     //adc3.setGain(GAIN_FOUR);           // +/- 1.024V  1 bit = 0.03125mV
    //     // adc3.setGain(GAIN_EIGHT);            // +/- 0.512V  1 bit = 0.015625mV
    //     //adc3.setGain(GAIN_SIXTEEN);        // +/- 0.256V  1 bit = 0.0078125mV

    //     adc1.begin();

    //     vTaskDelay(100 / portTICK_RATE_MS);

    //     adc2.begin();

    //     vTaskDelay(100 / portTICK_RATE_MS);

    //     adc3.begin();

    //     vTaskDelay(100 / portTICK_RATE_MS);

}

void heat_exchanger_sensor_task::run(void * parameter){

   
    while(1) { // setup

        sensorSemaphore = xSemaphoreCreateMutex();

        heat_exchanger_sensor_task::begin();
        
        vTaskDelay(100 / portTICK_RATE_MS);
            
        int err = 1;

        while(err > 0){ //loop

            
            
            vTaskDelay(1000 / portTICK_RATE_MS);
            heat_exchanger_sensor_task::getTemperature();
            
            

            //need to initialise at least one value
            //ss.battery_voltage = 3.3;
                               
            // vTaskDelay(10 / portTICK_RATE_MS);  
            // ss.pressure_1_pressure =    adc1.readADC_SingleEnded(0) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION;
           
            // vTaskDelay(10 / portTICK_RATE_MS); 
            // //p_sensor_data->pressure_1_temp =     ((adc1.readADC_SingleEnded(1) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION) - 0.5) / FIVE_VOLT_TO_CENTIGRADE_SCALAR;
            // //p_sensor_data->pressure_1_temp =      ((heat_exchanger_sensor_task::adc_average(1,1, ADC_CYCLES) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION) - 0.5) / FIVE_VOLT_TO_CENTIGRADE_SCALAR;
            // ss.pressure_1_temp =        ((adc1.readADC_SingleEnded(3) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION) - 0.5) / FIVE_VOLT_TO_CENTIGRADE_SCALAR;

            // vTaskDelay(10 / portTICK_RATE_MS);
            // ss.pressure_2_pressure =    adc1.readADC_SingleEnded(2) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION;

            // vTaskDelay(10 / portTICK_RATE_MS);
            // //p_sensor_data->pressure_2_temp =          ((adc1.readADC_SingleEnded(3) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION) - 0.5) / FIVE_VOLT_TO_CENTIGRADE_SCALAR;
            // //p_sensor_data->pressure_2_temp =      ((heat_exchanger_sensor_task::adc_average(1,3, ADC_CYCLES) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION) - 0.5) / FIVE_VOLT_TO_CENTIGRADE_SCALAR;
            // ss.pressure_2_temp =        ((adc1.readADC_SingleEnded(1) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION) - 0.5) / FIVE_VOLT_TO_CENTIGRADE_SCALAR;
            
            // vTaskDelay(10 / portTICK_RATE_MS);  
            // ss.flow_1_flow =            adc2.readADC_SingleEnded(0) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION;
           
            // vTaskDelay(10 / portTICK_RATE_MS);    
            // //p_sensor_data->flow_1_temperature =   ((heat_exchanger_sensor_task::adc_average(2,1, ADC_CYCLES) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION) - 0.5) / FIVE_VOLT_TO_CENTIGRADE_SCALAR;
            // //p_sensor_data->flow_1_temperature =   ((adc2.readADC_SingleEnded(1) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION) - 0.5) / FIVE_VOLT_TO_CENTIGRADE_SCALAR;
            // ss.flow_1_temperature =     ((adc2.readADC_SingleEnded(3) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION) - 0.5) / FIVE_VOLT_TO_CENTIGRADE_SCALAR;
                        
            // vTaskDelay(10 / portTICK_RATE_MS);   
            // ss.flow_2_flow =            adc2.readADC_SingleEnded(2) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION;
                       
            // vTaskDelay(10 / portTICK_RATE_MS);    
            // ss.flow_2_temperature =     ((adc2.readADC_SingleEnded(1) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION) - 0.5) / FIVE_VOLT_TO_CENTIGRADE_SCALAR;
            // //p_sensor_data->flow_2_temperature =   ((heat_exchanger_sensor_task::adc_average(2,3, ADC_CYCLES) * ADC_GAIN_TWOTHIRDS_VOLTAGE_CONVERSION) - 0.5) / FIVE_VOLT_TO_CENTIGRADE_SCALAR;
            // //p_sensor_data->flow_2_temperature =   adc2.readADC_SingleEnded(3);
            
            // vTaskDelay(10 / portTICK_RATE_MS);   
            // ss.pt1 =                     (adc3.readADC_SingleEnded(2)- PTC_OFFSET)/PTC_BITS_PER_CENTIGRADE;
            // vTaskDelay(10 / portTICK_RATE_MS);  
            // ss.pt2 =                     (adc3.readADC_SingleEnded(1)- PTC_OFFSET)/PTC_BITS_PER_CENTIGRADE;

            // vTaskDelay(10 / portTICK_RATE_MS);  
            // ss.pt3 =                    (adc3.readADC_SingleEnded(0)- PTC_OFFSET)/PTC_BITS_PER_CENTIGRADE;

            // vTaskDelay(10 / portTICK_RATE_MS);  
            // ss.pt4 =                    (adc3.readADC_SingleEnded(3)- PTC_OFFSET)/PTC_BITS_PER_CENTIGRADE;


            if( sensorSemaphore != NULL )
                {
                    /* See if we can obtain the semaphore.  If the semaphore is not
                    available wait 10 ticks to see if it becomes free. */
                    if( xSemaphoreTake( sensorSemaphore, ( TickType_t ) 20 ) == pdTRUE )
                    {

                            *p_sensor_data = ss;
                            
                            xSemaphoreGive(sensorSemaphore);
                            

                    }
                    else
                    {
                        /* We could not obtain the semaphore and can therefore not access
                        the shared resource safely. */
                        ESP_LOGW(TAG, "Failed to get sensorSemaphore");
                    }
                }



            //heat_exchanger_sensor_task::send(*p_sensor_data);
            
            vTaskDelay(8000 / portTICK_RATE_MS);

        }
        //free(data);
    }
}


void heat_exchanger_sensor_task::getTemperature(){

    
    sensors.begin();
    vTaskDelay(1000 / portTICK_RATE_MS);
    
    ESP_LOGI(TAG, "Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    ESP_LOGI(TAG, "DONE");
    
    vTaskDelay(2000 / portTICK_RATE_MS);
  
    float sensor_0 = 0;
    while (1){
        sensor_0 = sensors.getTempC(onewire_address_array[0].addr);
        if  (sensor_0 == -127){
            ESP_LOGW(TAG, "ERROR Reading temperature value 0");
            vTaskDelay(1000 / portTICK_RATE_MS);
            // to do send management message
        }
        else {
            ss.onewire_0_temperature = sensor_0;
            break;
        }
        
    }
    ESP_LOGI(TAG, "Sensor 0(*C): %f", sensor_0);

    vTaskDelay(1000 / portTICK_RATE_MS);

    float sensor_1 = 0;
    while (1){
        sensor_1 = sensors.getTempC(onewire_address_array[1].addr);
        if  (sensor_1 == -127){
            ESP_LOGW(TAG, "ERROR Reading temperature value 1");
            vTaskDelay(1000 / portTICK_RATE_MS);
            // to do send management message
        }
        else {
            ss.onewire_1_temperature = sensor_1;
            break;
        }
        
    }
    ESP_LOGI(TAG, "Sensor 1(*C): %f", sensor_1);


    ss.delta_t = sensor_0 - sensor_1;


}


int heat_exchanger_sensor_task::find_one_wire_addresses(){

    byte i;
    int count = 0;
    byte addr[8];

    while (ds.search(addr)){
        
        // copy to address array
        for (i = 0; i < 8; i++) {
            onewire_address_array[count].addr[i] = addr[i];
        }
        // move to next array element
        count++;

    }

    ds.reset_search();
    delay(500);
    ESP_LOGI(TAG, "No more addresses %i found", count);
    return count;
    

}
