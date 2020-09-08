







#define SOFTWARE_VERSION "2"
//#define DEVELOPMENT



#ifdef DEVELOPMENT

// networking configuration
#define DEVICE_ID "development"
#define WIFI_SSID "BTWholeHome-2CC"
#define WIFI_PASSWORD "tc270498"
#define LOCAL_IP 192, 168, 1, 210
#define GATEWAY 192, 168, 1 ,254
#define DNS 192, 168, 1 ,254
#define SUBNET 255, 255, 255, 0
#define TELNET_PORT 29767
#define FOTA_PORT 29768
// #define CONFIG_FIRMWARE_UPGRADE_URL "dev.netsensors.co.uk"
// #define CONFIG_BROKER_URL "mqtt://dev.netsensors.co.uk"

#define CONFIG_FIRMWARE_UPGRADE_URL "http://dev.netsensors.co.uk/static-2/ota_files/delta/dev/firmware_dev_delta.bin"
#define CONFIG_BROKER_URL "mqtt://dev.netsensors.co.uk"
#define METADATA_LOCATION_PATH "http://dev.netsensors.co.uk/static-2/ota_files/delta/dev/metadata.txt"

#else

// networking configuration
#define DEVICE_ID "production"
#define WIFI_SSID "Essco_HIU"
#define WIFI_PASSWORD "Comf0rt7"
#define LOCAL_IP 192,168,0,167
#define GATEWAY 192,168,0,199
#define SUBNET 255, 255, 255, 0
#define DNS 192,168,0,81
#define TELNET_PORT 29767
#define FOTA_PORT 29768


#define CONFIG_FIRMWARE_UPGRADE_URL "http://dev.netsensors.co.uk/static-2/ota_files/delta/production/firmware_production_delta.bin"
#define CONFIG_BROKER_URL "mqtt://dev.netsensors.co.uk"
#define METADATA_LOCATION_PATH "http://dev.netsensors.co.uk/static-2/ota_files/delta/production/metadata.txt"

#endif

#define WIFI_TIMEOUT_MS 10000 // 10 second WiFi connection timeout
#define WIFI_RECOVER_TIME_MS 20000 // Wait 20 seconds after a failed connection attempt

// pwm configuration
#define PWM1_PIN_VALVE_1 18
#define PWM2_PIN_VALVE_2 19
#define PWM3_PIN_PUMP    14
#define RELAY_PIN_SOLENOID 13

#define PWM_FREQUENCY 100
#define PWM1_CHANNEL_VALVE_1 1
#define PWM2_CHANNEL_VALVE_2 2
#define PWM3_CHANNEL_PUMP 3
#define PWM_RESOUTION 8

// watchdog pins
#define WATCHDOG_DONE 4
#define WATCHDOG_WAKE 39

// timer pins
#define TIMER_INPUT 34

#define TEMP_UPPER_RANGE 100
#define TEMP_LOWER_RANGE 0