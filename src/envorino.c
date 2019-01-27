#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>

#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>

#include <semphr.h>
#include "bme680.h"

#define MQTT_HOST ("172.16.1.56")
#define MQTT_PORT 1883

#define MQTT_USER "envorino"
#define MQTT_PASS "********"

#define WIFI_SSID "IoT"
#define WIFI_PASS "********"

#define I2C_BUS       0 
#define I2C_SCL_PIN   5 
#define I2C_SDA_PIN   4
#define I2C_FREQ      I2C_FREQ_100K

static bme680_sensor_t* sensor = 0;

SemaphoreHandle_t wifi_alive;
SemaphoreHandle_t mqtt_alive;

QueueHandle_t publish_queue;

#define PUB_MSG_LEN 128
#define MQTT_BUF_LEN 256
#define TOPIC_LEN 100

#define GAS_BURNIN_CYCLES 50
#define GAS_MIN_THRESHOLD 2500.0

#define GAS_HUM_BASELINE 40.0
#define GAS_HUM_WEIGHT 0.25

static void measure_task(void *pvParameters) {
    while(1) {
    	xSemaphoreTake(mqtt_alive, portMAX_DELAY);
        printf("%s: started\n", __func__);

        bme680_values_float_t values;
        TickType_t last_wakeup = xTaskGetTickCount();
        uint32_t duration = bme680_get_measurement_duration(sensor);

        // IAQ burn-in
        float gas_accum = 0;
        int gas_cycles = 0;
        float gas_baseline = 0;

        while(1) {
	    if (bme680_force_measurement (sensor)) {
	        vTaskDelay (duration);

	        if (bme680_get_results_float (sensor, &values)) {
                float iaq = 0.0;

                if (gas_cycles >= GAS_BURNIN_CYCLES && gas_baseline == 0.0) {
                    gas_baseline = gas_accum / gas_cycles;
                } else if (gas_baseline == 0.0 && values.gas_resistance > GAS_MIN_THRESHOLD) {
                    gas_accum += values.gas_resistance;
                    gas_cycles++;
                } else if (gas_baseline > 0) {
                    float gas_offset = gas_baseline - values.gas_resistance;
                    float hum_offset = GAS_HUM_BASELINE - values.humidity;
                    float hum_score = 0.0;
                    float gas_score = 0.0;

                    if (hum_offset > 0) {
                        hum_score = (100 - GAS_HUM_BASELINE - hum_offset);
                        hum_score /= (100 - GAS_HUM_BASELINE);
                        hum_score *= (GAS_HUM_WEIGHT * 100);
                    } else {
                        hum_score = (GAS_HUM_BASELINE + hum_offset);
                        hum_score /= GAS_HUM_BASELINE;
                        hum_score *= (GAS_HUM_WEIGHT * 100);
                    }

                    if (gas_offset > 0) {
                        gas_score = (values.gas_resistance / gas_baseline);
                        gas_score *= (100 - (GAS_HUM_WEIGHT * 100));
                    } else {
                        gas_score = 100 - (GAS_HUM_WEIGHT * 100);
                    }

                    iaq = gas_score + hum_score;
                }

		    char msg[PUB_MSG_LEN];
		    snprintf(msg, PUB_MSG_LEN, 
			     "humidity=%.2f temperature=%.2f gas_resistance=%.2f,iaq=%.2f", 
			     values.humidity, values.temperature, values.gas_resistance, iaq);
		    if (xQueueSend(publish_queue, (void*)msg, 0) == pdFALSE) {
	                printf("%s: publish queue overflow.\n", __func__);
	            }
		}
	    } else {
	        printf("%s: failed to read BME680 sensor data.\n", __func__);
		break;
	    }

	    vTaskDelayUntil(&last_wakeup, 1000 / portTICK_PERIOD_MS);
	}

	printf("%s: redoing BME680 sensor readings.\n", __func__);
	taskYIELD();
    }
}

static const char* get_my_id(void) {
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;

    if (my_id_done)
        return my_id;

    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t*)my_id))
        return NULL;
    
    for (i = 5; i >= 0; --i) {
        x = my_id[i] & 0x0F;
        if (x > 9) x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9) x += 7;
        my_id[i*2] = x + '0';
    }

    my_id[12] = '\0';
    my_id_done = true;
    
    return my_id;
}

static const char* get_sensor_name(void) {
    const char* mac_address = get_my_id();

    if (!strcmp(mac_address, "5CCF7F6C7502")) {
	return "jesper_office";
    } else {
	return "unbound";
    }
}

static void mqtt_task(void *pvParameters) {
    int ret = 0;
    struct mqtt_network network;
    mqtt_client_t client = mqtt_client_default;
    char mqtt_client_id[20];
    uint8_t mqtt_buf[MQTT_BUF_LEN];
    uint8_t mqtt_readbuf[MQTT_BUF_LEN];
    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

    mqtt_network_new(&network);
    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "ESP-");
    strcat(mqtt_client_id, get_my_id());

    const char* sensor_name = get_sensor_name();

    printf("%s: MAC address: %s\n", __func__, mqtt_client_id); 
    printf("%s: Sensor name: %s\n", __func__, sensor_name);

    char topic[TOPIC_LEN];
    snprintf(topic, TOPIC_LEN, "sensors/%s", sensor_name);

    while(1) {
        xSemaphoreTake(wifi_alive, portMAX_DELAY);
        printf("%s: started\n\r", __func__);
        printf("%s: (Re)connecting to MQTT server %s ... ",__func__,
               MQTT_HOST);
        ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT);
        if(ret){
            printf("error: %d\n\r", ret);
            taskYIELD();
            continue;
        } else {
	    xSemaphoreGive(mqtt_alive);
	    taskYIELD();
	}
        printf("done\n\r");
        mqtt_client_new(&client, &network, 5000, mqtt_buf, MQTT_BUF_LEN,
                      mqtt_readbuf, MQTT_BUF_LEN);

        data.willFlag          = 0;
        data.MQTTVersion       = 3;
        data.clientID.cstring  = mqtt_client_id;
        data.username.cstring  = MQTT_USER;
        data.password.cstring  = MQTT_PASS;
        data.keepAliveInterval = 10;
        data.cleansession      = 0;

        printf("Send MQTT connect ... ");
        ret = mqtt_connect(&client, &data);
        if(ret){
            printf("error: %d\n\r", ret);
            mqtt_network_disconnect(&network);
            taskYIELD();
            continue;
        }
        
	printf("done\r\n");
        xQueueReset(publish_queue);

        while(1){
            char msg[MQTT_BUF_LEN - 1] = "\0";
            while(xQueueReceive(publish_queue, (void *)msg, 0) ==
                  pdTRUE){
		char rewrite[MQTT_BUF_LEN];
		memset(rewrite, 0, sizeof(rewrite));
		snprintf(rewrite, MQTT_BUF_LEN, "envorino,sensor=%s %s", sensor_name, msg);

		mqtt_message_t message;
                message.payload = rewrite;
                message.payloadlen = strlen(rewrite);
                message.dup = 0;
                message.qos = MQTT_QOS1;
                message.retained = 0;
                ret = mqtt_publish(&client, topic, &message);
                if (ret != MQTT_SUCCESS ){
                    printf("error while publishing message: %d\n", ret );
                    break;
                }
            }

            ret = mqtt_yield(&client, 1000);
            if (ret == MQTT_DISCONNECTED)
                break;
        }

        printf("Connection dropped, request restart\n\r");
        mqtt_network_disconnect(&network);
        taskYIELD();
    }
}

static void wifi_task(void *pvParameters) {
    uint8_t status  = 0;
    uint8_t retries = 30;
    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    printf("WiFi: connecting to WiFi\n\r");
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    while(1)
    {
        while ((status != STATION_GOT_IP) && (retries)){
            status = sdk_wifi_station_get_connect_status();
            printf("%s: status = %d\n\r", __func__, status );

            if (status == STATION_WRONG_PASSWORD){
                printf("WiFi: wrong password\n\r");
                break;
            } else if (status == STATION_NO_AP_FOUND) {
                printf("WiFi: AP not found\n\r");
                break;
            } else if (status == STATION_CONNECT_FAIL) {
                printf("WiFi: connection failed\r\n");
                break;
            }
            
	    vTaskDelay( 1000 / portTICK_PERIOD_MS );
            --retries;
        
	}
        if (status == STATION_GOT_IP) {
            printf("WiFi: Connected\n\r");
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }

        while ((status = sdk_wifi_station_get_connect_status()) == STATION_GOT_IP) {
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        
	}
        printf("WiFi: disconnected\n\r");
        sdk_wifi_station_disconnect();
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}

void user_init(void) {
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);
    sensor = bme680_init_sensor(I2C_BUS, BME680_I2C_ADDRESS_2, 0);

    if (sensor) {
	printf("%s: found BME680 sensor.\n", __func__);
	bme680_set_oversampling_rates(sensor, osr_4x, osr_none, osr_2x);
	bme680_set_filter_size(sensor, iir_size_7);
	bme680_set_ambient_temperature (sensor, 25);

	xTaskCreate(&measure_task, "measure_task", 1024, NULL, 5, NULL);
    } else {
	printf("%s: could not init BME680 sensor.\n", __func__);
    }

    vSemaphoreCreateBinary(wifi_alive);
    vSemaphoreCreateBinary(mqtt_alive);

    publish_queue = xQueueCreate(10, PUB_MSG_LEN);
    
    xTaskCreate(&wifi_task, "wifi_task",  256, NULL, 2, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 4, NULL);
}
