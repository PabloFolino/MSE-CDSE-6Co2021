/** \file	mqtt.c
 *  Mar 2022
 *  Maestría en Sistemas Embebidos - Sistemas embebidos distribuidos
 * \brief Contiene las funciones de inicializacion y manejo del protocolo mqtt
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "mqtt_client.h"

#include "config.h"
#include "wifi.h"
#include "scd_folino.h"
#include "mqtt.h"

/* TAGs */
static const char TAG[] = "MQTT";

/********************************** PID ***************************************/
extern struct pid_t pid;
extern struct pwm_t pwm;

/********************************** MQTT **************************************/

/* Definiciones */
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t);
static void mqtt_event_handler(void *, esp_event_base_t , int32_t , void *);
const char* mqtt_server = IP_BROKER_MQTT;
const int mqttPort = PORT_MQTT;
//const char* mqttUser = USER_MQTT;
//const char* mqttPassword = PASSWD_MQTT;
static esp_mqtt_client_handle_t client;
esp_mqtt_client_handle_t MQTT_getClient(void);

// parámetros Wifi
extern wifi_ap_record_t wifidata;

/*******************************************************************************
MQTT_subscribe(): Subscripción al topic especificado.
*******************************************************************************/
void MQTT_subscribe(const char * topic){

  esp_mqtt_client_subscribe(client, topic, 0);

}

/*******************************************************************************
 MQTT_publish(): publica mensaje en el topic especificado.
*******************************************************************************/
void MQTT_publish(const char * topic, const char * mensaje) {

  /* CON PUPLISH *********************************************************
  esp_mqtt_client_publish(client, topic, data, len, qos, retain) */
  esp_mqtt_client_publish(client, topic, mensaje, 0, 0, 0);

  /* CON ENQUEUE ********************************************************
  esp_mqtt_client_enqueue(client, topic, data, len, qos, retain, store);
  //esp_mqtt_client_enqueue(client, topic, mensaje, 0, 0, 0, 1);*/
}

/*******************************************************************************
 MQTT_processTopic(): lee el mensaje MQTT recibido cuando se dispara el evento
 ******************************************************************************/
 void MQTT_processTopic(const char * topic, const char * msg){

   /* Acciones a ejecutar para cada topic recibido */

   // Ingresar código aquí
   if(strcmp("test/led", topic)==0) {
        printf("MQTT: Mensaje recibido: %s\n", msg);
        IO_toggleLed();
    }
   if(strcmp("pwm/modo", topic)==0) {
        printf("MQTT: PWM-> modo: %s\n", msg);
        pwm.modo=atoi(msg);
   }
   if(strcmp("pid/kp", topic)==0) {
        printf("MQTT: PID-> kp: %s\n", msg);
        pid.kp=atoi(msg);
   }
   if(strcmp("pid/ki", topic)==0) {
        printf("MQTT: PID-> ki: %s\n", msg);
        pid.ki=atoi(msg);
   }
   if(strcmp("pid/kd", topic)==0) {
        printf("MQTT: PID-> kd: %s\n", msg);
        pid.kd=atoi(msg);
   }
   if(strcmp("pid/b", topic)==0) {
        printf("MQTT: PID-> b: %s\n", msg);
        pid.b=atoi(msg);
   }
   if(strcmp("pid/N", topic)==0) {
        printf("MQTT: PID-> N: %s\n", msg);
        pid.N=atoi(msg);
   }
   if(strcmp("pid/h", topic)==0) {
        printf("MQTT: PID-> h: %s\n", msg);
        pid.h=atoi(msg);
   }   
   if(strcmp("pwm/setpoint", topic)==0) {
        printf("MQTT: PWM-> setpoint_mqtt: %s\n", msg);
        pwm.setpoint_mqtt=(float) atoi(msg);
        if(pwm.setpoint_mqtt>=(float) (ADC_MAX_COUNT-DELTA_ADC*3)){             // Limitador
                pwm.setpoint_mqtt=(float) (ADC_MAX_COUNT-DELTA_ADC*3); 
                printf("Se limitó el setpoint a=%4.2f \n",pwm.setpoint_mqtt);
        }      
        if(pwm.setpoint_mqtt<=(float) (ADC_MIN_COUNT+DELTA_ADC*3)){
                pwm.setpoint_mqtt=(float) (ADC_MIN_COUNT+DELTA_ADC*3); 
                printf("Se limitó el setpoint a=%4.2f \n",pwm.setpoint_mqtt);
        }  
   }  
   if(strcmp("pwm/muestras", topic)==0) {
        printf("MQTT: PWM-> muestras: %s\n", msg);
        pwm.c_muestras=atoi(msg);
        if(pwm.c_muestras > N_MUESTRAS_MAX){                    // limitador
                pwm.c_muestras=N_MUESTRAS_MAX; 
                printf("Se limitó el muestras a=%4d \n",pwm.c_muestras);
        } 
        if(pwm.c_muestras < N_MUESTRAS_MIN){                    
                pwm.c_muestras=N_MUESTRAS_MIN; 
                printf("Se limitó el muestras a=%4d \n",pwm.c_muestras);
        } 
   };
   if(strcmp("pwm/disparo", topic)==0) {
        printf("MQTT: PWM-> disparo_mqtt: %s\n", msg);
        pwm.disparo=atoi(msg);
        //printf("Disparo(INT)=%d \n",pwm.disparo);
   }  
     
 }

/*******************************************************************************
 MQTT_suscripciones(): lee el mensaje MQTT recibido cuando se dispara el evento
 ******************************************************************************/
 void MQTT_suscripciones(void){
        MQTT_subscribe("test/led");
        MQTT_subscribe("pwm/modo");
        MQTT_subscribe("pwm/setpoint");
        MQTT_subscribe("pwm/disparo");
        MQTT_subscribe("pwm/muestras");
        MQTT_subscribe("pid/kp");
        MQTT_subscribe("pid/ki");
        MQTT_subscribe("pid/kd");
        MQTT_subscribe("pid/b");
        MQTT_subscribe("pid/N");
        MQTT_subscribe("pid/h");
 };

 /*******************************************************************************
  MQTT_init(): Inicialización de MQTT
  ******************************************************************************/
 void MQTT_init(void){

   esp_mqtt_client_config_t mqtt_cfg = {
           //.uri = CONFIG_BROKER_URL,
           .host= mqtt_server,
           //.username = mqttUser,
           //.password = mqttPassword,
           .port = mqttPort,
   };

   ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
   client = esp_mqtt_client_init(&mqtt_cfg); //   Creates mqtt client handle based on the configuration.
   esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
   esp_mqtt_client_start(client); // Starts mqtt client with already created client handle.
   vTaskDelay(50 / portTICK_PERIOD_MS); // waiting 50 ms

   MQTT_suscripciones();

 }

 /* handle cliente MQTT ************************************************/
 esp_mqtt_client_handle_t MQTT_getClient(void)
 {
         return client;
 }

/*****************************************************************************/
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
        //esp_mqtt_client_handle_t client = event->client;
        client = event->client;

        // your_context_t *context = event->context;
        switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
                ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
                break;

        case MQTT_EVENT_DISCONNECTED:
                ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
                break;

        case MQTT_EVENT_SUBSCRIBED:
                ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
                break;

        case MQTT_EVENT_UNSUBSCRIBED:
                ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
                break;

        case MQTT_EVENT_PUBLISHED:
                ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
                break;

        case MQTT_EVENT_DATA:   // Cuando recibo un mensaje a un topic que estoy subcripto.
                ESP_LOGI(TAG, "MQTT_EVENT_DATA");

                // obtener topic y mensaje recibido
                char rcv_topic[MAX_TOPIC_LENGTH]="";
                char rcv_message[MAX_MSG_LENGTH]="";
                strncpy(rcv_topic, event->topic, event->topic_len);
                strncpy(rcv_message, event->data, event->data_len);
                //ESP_LOGI(TAG, "TOPIC RECEIVED: %s", rcv_topic );
                //ESP_LOGI(TAG, "MESSAGE RECEIVED: %s", rcv_message);
                MQTT_processTopic(rcv_topic, rcv_message);
                break;

        case MQTT_EVENT_ERROR:
                ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
                break;

        default:
                ESP_LOGI(TAG, "Other event id:%d", event->event_id);
                break;
        }
        return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
        ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
        mqtt_event_handler_cb(event_data);
}