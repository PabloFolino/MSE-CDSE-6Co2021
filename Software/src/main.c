/* sdkconfig.h
#define CONFIG_FREERTOS_HZ 1000    -> <ntes era 100
*/



#include "main.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define Delay_500US 500
#define Delay_10MS 10000
#define Delay_100MS 100000
#define Delay_500MS 500000

const char *TAG = "MAIN";
volatile uint16_t  data_ADC1,data_ADC2;
volatile uint32_t data_max=0; // lectura máxima del ADC registrada

// MQTT
#define msg "Hola estoy conectado ESP32-MQTT"
#define TAG1 "MQQT"
#define CONTADOR "Count:"

// Prototipo de funcion de la tarea
void control( void* taskParmPtr );

extern struct pwm_t pwm;

/*******************************************************************************
 Programa principal
******************************************************************************/
void app_main(void)
{
  // Declaración de variables

  int8_t rssi;
  char msg_rssi[10];
  uint16_t contador=0;
  char msg_send[100];
  char msg_contador_data[5];
  char msg_datos[10];

  // Inicializaciones

  // Use POSIX and C standard library functions to work with files.
  IO_adcInits();                   // Inicio los ADCs
  IO_gpioInit();                   // Inicia los gpio y el pwm
  RTOS_timerInit();                // Instancia la tareas de toma de muestras
  

  // Verificación de lo leido

  
  // Inicializo red 
  WIFI_init();
  // Inicializo MQTT
  /* 1) Instale MQTT Mosquito en una termanial de Linux -->
  *     sudo apt-get install mosquitto y sudo apt-get install mosquitto-clients
  *  2) Inicie el broker mosquitto --> sudo service mosquitto start
  *  3) Verifique que esta corriendo --> Verificar estado actual del broker mosquitto
  *  4) Suscribase a un topico --> 
  *     mosquitto_sub -h 192.168.0.113 -t "test/topic1" -t "test/rssi" -v
  */
  MQTT_init();
  
  // Inicializo SNPT
 

  // Crear tarea en freeRTOS
  BaseType_t res = xTaskCreate(
    control,                     	  // Funcion de la tarea a ejecutar
    ( const char * )"print1",   	  // Nombre de la tarea como String amigable para el usuario
    configMINIMAL_STACK_SIZE*2, 		// Cantidad de stack de la tarea
    0,                          		// Parametros de tarea
    tskIDLE_PRIORITY+1,         		// Prioridad de la tarea -> Queremos que este un nivel encima de IDLE
    0                          			// Puntero a la tarea creada en el sistema
  );

  // Gestion de errores
	if(res == pdFAIL)
	{
		printf( "Error al crear las tareas.\r\n" );
		while(1);						            //Se bloquea el programa
	}

 
  while(1){
    contador++;
    RTOS_delayMs(1000);             // Delay de 1seg
    
    strcpy(msg_send,CONTADOR);
    itoa(contador,msg_contador_data,5);
    rssi=WIFI_getRSSI();
    itoa(rssi,msg_rssi,10);
    strcat(msg_send,msg_contador_data);
    strcat(msg_send," dBm:");
    strcat(msg_send,msg_rssi);

    MQTT_publish("test/topic1",msg);
    MQTT_publish("test/rssi", msg_send);
    debug("Programa principal");

    itoa(data_ADC2,msg_datos,10);
    MQTT_publish("test/adc_motor",msg_datos);
    itoa(data_ADC1,msg_datos,10);
    MQTT_publish("test/adc_setpoint",msg_datos);


    strcat(msg_send," pwm:");
    itoa((int)pwm.pwm,msg_datos,10);
    strcat(msg_send,msg_datos);
    strcat(msg_send,"  pwm.p:");
    itoa((int)pwm.p,msg_datos,10);
    strcat(msg_send,msg_datos);
    strcat(msg_send,"  pwm.i:");
    itoa((int)pwm.i,msg_datos,10);
    strcat(msg_send,msg_datos);
    strcat(msg_send,"  pwm.d:");
    itoa((int)pwm.d,msg_datos,10);
    strcat(msg_send,msg_datos);
    MQTT_publish("test/pwm",msg_send);

    // ESP_LOGI(TAG,"test/topic1, mensaje --> %s\n", msg);
    // ESP_LOGI(TAG,"test/rssi, rssi=%d\n", rssi);
    //printf("n=%6d  Estado=%d \n",contador, estado);

  }

}

void control( void* taskParmPtr )
{
  // ---------- CONFIGURACIONES ------------------------------
  TickType_t xPeriodicity =  20 / portTICK_RATE_MS;			// Tarea periodica cada 20 ms
  //TickType_t xPeriodicity =  1000 / portTICK_RATE_MS;			// Tarea periodica cada 1s
  
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // ---------- REPETIR POR SIEMPRE --------------------------
  while(1){
    //debug("Tarea");
    // Leo los conversores AD
    IO_readAdc();
    // Calculo el PWM
    IO_gpioPWM(PWM_calculo()+PWM_OFFSET);
    // Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
    vTaskDelayUntil( &xLastWakeTime , xPeriodicity );
    }
}
