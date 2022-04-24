#include "scd_folino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "mqtt.h"
#include <string.h>

#define OFF 0
#define ON 1

#define ESPACIO  " "

//static int contador=0;

/* TAGs */
//static const char *TAG1 = "RTOS/TIMER";


/********************************** TIMER *************************************/
esp_timer_handle_t periodic_timer,pwm_timer;
extern volatile uint16_t data_ADC1,data_ADC2;
extern volatile uint32_t data_max;

/************************************* ADC ***********************************/
#define DEFAULT_VREF  1100
static esp_adc_cal_characteristics_t *adc_chars1;
static esp_adc_cal_characteristics_t *adc_chars2;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_channel_t channel1 = ADC_CHANNEL_6;     //GPIO34 if ADC1
static const adc_channel_t channel2 = ADC_CHANNEL_7;     //GPIO35 if ADC1 
static const adc_atten_t atten = ADC_ATTEN_MAX ;         //ADC_ATTEN_DB_0; ADC_ATTEN_DB_11
//static const adc_unit_t unit = ADC_UNIT_1;

// Prototipo de funcion de la tarea
void task_muestra(void* taskParmPtr);

/************************************* PWM ***********************************/
struct pid_t pid;
struct pwm_t pwm;

static struct test_muestra {
  float setpoint;
  float encoder;
} muestra[N_MUESTRAS_MAX];

char mensaj[50];

/*******************************************************************************
 IO_readAdc1(): Devuelve el valor leido como un entero de 12-bits de resolución
               del CANAL 1-GPIO34-ADC1_6
*******************************************************************************/
uint16_t IO_readAdc1(){

  return (uint16_t)adc1_get_raw((adc1_channel_t)channel1);  // entero 12 bitmeasure;

}

/*******************************************************************************
 IO_readAdc2(): Devuelve el valor leido como un entero de 12-bits de resolución
               del CANAL 2-GPIO35-ADC1_7
*******************************************************************************/
uint16_t IO_readAdc2(){

  return (uint16_t)adc1_get_raw((adc1_channel_t)channel2);  // entero 12 bitmeasure;

}

/*******************************************************************************
 IO_readAdc(): lee los dos ADC
 estado=0--> PID asociado al potenciómetro
       =1--> PID asociado a MQTT, el setpoint se envía x MQTT
       =2--> Disparo único asociado a MQTT
*******************************************************************************/
void IO_readAdc(){
  switch (pwm.modo){
    case 0:                       // Estado inicial del banco de pueba
      data_ADC1=IO_readAdc1();
      break;
    case 1:
      data_ADC1=pwm.setpoint_mqtt;
      break;
    case 2:
      break;
    default:
      data_ADC1=ADC_MAX_COUNT/2;
  }
  data_ADC2=ADC_MAX_COUNT-IO_readAdc2();
}

/*******************************************************************************
 IO_adcInit(): Inicialización del conversor analógico digital
*******************************************************************************/
void IO_adcInits(){

  adc1_config_width(width);
  adc1_config_channel_atten(channel1, atten);
  adc1_config_channel_atten(channel2, atten);
  adc_chars1 = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  adc_chars2 = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  //esp_adc_cal_value_t val_type1 = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars1);
  //esp_adc_cal_value_t val_type2 = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars2);

}

/*****************************************************************************
 clear_muestras(void): limoia el vector de muestras
*******************************************************************************/
void clear_muestras(void)
{
  for(int i=0;i<N_MUESTRAS_MAX;i++){
    muestra[i].encoder=0.0;
    muestra[i].setpoint=0.0;
  }
}

/*****************************************************************************
 take_muestras(void): callback para la interrpción de timer.
*******************************************************************************/
void take_muestras(void){
    IO_readAdc();
    muestra[pwm.n_muestra].encoder=data_ADC2;
    muestra[pwm.n_muestra].setpoint=data_ADC1;
    pwm.n_muestra++;
    if(pwm.n_muestra > pwm.c_muestras){           // termina de tomar muestras
      pwm.disparo=3;
      pwm.n_muestra=0;
    }
}

/*****************************************************************************
 tx_muestras(void): callback para la interrupción de timer.
*******************************************************************************/
void tx_muestras(void){
  char msg_encoder[5];
  char msg_setpoint[5];
  
  itoa((int)pwm.n_muestra,mensaj,10);
  itoa(muestra[pwm.n_muestra].encoder,msg_encoder,10);
  itoa(muestra[pwm.n_muestra].setpoint,msg_setpoint,10);

  strcat(mensaj,ESPACIO);
  strcat(mensaj,msg_encoder);
  strcat(mensaj,ESPACIO);
  strcat(mensaj,msg_setpoint);
  
  MQTT_publish("pid/muestras_rx",mensaj);
  pwm.n_muestra++;
  if(pwm.n_muestra > pwm.c_muestras){         // termina de transmitir muestras
    pwm.n_muestra=0;
    pwm.disparo=0;
    pwm.modo=0;                             // Vuelvo al estado inicial
                                            // del banco de prueba
  }  
}

/*****************************************************************************
 RTOS_timerCallback(void* arg): callback para la interrpción de timer.
*******************************************************************************/
void task_muestra( void* taskParmPtr )
{
  // ---------- CONFIGURACIONES ------------------------------
  TickType_t xPeriodicity =  2 / portTICK_RATE_MS;			// Tarea periodica cada 10 ms
  
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // ---------- REPETIR POR SIEMPRE --------------------------
  while(1){
    if (pwm.modo==2){                     // pregunto si estoy en el modo muestras
      switch(pwm.disparo){
        case 1:
          pwm.n_muestra=0;
          clear_muestras();
          pwm.disparo=2;
          data_ADC1=pwm.setpoint_mqtt;
          break;
        case 2:
          take_muestras();
          break;
        case 3:
          tx_muestras();
          break;
      }
    }

  // Envia la tarea al estado bloqueado durante xPeriodicity (delay periodico)
  vTaskDelayUntil( &xLastWakeTime , xPeriodicity );
  }
}
  
/*****************************************************************************
 RTOS_timerInit(): inicialización del temporizador de alta presición.
*******************************************************************************/
void RTOS_timerInit(void){
  
  BaseType_t res = xTaskCreate(
    task_muestra,                     	  // Funcion de la tarea a ejecutar
    ( const char * )"task_muestra",   	  // Nombre de la tarea como String amigable para el usuario
    configMINIMAL_STACK_SIZE*2, 		// Cantidad de stack de la tarea
    0,                          		// Parametros de tarea
    tskIDLE_PRIORITY+1,         		// Prioridad de la tarea -> Queremos que este un nivel encima de IDLE
    0                          			// Puntero a la tarea creada en el sistema
  );

  // Gestion de errores
	if(res == pdFAIL)
	{
		printf( "Error al crear la tarea muestra.\r\n" );
		while(1);						            //Se bloquea el programa
	}

}

/*****************************************************************************
 RTOS_timerStart(): arranca el temporizador periodico con un periodo de interrupción
"interval" en microsegundos.
IMPORTANTE: no volver a ejecutar esta función si el timer ya está corriendo.
Para reiniciar el timer, primero debe detenerse.
*******************************************************************************/
void RTOS_timerStart(uint64_t interval_us){

  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, interval_us));
 // ESP_LOGI(TAG1, "Started timers, time since boot: %lld us", esp_timer_get_time());

}

/*****************************************************************************
RTOS_timerStop(): detiene el temporizador periodico.
*******************************************************************************/
void RTOS_timerStop(void){

  esp_timer_stop(periodic_timer);

}

/******************************************************************************
RTOS_delayMs(): introduce un delay de "delay_ms" milisegundos
******************************************************************************/
void RTOS_delayMs(int time_ms){

  vTaskDelay(time_ms / portTICK_PERIOD_MS);

}

/******************************************************************************
 IO_setLed(): Setea el estado del LED configurado por defecto en el módulo ESP32
******************************************************************************/
int IO_setLed(int estado){

  gpio_set_level(BLINK_GPIO, estado);
  return estado;

}

/******************************************************************************
 IO_toggleLed(): Togglea estado del LED configurado por defecto en el módulo ESP32
******************************************************************************/
void IO_toggleLed(void){
  static int estado = 0;
  estado = 1 - estado;
  gpio_set_level(BLINK_GPIO, estado);
  //MQTT_publish("test/led","Toggle Led");
}

/******************************************************************************
IO_gpioInit(): inicializa perifericos de entrada/salida
*******************************************************************************/
void IO_gpioInit(){

  /* Configure the IOMUX register for pad GPIO_EN3,GPIO_EN4 y GPIO_PWM (some pads 
  are muxed to GPIO on reset already, but some default to other functions and need 
  to be switched to GPIO. Consult the Technical Reference for a list of pads and 
  their default  functions.) */
  /* Importante: el puente L289, también tiene puestas las resistencias de pull-up 
  */
 
  // Inicio las constantes
  pid.kp=KP_INIT;
  pid.ki=KI_INIT;
  pid.kd=KD_INIT;
  pid.b=B_INIT;
  pid.h=H_INIT;
  pid.N=N_INI;


  // pwm.acc=0.0;
  pwm.p=0.0;
  pwm.i=0.0;
  pwm.d=0.0;
  pwm.past_y=0.0;
  pwm.setpoint_mqtt= SETPOINT_MQTT; 
  pwm.c_muestras=N_MUESTRAS;
  pwm.disparo=0;        // no se encuentra disparado
  pwm.modo=0;           // modo=0--> PID asociado al potenciómetro
                        //       =1--> PID asociado a MQTT, el setpoint se envía x MQTT
                        //       =2--> Disparo único asociado a MQTT



  // Se selecciona la unidad de PWM y los pines de salida
  mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM0A,GPIO_EN3);         
  mcpwm_gpio_init(MCPWM_UNIT_0,MCPWM0B,GPIO_EN4);         
  // Se asigna el timmer y la configuración del PWM
  mcpwm_config_t pwm_config = {                           
    .frequency = PWM_FREQ_HZ,
    .cmpr_a = 0,
    .cmpr_b = 0,
    .counter_mode = MCPWM_UP_COUNTER,
    .duty_mode = MCPWM_DUTY_MODE_0,
  };
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);  
  
  // Se setea que el MCPWM0A trabaje en modo no invertido --> MCPWM_DUTY_MODE_0
  mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, MCPWM_DUTY_MODE_0);
  // Se setea que el MCPWM0B trabaje en modo invertido --> MCPWM_DUTY_MODE_1
  mcpwm_set_duty_type(MCPWM_UNIT_0,MCPWM_TIMER_0 , MCPWM_GEN_B , MCPWM_DUTY_MODE_1);

  
  // Configuro y  deshabilito el PWM
  gpio_reset_pin(GPIO_ENABLE);
  gpio_set_direction(GPIO_ENABLE, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_ENABLE, OFF);

  /* Set the GPIO as a push/pull output para el led */
  gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_level(BLINK_GPIO, 0);

}

/******************************************************************************
 IO_gpioPWM(): setea la velocidad del PWM(16bits), el puente H se habilita 
*******************************************************************************/
void IO_gpioPWM(float set_pwm){

  //mcpwm_set_signal_low(MCPWM_UNIT_0,MCPWM_TIMER_0,MCPWM_GEN_A);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, set_pwm);  
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, set_pwm);   
  
  // Habilito el PWM
  gpio_set_level(GPIO_ENABLE, 1);
}

/******************************************************************************
IO_gpioStop(): para el motor
*******************************************************************************/
void IO_gpioStop(){

  IO_gpioPWM(50.0);

}

/******************************************************************************
PWM_calculo(): devuelve el pwm en formato float
*******************************************************************************/
float PWM_calculo(void){
  float err_temp;

  vTaskSuspendAll();
  // Término proporcional P[k] = Kp * (b*r[k] - y[k])
  pwm.error=(float) ((pid.b*data_ADC1-data_ADC2)*(PWM_OFFSET/ADC_MAX_COUNT));
  pwm.p=pid.kp*pwm.error;

  // Término integral I[k+1] = I[k] + Ki*h*e[k], con e[k] = r[k] - y[k]
  err_temp=(float)(data_ADC1-data_ADC2);
  if((pwm.pwm>=PWM_MAX_POS)||(pwm.pwm<=PWM_MAX_NEG)
    || abs((int) err_temp)<DELTA_ADC ) { // Rutina reset Anti Wind-Up
    pwm.i=0;
    }
  else{
    pwm.i=pwm.i+pid.ki*pid.h*err_temp;
    if (pwm.i>PI_MAX_POS) pwm.i=PI_MAX_POS;
    if (pwm.i<PI_MAX_NEG) pwm.i=PI_MAX_NEG;
  };


  //*********************************************************************
  // pwm.acc=pwm.acc+pwm.error;
  // if (pwm.pwm>=PWM_MAX_POS) pwm.acc=0;   // Rutina reset Anti Wind-Up
  // if (pwm.pwm<=PWM_MAX_NEG) pwm.acc=0;
  // pwm.i=pid.ki*pwm.acc;
  //*********************************************************************

  // Término derivativo D[k] = (Kd/(Kd + N*h)) * D[k-1] - (N*h*Kd/(Kd+N*h)) * (y[k]-y[k-1])
  pwm.d=(pid.kd*pwm.past_d-pid.N*pid.h*pid.kd*(data_ADC2-pwm.past_y))/(pid.kd+pid.N*pid.h);
  //pwm.d= pid.kd*(data_ADC2-pwm.past_y);
  pwm.past_y=data_ADC2;


  pwm.d=0;
  pwm.i=0;


  // Calculo de PID
  pwm.pwm=pwm.p+pwm.i+pwm.d;

  // Limitador
  if (pwm.pwm>PWM_MAX_POS) pwm.pwm=PWM_MAX_POS;
  if (pwm.pwm<PWM_MAX_NEG) pwm.pwm=PWM_MAX_NEG;
  xTaskResumeAll();

  return(pwm.pwm);
}

/******************************************************************************
debug(char *): publca MQTT para debug
*******************************************************************************/
void debug(char * mensa){
  MQTT_publish("test/debug", mensa);
}