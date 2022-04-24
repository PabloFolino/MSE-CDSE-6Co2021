/** \file scd-folino.h
 *  Mar 2022
 *  Maestría en SIstemas Embebidos - Sistemas embebidos distribuidos
 */
#include "esp_adc_cal.h"
#include "driver/mcpwm.h"
#include "config.h"

#ifndef SCD_FOLINO_H_
#define SCD_FOLINO_H_

// PWM
#define KP_INIT 16.0
#define KI_INIT 0       // 1.001
#define KD_INIT 0       // 1.01
#define B_INIT  1.0
#define H_INIT  1.0
#define N_INI   10.0
#define N_MUESTRAS 200
#define N_MUESTRAS_MAX 250
#define N_MUESTRAS_MIN 10
#define PWM_MAX_POS 50.0
#define PWM_MAX_NEG -50.0
#define PI_MAX_POS 35.0
#define PI_MAX_NEG -35.0
#define PWM_OFFSET 50.0
#define ADC_MAX_COUNT 4096.0        // Con atten = ADC_ATTEN_MAX
#define ADC_MIN_COUNT 0
#define SETPOINT_MQTT (ADC_MAX_COUNT/2.0)
#define DELTA_ADC 100
#define GPIO_EN3 GPIO_NUM_25        // GIRO 1
#define GPIO_EN4 GPIO_NUM_33        // GIRO 2
#define GPIO_ENABLE GPIO_NUM_32     // Enable el PWM
#define PWM_FREQ_HZ 30000
float PWM_calculo(void);

// GPIO -PWM
#define TIME_50US 50
void IO_gpioInit(void);
void IO_gpioPWM(float);
void IO_gpioStop(void);
// GPIO -Generales
int IO_setLed(int);
void IO_toggleLed(void);

// ADC
void IO_adcInits(void);
void IO_readAdc(void);

// RTOS
//static void RTOS_timerCallback(void*);
void RTOS_timerInit(void);
void RTOS_timerStart(uint64_t);
void RTOS_timerStop(void);
void RTOS_delayMs(int);

// Debug
void debug(char * );

// WTD
#define TWDT_TIMEOUT_S 5

struct pid_t {
  float kp;
  float ki;
  float kd;
  float b;            // Beta-- > ponderación del setpoint
  float N;            // longitud del filtro de la derivada
  float h;
};

struct pwm_t{
  float p;
  float i;
  float d;
  float past_d;
  float past_y;
  // float acc;      // Se usa para el calculo de la integral
  float error;
  float pwm;
  float setpoint_mqtt;
  int16_t c_muestras; // Cantidad de muestras a tomar en el disparo único
  int16_t n_muestra;
  uint8_t disparo;    // usa para el disparo único 
                      //    --> 0 no disp           --> 1 disparado 
                      //    --> 2 tomando muestras  --> 3 transmitiendo resultados
  uint8_t modo;       // modo=0--> PID asociado al potenciómetro
                      //       =1--> PID asociado a MQTT, el setpoint se envía x MQTT
                      //       =2--> Disparo único asociado a MQTT
};


#endif
