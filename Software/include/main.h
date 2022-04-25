/** \file	main.h
 *  Mar 2022
 *  Maestría en SIstemas Embebidos / Sistemas embebidos distribuidos
 *  \brief	Contiene las bibliotecas y definiciones generales de configuración
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

/* Configuración general  */
#include "config.h"

/* Nivel de abstracción APID */
#include "mqtt.h"
#include "wifi.h"

/* Configuración pwm */
#include "pwm.h"

#endif /* MAIN_H_ */
