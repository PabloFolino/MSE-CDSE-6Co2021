# MSE-CDSE-6Co2021
Materia de Control Digital de Sistemas Embebidos  de la Maestría de Sistemas Embebidos de la FIUBA.

## Alumnos: 
Leonardo Daniel Del Sancio 
Mauricio	Barroso Benavides
Pablo Daniel Folino
         
## Profesores:
 Leonardo Carducci 
 Sebastían García Marra

## Título del proyecto
 
 Sistema didáctico para el control de posiciones de un motor de CC
 
## Banco de Prueba

![](/Videos/banco.jpeg)

## Descripción
 
 En este trabajo se estudia el control de posición de un motor de CC. Se
 utiliza un puente H con señales PWM(modulación de ancho de pulso) para
 controlar al motor.
 Una placa MCU-ESP32 posee la lógica de control con un sistema PID(proporcinal-integral y derivativo), además del
 sistema de comunicación con el mundo exterior Wifi, mediante el protocolo MQTT.
 Una computadora con un broker Mosquitto se encarga de enviar y recibir información a módulo ESP32.
 Desde la computadora se puede:
 
	1) Confidurar las constantes del PID kp,ki,kd,N,b,h.
	2) Configurar los modos de trabajo del banco de prueba:
			a)modo=0--> PID asociado al potenciómetro
   			b)     =1--> PID asociado a MQTT, el setpoint se envía x MQTT
			c)     =2--> Disparo único asociado a MQTT
	3) Setear una posición del controlador.
	4) Configurara en número de muestras(10-250)
 
 
## Importante:
 Configurar en el archivo **sdkconfig.h**  la línea **"define CONFIG_FREERTOS_HZ 1000"**. 
