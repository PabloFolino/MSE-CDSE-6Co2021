##!/bin/bash
# Configuración del broker y topic a escuchar.
# Configuración inicial:
broker="192.168.0.113"
port="1883"
topic1="pid/muestras_rx"
archivo="muestras_recibidas.txt"
#--------------------------------------------------
# Ponemos el cliente de mosquitto a escuchar
mosquitto_sub -t $topic1 -h $broker -p $port -v | while read value;    do
# Guardamos valores:
echo "$value" >> $archivo   # guardamos datos en archivo
echo " $value"              # mostramos el resultado por consola
done
# Ejecútelo mediante ./muestras_mqtt.sh.
