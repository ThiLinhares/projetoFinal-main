#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include "pico/stdlib.h"

// Estrutura de dados para envio via MQTT
// Contém exatamente o que os sensores leem
typedef struct {
    float    temperature;
    float    pressure;
    float    humidity;   
    float    voltagem_mq2; 
    float    voltagem_mq7; 
    float    voltagem_ir;  
    bool     bmeOK;   
    bool     adsOK;       
} MqttSensorData_t;

// Protótipo da tarefa MQTT
void vTaskMQTT(void *pvParameters);

#endif