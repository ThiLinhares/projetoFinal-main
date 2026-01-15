#ifndef BME280_DRIVER_H
#define BME280_DRIVER_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Endereço padrão do BME280 (pode ser 0x76 ou 0x77)
#define BME280_ADDR 0x76

// Estrutura para armazenar os dados lidos já compensados
typedef struct {
    float temperature; // Graus Celsius
    float pressure;    // hPa
    float humidity;    // %
} bme280_data_t;

// Inicializa o BME280 na instância I2C fornecida
// Retorna true se sucesso (chip ID verificado)
bool bme280_init(i2c_inst_t *i2c, uint8_t sda_pin, uint8_t scl_pin);

// Lê os dados compensados do sensor
void bme280_read(bme280_data_t *data);

#endif