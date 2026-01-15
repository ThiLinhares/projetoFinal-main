#include "bme280_driver.h"
#include <stdio.h>

// Registradores
#define REG_ID            0xD0
#define REG_RESET         0xE0
#define REG_CTRL_HUM      0xF2
#define REG_STATUS        0xF3
#define REG_CTRL_MEAS     0xF4
#define REG_CONFIG        0xF5
#define REG_PRESS_MSB     0xF7

// Calibração
#define REG_CALIB_1       0x88
#define REG_CALIB_2       0xE1
#define REG_CALIB_H1      0xA1 

static i2c_inst_t *g_i2c = NULL;
static uint8_t g_addr = BME280_ADDR;
static bool g_is_bme280 = true; // true = BME280, false = BMP280

struct {
    uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1, dig_H3; int16_t dig_H2, dig_H4, dig_H5; int8_t  dig_H6;
} calib;

int32_t t_fine; 

static void write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_write_blocking(g_i2c, g_addr, buf, 2, false);
}

static void read_calibration_data() {
    uint8_t buf[26];
    uint8_t reg;

    // T1-T3, P1-P9
    reg = 0x88;
    i2c_write_blocking(g_i2c, g_addr, &reg, 1, true);
    i2c_read_blocking(g_i2c, g_addr, buf, 24, false);

    calib.dig_T1 = (buf[1] << 8) | buf[0];
    calib.dig_T2 = (int16_t)((buf[3] << 8) | buf[2]);
    calib.dig_T3 = (int16_t)((buf[5] << 8) | buf[4]);
    calib.dig_P1 = (buf[7] << 8) | buf[6];
    calib.dig_P2 = (int16_t)((buf[9] << 8) | buf[8]);
    calib.dig_P3 = (int16_t)((buf[11] << 8) | buf[10]);
    calib.dig_P4 = (int16_t)((buf[13] << 8) | buf[12]);
    calib.dig_P5 = (int16_t)((buf[15] << 8) | buf[14]);
    calib.dig_P6 = (int16_t)((buf[17] << 8) | buf[16]);
    calib.dig_P7 = (int16_t)((buf[19] << 8) | buf[18]);
    calib.dig_P8 = (int16_t)((buf[21] << 8) | buf[20]);
    calib.dig_P9 = (int16_t)((buf[23] << 8) | buf[22]);

    // Umidade (apenas se for BME280)
    if (g_is_bme280) {
        reg = 0xA1;
        i2c_write_blocking(g_i2c, g_addr, &reg, 1, true);
        i2c_read_blocking(g_i2c, g_addr, &calib.dig_H1, 1, false);

        reg = 0xE1;
        i2c_write_blocking(g_i2c, g_addr, &reg, 1, true);
        i2c_read_blocking(g_i2c, g_addr, buf, 7, false);

        calib.dig_H2 = (int16_t)((buf[1] << 8) | buf[0]);
        calib.dig_H3 = buf[2];
        calib.dig_H4 = (int16_t)((buf[3] << 4) | (buf[4] & 0x0F));
        calib.dig_H5 = (int16_t)((buf[5] << 4) | (buf[4] >> 4));
        calib.dig_H6 = (int8_t)buf[6];
    }
}

bool bme280_init(i2c_inst_t *i2c, uint8_t sda_pin, uint8_t scl_pin) {
    g_i2c = i2c;
    i2c_init(g_i2c, 100 * 1000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    sleep_ms(100);

    uint8_t id_reg = REG_ID;
    uint8_t chip_id;
    int ret;

    // Scan simples para encontrar o endereço correto
    g_addr = 0x76;
    ret = i2c_write_blocking(g_i2c, g_addr, &id_reg, 1, true);
    if (ret < 0) {
        g_addr = 0x77;
        ret = i2c_write_blocking(g_i2c, g_addr, &id_reg, 1, true);
        if (ret < 0) return false; 
    }

    i2c_read_blocking(g_i2c, g_addr, &chip_id, 1, false);

    if (chip_id == 0x60) {
        g_is_bme280 = true;
    } else if (chip_id == 0x58) {
        g_is_bme280 = false; // Identificado como BMP280
    } else {
        return false; // ID desconhecido
    }

    write_reg(REG_RESET, 0xB6);
    sleep_ms(100);
    read_calibration_data();
    
    // Configura umidade apenas se for BME280
    if (g_is_bme280) write_reg(REG_CTRL_HUM, 0x01);
    
    write_reg(REG_CTRL_MEAS, 0x27); 
    write_reg(REG_CONFIG, 0xA0); 
    sleep_ms(100);

    return true;
}

int32_t compensate_temp(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t compensate_pressure(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);
    return (uint32_t)p;
}

uint32_t compensate_humidity(int32_t adc_H) {
    // Se for BMP280, não há leitura de umidade, retorna 0
    if (!g_is_bme280) return 0;
    
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) - (((int32_t)calib.dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) *
                   (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                   ((int32_t)calib.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

void bme280_read(bme280_data_t *data) {
    uint8_t buf[8];
    uint8_t reg = REG_PRESS_MSB;
    i2c_write_blocking(g_i2c, g_addr, &reg, 1, true);
    
    // O BMP280 só tem dados de Pressão e Temperatura (6 bytes), enquanto o BME280 tem + Umidade (8 bytes)
    // No entanto, ler 8 bytes no BMP280 geralmente não causa erro, apenas retorna lixo nos últimos 2 bytes.
    // Para ser seguro, lemos 8 bytes sempre.
    i2c_read_blocking(g_i2c, g_addr, buf, 8, false);

    int32_t adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    int32_t adc_T = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
    int32_t adc_H = (buf[6] << 8) | buf[7];

    data->temperature = compensate_temp(adc_T) / 100.0f;
    data->pressure = compensate_pressure(adc_P) / 256.0f / 100.0f;
    
    // A função compensate_humidity agora lida com a verificação de g_is_bme280
    data->humidity = compensate_humidity(adc_H) / 1024.0f;
}