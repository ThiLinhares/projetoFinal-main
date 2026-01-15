#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h" 
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h" 

// --- Inclusões do FreeRTOS ---
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "u8g2pico.h"
#include "bme280_driver.h"

/* ================= HARDWARE ================= */

// DISPLAY (I2C1)
#define DISPLAY_I2C_PORT    i2c1    
#define DISPLAY_SDA_PIN     14      
#define DISPLAY_SCL_PIN     15      
#define DISPLAY_I2C_ADDRESS 0x3C    

// BME280 (I2C0)
#define BME280_I2C_PORT     i2c0
#define BME280_SDA_PIN      0
#define BME280_SCL_PIN      1

// ADS1115 (Soft I2C)
#define SOFT_SDA_PIN        2
#define SOFT_SCL_PIN        3
#define ADS1115_ADDR        0x48

// PERIFÉRICOS
#define LED_G_PIN 11 
#define LED_B_PIN 12 
#define LED_R_PIN 13 
#define BUZZER_PIN 21

/* ================= CALIBRAÇÃO (LIMITES) ================= */

// MQ-2 (Gás/Fumaça)
#define MQ2_LIMIT_WARNING     0.8f   
#define MQ2_LIMIT_ALARM       1.2f   

// MQ-7 (Monóxido)
#define MQ7_LIMIT_WARNING     1.8f   
#define MQ7_LIMIT_ALARM       2.5f   

// IR (Fogo) - DESCE com fogo
#define FIRE_LIMIT_ALARM      1.5f   

#define TEMP_LIMIT_WARNING    45.0f 
#define TEMP_LIMIT_ALARM      57.0f 

#define BRILHO_LED 25

typedef struct {
    float    temperature;
    float    pressure;
    float    humidity;   
    float    voltagem_mq2; 
    float    voltagem_mq7; 
    float    voltagem_ir;  
    bool     bmeOK;   
    bool     adsOK;       
} SensorData_t;

QueueHandle_t xQueueSensores;
u8g2_t u8g2_global; 

// Protótipos
void inicializarDisplay(u8g2_t *u8g2);
void inicializarHardware();
void tocar_buzzer_pwm(uint frequency, uint duration_ms); 
void tocarAlarmeProfissional(bool critico); 
void definirBrilhoLED(uint gpio, uint8_t brilho);
float ler_adc_soft_i2c(uint8_t canal, bool *sucesso); 
void executar_autoteste_inicial(); 
void vTaskDisplayControl(void *pvParameters);
void vTaskSensores(void *pvParameters);

// Hooks FreeRTOS
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask; (void)pcTaskName;
    printf("\n[FATAL] Stack Overflow: %s\n", pcTaskName);
    while (true) {};
}
void vApplicationMallocFailedHook(void) {
    printf("\n[FATAL] Malloc Failed\n");
    while(true) {};
}
void vApplicationTickHook(void) {}

/* ================= SOFT I2C ================= */
void soft_i2c_init() {
    gpio_init(SOFT_SDA_PIN); gpio_init(SOFT_SCL_PIN);
    gpio_set_dir(SOFT_SDA_PIN, GPIO_IN); gpio_set_dir(SOFT_SCL_PIN, GPIO_IN);
    gpio_pull_up(SOFT_SDA_PIN); gpio_pull_up(SOFT_SCL_PIN);
}
void soft_i2c_delay() { sleep_us(5); }
void sda_high() { gpio_set_dir(SOFT_SDA_PIN, GPIO_IN); }
void scl_high() { gpio_set_dir(SOFT_SCL_PIN, GPIO_IN); }
void sda_low() { gpio_set_dir(SOFT_SDA_PIN, GPIO_OUT); gpio_put(SOFT_SDA_PIN, 0); }
void scl_low() { gpio_set_dir(SOFT_SCL_PIN, GPIO_OUT); gpio_put(SOFT_SCL_PIN, 0); }
bool sda_read() { return gpio_get(SOFT_SDA_PIN); }

void soft_i2c_start() {
    sda_high(); scl_high(); soft_i2c_delay();
    sda_low();  soft_i2c_delay();
    scl_low();  soft_i2c_delay();
}
void soft_i2c_stop() {
    sda_low(); scl_low(); soft_i2c_delay();
    scl_high(); soft_i2c_delay();
    sda_high(); soft_i2c_delay();
}
void soft_i2c_write_byte(uint8_t data) {
    for(int i=0; i<8; i++) {
        if (data & 0x80) sda_high(); else sda_low();
        data <<= 1;
        soft_i2c_delay(); scl_high(); soft_i2c_delay(); scl_low(); soft_i2c_delay();
    }
    sda_high(); soft_i2c_delay(); scl_high(); soft_i2c_delay(); scl_low(); soft_i2c_delay();
}
uint8_t soft_i2c_read_byte(bool ack) {
    uint8_t data = 0;
    sda_high();
    for(int i=0; i<8; i++) {
        data <<= 1;
        scl_high(); soft_i2c_delay();
        if(sda_read()) data |= 1;
        scl_low(); soft_i2c_delay();
    }
    if(ack) sda_low(); else sda_high();
    soft_i2c_delay(); scl_high(); soft_i2c_delay(); scl_low(); soft_i2c_delay(); sda_high(); 
    return data;
}

float ler_adc_soft_i2c(uint8_t canal, bool *sucesso) {
    uint8_t config_msb;
    if (canal == 0) config_msb = 0xC1;
    else if (canal == 1) config_msb = 0xD1;
    else if (canal == 2) config_msb = 0xE1;
    else config_msb = 0xF1;

    soft_i2c_start();
    soft_i2c_write_byte(ADS1115_ADDR << 1); soft_i2c_write_byte(0x01); 
    soft_i2c_write_byte(config_msb); soft_i2c_write_byte(0x83); 
    soft_i2c_stop();
    sleep_ms(15); 
    soft_i2c_start();
    soft_i2c_write_byte(ADS1115_ADDR << 1); soft_i2c_write_byte(0x00);
    soft_i2c_stop();
    soft_i2c_start();
    soft_i2c_write_byte((ADS1115_ADDR << 1) | 1); 
    uint8_t msb = soft_i2c_read_byte(true); uint8_t lsb = soft_i2c_read_byte(false); 
    soft_i2c_stop();

    int16_t raw = (msb << 8) | lsb;
    if (raw < 0) raw = 0;
    *sucesso = true; 
    return raw * 0.0001875f;
}

/* ================= SISTEMA DE SOM PROFISSIONAL ================= */

void tocar_buzzer_pwm(uint frequency, uint duration_ms) {
    if (frequency == 0) {
        pwm_set_gpio_level(BUZZER_PIN, 0);
        sleep_ms(duration_ms);
        return;
    }

    uint32_t clock_freq = 125000000; 
    uint32_t divider = 16; 
    uint32_t wrap = (clock_freq / divider) / frequency;

    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, (float)divider);
    pwm_config_set_wrap(&config, wrap);
    pwm_init(slice_num, &config, true);

    pwm_set_gpio_level(BUZZER_PIN, wrap / 2); 
    
    sleep_ms(duration_ms);
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

void tocarAlarmeProfissional(bool critico) {
    if (critico) {
        // Sirene de 400 a 3000Hz
        for (int f = 400; f < 3000; f += 200) {
            tocar_buzzer_pwm(f, 10); 
        }
    } else {
        // Aviso simples
        tocar_buzzer_pwm(3000, 100); 
        sleep_ms(50);
        tocar_buzzer_pwm(0, 50);     
    }
}

/* ================= POST ================= */
void executar_autoteste_inicial() {
    printf("\n[POST] --- AUTO-TESTE (Interface Pro) ---\n");
    inicializarDisplay(&u8g2_global);
    u8g2_SetFont(&u8g2_global, u8g2_font_6x10_tf);
    
    printf("[POST] Testando Sirene...\n");
    u8g2_DrawStr(&u8g2_global, 20, 30, "INICIANDO...");
    u8g2_DrawStr(&u8g2_global, 20, 45, "Calibrando...");
    u8g2_SendBuffer(&u8g2_global);
    
    tocar_buzzer_pwm(1000, 100);
    tocar_buzzer_pwm(1500, 100);
    tocar_buzzer_pwm(2000, 100);
    
    definirBrilhoLED(LED_R_PIN, 255); sleep_ms(100); definirBrilhoLED(LED_R_PIN, 0);
    definirBrilhoLED(LED_G_PIN, 255); sleep_ms(100); definirBrilhoLED(LED_G_PIN, 0);
    definirBrilhoLED(LED_B_PIN, 255); sleep_ms(100); definirBrilhoLED(LED_B_PIN, 0);

    bool bme_ok = bme280_init(BME280_I2C_PORT, BME280_SDA_PIN, BME280_SCL_PIN);
    bool ads_ok = false;
    ler_adc_soft_i2c(0, &ads_ok); 

    u8g2_ClearBuffer(&u8g2_global);
    if (bme_ok && ads_ok) {
        u8g2_DrawStr(&u8g2_global, 0, 40, "SISTEMA OK");
        u8g2_SendBuffer(&u8g2_global);
        sleep_ms(1000);
    } else {
        u8g2_DrawStr(&u8g2_global, 0, 40, "FALHA HW");
        u8g2_SendBuffer(&u8g2_global);
        tocar_buzzer_pwm(200, 500); 
        sleep_ms(1000);
    }
}

/* ================= MAIN ================= */
int main() {
    stdio_init_all();
    sleep_ms(2000); 

    inicializarHardware();
    soft_i2c_init(); 
    executar_autoteste_inicial();

    xQueueSensores = xQueueCreate(5, sizeof(SensorData_t));
    xTaskCreate(vTaskDisplayControl, "DisplayCtrl", 2048, NULL, 1, NULL);
    xTaskCreate(vTaskSensores, "LeituraSens", 1024, NULL, 2, NULL);

    vTaskStartScheduler();
    while (true) {};
}

/* ================= TASKS ================= */
void vTaskSensores(void *pvParameters) {
    (void)pvParameters;
    SensorData_t data;
    bool bme_conectado = bme280_init(BME280_I2C_PORT, BME280_SDA_PIN, BME280_SCL_PIN);
    
    while (true) {
        if (bme_conectado) {
            bme280_data_t readings;
            bme280_read(&readings);
            data.temperature = readings.temperature;
            data.pressure = readings.pressure;
            data.humidity = readings.humidity;
            data.bmeOK = true;
        } else {
            bme_conectado = bme280_init(BME280_I2C_PORT, BME280_SDA_PIN, BME280_SCL_PIN);
            data.bmeOK = false;
        }

        float soma_mq2 = 0.0f, soma_mq7 = 0.0f, soma_ir = 0.0f;
        bool leitura_ok = false;
        bool todas_ok = true;
        int amostras = 5;

        for (int i = 0; i < amostras; i++) {
            soma_mq2 += ler_adc_soft_i2c(0, &leitura_ok); if(!leitura_ok) todas_ok = false;
            soma_mq7 += ler_adc_soft_i2c(1, &leitura_ok); if(!leitura_ok) todas_ok = false;
            soma_ir  += ler_adc_soft_i2c(2, &leitura_ok); if(!leitura_ok) todas_ok = false;
            sleep_ms(5); 
        }
        data.voltagem_mq2 = soma_mq2 / amostras;
        data.voltagem_mq7 = soma_mq7 / amostras;
        data.voltagem_ir  = soma_ir  / amostras;
        data.adsOK = todas_ok;

        if (data.bmeOK && data.adsOK) {
            // No serial mantemos os dados técnicos para debug
            printf("[MONITOR] MQ2:%.2fV | MQ7:%.2fV | CHAMA:%.2fV\n", 
                   data.voltagem_mq2, data.voltagem_mq7, data.voltagem_ir);
        }
        xQueueSend(xQueueSensores, &data, 0);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

// --- TAREFA DE DISPLAY ATUALIZADA (Interface Amigável + Tela Cheia) ---
void vTaskDisplayControl(void *pvParameters) {
    (void)pvParameters;
    u8g2_t *u8g2 = &u8g2_global; 
    SensorData_t sData;
    char strBuffer[32];

    while (true) {
        if (xQueueReceive(xQueueSensores, &sData, pdMS_TO_TICKS(4000)) == pdTRUE) {
            u8g2_ClearBuffer(u8g2);

            if (!sData.adsOK) {
                u8g2_SetFont(u8g2, u8g2_font_6x10_tf);
                u8g2_DrawStr(u8g2, 0, 30, "ERRO SENSORES");
                u8g2_DrawStr(u8g2, 0, 45, "Verificar Cabos");
            } else {
                // Analise de Status
                bool fogo_ir  = (sData.voltagem_ir < FIRE_LIMIT_ALARM);
                bool mq2_gas  = (sData.voltagem_mq2 > MQ2_LIMIT_ALARM);
                bool mq7_co   = (sData.voltagem_mq7 > MQ7_LIMIT_ALARM);
                
                // Avisos (Amarelo)
                bool mq2_warn = (sData.voltagem_mq2 > MQ2_LIMIT_WARNING);
                bool mq7_warn = (sData.voltagem_mq7 > MQ7_LIMIT_WARNING);

                bool perigo_critico = (fogo_ir || mq2_gas || mq7_co);
                bool atencao = (mq2_warn || mq7_warn);

                // --- HARDWARE (LED/BUZZER) ---
                if (perigo_critico) {
                    definirBrilhoLED(LED_R_PIN, 255); definirBrilhoLED(LED_G_PIN, 0); definirBrilhoLED(LED_B_PIN, 0);
                    tocarAlarmeProfissional(true); 
                } else if (atencao) {
                    definirBrilhoLED(LED_R_PIN, 0); definirBrilhoLED(LED_G_PIN, 0); definirBrilhoLED(LED_B_PIN, BRILHO_LED);
                    tocarAlarmeProfissional(false); 
                } else {
                    definirBrilhoLED(LED_R_PIN, 0); definirBrilhoLED(LED_G_PIN, BRILHO_LED); definirBrilhoLED(LED_B_PIN, 0);
                }

                // --- DESENHO NA TELA (Lógica Nova) ---

                if (perigo_critico) {
                    // MODO TELA CHEIA (FULL SCREEN ALERT)
                    // Fundo Branco (Invertido) para chamar atenção
                    u8g2_SetDrawColor(u8g2, 1);
                    u8g2_DrawBox(u8g2, 0, 0, 128, 64);
                    u8g2_SetDrawColor(u8g2, 0); // Texto Preto (Vazado)
                    
                    // Fonte maior (Negrito)
                    u8g2_SetFont(u8g2, u8g2_font_ncenB14_tr); 

                    if (fogo_ir) {
                        u8g2_DrawStr(u8g2, 10, 30, "! FOGO !");
                        u8g2_SetFont(u8g2, u8g2_font_6x10_tf);
                        u8g2_DrawStr(u8g2, 20, 50, "Evacuar Area");
                    } else if (mq2_gas) {
                        u8g2_DrawStr(u8g2, 15, 30, "GAS VAZ.");
                        u8g2_SetFont(u8g2, u8g2_font_6x10_tf);
                        u8g2_DrawStr(u8g2, 10, 50, "Risco Explosao");
                    } else if (mq7_co) {
                        u8g2_DrawStr(u8g2, 5, 30, "CO ALTO!");
                        u8g2_SetFont(u8g2, u8g2_font_6x10_tf);
                        u8g2_DrawStr(u8g2, 20, 50, "Gas Toxico");
                    }
                    // Restaura cor normal para próxima volta
                    u8g2_SetDrawColor(u8g2, 1); 

                } else {
                    // MODO MONITORAMENTO (Interface Limpa)
                    u8g2_SetFont(u8g2, u8g2_font_6x10_tf);
                    
                    // Cabeçalho
                    if (atencao) u8g2_DrawStr(u8g2, 35, 10, "ATENCAO");
                    else u8g2_DrawStr(u8g2, 35, 10, "MONITORAMENTO");
                    u8g2_DrawHLine(u8g2, 0, 12, 128);

                    // Linha 1: Temperatura (Isso todo mundo entende)
                    sprintf(strBuffer, "Ambiente: %.1f C", sData.temperature);
                    u8g2_DrawStr(u8g2, 0, 25, strBuffer);

                    // Linha 2: Fumaça/Gás (Tradução para Humano)
                    char *status_mq2 = "Normal";
                    if(mq2_warn) status_mq2 = "Alto";
                    sprintf(strBuffer, "Fumaca: %s", status_mq2);
                    u8g2_DrawStr(u8g2, 0, 37, strBuffer);

                    // Linha 3: Monóxido (Tradução para Humano)
                    char *status_co = "Seguro";
                    if(mq7_warn) status_co = "Detectado";
                    sprintf(strBuffer, "CO: %s", status_co);
                    u8g2_DrawStr(u8g2, 0, 49, strBuffer);

                    // Linha 4: Fogo
                    u8g2_DrawStr(u8g2, 0, 61, "Fogo: Ausente");
                }
            }
            u8g2_SendBuffer(u8g2);
        }
    }
}

void configurarPWM_LED(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f);
    pwm_init(slice_num, &config, true);
}
void definirBrilhoLED(uint gpio, uint8_t brilho) {
    pwm_set_gpio_level(gpio, (uint16_t)brilho * 257); 
}
void inicializarHardware() {
    configurarPWM_LED(LED_R_PIN); configurarPWM_LED(LED_G_PIN); configurarPWM_LED(LED_B_PIN);
    definirBrilhoLED(LED_R_PIN, 0); definirBrilhoLED(LED_G_PIN, 0); definirBrilhoLED(LED_B_PIN, 0);
    
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
}
void inicializarDisplay(u8g2_t *u8g2) {
    u8g2_Setup_ssd1306_i2c_128x64_noname_f_pico((u8g2pico_t *)u8g2, DISPLAY_I2C_PORT, DISPLAY_SDA_PIN, DISPLAY_SCL_PIN, U8G2_R0, DISPLAY_I2C_ADDRESS);
    u8g2_InitDisplay(u8g2); 
    u8g2_SetPowerSave(u8g2, 0); 
    u8g2_ClearBuffer(u8g2);
}