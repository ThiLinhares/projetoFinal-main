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

/* ================= HARDWARE (BitDogLab V6.3) ================= */

// DISPLAY (I2C1 - Fixo na placa)
#define DISPLAY_I2C_PORT    i2c1    
#define DISPLAY_SDA_PIN     14      
#define DISPLAY_SCL_PIN     15      
#define DISPLAY_I2C_ADDRESS 0x3C    

// BME280 (I2C0 - Sensor de Temperatura/Pressão)
#define BME280_I2C_PORT     i2c0
#define BME280_SDA_PIN      0
#define BME280_SCL_PIN      1

// ADS1115 (Soft I2C Manual - Para evitar conflitos)
#define SOFT_SDA_PIN        2
#define SOFT_SCL_PIN        3
#define ADS1115_ADDR        0x48

// PERIFÉRICOS DE SAÍDA
#define LED_G_PIN 11 
#define LED_B_PIN 12 
#define LED_R_PIN 13 
#define BUZZER_PIN 21

// PERIFÉRICOS DE ENTRADA
#define BUTTON_A_PIN 5  

/* ================= LIMITES COM HISTERESE ================= */

// MQ-2 (Gás/Fumo)
#define MQ2_SET_POINT         1.2f   // Ativa
#define MQ2_RESET_POINT       1.0f   // Desativa
#define MQ2_WARNING           0.8f

// MQ-7 (Monóxido)
#define MQ7_SET_POINT         2.5f   // Ativa
#define MQ7_RESET_POINT       2.2f   // Desativa
#define MQ7_WARNING           1.8f

// IR (Fogo) - Lógica "Low Active" (Tensão DESCE com fogo)
#define FIRE_SET_POINT        1.5f   // Ativa (< 1.5V)
#define FIRE_RESET_POINT      1.8f   // Desativa (> 1.8V)

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

/* ================= SOFT I2C (DRIVER ADS1115) ================= */
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

/* ================= SISTEMA DE SOM ================= */

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
        for (int f = 400; f < 3000; f += 200) {
            tocar_buzzer_pwm(f, 10); 
        }
    } else {
        tocar_buzzer_pwm(3000, 100); 
        sleep_ms(50);
        tocar_buzzer_pwm(0, 50);     
    }
}

/* ================= POST / SETUP ================= */
void executar_autoteste_inicial() {
    printf("\n[POST] --- AUTO-TESTE INICIADO ---\n");
    inicializarDisplay(&u8g2_global);
    u8g2_SetFont(&u8g2_global, u8g2_font_6x10_tf);
    
    u8g2_DrawStr(&u8g2_global, 20, 30, "INICIANDO...");
    u8g2_DrawStr(&u8g2_global, 20, 45, "A calibrar...");
    u8g2_SendBuffer(&u8g2_global);
    
    tocar_buzzer_pwm(1000, 100);
    
    // Pisca cores
    definirBrilhoLED(LED_R_PIN, 255); sleep_ms(100); definirBrilhoLED(LED_R_PIN, 0);
    definirBrilhoLED(LED_G_PIN, 255); sleep_ms(100); definirBrilhoLED(LED_G_PIN, 0);
    definirBrilhoLED(LED_B_PIN, 255); sleep_ms(100); definirBrilhoLED(LED_B_PIN, 0);

    // Teste Sensores
    bool ads_ok = false;
    ler_adc_soft_i2c(0, &ads_ok); 
    bool bme_ok = bme280_init(BME280_I2C_PORT, BME280_SDA_PIN, BME280_SCL_PIN);

    u8g2_ClearBuffer(&u8g2_global);
    if (ads_ok && bme_ok) {
        u8g2_DrawStr(&u8g2_global, 0, 40, "SISTEMA OK");
        printf("[POST] Hardware OK.\n");
    } else {
        u8g2_DrawStr(&u8g2_global, 0, 30, "ERRO HARDWARE");
        if (!ads_ok) u8g2_DrawStr(&u8g2_global, 0, 45, "ADS1115 Falhou");
        if (!bme_ok) u8g2_DrawStr(&u8g2_global, 0, 55, "BME280 Falhou");
        printf("[POST] Falha de Hardware Detetada.\n");
        sleep_ms(2000);
    }
    u8g2_SendBuffer(&u8g2_global);
    sleep_ms(1000);
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

/* ================= TASK DE SENSORES ================= */
void vTaskSensores(void *pvParameters) {
    (void)pvParameters;
    SensorData_t data;
    bool bme_conectado = bme280_init(BME280_I2C_PORT, BME280_SDA_PIN, BME280_SCL_PIN);
    
    while (true) {
        // --- 1. BME280 ---
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
            data.temperature = 0.0f; 
            data.humidity = 0.0f;
        }

        // --- 2. Sensores Analógicos ---
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

        // --- 3. Monitor Serial ---
        if (todas_ok) {
            char *status_fumo = (data.voltagem_mq2 > MQ2_SET_POINT) ? "PERIGO" : "Normal";
            char *status_co   = (data.voltagem_mq7 > MQ7_SET_POINT) ? "PERIGO" : "Normal";
            char *status_fogo = (data.voltagem_ir < FIRE_SET_POINT) ? "FOGO DETETADO!" : "Seguro";

            if (data.bmeOK) {
                printf("[AMBIENTE] Temp: %.1fC | Humid: %.1f%% || [SENSORES] Fumo: %s (%.2fV) | CO: %s (%.2fV) | Fogo: %s (%.2fV)\n", 
                       data.temperature, data.humidity,
                       status_fumo, data.voltagem_mq2,
                       status_co, data.voltagem_mq7,
                       status_fogo, data.voltagem_ir);
            } else {
                printf("[AMBIENTE] Erro BME280 || [SENSORES] Fumo: %s (%.2fV) | CO: %s | Fogo: %s\n",
                       status_fumo, data.voltagem_mq2, status_co, status_fogo);
            }
        } else {
            printf("[ERRO] Falha ADS1115.\n");
        }

        xQueueSend(xQueueSensores, &data, 0);
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

/* ================= TASK DE DISPLAY ================= */
void vTaskDisplayControl(void *pvParameters) {
    (void)pvParameters;
    u8g2_t *u8g2 = &u8g2_global; 
    SensorData_t sData;
    char strBuffer[32];

    // Estados com Histerese
    static bool state_fire = false;
    static bool state_gas = false;
    static bool state_co = false;

    // Silêncio
    bool modo_silencioso = false;
    absolute_time_t fim_do_silencio;

    // Heartbeat
    static bool heartbeat = false;

    while (true) {
        if (xQueueReceive(xQueueSensores, &sData, pdMS_TO_TICKS(100)) == pdTRUE) {
            
            // --- 1. Histerese ---
            if (sData.voltagem_ir < FIRE_SET_POINT) state_fire = true;
            else if (sData.voltagem_ir > FIRE_RESET_POINT) state_fire = false;

            if (sData.voltagem_mq2 > MQ2_SET_POINT) state_gas = true;
            else if (sData.voltagem_mq2 < MQ2_RESET_POINT) state_gas = false;

            if (sData.voltagem_mq7 > MQ7_SET_POINT) state_co = true;
            else if (sData.voltagem_mq7 < MQ7_RESET_POINT) state_co = false;

            bool perigo_critico = (state_fire || state_gas || state_co);
            bool atencao = (!perigo_critico) && (sData.voltagem_mq2 > MQ2_WARNING || sData.voltagem_mq7 > MQ7_WARNING);

            // --- 2. Heartbeat (Alterna estado) ---
            heartbeat = !heartbeat; 

            // --- 3. Botão A (Silenciar) ---
            if (gpio_get(BUTTON_A_PIN) == 0) { 
                if (!modo_silencioso) {
                    printf("[USER] SILENCIAR ATIVO (5s)\n");
                    modo_silencioso = true;
                    fim_do_silencio = make_timeout_time_ms(5000);
                    // Apaga tudo
                    definirBrilhoLED(LED_R_PIN, 0); definirBrilhoLED(LED_G_PIN, 0); definirBrilhoLED(LED_B_PIN, 0);
                    pwm_set_gpio_level(BUZZER_PIN, 0);
                }
            }

            if (modo_silencioso) {
                if (time_reached(fim_do_silencio)) {
                    printf("[SYSTEM] SILENCIO EXPIROU\n");
                    modo_silencioso = false;
                }
            }

            // --- 4. Display ---
            u8g2_ClearBuffer(u8g2);

            if (!sData.adsOK) {
                u8g2_SetFont(u8g2, u8g2_font_6x10_tf);
                u8g2_DrawStr(u8g2, 0, 30, "ERRO SENSORES");
                definirBrilhoLED(LED_B_PIN, 50); 
            } else {
                
                if (modo_silencioso) {
                    // SILÊNCIO
                    u8g2_SetFont(u8g2, u8g2_font_ncenB10_tr);
                    u8g2_DrawStr(u8g2, 10, 20, "ALARME");
                    u8g2_DrawStr(u8g2, 10, 40, "SILENCIADO");
                    
                    int tempo = absolute_time_diff_us(get_absolute_time(), fim_do_silencio) / 1000;
                    if(tempo < 0) tempo = 0;
                    u8g2_DrawBox(u8g2, 0, 55, (tempo * 128) / 5000, 5);

                    definirBrilhoLED(LED_R_PIN, 0); pwm_set_gpio_level(BUZZER_PIN, 0);

                } else {
                    // MODO ATIVO
                    if (perigo_critico) {
                        // CRÍTICO
                        u8g2_SetDrawColor(u8g2, 1);
                        u8g2_DrawBox(u8g2, 0, 0, 128, 64);
                        u8g2_SetDrawColor(u8g2, 0); 
                        
                        u8g2_SetFont(u8g2, u8g2_font_ncenB14_tr);
                        if (state_fire) {
                            u8g2_DrawStr(u8g2, 15, 30, "! FOGO !");
                            u8g2_SetFont(u8g2, u8g2_font_6x10_tf);
                            u8g2_DrawStr(u8g2, 10, 50, "Aperte A p/ Parar");
                        } else if (state_gas) {
                            u8g2_DrawStr(u8g2, 15, 30, "GAS VAZ.");
                        } else {
                            u8g2_DrawStr(u8g2, 5, 30, "CO TOXICO");
                        }
                        
                        u8g2_SetDrawColor(u8g2, 1);
                        definirBrilhoLED(LED_R_PIN, 255);
                        tocarAlarmeProfissional(true);

                    } else if (atencao) {
                        // ATENÇÃO
                        u8g2_SetFont(u8g2, u8g2_font_6x10_tf);
                        u8g2_DrawStr(u8g2, 35, 10, "ATENCAO");
                        sprintf(strBuffer, "Gas: %.1fV", sData.voltagem_mq2);
                        u8g2_DrawStr(u8g2, 0, 30, strBuffer);
                        
                        definirBrilhoLED(LED_B_PIN, BRILHO_LED);
                        tocarAlarmeProfissional(false);

                    } else {
                        // SEGURO
                        u8g2_SetFont(u8g2, u8g2_font_6x10_tf);
                        u8g2_DrawStr(u8g2, 30, 10, "MONITORAMENTO");
                        u8g2_DrawHLine(u8g2, 0, 12, 128);
                        
                        if (sData.bmeOK) sprintf(strBuffer, "T:%.1fC H:%.0f%%", sData.temperature, sData.humidity);
                        else sprintf(strBuffer, "T:-- C H:-- %%");
                        u8g2_DrawStr(u8g2, 0, 25, strBuffer);
                        
                        sprintf(strBuffer, "Fumo:%.1f CO:%.1f", sData.voltagem_mq2, sData.voltagem_mq7);
                        u8g2_DrawStr(u8g2, 0, 40, strBuffer);
                        
                        u8g2_DrawStr(u8g2, 0, 60, "Status: Seguro");

                        // HEARTBEAT
                        // Verde pisca (Heartbeat) - Azul desligado
                        if(heartbeat) {
                            u8g2_DrawBox(u8g2, 122, 0, 4, 4); // Ponto no display
                            definirBrilhoLED(LED_G_PIN, BRILHO_LED); // Verde Aceso
                        } else {
                             definirBrilhoLED(LED_G_PIN, 0); // Verde Apagado
                        }

                        // Garante que os outros estão apagados
                        definirBrilhoLED(LED_R_PIN, 0); 
                        definirBrilhoLED(LED_B_PIN, 0); 
                        pwm_set_gpio_level(BUZZER_PIN, 0);
                    }
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
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);
}
void inicializarDisplay(u8g2_t *u8g2) {
    u8g2_Setup_ssd1306_i2c_128x64_noname_f_pico((u8g2pico_t *)u8g2, DISPLAY_I2C_PORT, DISPLAY_SDA_PIN, DISPLAY_SCL_PIN, U8G2_R0, DISPLAY_I2C_ADDRESS);
    u8g2_InitDisplay(u8g2); 
    u8g2_SetPowerSave(u8g2, 0); 
    u8g2_ClearBuffer(u8g2);
}