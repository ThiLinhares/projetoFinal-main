#include "mqtt_handler.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// --- SUAS CREDENCIAIS WI-FI ---
#define WIFI_SSID "NOME_DA_SUA_REDE"
#define WIFI_PASSWORD "SENHA_DA_SUA_REDE"

#define MQTT_SERVER_HOST "broker.hivemq.com"
#define MQTT_SERVER_PORT 1883

// Fila externa (definida no main)
extern QueueHandle_t xQueueMQTT; 

typedef struct {
    mqtt_client_t *client;
    ip_addr_t remote_addr;
} mqtt_state_t;

// Callback de conexão
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Conectado ao Broker!\n");
    } else {
        printf("[MQTT] Falha na conexao: %d\n", status);
    }
}

static void mqtt_pub_request_cb(void *arg, err_t err) {
    if (err != ERR_OK) printf("[MQTT] Erro publicação: %d\n", err);
}

static void dns_found(const char *name, const ip_addr_t *ipaddr, void *callback_arg) {
    mqtt_state_t *state = (mqtt_state_t*)callback_arg;
    if (ipaddr) {
        state->remote_addr = *ipaddr;
        printf("[MQTT] DNS OK: %s\n", ip4addr_ntoa(ipaddr));
    }
}

// Tarefa Principal do MQTT
void vTaskMQTT(void *pvParameters) {
    mqtt_state_t state = {0};
    MqttSensorData_t data;
    char payload[64];
    struct mqtt_connect_client_info_t ci = {0};
    
    // 1. Inicializa Wi-Fi
    if (cyw43_arch_init()) {
        printf("[WIFI] Erro Init Wi-Fi\n");
        vTaskDelete(NULL);
    }
    cyw43_arch_enable_sta_mode();

    printf("[WIFI] A conectar...\n");
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("[WIFI] A tentar reconectar...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("[WIFI] Conectado!\n");

    // 2. Resolve DNS
    state.client = mqtt_client_new();
    cyw43_arch_lwip_begin(); 
    dns_gethostbyname(MQTT_SERVER_HOST, &state.remote_addr, dns_found, &state);
    cyw43_arch_lwip_end();

    while (state.remote_addr.addr == 0) vTaskDelay(pdMS_TO_TICKS(100));

    // 3. Configura Cliente
    ci.client_id = "PicoW_Monitor_Reciclagem";
    
    cyw43_arch_lwip_begin();
    mqtt_client_connect(state.client, &state.remote_addr, MQTT_SERVER_PORT, mqtt_connection_cb, &state, &ci);
    cyw43_arch_lwip_end();

    // 4. Loop de Publicação
    while (true) {
        if (xQueueReceive(xQueueMQTT, &data, pdMS_TO_TICKS(1000)) == pdTRUE) {
            
            if (mqtt_client_is_connected(state.client)) {
                cyw43_arch_lwip_begin(); 

                // Fumo (Gás)
                snprintf(payload, sizeof(payload), "%.2f", data.voltagem_mq2);
                mqtt_publish(state.client, "galpao/reciclagem/mq2_fumaça", payload, strlen(payload), 0, 0, mqtt_pub_request_cb, &state);

                // CO
                snprintf(payload, sizeof(payload), "%.2f", data.voltagem_mq7);
                mqtt_publish(state.client, "galpao/reciclagem/mq7_co", payload, strlen(payload), 0, 0, mqtt_pub_request_cb, &state);

                // Fogo
                snprintf(payload, sizeof(payload), "%.2f", data.voltagem_ir);
                mqtt_publish(state.client, "galpao/reciclagem/sensor_ir", payload, strlen(payload), 0, 0, mqtt_pub_request_cb, &state);

                // Temperatura
                if (data.bmeOK) {
                    snprintf(payload, sizeof(payload), "%.1f", data.temperature);
                    mqtt_publish(state.client, "galpao/reciclagem/temperatura", payload, strlen(payload), 0, 0, mqtt_pub_request_cb, &state);
                }

                cyw43_arch_lwip_end(); 
            } else {
                printf("[MQTT] Reconectando...\n");
                cyw43_arch_lwip_begin();
                mqtt_client_connect(state.client, &state.remote_addr, MQTT_SERVER_PORT, mqtt_connection_cb, &state, &ci);
                cyw43_arch_lwip_end();
            }
        }
    }
}