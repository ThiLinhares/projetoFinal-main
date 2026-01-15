## Leitura de Temperatura e Umidade com AHT10 e Raspberry Pi Pico

Este projeto realiza a leitura de temperatura e umidade usando o sensor AHT10 conectado ao Raspberry Pi Pico, exibindo os dados em um display OLED SSD1306 (128x64) via I²C e enviando-os por UART para um terminal serial.

### Funcionalidades

- Leitura precisa de temperatura (°C) e umidade relativa (%)
- Exibição dos dados em display OLED SSD1306
- Alertas visuais para alta umidade (>70%) e baixa temperatura (<20°C)
- Envio dos dados via UART (115200 baud)

---

### Hardware Necessário

- Raspberry Pi Pico ou Pico W
- Sensor AHT10 (I²C)
- Display OLED SSD1306 128x64 (I²C)
- Cabos para conexão

#### Ligações sugeridas

| Dispositivo         | Pico GPIO | Função         |
|---------------------|-----------|---------------|
| AHT10 VCC           | 3V3       | Alimentação   |
| AHT10 GND           | GND       | Terra         |
| AHT10 SDA           | GP0       | SDA (I²C0)    |
| AHT10 SCL           | GP1       | SCL (I²C0)    |
| SSD1306 VCC         | 3V3       | Alimentação   |
| SSD1306 GND         | GND       | Terra         |
| SSD1306 SDA         | GP14      | SDA (I²C1)    |
| SSD1306 SCL         | GP15      | SCL (I²C1)    |
| UART TX (opcional)  | GP8       | UART1 TX      |
| UART RX (opcional)  | GP9       | UART1 RX      |

---

### Como Compilar e Gravar

1. Instale o [Pico SDK](https://github.com/raspberrypi/pico-sdk) (v2.1.1 ou superior) e o toolchain GCC ARM.
2. Clone este repositório e acesse a pasta do projeto.
3. Crie a pasta de build:
   ```bash
   mkdir build && cd build
   ```
4. Configure o projeto:
   ```bash
   cmake ..
   ```
5. Compile o firmware:
   ```bash
   cmake --build .
   ```
6. Grave o firmware no Pico usando a task `Run Project` ou `Flash` do VS Code, ou utilize o UF2 gerado.

---

### Como Usar

1. Conecte o Raspberry Pi Pico ao PC via USB.
2. Grave o firmware compilado.
3. Abra um terminal serial (baudrate 115200) para visualizar os dados enviados.
4. O display OLED mostrará temperatura e umidade em tempo real, com alertas visuais.

---

### Estrutura dos Arquivos

- `AHT10.h` / `AHT10.c`: Funções e definições para comunicação com o sensor AHT10
- `main.c`: Inicialização, leitura dos sensores, exibição e envio UART
- `lib/u8g2pico/`: Biblioteca para controle do display OLED
- `CMakeLists.txt`: Configuração do build

---

### Bibliotecas Utilizadas

- [U8G2Pico](https://github.com/georgines/u8g2pico) – Wrapper U8G2 para displays SSD1306 no Pico

---

### Licença

MIT. Veja o arquivo LICENSE para detalhes.

---


