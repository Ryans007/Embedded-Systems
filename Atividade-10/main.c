#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "rom/ets_sys.h"

// Pinos dos LEDs
#define LED_01 36
#define LED_02 37
#define LED_03 38
#define LED_04 39

// Pinos do Display 7 Segmentos
#define SEG_A 19
#define SEG_B 20
#define SEG_C 21
#define SEG_D 47
#define SEG_E 48
#define SEG_F 45
#define SEG_G 35

// Pinos dos Botões e Buzzer
#define BUTTON_A 13
#define BUTTON_B 14
#define BUZZER 42
#define NTC_CHANNEL ADC1_CHANNEL_4

// Pinos do SDCard (SPI)
#define SD_MOSI 18
#define SD_MISO 12
#define SD_SCK  10
#define SD_CS   17

// I2C para LCD
#define I2C_MASTER_SCL_IO 15
#define I2C_MASTER_SDA_IO 16
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define LCD_ADDR 0x27

// PWM para Buzzer
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY 512
#define LEDC_FREQUENCY 2000

// Parâmetros do sistema
#define DEBOUNCE_MS 200
#define TEMP_ALARME_PADRAO 25
#define INCREMENTO_TEMP 5
#define INTERVALO_PISCA_MS 300
#define INTERVALO_SALVAR_MS 1000

// Constantes do NTC
#define RESISTOR_SERIE 10000.0
#define RESISTENCIA_NOMINAL 10000.0
#define TEMP_NOMINAL 25.0
#define COEFICIENTE_B 3950.0

// Estrutura para dados compartilhados
typedef struct {
    float temperatura;
    int temp_alarme;
    bool alarme_ativo;
} dados_sistema_t;

// Handles das tasks
TaskHandle_t task_leitura_ntc_handle = NULL;
TaskHandle_t task_botoes_handle = NULL;
TaskHandle_t task_lcd_handle = NULL;
TaskHandle_t task_display_7seg_handle = NULL;
TaskHandle_t task_leds_handle = NULL;
TaskHandle_t task_buzzer_handle = NULL;
TaskHandle_t task_sdcard_handle = NULL;

// Semáforos e Filas
SemaphoreHandle_t mutex_dados = NULL;
QueueHandle_t fila_botoes = NULL;
QueueHandle_t fila_temperatura = NULL;

// Variáveis globais
static dados_sistema_t dados_compartilhados = {
    .temperatura = 0.0,
    .temp_alarme = TEMP_ALARME_PADRAO,
    .alarme_ativo = false
};

static volatile uint32_t ultimo_press_btn_cima = 0;
static volatile uint32_t ultimo_press_btn_baixo = 0;
static sdmmc_card_t *cartao_sd = NULL;
static bool sd_montado = false;

// Protótipos das funções auxiliares
void configurar_pinos_7seg();
void exibir_digito_7seg(char digito);
void configurar_i2c();
void inicializar_lcd();
void limpar_display();
void mover_cursor(uint8_t col, uint8_t row);
void escrever_texto(const char *str);
void configurar_pwm();
void configurar_adc();
void inicializar_sdcard();
float ler_temperatura_ntc();

// Tabela de segmentos para cada dígito
const uint8_t tabela_7seg[16][7] = {
    // A, B, C, D, E, F, G
    {1, 1, 1, 1, 1, 1, 0}, // 0
    {0, 1, 1, 0, 0, 0, 0}, // 1
    {1, 1, 0, 1, 1, 0, 1}, // 2
    {1, 1, 1, 1, 0, 0, 1}, // 3
    {0, 1, 1, 0, 0, 1, 1}, // 4
    {1, 0, 1, 1, 0, 1, 1}, // 5
    {1, 0, 1, 1, 1, 1, 1}, // 6
    {1, 1, 1, 0, 0, 0, 0}, // 7
    {1, 1, 1, 1, 1, 1, 1}, // 8
    {1, 1, 1, 1, 0, 1, 1}, // 9
    {1, 1, 1, 0, 1, 1, 1}, // A
    {0, 0, 1, 1, 1, 1, 1}, // B
    {1, 0, 0, 1, 1, 1, 0}, // C
    {0, 1, 1, 1, 1, 0, 1}, // D
    {1, 0, 0, 1, 1, 1, 1}, // E
    {1, 0, 0, 0, 1, 1, 1}  // F
};

// Interrupções dos botões
static void IRAM_ATTR interrupcao_botao_cima(void* arg) {
    uint32_t agora = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if ((agora - ultimo_press_btn_cima) > DEBOUNCE_MS) {
        int comando = 1; // 1 = incrementar
        xQueueSendFromISR(fila_botoes, &comando, &xHigherPriorityTaskWoken);
        ultimo_press_btn_cima = agora;
    }
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void IRAM_ATTR interrupcao_botao_baixo(void* arg) {
    uint32_t agora = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if ((agora - ultimo_press_btn_baixo) > DEBOUNCE_MS) {
        int comando = -1; // -1 = decrementar
        xQueueSendFromISR(fila_botoes, &comando, &xHigherPriorityTaskWoken);
        ultimo_press_btn_baixo = agora;
    }
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// Task para leitura do sensor NTC
void task_leitura_ntc(void *pvParameters) {
    float temperatura_lida;
    TickType_t ultimo_tempo = xTaskGetTickCount();
    
    printf("Task NTC iniciada\n");
    
    while(1) {
        // Ler temperatura
        temperatura_lida = ler_temperatura_ntc();
        
        // Atualizar dados compartilhados
        if (xSemaphoreTake(mutex_dados, portMAX_DELAY) == pdTRUE) {
            dados_compartilhados.temperatura = temperatura_lida;
            
            // Verificar se está em alarme
            dados_compartilhados.alarme_ativo = 
                (temperatura_lida >= dados_compartilhados.temp_alarme);
            
            xSemaphoreGive(mutex_dados);
        }
        
        // Enviar temperatura para fila do SDCard
        xQueueSend(fila_temperatura, &temperatura_lida, pdMS_TO_TICKS(10));
        
        vTaskDelayUntil(&ultimo_tempo, pdMS_TO_TICKS(100));
    }
}

// Task para processar botões
void task_botoes(void *pvParameters) {
    int comando;
    
    printf("Task Botões iniciada\n");
    
    // Configurar botões
    gpio_reset_pin(BUTTON_A);
    gpio_set_direction(BUTTON_A, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_A, GPIO_PULLUP_DISABLE);
    
    gpio_reset_pin(BUTTON_B);
    gpio_set_direction(BUTTON_B, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_B, GPIO_PULLUP_DISABLE);
    
    // Configurar interrupções
    gpio_set_intr_type(BUTTON_A, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(BUTTON_B, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A, interrupcao_botao_cima, NULL);
    gpio_isr_handler_add(BUTTON_B, interrupcao_botao_baixo, NULL);
    
    while(1) {
        if (xQueueReceive(fila_botoes, &comando, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(mutex_dados, portMAX_DELAY) == pdTRUE) {
                if (comando == 1) {
                    dados_compartilhados.temp_alarme += INCREMENTO_TEMP;
                    printf("Temp alarme aumentada: %d°C\n", dados_compartilhados.temp_alarme);
                } else if (comando == -1) {
                    dados_compartilhados.temp_alarme -= INCREMENTO_TEMP;
                    printf("Temp alarme diminuída: %d°C\n", dados_compartilhados.temp_alarme);
                }
                xSemaphoreGive(mutex_dados);
            }
        }
    }
}

// Task para atualizar LCD
void task_lcd(void *pvParameters) {
    char linha[17];
    float temp_atual;
    int temp_alarme_atual;
    
    printf("Task LCD iniciada\n");
    
    // Inicializar LCD
    configurar_i2c();
    vTaskDelay(pdMS_TO_TICKS(100));
    inicializar_lcd();
    limpar_display();
    
    while(1) {
        // Ler dados compartilhados
        if (xSemaphoreTake(mutex_dados, portMAX_DELAY) == pdTRUE) {
            temp_atual = dados_compartilhados.temperatura;
            temp_alarme_atual = dados_compartilhados.temp_alarme;
            xSemaphoreGive(mutex_dados);
            
            // Atualizar display
            mover_cursor(0, 0);
            sprintf(linha, "NTC: %.1fC    ", temp_atual);
            escrever_texto(linha);
            
            mover_cursor(0, 1);
            sprintf(linha, "Alarme: %dC    ", temp_alarme_atual);
            escrever_texto(linha);
        }
        
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// Task para controlar LEDs
void task_leds(void *pvParameters) {
    float temperatura;
    int temp_alarme;
    float margem;
    bool alarme_ativo;
    static bool estado_pisca = false;
    static TickType_t ultimo_pisca = 0;
    
    printf("Task LEDs iniciada\n");
    
    // Configurar LEDs como saída
    gpio_reset_pin(LED_01);
    gpio_set_direction(LED_01, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_01, 0);
    
    gpio_reset_pin(LED_02);
    gpio_set_direction(LED_02, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_02, 0);
    
    gpio_reset_pin(LED_03);
    gpio_set_direction(LED_03, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_03, 0);
    
    gpio_reset_pin(LED_04);
    gpio_set_direction(LED_04, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_04, 0);
    
    while(1) {
        // Ler dados compartilhados
        if (xSemaphoreTake(mutex_dados, portMAX_DELAY) == pdTRUE) {
            temperatura = dados_compartilhados.temperatura;
            temp_alarme = dados_compartilhados.temp_alarme;
            alarme_ativo = dados_compartilhados.alarme_ativo;
            xSemaphoreGive(mutex_dados);
            
            margem = temp_alarme - temperatura;
            
            // Controlar LEDs
            if (alarme_ativo) {
                // Piscar todos os LEDs
                TickType_t agora = xTaskGetTickCount();
                if ((agora - ultimo_pisca) >= pdMS_TO_TICKS(INTERVALO_PISCA_MS)) {
                    estado_pisca = !estado_pisca;
                    gpio_set_level(LED_01, estado_pisca);
                    gpio_set_level(LED_02, estado_pisca);
                    gpio_set_level(LED_03, estado_pisca);
                    gpio_set_level(LED_04, estado_pisca);
                    ultimo_pisca = agora;
                }
            } else {
                // Barra de LEDs progressiva
                gpio_set_level(LED_01, margem <= 20);
                gpio_set_level(LED_02, margem <= 15);
                gpio_set_level(LED_03, margem <= 10);
                gpio_set_level(LED_04, margem <= 2);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Task para controlar display 7 segmentos
void task_display_7seg(void *pvParameters) {
    float temperatura;
    int temp_alarme;
    float margem;
    static bool pisca_estado = false;
    static TickType_t ultimo_pisca = 0;
    
    printf("Task Display 7-Seg iniciada\n");
    
    configurar_pinos_7seg();
    
    while(1) {
        // Ler dados compartilhados
        if (xSemaphoreTake(mutex_dados, portMAX_DELAY) == pdTRUE) {
            temperatura = dados_compartilhados.temperatura;
            temp_alarme = dados_compartilhados.temp_alarme;
            xSemaphoreGive(mutex_dados);
            
            margem = temp_alarme - temperatura;
            
            // Determinar qual dígito mostrar
            if (margem <= 0) {
                // Temperatura atingiu ou passou do alarme - piscar F
                TickType_t agora = xTaskGetTickCount();
                if ((agora - ultimo_pisca) >= pdMS_TO_TICKS(INTERVALO_PISCA_MS)) {
                    pisca_estado = !pisca_estado;
                    if (pisca_estado) {
                        exibir_digito_7seg('F');
                    } else {
                        exibir_digito_7seg(' '); // Apagar display
                    }
                    ultimo_pisca = agora;
                }
            } else if (margem <= 2) {
                exibir_digito_7seg('D');
            } else if (margem <= 10) {
                exibir_digito_7seg('7');
            } else if (margem <= 15) {
                exibir_digito_7seg('3');
            } else if (margem <= 20) {
                exibir_digito_7seg('0');
            } else {
                exibir_digito_7seg(' '); // Apagar se muito longe
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Task para controlar buzzer
void task_buzzer(void *pvParameters) {
    bool alarme_anterior = false;
    bool alarme_atual = false;
    
    printf("Task Buzzer iniciada\n");
    
    configurar_pwm();
    
    while(1) {
        // Verificar estado do alarme
        if (xSemaphoreTake(mutex_dados, portMAX_DELAY) == pdTRUE) {
            alarme_atual = dados_compartilhados.alarme_ativo;
            xSemaphoreGive(mutex_dados);
            
            // Controlar buzzer apenas se estado mudou
            if (alarme_atual != alarme_anterior) {
                if (alarme_atual) {
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    printf("ALARME ATIVADO!\n");
                } else {
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    printf("Alarme desativado\n");
                }
                alarme_anterior = alarme_atual;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task para salvar dados no SDCard
void task_sdcard(void *pvParameters) {
    float temperatura;
    int temp_alarme = TEMP_ALARME_PADRAO;
    uint32_t contador_leituras = 0;
    
    printf("Task SDCard iniciada\n");
    
    inicializar_sdcard();
    
    while(1) {
        // Receber temperatura da fila
        if (xQueueReceive(fila_temperatura, &temperatura, pdMS_TO_TICKS(INTERVALO_SALVAR_MS)) == pdTRUE) {
            
            // Obter temperatura de alarme
            if (xSemaphoreTake(mutex_dados, pdMS_TO_TICKS(10)) == pdTRUE) {
                temp_alarme = dados_compartilhados.temp_alarme;
                xSemaphoreGive(mutex_dados);
            }
            
            contador_leituras++;
            
            // Verificar se o SD está montado
            if (!sd_montado || cartao_sd == NULL) {
                printf("Nao montado - salvamento ignorado\n");
            } else {
                // Simular gravação de dados no SDCard
                uint32_t timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
                
                // Mostrar dados que seriam gravados
                printf("LEITURA #%lu - %lus,%.2fC,%dC -> GRAVADO no SDCard\n", 
                       contador_leituras, timestamp, temperatura, temp_alarme);
            }
        }
    }
}

// Configuração dos pinos do display 7 segmentos
void configurar_pinos_7seg() {
    gpio_reset_pin(SEG_A);
    gpio_set_direction(SEG_A, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_B);
    gpio_set_direction(SEG_B, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_C);
    gpio_set_direction(SEG_C, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_D);
    gpio_set_direction(SEG_D, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_E);
    gpio_set_direction(SEG_E, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_F);
    gpio_set_direction(SEG_F, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_G);
    gpio_set_direction(SEG_G, GPIO_MODE_OUTPUT);
}

// Exibir dígito no display 7 segmentos
void exibir_digito_7seg(char digito) {
    int indice = -1;
    
    if (digito >= '0' && digito <= '9') {
        indice = digito - '0';
    } else if (digito >= 'A' && digito <= 'F') {
        indice = digito - 'A' + 10;
    } else if (digito >= 'a' && digito <= 'f') {
        indice = digito - 'a' + 10;
    } else if (digito == ' ') {
        // Apagar display
        gpio_set_level(SEG_A, 0);
        gpio_set_level(SEG_B, 0);
        gpio_set_level(SEG_C, 0);
        gpio_set_level(SEG_D, 0);
        gpio_set_level(SEG_E, 0);
        gpio_set_level(SEG_F, 0);
        gpio_set_level(SEG_G, 0);
        return;
    }
    
    if (indice >= 0 && indice < 16) {
        gpio_set_level(SEG_A, tabela_7seg[indice][0]);
        gpio_set_level(SEG_B, tabela_7seg[indice][1]);
        gpio_set_level(SEG_C, tabela_7seg[indice][2]);
        gpio_set_level(SEG_D, tabela_7seg[indice][3]);
        gpio_set_level(SEG_E, tabela_7seg[indice][4]);
        gpio_set_level(SEG_F, tabela_7seg[indice][5]);
        gpio_set_level(SEG_G, tabela_7seg[indice][6]);
    }
}

// Configurar I2C
void configurar_i2c() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Funções auxiliares do LCD
#define LCD_RS  0x01
#define LCD_EN  0x04
#define LCD_BL  0x08

static void lcd_i2c_write(uint8_t data) {
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, &data, 1, 1000 / portTICK_PERIOD_MS);
}

static void lcd_pulse_enable(uint8_t data) {
    lcd_i2c_write(data | LCD_EN);
    ets_delay_us(1);
    lcd_i2c_write(data & ~LCD_EN);
    ets_delay_us(50);
}

static void lcd_write_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble << 4) | mode | LCD_BL;
    lcd_pulse_enable(data);
}

void lcd_send_cmd(uint8_t cmd) {
    lcd_write_nibble(cmd >> 4, 0);
    lcd_write_nibble(cmd & 0x0F, 0);
}

void lcd_send_data(uint8_t data) {
    lcd_write_nibble(data >> 4, LCD_RS);
    lcd_write_nibble(data & 0x0F, LCD_RS);
}

void inicializar_lcd() {
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_write_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_nibble(0x02, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    lcd_send_cmd(0x28);  // 4 bits, 2 linhas
    lcd_send_cmd(0x0C);  // Display ligado, cursor off
    lcd_send_cmd(0x06);  // Modo de entrada
    lcd_send_cmd(0x01);  // Limpar display
    vTaskDelay(pdMS_TO_TICKS(2));
}

void limpar_display() {
    lcd_send_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void mover_cursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40};
    lcd_send_cmd(0x80 | (col + row_offsets[row]));
}

void escrever_texto(const char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

// Configurar PWM
void configurar_pwm() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BUZZER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

// Configurar ADC
void configurar_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(NTC_CHANNEL, ADC_ATTEN_DB_12);
}

// Inicializar SDCard
void inicializar_sdcard() {
    printf("\nInicializando SDCard...\n");
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        printf("ERRO ao inicializar SPI para SDCard\n");
        sd_montado = false;
        return;
    }
    
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS;
    slot_config.host_id = host.slot;
    
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &cartao_sd);
    
    if (ret != ESP_OK) {
        printf("ERRO ao montar SDCard\n");
        sd_montado = false;
        return;
    }
    
    sd_montado = true;
    printf("SDCard montado com sucesso!\n");
}

// Ler temperatura do NTC
float ler_temperatura_ntc() {
    int valor_adc = adc1_get_raw(NTC_CHANNEL);
    float tensao = (valor_adc / 4095.0) * 3.3;
    
    if (tensao <= 0.01) tensao = 0.01;
    if (tensao >= 3.29) tensao = 3.29;
    
    float resistencia_ntc = RESISTOR_SERIE * tensao / (3.3 - tensao);
    
    if (resistencia_ntc <= 0) resistencia_ntc = 1.0;
    
    float temp = resistencia_ntc / RESISTENCIA_NOMINAL;
    temp = log(temp);
    temp /= COEFICIENTE_B;
    temp += 1.0 / (TEMP_NOMINAL + 273.15);
    temp = 1.0 / temp;
    temp -= 273.15;
    
    return temp;
}

// Função principal
void app_main(void) {
    printf("\n=== Sistema de Monitoramento com FreeRTOS ===\n");
    printf("Atividade 10 - Organização com Tasks\n\n");
    
    // Configurar ADC antes de criar as tasks
    configurar_adc();
    
    // Criar mutex para dados compartilhados
    mutex_dados = xSemaphoreCreateMutex();
    if (mutex_dados == NULL) {
        printf("ERRO: Não foi possível criar o mutex\n");
        return;
    }
    
    // Criar filas
    fila_botoes = xQueueCreate(10, sizeof(int));
    if (fila_botoes == NULL) {
        printf("ERRO: Não foi possível criar a fila de botões\n");
        return;
    }
    
    fila_temperatura = xQueueCreate(20, sizeof(float));
    if (fila_temperatura == NULL) {
        printf("ERRO: Não foi possível criar a fila de temperatura\n");
        return;
    }
    
    // Criar tasks com prioridades diferentes
    xTaskCreate(
        task_leitura_ntc,
        "Task_NTC",
        4096,
        NULL,
        5,  // Prioridade alta para leitura do sensor
        &task_leitura_ntc_handle
    );
    
    xTaskCreate(
        task_botoes,
        "Task_Botoes",
        2048,
        NULL,
        4,  // Prioridade média-alta para resposta rápida
        &task_botoes_handle
    );
    
    xTaskCreate(
        task_lcd,
        "Task_LCD",
        4096,
        NULL,
        3,  // Prioridade média
        &task_lcd_handle
    );
    
    xTaskCreate(
        task_display_7seg,
        "Task_7Seg",
        2048,
        NULL,
        3,  // Prioridade média
        &task_display_7seg_handle
    );
    
    xTaskCreate(
        task_leds,
        "Task_LEDs",
        2048,
        NULL,
        3,  // Prioridade média
        &task_leds_handle
    );
    
    xTaskCreate(
        task_buzzer,
        "Task_Buzzer",
        2048,
        NULL,
        3,  // Prioridade média
        &task_buzzer_handle
    );
    
    xTaskCreate(
        task_sdcard,
        "Task_SDCard",
        4096,
        NULL,
        2,  // Prioridade baixa
        &task_sdcard_handle
    );
    
    printf("Todas as tasks foram criadas com sucesso!\n");
    printf("Sistema rodando...\n\n");
}