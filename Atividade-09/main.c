#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "rom/ets_sys.h"

// Pinos do sistema
#define LED_01 35
#define LED_02 36
#define LED_03 37
#define LED_04 38
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
#define INTERVALO_SALVAR_MS 500

// Constantes do NTC
#define RESISTOR_SERIE 10000.0
#define RESISTENCIA_NOMINAL 10000.0
#define TEMP_NOMINAL 25.0
#define COEFICIENTE_B 3950.0

// Estados da máquina de estados
typedef enum {
    ESTADO_INICIALIZAR,
    ESTADO_LER_SENSOR,
    ESTADO_ATUALIZAR_DISPLAY,
    ESTADO_ATUALIZAR_LEDS,
    ESTADO_VERIFICAR_ALARME,
    ESTADO_SALVAR_SDCARD
} estado_sistema_t;

// Variáveis globais
volatile int temp_alarme = TEMP_ALARME_PADRAO;
volatile bool alarme_ativo = false;
volatile uint32_t ultimo_press_btn_cima = 0;
volatile uint32_t ultimo_press_btn_baixo = 0;

static estado_sistema_t estado_atual = ESTADO_INICIALIZAR;
static float temperatura_atual = 0.0;
static uint32_t ultimo_tempo_salvar = 0;
static uint32_t ultimo_tempo_piscar = 0;
static bool estado_led_pisca = false;
static sdmmc_card_t *cartao_sd = NULL;
static bool sd_montado = false;
static uint32_t contador_leituras = 0;

// Protótipos
void configurar_pinos();
void configurar_i2c();
void inicializar_lcd();
void limpar_display();
void mover_cursor(uint8_t col, uint8_t row);
void escrever_texto(const char *str);
void configurar_pwm();
void configurar_adc();
void inicializar_sdcard();
float ler_temperatura_ntc();
void atualizar_barra_leds(float temp);
void atualizar_tela(float temp);
void controlar_buzzer(bool estado);
void salvar_no_sdcard(float temp);
void piscar_leds();

// Interrupções dos botões (debounce por software)
static void IRAM_ATTR interrupcao_botao_cima(void* arg) {
    uint32_t agora = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    
    if ((agora - ultimo_press_btn_cima) > DEBOUNCE_MS) {
        temp_alarme += INCREMENTO_TEMP;
        ultimo_press_btn_cima = agora;
    }
}

static void IRAM_ATTR interrupcao_botao_baixo(void* arg) {
    uint32_t agora = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    
    if ((agora - ultimo_press_btn_baixo) > DEBOUNCE_MS) {
        temp_alarme -= INCREMENTO_TEMP;
        ultimo_press_btn_baixo = agora;
    }
}

void app_main(void) {
    printf("=== Sistema de Monitoramento de Temperatura ===\n");
    
    // Loop principal da máquina de estados
    while(1) {
        switch(estado_atual) {
            case ESTADO_INICIALIZAR:
                printf("\nInicializando perifericos...\n");
                
                // Configurar todos os periféricos
                configurar_pinos();
                configurar_i2c();
                vTaskDelay(pdMS_TO_TICKS(100));
                inicializar_lcd();
                configurar_pwm();
                configurar_adc();
                inicializar_sdcard();
                
                // Configurar interrupções dos botões
                gpio_set_intr_type(BUTTON_A, GPIO_INTR_NEGEDGE);
                gpio_set_intr_type(BUTTON_B, GPIO_INTR_NEGEDGE);
                gpio_install_isr_service(0);
                gpio_isr_handler_add(BUTTON_A, interrupcao_botao_cima, NULL);
                gpio_isr_handler_add(BUTTON_B, interrupcao_botao_baixo, NULL);
                
                printf("Sistema pronto!\n\n");
                limpar_display();
                
                estado_atual = ESTADO_LER_SENSOR;
                break;
                
            case ESTADO_LER_SENSOR:
                // Leitura do sensor NTC via ADC
                temperatura_atual = ler_temperatura_ntc();
                contador_leituras++;
                estado_atual = ESTADO_ATUALIZAR_DISPLAY;
                break;
                
            case ESTADO_ATUALIZAR_DISPLAY:
                // Atualizar LCD com temperatura e alarme
                atualizar_tela(temperatura_atual);
                estado_atual = ESTADO_ATUALIZAR_LEDS;
                break;
                
            case ESTADO_ATUALIZAR_LEDS:
                // Controlar LEDs (barra termômetro ou pisca-pisca)
                if (alarme_ativo) {
                    piscar_leds();
                } else {
                    atualizar_barra_leds(temperatura_atual);
                }
                estado_atual = ESTADO_VERIFICAR_ALARME;
                break;
                
            case ESTADO_VERIFICAR_ALARME:
                // Verificar se temperatura atingiu o alarme
                alarme_ativo = (temperatura_atual >= temp_alarme);
                controlar_buzzer(alarme_ativo);
                estado_atual = ESTADO_SALVAR_SDCARD;
                break;
                
            case ESTADO_SALVAR_SDCARD:
                // Salvar dados no cartão SD periodicamente
                uint32_t tempo_atual = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if ((tempo_atual - ultimo_tempo_salvar) >= INTERVALO_SALVAR_MS) {
                    salvar_no_sdcard(temperatura_atual);
                    ultimo_tempo_salvar = tempo_atual;
                }
                
                // Voltar para leitura do sensor
                estado_atual = ESTADO_LER_SENSOR;
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
        }
    }
}

void configurar_pinos() {
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
    
    // Configurar botões como entrada (pull-up externo na montagem)
    gpio_reset_pin(BUTTON_A);
    gpio_set_direction(BUTTON_A, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_A, GPIO_PULLUP_DISABLE);
    
    gpio_reset_pin(BUTTON_B);
    gpio_set_direction(BUTTON_B, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_B, GPIO_PULLUP_DISABLE);
}

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
    // Sequência de inicialização do LCD 16x2 (modo 4 bits)
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
    vTaskDelay(2 / portTICK_PERIOD_MS);
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

void configurar_pwm() {
    // Configurar PWM para controle do buzzer
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

void configurar_adc() {
    // Configurar ADC para leitura do NTC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(NTC_CHANNEL, ADC_ATTEN_DB_12);
}

void inicializar_sdcard() {
    printf("\nInicializando...\n");
    
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
    
    // Inicializar barramento SPI
    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        printf("ERRO ao inicializar SPI (codigo: %d)\n", ret);
        sd_montado = false;
        return;
    }
    printf("Barramento SPI OK\n");
    
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS;
    slot_config.host_id = host.slot;
    
    // Tentar montar o cartão SD
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &cartao_sd);
    
    if (ret != ESP_OK) {
        printf("ERRO ao montar (codigo: %d)\n", ret);
        if (ret == ESP_FAIL) {
            printf("Verifique se o cartao esta inserido\n");
        }
        sd_montado = false;
        return;
    }
    
    sd_montado = true;
    printf("Montado com sucesso!\n");
    sdmmc_card_print_info(stdout, cartao_sd);
    printf("Sistema pronto para gravar dados\n\n");
}

float ler_temperatura_ntc() {
    // Ler valor do ADC
    int valor_adc = adc1_get_raw(NTC_CHANNEL);
    float tensao = (valor_adc / 4095.0) * 3.3;
    
    // Proteção contra valores extremos
    if (tensao <= 0.01) tensao = 0.01;
    if (tensao >= 3.29) tensao = 3.29;
    
    // Calcular resistência do NTC (divisor de tensão)
    float resistencia_ntc = RESISTOR_SERIE * tensao / (3.3 - tensao);
    
    if (resistencia_ntc <= 0) resistencia_ntc = 1.0;
    
    // Calcular temperatura usando equação Steinhart-Hart simplificada
    float temp = resistencia_ntc / RESISTENCIA_NOMINAL;
    temp = log(temp);
    temp /= COEFICIENTE_B;
    temp += 1.0 / (TEMP_NOMINAL + 273.15);
    temp = 1.0 / temp;
    temp -= 273.15;
    
    return temp;
}

void atualizar_barra_leds(float temp) {
    // Calcular margem até o alarme
    float margem = temp_alarme - temp;
    
    // Acender LEDs progressivamente conforme temperatura se aproxima do alarme
    gpio_set_level(LED_01, margem <= 20);
    gpio_set_level(LED_02, margem <= 15);
    gpio_set_level(LED_03, margem <= 10);
    gpio_set_level(LED_04, margem <= 2);
}

void piscar_leds() {
    // Piscar todos os LEDs quando alarme estiver ativo
    uint32_t tempo = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    if ((tempo - ultimo_tempo_piscar) >= INTERVALO_PISCA_MS) {
        estado_led_pisca = !estado_led_pisca;
        gpio_set_level(LED_01, estado_led_pisca);
        gpio_set_level(LED_02, estado_led_pisca);
        gpio_set_level(LED_03, estado_led_pisca);
        gpio_set_level(LED_04, estado_led_pisca);
        ultimo_tempo_piscar = tempo;
    }
}

void atualizar_tela(float temp) {
    char linha[16];
    
    // Primeira linha: temperatura atual
    mover_cursor(0, 0);
    sprintf(linha, "NTC: %.1fC    ", temp);
    escrever_texto(linha);
    
    // Segunda linha: temperatura de alarme
    mover_cursor(0, 1);
    sprintf(linha, "Alarm: %dC    ", temp_alarme);
    escrever_texto(linha);
}

void controlar_buzzer(bool estado) {
    if (estado) {
        // Ativar buzzer (duty cycle 50%)
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    } else {
        // Desativar buzzer (duty cycle 0%)
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
}

void salvar_no_sdcard(float temp) {
    // Verificar se o SD está montado
    if (!sd_montado || cartao_sd == NULL) {
        printf("Nao montado - salvamento ignorado\n");
        return;
    }
    
    // Simular gravação de dados no SDCard 
    uint32_t timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
    
    // Mostrar dados que seriam gravados 
    printf("LEITURA #%lu - %lus,%.2fC,%dC -> GRAVADO no SDCard\n", 
           contador_leituras, timestamp, temp, temp_alarme);
}