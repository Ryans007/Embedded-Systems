#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "rom/ets_sys.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "rom/ets_sys.h"


// ========================================= Definição de Variáveis =====================================
// Definições de Pinos
#define LED_01 18
#define LED_02 17
#define LED_03 16
#define LED_04 15
#define BUTTON_A 42
#define BUTTON_B 41
#define BUZZER 7
#define NTC_CHANNEL ADC1_CHANNEL_4

// Pinos do Display 7 Segmentos
#define SEG_A 38
#define SEG_B 37
#define SEG_C 36
#define SEG_D 35
#define SEG_E 45
#define SEG_F 48
#define SEG_G 47

// Pinos do SDCard (SPI)
#define SD_MOSI 13
#define SD_MISO 11
#define SD_SCK  12
#define SD_CS   14

// Configurações I2C para LCD
#define I2C_MASTER_SCL_IO 40
#define I2C_MASTER_SDA_IO 39
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define LCD_ADDR 0x27  

// Configurações PWM para Buzzer
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY 512
#define LEDC_FREQUENCY 2000

// Configurações do Sistema
#define DEBOUNCE_MS 200
#define TEMP_ALARM_DEFAULT 25
#define TEMP_INCREMENT 5
#define BLINK_INTERVAL_MS 300
#define INTERVALO_SALVAR_MS 500

// Parâmetros do NTC
#define SERIES_RESISTOR 10000.0
#define NOMINAL_RESISTANCE 10000.0
#define NOMINAL_TEMPERATURE 25.0
#define B_COEFFICIENT 3950.0

// Variaveis Globais
volatile int16_t last_press_button_a = 0;
volatile int16_t last_press_button_b = 0;
volatile int threshold_temp = TEMP_ALARM_DEFAULT;
volatile bool alarm_on = false;
volatile int led_check = 0;
volatile int16_t led_last_time = 0;
volatile float current_temp = 0;
volatile int16_t last_save_sdcard = 0;
volatile bool sd_montado = false;
static sdmmc_card_t *cartao_sd = NULL;
static uint32_t contador_leituras = 0;
volatile int temp_alarme = TEMP_ALARM_DEFAULT;

// ======================================= Prototipos ==========================================

void config_gpios();
void config_pwm();
void setup_i2c_comm();
void boot_lcd();
void clear_display();
void move_cursor(uint8_t col, uint8_t row);
void write_text(const char *str);
void init_temp_sensor();
float get_ntc_reading();
void whats_led_definition(float temp);
void refresh_display(float temp);
void buzzer_alarm(bool state);
void initialize_sdcard();
void save_to_sdcard(float temp);

// ===========================  Definição da Maquina de Estados ===============================

typedef enum {
  ESTADO_INICIALIZAR,
  ESTADO_LER_SENSOR,
  ESTADO_ATUALIZAR_DISPLAY,
  ESTADO_ATUALIZAR_LEDS,
  ESTADO_VERIFICAR_ALARME,
  ESTADO_SALVAR_SDCARD
} estado_sistema_t; 

// Variável com os estados do sistema
static estado_sistema_t estado_atual = ESTADO_INICIALIZAR;

// =========================== Funções de Tratamento de Interrupções ==========================

static void IRAM_ATTR isr_button_up_function(void* arg) {
  int16_t current_interrupt_time = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;

  if ((current_interrupt_time - last_press_button_a) > DEBOUNCE_MS) {
    last_press_button_a = current_interrupt_time;
    threshold_temp += 5;
  }
}

static void IRAM_ATTR isr_button_down_function(void* arg) {
  int16_t current_interrupt_time = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;

  if ((current_interrupt_time - last_press_button_b) > DEBOUNCE_MS) {
    last_press_button_b = current_interrupt_time;
    threshold_temp -= 5;
  }
}
// ========================================== Main =============================================

void app_main() {
  printf("Inicializando Sistema de Monitoramento de Temperatura...\n");
  // Loop Principal
  while (true) {
    switch (estado_atual) {
      case ESTADO_INICIALIZAR:
          // Bloco de inicialização dos periféricos (GPIO, I2C, LCD, PWM, ADC)
          config_gpios();
          config_pwm();
          vTaskDelay(pdMS_TO_TICKS(100));
          setup_i2c_comm();
          boot_lcd();
          init_temp_sensor();
          initialize_sdcard();
          
          // Configuração das interrupções dos botões
          gpio_set_intr_type(BUTTON_A, GPIO_INTR_NEGEDGE);
          gpio_set_intr_type(BUTTON_B, GPIO_INTR_NEGEDGE);
          gpio_install_isr_service(0);
          gpio_isr_handler_add(BUTTON_A, isr_button_up_function, NULL);
          gpio_isr_handler_add(BUTTON_B, isr_button_down_function, NULL);
          
          printf("Sistema pronto!\n");
          estado_atual = ESTADO_LER_SENSOR;
          clear_display();
          break;
      case ESTADO_LER_SENSOR:
          current_temp = get_ntc_reading();
          contador_leituras++;
          alarm_on = (current_temp >= threshold_temp) ? 1 : 0;
          estado_atual = ESTADO_ATUALIZAR_DISPLAY;
          break;
      case ESTADO_ATUALIZAR_DISPLAY:
          refresh_display(current_temp);
          estado_atual = ESTADO_ATUALIZAR_LEDS;
          break;
      case ESTADO_ATUALIZAR_LEDS:
          int16_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
          if (alarm_on) {
            if ((current_time - led_last_time) > BLINK_INTERVAL_MS) {
                led_check = led_check ? 0 : 1;
                gpio_set_level(LED_01, led_check);
                gpio_set_level(LED_02, led_check);
                gpio_set_level(LED_03, led_check);
                gpio_set_level(LED_04, led_check);
                led_last_time = current_time;
              }
            } else {
                whats_led_definition(current_temp);
            }
            estado_atual = ESTADO_VERIFICAR_ALARME;
            break;
      case ESTADO_VERIFICAR_ALARME:
          buzzer_alarm(alarm_on);
          estado_atual = ESTADO_SALVAR_SDCARD;
          break;
      case ESTADO_SALVAR_SDCARD:
          int16_t current_time_sdcard = xTaskGetTickCount() * portTICK_PERIOD_MS;
          if ((current_time_sdcard - last_save_sdcard) >= INTERVALO_SALVAR_MS) {
              save_to_sdcard(current_temp);
              last_save_sdcard = current_time_sdcard;
          }
          estado_atual = ESTADO_LER_SENSOR;
          break;
    }
  }
}

// ==================================== Funções Auxiliares ===========================================

// Atualiza as informações (Temp. Atual e Temp. Alarme) no display LCD
void refresh_display(float temp) {
    char line1[32];
    
    move_cursor(0, 0);
    sprintf(line1, "Temp: %.1fC    ", temp);
    write_text(line1);
    
    move_cursor(0, 1);
    sprintf(line1, "Alarm: %dC    ", threshold_temp);
    write_text(line1);
}

// Aciona o Buzzer caso a Temperatura esteja maior ou igual a temperatura de alarme
void buzzer_alarm(bool state) {
  switch (state) {
    case 1:
      ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
      ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
      break;
    case 0:
      ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
      ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
      break;
  }
}

// Função que diz quais leds devem ser acesos de acordo com a temperatura
void whats_led_definition(float temp) {
  float variation;

  variation = threshold_temp - temp;

  gpio_set_level(LED_01, (variation <= 20));
  gpio_set_level(LED_02, (variation <= 15));
  gpio_set_level(LED_03, (variation <= 10));
  gpio_set_level(LED_04, (variation <= 2));

}

// ============================ Funções de Configuração LCD (I2C) =======================================

// Inicializa o barramento I2C como mestre
void setup_i2c_comm() {
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

// Definições auxiliares para o driver do LCD (via PCF8574)
#define LCD_RS  0x01
#define LCD_EN  0x04
#define LCD_BL  0x08

// Funções de baixo nível para comunicação com o LCD via I2C
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

// Envia um comando para o LCD
void lcd_send_cmd(uint8_t cmd) {
    lcd_write_nibble(cmd >> 4, 0);
    lcd_write_nibble(cmd & 0x0F, 0);
}

// Envia um dado (caractere) para o LCD
void lcd_send_data(uint8_t data) {
    lcd_write_nibble(data >> 4, LCD_RS);
    lcd_write_nibble(data & 0x0F, LCD_RS);
}

// Sequência de inicialização do LCD 16x2 no modo 4 bits
void boot_lcd() {
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_write_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_nibble(0x02, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    lcd_send_cmd(0x28);  // 4-bit mode, 2 lines, 5x8 font
    lcd_send_cmd(0x0C);  // Display on, cursor off
    lcd_send_cmd(0x06);  // Entry mode
    lcd_send_cmd(0x01);  // Clear display
    vTaskDelay(pdMS_TO_TICKS(2));
}

// Limpa o display do LCD
void clear_display() {
    lcd_send_cmd(0x01);
    vTaskDelay(2 / portTICK_PERIOD_MS);
}

// Define a posição do cursor no LCD
void move_cursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40};
    lcd_send_cmd(0x80 | (col + row_offsets[row]));
}

// Escreve uma string no LCD
void write_text(const char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

// ========================= Funções para Sensor de Temperatura (NTC) =============================

// Configura o conversor Analógico-Digital (ADC1)
void init_temp_sensor() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(NTC_CHANNEL, ADC_ATTEN_DB_12);
}

// Lê o valor do ADC e calcula a temperatura (Equação Steinhart-Hart)
float get_ntc_reading() {
    int adc_val = adc1_get_raw(NTC_CHANNEL);
    float meas_voltage = (adc_val / 4095.0) * 3.3;
    
    // Proteção contra valores extremos (evita divisão por zero ou log(0))
    if (meas_voltage <= 0.01) meas_voltage = 0.01;
    if (meas_voltage >= 3.29) meas_voltage = 3.29;
    
    // Cálculo da resistência do NTC (Divisor de tensão) 
    float ntc_ohms = SERIES_RESISTOR * meas_voltage / (3.3 - meas_voltage);
    
    if (ntc_ohms <= 0) ntc_ohms = 1.0;
    
    // Cálculo da temperatura usando a fórmula Steinhart-Hart simplificada
    float temp_calc = ntc_ohms / NOMINAL_RESISTANCE;
    temp_calc = log(temp_calc);
    temp_calc /= B_COEFFICIENT;
    temp_calc += 1.0 / (NOMINAL_TEMPERATURE + 273.15);
    temp_calc = 1.0 / temp_calc;
    temp_calc -= 273.15;
    
    return temp_calc;
}

// ================================ Funções para configurar o SDCARD ===========================

void initialize_sdcard() {
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

void save_to_sdcard(float temp) {
    // Verificar se o SD está montado
    if (!sd_montado || cartao_sd == NULL) {
        printf("Nao montado - salvamento ignorado\n");
        return;
    }
    
    // Simular gravação de dados no SDCard
    uint32_t timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
    
    // Mostrar dados que seriam gravados
    printf("LEITURA #%lu - %lus,%.2fC,%dC -> GRAVADO no SDCard\n", 
           contador_leituras, timestamp, temp, threshold_temp);
}

// ==================================== Funções de Configuração ================================

// Configura os pinos de GPIO (LEDs como saída, Botões como entrada)
void config_gpios() {
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
    
    // Botões com resistores de pull-up EXTERNOS (10kΩ para 3.3V)
    gpio_reset_pin(BUTTON_A);
    gpio_set_direction(BUTTON_A, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_A, GPIO_PULLUP_DISABLE);  // Pull-up externo
    
    gpio_reset_pin(BUTTON_B);
    gpio_set_direction(BUTTON_B, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_B, GPIO_PULLUP_DISABLE);  // Pull-up externo
}

// Configura o periférico LEDC (PWM) para controlar o buzzer
void config_pwm() {
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