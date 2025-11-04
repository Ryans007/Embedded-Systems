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

// Definições de Pinos
#define LED_01 35
#define LED_02 36
#define LED_03 37
#define LED_04 38
#define BUTTON_A 13
#define BUTTON_B 14
#define BUZZER 42
#define NTC_CHANNEL ADC1_CHANNEL_4

// Configurações I2C para LCD
#define I2C_MASTER_SCL_IO 15
#define I2C_MASTER_SDA_IO 16
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define LCD_ADDR 0x27  // Mudado para 0x27 (padrão do Wokwi)

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

// Parâmetros do NTC
#define SERIES_RESISTOR 10000.0
#define NOMINAL_RESISTANCE 10000.0
#define NOMINAL_TEMPERATURE 25.0
#define B_COEFFICIENT 3950.0

// Variáveis Globais
volatile int threshold_temp = TEMP_ALARM_DEFAULT;
volatile bool alert_on = false;
volatile uint32_t last_press_btn_up = 0;
volatile uint32_t last_press_btn_down = 0;

// Protótipos
void setup_all_pins();
void setup_i2c_comm();
void boot_lcd();
void clear_display();
void move_cursor(uint8_t col, uint8_t row);
void write_text(const char *str);
void init_sound_driver();
void init_temp_sensor();
float get_ntc_reading();
void set_led_bar(float temp);
void refresh_display(float temp);
void toggle_alarm_sound(bool state);

// ISR (Rotina de Interrupção) do Botão A (Incrementa alarme)
static void IRAM_ATTR isr_button_up(void* arg) {
    uint32_t now = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    
    if ((now - last_press_btn_up) > DEBOUNCE_MS) {
        threshold_temp += TEMP_INCREMENT;
        last_press_btn_up = now;
    }
}

// ISR (Rotina de Interrupção) do Botão B (Decrementa alarme)
static void IRAM_ATTR isr_button_down(void* arg) {
    uint32_t now = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    
    if ((now - last_press_btn_down) > DEBOUNCE_MS) {
        threshold_temp -= TEMP_INCREMENT;
        last_press_btn_down = now;
    }
}

// Função principal (entry point) da aplicação
void app_main(void) {
    printf("Iniciando sistema...\n");
    
    // Bloco de inicialização dos periféricos (GPIO, I2C, LCD, PWM, ADC)
    setup_all_pins();
    setup_i2c_comm();
    vTaskDelay(pdMS_TO_TICKS(100));
    boot_lcd();
    init_sound_driver();
    init_temp_sensor();
    
    // Configuração das interrupções dos botões
    gpio_set_intr_type(BUTTON_A, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type(BUTTON_B, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A, isr_button_up, NULL);
    gpio_isr_handler_add(BUTTON_B, isr_button_down, NULL);
    
    printf("Sistema pronto!\n");
    clear_display();
    
    uint32_t last_toggle = 0;
    bool led_blink_state = false;
    
    // Loop principal da aplicação
    while(1) {
        // Leitura da temperatura e verificação do alarme
        float current_temp = get_ntc_reading();
        
        alert_on = (current_temp >= threshold_temp);
        toggle_alarm_sound(alert_on);
        refresh_display(current_temp);
        
        uint32_t tick_now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Lógica de piscar LEDs se o alarme estiver ativo
        if (alert_on) {
            if ((tick_now - last_toggle) >= BLINK_INTERVAL_MS) {
                led_blink_state = !led_blink_state;
                gpio_set_level(LED_01, led_blink_state);
                gpio_set_level(LED_02, led_blink_state);
                gpio_set_level(LED_03, led_blink_state);
                gpio_set_level(LED_04, led_blink_state);
                last_toggle = tick_now;
            }
        } else {
            // Lógica de termômetro (barra de LEDs) se o alarme estiver inativo
            set_led_bar(current_temp);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Configura os pinos de GPIO (LEDs como saída, Botões como entrada)
void setup_all_pins() {
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

// Configura o periférico LEDC (PWM) para controlar o buzzer
void init_sound_driver() {
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

// Atualiza a barra de LEDs (termômetro) com base na proximidade do alarme
void set_led_bar(float temp) {
    float margin = threshold_temp - temp;
    
    gpio_set_level(LED_01, margin <= 20);
    gpio_set_level(LED_02, margin <= 15);
    gpio_set_level(LED_03, margin <= 10);
    gpio_set_level(LED_04, margin <= 2);
}

// Atualiza as informações (Temp. Atual e Temp. Alarme) no display LCD
void refresh_display(float temp) {
    char line1[16];
    
    move_cursor(0, 0);
    sprintf(line1, "NTC: %.1fC    ", temp);
    write_text(line1);
    
    move_cursor(0, 1);
    sprintf(line1, "Alarm: %dC    ", threshold_temp);
    write_text(line1);
}

// Ativa ou desativa o buzzer (PWM)
void toggle_alarm_sound(bool state) {
    if (state) {
        // Ativa o som definindo o duty cycle para 50% (LEDC_DUTY = 512)
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    } else {
        // Desativa o som definindo o duty cycle para 0%
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
}