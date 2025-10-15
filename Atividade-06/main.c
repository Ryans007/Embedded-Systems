#include <stdio.h>
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BUTTON_A 16
#define BUTTON_B 15
#define BUZZER 42
#define LED_01 48
#define LED_02 47

// Definições PWM
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_BUZZER LEDC_TIMER_0
#define LEDC_CHANNEL_BUZZER LEDC_CHANNEL_0

// Variáveis globais
int led1_state = 0;
int led2_state = 0;
volatile bool button_b_enabled = true;

// Controle do buzzer
volatile bool buzzer_active = false;
volatile uint32_t buzzer_start_time = 0;

// Protótipos
void my_gpio_config();
void config_pwm(int gpio_num, ledc_timer_t timer_num, ledc_channel_t channel, int duty);
static void IRAM_ATTR botao_a_pressionado(void* arg);
static void IRAM_ATTR botao_b_pressionado(void* arg);
static bool IRAM_ATTR pisca_led2(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data);
void problem_01();
void problem_02();
void problem_03();
void problem_04();
void check_buzzer();
void check_uart();

void app_main() {
    // Configurações iniciais
    my_gpio_config();
    
    // i. Botão A (GPIO com borda de descida): alterna o estado do LED1.
    problem_01();
    
    // ii. Botão B (GPIO com borda de descida): liga o buzzer por 1500 ms.
    problem_02();
    
    // iii. Timer (a cada 2 segundos): alterna o estado do LED2, simulando um piscar automático.
    problem_03();
    
    // iv. UART: ao digitar o caractere “a” no terminal desative a função do botão B e ao digitar “b” ative a função do Botão B.
    problem_04();
    
    // Loop principal - monitora buzzer e UART
    while(1) {
        check_buzzer();
        check_uart();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void config_pwm(int gpio_num, ledc_timer_t timer_num, ledc_channel_t channel, int duty) {
    // Configurar timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = timer_num,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    
    // Configurar canal
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = channel,
        .timer_sel = timer_num,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = gpio_num,
        .duty = duty,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

void my_gpio_config() {
    // Configuração dos botões
    gpio_reset_pin(BUTTON_A);
    gpio_set_direction(BUTTON_A, GPIO_MODE_INPUT);
    
    gpio_reset_pin(BUTTON_B);
    gpio_set_direction(BUTTON_B, GPIO_MODE_INPUT);
    
    // Configuração dos LEDs
    gpio_reset_pin(LED_01);
    gpio_set_direction(LED_01, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_01, 0);
    
    gpio_reset_pin(LED_02);
    gpio_set_direction(LED_02, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_02, 0);
    
    // Configuração do Buzzer com PWM
    config_pwm(BUZZER, LEDC_TIMER_BUZZER, LEDC_CHANNEL_BUZZER, 0);
}

void problem_01() {
    // i. Botão A (GPIO com borda de descida): alterna o estado do LED1.
    gpio_set_intr_type(BUTTON_A, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_A, botao_a_pressionado, NULL);
}

static void IRAM_ATTR botao_a_pressionado(void* arg) {
    static uint32_t last_interrupt_time = 0;
    uint32_t interrupt_time = xTaskGetTickCountFromISR();
    
    if (interrupt_time - last_interrupt_time > pdMS_TO_TICKS(200)) {
        led1_state = led1_state ? 0 : 1;
        gpio_set_level(LED_01, led1_state);
    }
    
    last_interrupt_time = interrupt_time;
}

void problem_02() {
    // ii. Botão B (GPIO com borda de descida): liga o buzzer por 1500 ms.
    gptimer_handle_t timer_handle;
    
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    gptimer_new_timer(&timer_config, &timer_handle);
    
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 2000000,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(timer_handle, &alarm_config);
    
    gptimer_event_callbacks_t cbs = {
        .on_alarm = pisca_led2,
    };
    gptimer_register_event_callbacks(timer_handle, &cbs, NULL);
    
    gptimer_enable(timer_handle);
    gptimer_start(timer_handle);
}

static bool IRAM_ATTR pisca_led2(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    led2_state = led2_state ? 0 : 1;
    gpio_set_level(LED_02, led2_state);
    return false;
}

void problem_03() {
    // iii. Timer (a cada 2 segundos): alterna o estado do LED2, simulando um piscar automático.
    gpio_set_intr_type(BUTTON_B, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(BUTTON_B, botao_b_pressionado, NULL);
}

static void IRAM_ATTR botao_b_pressionado(void* arg) {
    if (!button_b_enabled) return;
    
    static uint32_t last_interrupt_time = 0;
    uint32_t interrupt_time = xTaskGetTickCountFromISR();
    
    if (interrupt_time - last_interrupt_time > pdMS_TO_TICKS(200)) {
        // Liga o buzzer com frequência de 2000Hz e duty cycle de 50%
        ledc_set_freq(LEDC_MODE, LEDC_TIMER_BUZZER, 2000);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER, 512);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER);
        
        buzzer_active = true;
        buzzer_start_time = interrupt_time;
    }
    
    last_interrupt_time = interrupt_time;
}

void check_buzzer() {
    if (buzzer_active) {
        uint32_t current_time = xTaskGetTickCount();
        
        if ((current_time - buzzer_start_time) >= pdMS_TO_TICKS(1500)) {
            // Desliga o buzzer
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER);
            
            buzzer_active = false;
        }
    }
}

void problem_04() {
    // iv. UART: ao digitar o caractere “a” no terminal desative a função do botão B e ao digitar “b” ative a função do Botão B.
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    
    printf("UART pronto! Digite 'a' para desabilitar ou 'b' para habilitar o Botao B\n");
}

void check_uart() {
    uint8_t data;
    int len = uart_read_bytes(UART_NUM_0, &data, 1, 0);
    
    if(len > 0) {
        if(data == 'a' || data == 'A') {
            button_b_enabled = false;
            printf("Botao B DESABILITADO\n");
        }
        else if(data == 'b' || data == 'B') {
            button_b_enabled = true;
            printf("Botao B HABILITADO\n");
        }
    }
}