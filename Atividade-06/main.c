#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "driver/ledc.h"

// =================== Definição das Variáveis ===========================
#define BUZZER 15

#define LED_01 17
#define LED_02 16

#define BUTTON_01 42
#define BUTTON_02 41

#define DEBOUNCE_MS 200

int led_01_state = 0;
int led_02_state = 0; 

// Varíaveis PWM para buzzer

#define LEDC_FREQUENCY 1000 // Frequência = 1000 HZ ou 1KHZ
#define LEDC_RESOLUTION LEDC_TIMER_8_BIT // 8 bits = 0-255 níveis de PWM
#define LEDC_TIMER_BUZZER LEDC_TIMER_1 // Timer Para o Buzzer
#define LEDC_MODE LEDC_LOW_SPEED_MODE // Modo de Velocidade

#define LEDC_CHANNEL_BUZZER LEDC_CHANNEL_0 // Canal 0 - Buzzer

volatile bool buzzer_active = false;
volatile uint32_t last_buzzer_active_time = 0;

volatile bool botao_b_desativado = false;


// =========================== Prototipos ===============================

void task_01();
void task_02();
void task_03();
void task_04();

void config_gpios();
void config_pwm(int custom_pin, int custom_timer, int custom_channel, int duty);

void botao_a_pressionado_function(void *arg);
void botao_b_pressionado_function(void *arg);
static bool IRAM_ATTR pisca_led2_function(
  gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data);

void check_buzzer();
void check_uart();

// ============================= Main ===================================
void app_main() {
  config_gpios();
  task_01();
  task_02();
  task_03();
  task_04();
  while (true) {
    check_buzzer();
    check_uart();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ====================== Tarefas Principais =====================

void task_01(){
  /*
  Botão A (GPIO com borda de descida): alterna o estado do LED1.
  */

  // Setar o tipo de interrupção (Borda de descida ou borda de subida)
  gpio_set_intr_type(BUTTON_01, GPIO_INTR_NEGEDGE);

  // Instala o gerenciador de interrupções
  gpio_install_isr_service(0);

  // Registra qual função executar quando o botão é pressionado
  gpio_isr_handler_add(BUTTON_01, botao_a_pressionado_function, NULL);
}

void task_02(){
  /*
  Botão B (GPIO com borda de descida): liga o buzzer por 1500 ms.
  */

  // Setar o tipo de interrupção (Borda de descida ou borda de subida)
  gpio_set_intr_type(BUTTON_02, GPIO_INTR_NEGEDGE);

  // Instala o gerenciador de interrupções
  gpio_install_isr_service(0);

  // Registra qual função executar quando o botão é pressionado
  gpio_isr_handler_add(BUTTON_02, botao_b_pressionado_function, NULL);
}

void task_03(){
  /*
  Timer (a cada 2 segundos): alterna o estado do LED2, simulando um
  piscar automático.
  */
  
  gptimer_handle_t timer_handle; // Identificador do timer

  // Configuração básica do timer
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,  // Usa clock padrão
      .direction = GPTIMER_COUNT_UP,       // Conta para CIMA (0,1,2...)
      .resolution_hz = 1000000,            // 1 MHz = resolução de 1 microsegundo
  };
  gptimer_new_timer(&timer_config, &timer_handle);

  // Configuração do ALARME
  gptimer_alarm_config_t alarm_config = {
    .alarm_count = 2000000,              // Dispara em 2.000.000 microssegundos = 2 segundos    
    .reload_count = 0,                   // Reinicia de 0 após o alarme
    .flags.auto_reload_on_alarm = true,  // Recarrega automaticamente
  };
  gptimer_set_alarm_action(timer_handle, &alarm_config);
  
  // Registrar a função que executa quando alarma
  gptimer_event_callbacks_t cbs = {
      .on_alarm = pisca_led2_function,   // Função que executa
  };
  gptimer_register_event_callbacks(timer_handle, &cbs, NULL);
  
  // Ligar o timer
  gptimer_enable(timer_handle);
  gptimer_start(timer_handle);
}

void task_04() {
  /*
  UART: ao digitar o caractere “a” no terminal desative a função do
  botão B e ao digitar “b” ative a função do Botão B.
  */
  // Configuração da porta serial
  uart_config_t uart_config = {
      .baud_rate = 115200,               // Velocidade: 115200 bps
      .data_bits = UART_DATA_8_BITS,     // 8 bits de dados
      .parity = UART_PARITY_DISABLE,     // Sem paridade
      .stop_bits = UART_STOP_BITS_1,     // 1 stop bit
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // Sem controle de fluxo
  };
  
  uart_param_config(UART_NUM_0, &uart_config);    // Aplica config
  uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);  // Instala driver
  printf("Digite 'a' para desativar a função do Botão B, e digite 'b' para ativa-la novamente.\n");
}

// ================== Funções Auxiliares ============================

// Função Auxiliar Task_01
void IRAM_ATTR botao_a_pressionado_function(void *arg){
  /*
  Função que altera o estado do LED 01 quando o botão 1 é pressionado
  */
  static uint32_t last_interrupt_time = 0;
  uint32_t interrupt_time = xTaskGetTickCountFromISR();

  // Debounce
  if ((interrupt_time - last_interrupt_time) > pdMS_TO_TICKS(DEBOUNCE_MS)) {
    last_interrupt_time = interrupt_time;
    
    // Se o led estiver ligado eu desligo, e vice versa
    led_01_state = (led_01_state == 1) ? 0 : 1;

    // Alterar de fato o estado do LED
    gpio_set_level(LED_01, led_01_state); 
  }
}

// Função Auxiliar Task_02
void IRAM_ATTR botao_b_pressionado_function(void *arg){
  /*
  Função que liga o buzzer quando o botão 2 é pressionado
  */

  // Se o UART desativou essa funcionalidade, apenas saia da função, rode nada
  if (botao_b_desativado) return;

  static uint32_t last_interrupt_time = 0;
  uint32_t interrupt_time = xTaskGetTickCountFromISR();

  // Debounce
  if ((interrupt_time - last_interrupt_time) > pdMS_TO_TICKS(DEBOUNCE_MS)) {
    last_interrupt_time = interrupt_time;

    ledc_set_freq(LEDC_MODE, LEDC_TIMER_BUZZER, 2000);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER, 128);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER);

    buzzer_active = true;
    last_buzzer_active_time = interrupt_time;
  }
}

// Função Auxiliar para a TASK_02
void check_buzzer(){
  /* 
  Função que checa se o buzzer está ativo, e se tiver espera 1500 MS e o desliga.
  */

  uint32_t current_time = xTaskGetTickCount();
  
  if(buzzer_active) {
    if ((current_time - last_buzzer_active_time) > pdMS_TO_TICKS(1500)) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BUZZER);
        buzzer_active = false;
    }
  }
}

// Função auxiliar para a TASK_03
static bool IRAM_ATTR pisca_led2_function(
  gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
  /* Função para piscar um LED a cada 2 segundos
  */

  // Se o led estiver desligado, liga, e vice versa
  led_02_state = (led_02_state == 1) ? 0 : 1;
  gpio_set_level(LED_02, led_02_state);

  return false; // Continua o TIMER normal
}

// Função para auxiliar a TASK_04
void check_uart(){
  uint8_t data; // Váriavel que tera o caractere lido

  // Tenta ler 1 byte com timeout 0 (não espera)
  int len = uart_read_bytes(UART_NUM_0, &data, 1, 0);

  if (len) {
    if (data == 'a' || data == 'A') {
      botao_b_desativado = true;
      printf("Botão B desativado!!!\n");
    } else if ((data == 'b' || data == 'B')) {
      botao_b_desativado = false;
      printf("Botão B ativado!!!\n");
    }
  }
}

// ============================= Funções de Configurações Iniciais ===========================================

// Configuração dos GPIOs
void config_gpios() {
  /*
  Configura Todos os GPIOs.
  */

  // Led 01
  gpio_reset_pin(LED_01);
  gpio_set_direction(LED_01, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_01, 0);
  // Led 02
  gpio_reset_pin(LED_02);
  gpio_set_direction(LED_02, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_02, 0);

  // Configuração dos botões
  gpio_reset_pin(BUTTON_01);
  gpio_set_direction(BUTTON_01, GPIO_MODE_INPUT);
  
  gpio_reset_pin(BUTTON_02);
  gpio_set_direction(BUTTON_02, GPIO_MODE_INPUT);

  // Configuração do PWM
  config_pwm(BUZZER, LEDC_TIMER_BUZZER, LEDC_CHANNEL_BUZZER, 0);
}

void config_pwm(int custom_pin, int custom_timer, int custom_channel, int duty) {
  /*
  Essa função tem o objetivo de configurar e aplicar PWM em um pino especifico.

  Params:
    custom_pin: Qual pino (15, 16, 17, etc)
    custom_timer: Qual timer
    custom_channel: Qual canal
    custom_duty: brilho inicial (0-255)
  */

  // Configurando TIMER - Define como o PWM irá funcionar (Frequência e Resolução)
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_MODE, // Modo LOW-SPEED
    .timer_num = custom_timer, // Timer 0 ou 1
    .duty_resolution = LEDC_RESOLUTION, // 8 bits (0-255)
    .freq_hz = LEDC_FREQUENCY, // 1000 HZ
    .clk_cfg = LEDC_AUTO_CLK // Clock Automático
  };
  ledc_timer_config(&ledc_timer); // Aplica a configuração

  // Configurar o Canal - Liga o TIMER ao Pino (Deixa o PWM pronto para uso)
  ledc_channel_config_t ledc_channel = {
    .gpio_num = custom_pin, // Qual pino usar
    .speed_mode = LEDC_MODE, // Modo
    .channel = custom_channel, // Qual canal usar
    .intr_type = LEDC_INTR_DISABLE, // Sem interrupções
    .timer_sel = custom_timer, // Qual timer usar
    .duty = duty, // Brilho inicial
    .hpoint = 0 // Ponto de referência
  };
  ledc_channel_config(&ledc_channel); // Aplica a configuração
}