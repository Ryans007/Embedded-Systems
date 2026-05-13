#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

// ====================== Definição das Varíaveis ========================
#define LED_01 15
#define LED_02 16
#define LED_03 17
#define LED_04 18

#define DELAY_MS 500

// ====================== Definição dos Protitpos ========================
void config_gpios();
void phase_01();
void phase_02();
void set_led_level(int led, int level);

// ====================== Main ========================

void app_main() {
  config_gpios();
  while(true) {
    phase_01();
    phase_02();    
  }
}

// ====================== Funções Que Separam a Lógica Principal ========================

// Conta de 0 a 15 em binário
void phase_01() {
  for (int i = 0; i <= 16; i++) {
    gpio_set_level(LED_01, (i >> 0) & 1);
    gpio_set_level(LED_02, (i >> 1) & 1);
    gpio_set_level(LED_03, (i >> 2) & 1);
    gpio_set_level(LED_04, (i >> 3) & 1);
    vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
  }
}

// Acendem e Apagam os Leds em Sequência
void phase_02() {
  for (int i = 4; i >= 1; i--) {
    set_led_level(i, 1);
    vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    set_led_level(i, 0);
  }
  for (int i = 1; i <= 5; i++) {
    set_led_level(i, 1);
    vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    set_led_level(i, 0);
  }
}

// ====================== Funções Auxiliares ========================

// Função para apagar e desligar um led especifico 
void set_led_level(int led, int level){
  switch (led) {
    case 1: gpio_set_level(LED_01, level);
    case 2: gpio_set_level(LED_02, level);
    case 3: gpio_set_level(LED_03, level);
    case 4: gpio_set_level(LED_04, level);
  }
}

// Configuração Inicial dos Pinos
void config_gpios() {
  // Configura LED 01 
  gpio_reset_pin(LED_01);
  gpio_set_direction(LED_01, GPIO_MODE_OUTPUT);

  // Configura LED 02
  gpio_reset_pin(LED_02);
  gpio_set_direction(LED_02, GPIO_MODE_OUTPUT);

  // Configura LED 03
  gpio_reset_pin(LED_03);
  gpio_set_direction(LED_03, GPIO_MODE_OUTPUT);

  // Configura LED 04  
  gpio_reset_pin(LED_04);
  gpio_set_direction(LED_04, GPIO_MODE_OUTPUT);

  // Os leds começam apagados
  gpio_set_level(LED_01, 0);
  gpio_set_level(LED_02, 0);
  gpio_set_level(LED_03, 0);
  gpio_set_level(LED_04, 0);
}
