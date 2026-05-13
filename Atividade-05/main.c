#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// =========================== Definição das Váriaveis ============================
#define LED_01 15
#define LED_02 16
#define LED_03 17
#define LED_04 18

#define BUTTON_01 42
#define BUTTON_02 41

#define DEBOUNCE_MS 200

// ========================== Definição dos Prototipos ============================
void config_gpios();

// =================================== Main =======================================
void app_main() {
  // Configurar os GPIOs
  config_gpios();

  // Variaveis para armazenar quando foi a última vez que o botão foi pressionado, para DEBOUNCE
  uint32_t last_press_button_01 = 0;
  uint32_t last_press_button_02 = 0;

  // Variáveis que armazenam o último estado do botão
  int prev_state_button_01 = 1; 
  int prev_state_button_02 = 1;

  int contador = 0; // Para incrementar o valor binário toda vez que o usuário pressiona o botão
  int incremento = 1; // Variavel que diz a adição do incremento (1 ou 2)

  // Looping Principal 
  while (true) {
    uint32_t current_time =  xTaskGetTickCount() * portTICK_PERIOD_MS; // Pega o tempo atual para debounce

    // Váriaveis com o estado atual dos botões
    int current_state_button_01 = gpio_get_level(BUTTON_01);
    int current_state_button_02 = gpio_get_level(BUTTON_02);

    /* Botão A: a cada acionamento, deve incrementar o valor do contador
    conforme a unidade de incremento atual (padrão: +1). */
    if (!current_state_button_01 && prev_state_button_01 == 1) {
      if ((current_time - last_press_button_01) > 200) {
        last_press_button_01 = current_time;
        
        contador += incremento;
        printf("Contador: %d\n", contador);

        gpio_set_level(LED_01, (contador >> 0) & 1);
        gpio_set_level(LED_02, (contador >> 1) & 1);
        gpio_set_level(LED_03, (contador >> 2) & 1);
        gpio_set_level(LED_04, (contador >> 3) & 1);

        if (contador > 15) {
          contador = contador & 0x0F;
        }
      }
    }

    /*○ Botão B: alterna a unidade de incremento entre +1 e +2 a cada acionamento. */
    if (!current_state_button_02 && prev_state_button_02 == 1) {
      if ((current_time - last_press_button_02) > 200) {
        last_press_button_02 = current_time;

        // Se precinou o botão e o incremento era 1, vira 2, e vice versa
        incremento = (incremento == 1) ? 2 : 1;
        printf("Incremento Alterado Para: %d\n", incremento); 
      }
    }
    // Atualizar o estado anterior dos botões, com o estado atual         
    prev_state_button_01 = current_state_button_01;
    prev_state_button_02 = current_state_button_02;
  }
}

void config_gpios() {
  /* 
    Função para configurar todos os pinos utilizados
  */

  // Configuração dos botões
  
  // Botão 01
  gpio_reset_pin(BUTTON_01);
  gpio_set_direction(BUTTON_01, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_01, GPIO_PULLUP_ONLY);

  // Botão 02
  gpio_reset_pin(BUTTON_02);
  gpio_set_direction(BUTTON_02, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_02, GPIO_PULLUP_ONLY);

  // Configuração dos LEDs

  // LED 01
  gpio_reset_pin(LED_01);
  gpio_set_direction(LED_01, GPIO_MODE_OUTPUT);

  // LED 02
  gpio_reset_pin(LED_02);
  gpio_set_direction(LED_02, GPIO_MODE_OUTPUT);

  // LED 03
  gpio_reset_pin(LED_03);
  gpio_set_direction(LED_03, GPIO_MODE_OUTPUT);

  // LED 04
  gpio_reset_pin(LED_04);
  gpio_set_direction(LED_04, GPIO_MODE_OUTPUT);

  // Todos os LEDs começam desligados
  gpio_set_level(LED_01, 0);
  gpio_set_level(LED_02, 0);
  gpio_set_level(LED_03, 0);
  gpio_set_level(LED_04, 0);
}