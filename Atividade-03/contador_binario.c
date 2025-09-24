#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define DELAY_MS 500

#define LED01_GPIO 35
#define LED02_GPIO 36
#define LED03_GPIO 37
#define LED04_GPIO 38

void app_main() {
  // inicializar LED 1
  gpio_reset_pin(LED01_GPIO);
  gpio_set_direction(LED01_GPIO, GPIO_MODE_OUTPUT);
  
  // inicializar LED 2
  gpio_reset_pin(LED02_GPIO);
  gpio_set_direction(LED02_GPIO, GPIO_MODE_OUTPUT);
  
  // inicializar LED 3
  gpio_reset_pin(LED03_GPIO);
  gpio_set_direction(LED03_GPIO, GPIO_MODE_OUTPUT);
  
  // inicializar LED 4
  gpio_reset_pin(LED04_GPIO);
  gpio_set_direction(LED04_GPIO, GPIO_MODE_OUTPUT);

  // Inicia com os leds apagados
  gpio_set_level(LED01_GPIO, 0);
  gpio_set_level(LED02_GPIO, 0);
  gpio_set_level(LED03_GPIO, 0);
  gpio_set_level(LED04_GPIO, 0);

  void set_led_level(int num, int led_state) {
    if (num == 1) {
      gpio_set_level(LED01_GPIO, led_state);
    } else if (num == 2) {
      gpio_set_level(LED02_GPIO, led_state);
    } else if (num == 3) {
      gpio_set_level(LED03_GPIO, led_state);
    } else if (num == 4) {
      gpio_set_level(LED04_GPIO, led_state);
    }
  }

  while (1){
    for (int i = 0; i < 16; i++){
        gpio_set_level(LED01_GPIO, (i >> 0) & 1);
        gpio_set_level(LED02_GPIO, (i >> 1) & 1);
        gpio_set_level(LED03_GPIO, (i >> 2) & 1);
        gpio_set_level(LED04_GPIO, (i >> 3) & 1);

        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }

    for (int i = 1; i <= 4; i++) {
        set_led_level(i, 1);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        set_led_level(i, 0);
    }
    for (int i = 4; i >= 1; i--) {
        set_led_level(i, 1);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        set_led_level(i, 0);
    }
  }
}
