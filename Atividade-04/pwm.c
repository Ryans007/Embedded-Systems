#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define LED_ONE   15
#define LED_TWO   16
#define LED_THREE 17
#define LED_FOUR  18
#define BUZZER_PIN 14

// Frequência de 1 kHz e resolução de 8 bits (0–255 níveis de duty)
#define LEDC_FREQUENCY     1000
#define LEDC_RESOLUTION    LEDC_TIMER_8_BIT
#define LEDC_TIMER         LEDC_TIMER_0
#define LEDC_TIMER_BUZZER  LEDC_TIMER_1
#define LEDC_MODE          LEDC_LOW_SPEED_MODE

#define LEDC_CHANNEL_ONE   LEDC_CHANNEL_0
#define LEDC_CHANNEL_TWO   LEDC_CHANNEL_1
#define LEDC_CHANNEL_THREE   LEDC_CHANNEL_2
#define LEDC_CHANNEL_FOUR   LEDC_CHANNEL_3
#define LEDC_CHANNEL_BUZZER LEDC_CHANNEL_4

void phase_one();
void phase_two();
void phase_three();
void config_pwm();

void app_main() {
  while (1) {
    phase_one();
    phase_two();
    phase_three();
  }
}

void phase_one(){
    /*Fading Sincronizado dos LEDs 
      - Todos os LEDs variam o brilho de 0% → 100% → 0% em ciclos.
    */
    // Configuração pwm dos leds
    config_pwm(LED_ONE, LEDC_TIMER, LEDC_CHANNEL_ONE, 0);
    config_pwm(LED_TWO, LEDC_TIMER, LEDC_CHANNEL_TWO, 0);
    config_pwm(LED_THREE, LEDC_TIMER, LEDC_CHANNEL_THREE, 0);
    config_pwm(LED_FOUR, LEDC_TIMER, LEDC_CHANNEL_FOUR, 0);

    // 4. Loop para variar duty cycle nos dois LEDs
    // Aumenta o brilho de 0% para 100% (0 a 255)
    for (int duty = 0; duty <= 255; duty++) {
        // LED 1
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_ONE, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_ONE);

        // LED 2
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_TWO, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_TWO);

        // LED 3
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_THREE, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_THREE);

        // LED 4
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_FOUR, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_FOUR);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Aumenta o brilho de 100% para 0% (0 a 255)
    for (int duty = 255; duty >= 0; duty--) {
        // LED 1
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_ONE, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_ONE);

        // LED 2
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_TWO, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_TWO);

        // LED 3
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_THREE, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_THREE);

        // LED 4
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_FOUR, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_FOUR);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
};

void phase_two(){
    /*Fading Sequencial dos LEDs
    - ○ LED1 faz o ciclo → LED2 → LED3 → LED4, depois retorna.
    */

    // Configuração pwm dos leds
    config_pwm(LED_ONE, LEDC_TIMER, LEDC_CHANNEL_ONE, 0);
    config_pwm(LED_TWO, LEDC_TIMER, LEDC_CHANNEL_TWO, 0);
    config_pwm(LED_THREE, LEDC_TIMER, LEDC_CHANNEL_THREE, 0);
    config_pwm(LED_FOUR, LEDC_TIMER, LEDC_CHANNEL_FOUR, 0);

    void execute_fade(int led_channel){
        // Aumenta o brilho de 0% para 100% (0 a 255)
        for (int duty = 0; duty <= 255; duty++) {
            ledc_set_duty(LEDC_MODE, led_channel, duty);
            ledc_update_duty(LEDC_MODE, led_channel);

            vTaskDelay(pdMS_TO_TICKS(10));
        }
        // Aumenta o brilho de 100% para 0% (0 a 255)
        for (int duty = 255; duty >= 0; duty--) {
            ledc_set_duty(LEDC_MODE, led_channel, duty);
            ledc_update_duty(LEDC_MODE, led_channel);

            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    // LED 1
    execute_fade(LEDC_CHANNEL_ONE);
    // LED 2
    execute_fade(LEDC_CHANNEL_TWO);
    // LED 3
    execute_fade(LEDC_CHANNEL_THREE);
    // LED 4
    execute_fade(LEDC_CHANNEL_FOUR);
    // RETORNO
    // LED 3
    execute_fade(LEDC_CHANNEL_THREE);
    // LED 2
    execute_fade(LEDC_CHANNEL_TWO);
    // LED 1
    execute_fade(LEDC_CHANNEL_ONE);
}

void phase_three() {
  /* Fase 3: Teste Sonoro com o Buzzer
    - ○ O buzzer emite tons que variam de 500 Hz a 2000 Hz.
    - ○ Frequência sobe gradualmente (500 → 2000 Hz) e depois desce.
  */

    // Configuração do PWM no buzzer
    config_pwm(BUZZER_PIN, LEDC_TIMER_BUZZER, LEDC_CHANNEL_BUZZER, 128);

    for (int freq = 500; freq <= 2000 ; freq+=2){
      ledc_set_freq(LEDC_MODE, LEDC_TIMER_BUZZER, freq);
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    for (int freq = 2000; freq >= 500; freq-=2){
      ledc_set_freq(LEDC_MODE, LEDC_TIMER_BUZZER, freq);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    ledc_set_freq(LEDC_MODE, LEDC_TIMER_BUZZER, 0);
}

void config_pwm(int custom_pin, int custom_timer, int custom_channel, int duty) {
    // 1. Configuração do Timer 
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = custom_timer,
        .duty_resolution  = LEDC_RESOLUTION,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // 2. Configuração do Canal
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = custom_pin,
        .speed_mode     = LEDC_MODE,
        .channel        = custom_channel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = custom_timer,
        .duty           = duty, 
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}
