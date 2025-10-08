#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_01 15
#define LED_02 16
#define LED_03 17
#define LED_04 18

#define BUTTON_01 14
#define BUTTON_02 13

#define DEBOUNCE_MS 200

void my_gpio_config();

void app_main() {
    // Configuração dos GPIOS
    my_gpio_config();

    // Variáveis de controle
    uint32_t last_press_button_01 = 0;
    uint32_t last_press_button_02 = 0;
    
    // Estados anteriores dos botões (para detecção de borda)
    int prev_state_button_01 = 0;
    int prev_state_button_02 = 0;
    
    int contador = 0;
    int incremento = 1; 
    
    while(1) {
        // Obtém o tempo atual em milissegundos
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Lê o estado atual dos botões
        int current_state_button_01 = gpio_get_level(BUTTON_01);
        int current_state_button_02 = gpio_get_level(BUTTON_02);
        
        // BOTÃO A (Incrementa contador)
        // Detecta borda de subida (transição de 0 para 1)
        if (current_state_button_01 == 1 && prev_state_button_01 == 0) {
            // Verifica se passou o tempo de debounce
            if ((current_time - last_press_button_01) > DEBOUNCE_MS) {
                last_press_button_01 = current_time;
                
                // Incrementa o contador com tratamento de overflow
                contador += incremento;
                
                // Garante que o contador fique dentro de 4 bits (0-15)
                if (contador > 15) {
                    contador = contador & 0x0F; 
                }
                
                printf("Contador: %d (0x%X) - Incremento: %d\n", contador, contador, incremento);
                
                // Atualiza os LEDs com o valor do contador
                gpio_set_level(LED_01, (contador >> 0) & 1); 
                gpio_set_level(LED_02, (contador >> 1) & 1); 
                gpio_set_level(LED_03, (contador >> 2) & 1); 
                gpio_set_level(LED_04, (contador >> 3) & 1); 
            }
        }
        
        // BOTÃO B (Alterna incremento entre 1 e 2)
        // Detecta borda de subida (transição de 0 para 1)
        if (current_state_button_02 == 1 && prev_state_button_02 == 0) {
            // Verifica se passou o tempo de debounce
            if ((current_time - last_press_button_02) > DEBOUNCE_MS) {
                last_press_button_02 = current_time;
                
                // Alterna o valor do incremento entre 1 e 2
                incremento = (incremento == 1) ? 2 : 1;
                
                printf("Incremento alterado para: %d\n", incremento);
            }
        }
        
        // Atualiza os estados anteriores dos botões
        prev_state_button_01 = current_state_button_01;
        prev_state_button_02 = current_state_button_02;
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void my_gpio_config(){
    // Configuração dos botões
    // Botão 1 
    gpio_reset_pin(BUTTON_01);
    gpio_set_direction(BUTTON_01, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_01, GPIO_PULLDOWN_ONLY);

    // Botão 2 
    gpio_reset_pin(BUTTON_02);
    gpio_set_direction(BUTTON_02, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_02, GPIO_PULLDOWN_ONLY);

    // Configuração dos LEDs
    gpio_reset_pin(LED_01);
    gpio_set_direction(LED_01, GPIO_MODE_OUTPUT);
    
    gpio_reset_pin(LED_02);
    gpio_set_direction(LED_02, GPIO_MODE_OUTPUT);
    
    gpio_reset_pin(LED_03);
    gpio_set_direction(LED_03, GPIO_MODE_OUTPUT);
    
    gpio_reset_pin(LED_04);
    gpio_set_direction(LED_04, GPIO_MODE_OUTPUT);

    // Inicia com os LEDs apagados
    gpio_set_level(LED_01, 0);
    gpio_set_level(LED_02, 0);
    gpio_set_level(LED_03, 0);
    gpio_set_level(LED_04, 0);
}