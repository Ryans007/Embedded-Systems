# Sistemas Embarcados (Faculdade)

Este repositório agrupa as atividades práticas da cadeira de **Sistemas Embarcados** da faculdade. Cada pasta contém o código fonte de uma atividade distinta, desenvolvida em C para ESP32/FreeRTOS.

## Estrutura do Repositório

- Atividade-01/
- Atividade-02/
- Atividade-03/
  - `contador_binario.c`
- Atividade-04/
  - `pwm.c`
- Atividade-05/
  - `main.c`
- Atividade-06/
  - `main.c`
- Atividade-07/
  - `main.c`
- Atividade-08/
  - `main.c`
- Atividade-09/
  - `main.c`

## Atividades e Simulações Wokwi

### Atividade 03 – Contador Binário
- Descrição: Implementação de um contador binário de 0 a 15 utilizando 4 LEDs.
- Simulação no Wokwi:  
  https://wokwi.com/projects/447700133653697537

### Atividade 04 – PWM e Buzzer
- Descrição: Controle de brilho de LEDs via PWM e geração de tonalidades no buzzer (500 Hz → 2000 Hz → 500 Hz).
- Simulação no Wokwi:  
  https://wokwi.com/projects/447713609241958401

### Atividade 05 – Contador Binário com Botões
- Descrição: Contador binário de 0 a 15 com 4 LEDs e 2 botões: Botão A incrementa o contador (+1 ou +2) e Botão B alterna a unidade de incremento entre +1 e +2.
- Simulação no Wokwi:  
  https://wokwi.com/projects/447725339539824641

### Atividade 06 – Interrupções, Timer e UART
- Descrição: Uso de interrupções GPIO, timer por hardware e UART. Botão A alterna LED1 via interrupção, Botão B liga o buzzer por 1500 ms via PWM, timer dispara a cada 2 s alternando LED2, e comandos UART ('a'/'b') ativam ou desativam a função do Botão B.
- Simulação no Wokwi:  
  https://wokwi.com/projects/447732757246547969

---
*Desenvolvido para a disciplina de Sistemas Embarcados*


