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
- Atividade-10/
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

### Atividade 07 – Acelerômetro I2C com Display OLED
- Descrição: Leitura do acelerômetro MPU6050 via I2C com média móvel de 10 amostras. Os valores de aceleração (X, Y, Z) em m/s² são exibidos no display OLED SSD1306, e um LED pisca ao detectar variação brusca (≥ 0,5 m/s²) em qualquer eixo.
- Simulação no Wokwi:  
  https://wokwi.com/projects/447784831288053761

### Atividade 08 – Termômetro com Alarme
- Descrição: Leitura de temperatura via sensor NTC (equação Steinhart-Hart) com exibição em LCD 16x2 I2C. 4 LEDs indicam a proximidade da temperatura ao limiar de alarme. Ao atingir o limiar, o buzzer é acionado e os LEDs piscam. Botão A incrementa e Botão B decrementa o limiar de alarme em 5 °C via interrupção.
- Simulação no Wokwi:  
  https://wokwi.com/projects/447816426252018689

### Atividade 09 – Termômetro com Máquina de Estados e SD Card
- Descrição: Evolução da Atividade 08 com arquitetura baseada em máquina de estados (inicializar → ler sensor → atualizar display → atualizar LEDs → verificar alarme → salvar SD Card). Registra leituras de temperatura a cada 500 ms em cartão SD via SPI, mantendo o LCD I2C, buzzer, 4 LEDs indicadores e controle de limiar por interrupção.
- Simulação no Wokwi:  
  https://wokwi.com/projects/447863454176456705

### Atividade 10 – FreeRTOS com Display de 7 Segmentos
- Descrição: Evolução da Atividade 09 com adição de display de 7 segmentos (7 GPIOs dedicados) e uso de FreeRTOS para gerenciamento de tarefas. Mantém a mesma arquitetura de máquina de estados, LCD I2C, NTC, buzzer, 4 LEDs indicadores, SD Card via SPI e controle de limiar por interrupção.
- Simulação no Wokwi:  
  https://wokwi.com/projects/447870876107862017

---
*Desenvolvido para a disciplina de Sistemas Embarcados*


