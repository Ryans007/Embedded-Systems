#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"


// ================================ Typedefs ======================================

// Estrutura para armazenar dados de aceleração
typedef struct {
    float x;
    float y;
    float z;
} accel_data_t;

// ================================ Definição das Variaveis ===============================
#define LED 15
#define I2C_SDA 12
#define I2C_SCL 13
#define I2C_PORT I2C_NUM_0  
#define I2C_FREQ 100000

// Endereços I2C
#define MPU6050_ADDR 0x68
#define SSD1306_ADDR 0x3C

// Registradores do MPU6050
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

// Configurações do SSD1306
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

// Buffer para média móvel
#define BUFFER_SIZE 10
#define ACCEL_THRESHOLD 0.5
#define SAMPLE_PERIOD_MS 200

// Variaveis para buffer
static accel_data_t accel_buffer[10];  // Buffer circular
static int buffer_index = 0;           // Índice atual
static int buffer_count = 0;           // Quantos dados temos

// Guardar valores anteriores
static accel_data_t last_accel = {0, 0, 0};

// Buffer de display do SSD1306
static uint8_t ssd1306_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// ================================ Definição dos Prototipos ===========================

// Protótipos de funções
static esp_err_t i2c_initial_config();
static void initial_config_led();
static esp_err_t mpu6050_init();
static esp_err_t mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az);
static esp_err_t ssd1306_init();
static void ssd1306_clear();
static void ssd1306_display();
static void ssd1306_draw_text(uint8_t x, uint8_t y, const char *text, uint8_t size);
static void calculate_average(accel_data_t *avg);
static bool check_acceleration_change(accel_data_t current);
void add_to_buffer(accel_data_t data);

// Font 8x8 - mais legível
static const uint8_t font8x8[][8] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // ' '
    {0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 0x00, 0x00}, // '!'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '"'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '#'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '$'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '%'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '&'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '''
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '('
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // ')'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '*'
    {0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00}, // '+'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // ','
    {0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00}, // '-'
    {0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00}, // '.'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '/'
    {0x3E, 0x41, 0x49, 0x45, 0x43, 0x3E, 0x00, 0x00}, // '0'
    {0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, 0x00, 0x00}, // '1'
    {0x42, 0x61, 0x51, 0x49, 0x46, 0x00, 0x00, 0x00}, // '2'
    {0x21, 0x41, 0x45, 0x4B, 0x31, 0x00, 0x00, 0x00}, // '3'
    {0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, 0x00, 0x00}, // '4'
    {0x27, 0x45, 0x45, 0x45, 0x39, 0x00, 0x00, 0x00}, // '5'
    {0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00, 0x00, 0x00}, // '6'
    {0x01, 0x71, 0x09, 0x05, 0x03, 0x00, 0x00, 0x00}, // '7'
    {0x36, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00, 0x00}, // '8'
    {0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x00, 0x00}, // '9'
    {0x00, 0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00}, // ':'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // ';'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '<'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '='
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '>'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '?'
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // '@'
    {0x7C, 0x12, 0x11, 0x12, 0x7C, 0x00, 0x00, 0x00}, // 'A'
    {0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00, 0x00}, // 'B'
    {0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, 0x00, 0x00}, // 'C'
    {0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00}, // 'D'
    {0x7F, 0x49, 0x49, 0x49, 0x41, 0x00, 0x00, 0x00}, // 'E'
    {0x7F, 0x09, 0x09, 0x09, 0x01, 0x00, 0x00, 0x00}, // 'F'
    {0x3E, 0x41, 0x49, 0x49, 0x7A, 0x00, 0x00, 0x00}, // 'G'
    {0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x00, 0x00}, // 'H'
    {0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, 0x00, 0x00}, // 'I'
    {0x20, 0x40, 0x41, 0x3F, 0x01, 0x00, 0x00, 0x00}, // 'J'
    {0x7F, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00, 0x00}, // 'K'
    {0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00}, // 'L'
    {0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00, 0x00, 0x00}, // 'M'
    {0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00, 0x00, 0x00}, // 'N'
    {0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00, 0x00, 0x00}, // 'O'
    {0x7F, 0x09, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00}, // 'P'
    {0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00, 0x00, 0x00}, // 'Q'
    {0x7F, 0x09, 0x19, 0x29, 0x46, 0x00, 0x00, 0x00}, // 'R'
    {0x46, 0x49, 0x49, 0x49, 0x31, 0x00, 0x00, 0x00}, // 'S'
    {0x01, 0x01, 0x7F, 0x01, 0x01, 0x00, 0x00, 0x00}, // 'T'
    {0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00, 0x00, 0x00}, // 'U'
    {0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00, 0x00, 0x00}, // 'V'
    {0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00, 0x00, 0x00}, // 'W'
    {0x63, 0x14, 0x08, 0x14, 0x63, 0x00, 0x00, 0x00}, // 'X'
    {0x07, 0x08, 0x70, 0x08, 0x07, 0x00, 0x00, 0x00}, // 'Y'
    {0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x00, 0x00}, // 'Z'
};

void app_main() {
  printf("Iniciando sistema...\n");
  
  initial_config_led();
  
  if (i2c_initial_config() != ESP_OK) {
      printf("Erro ao configurar I2C\n");
      return;
  }
  
  vTaskDelay(100 / portTICK_PERIOD_MS);
  if (mpu6050_init() != ESP_OK) {
      printf("Erro ao inicializar MPU6050\n");
      return;
  }
  
  vTaskDelay(100 / portTICK_PERIOD_MS);
  if (ssd1306_init() != ESP_OK) {
      printf("Erro ao inicializar SSD1306\n");
      return;
  }
  
  printf("Sistema pronto!\n\n");
  
  int16_t ax_raw, ay_raw, az_raw;
  char buffer[32];
  while (true) {
    if(mpu6050_read_accel(&ax_raw, &ay_raw, &az_raw) == ESP_OK) {
        // Formula de Conversão
        accel_data_t current;
        current.x = (ax_raw / 16384.0) * 9.81;
        current.y = (ay_raw / 16384.0) * 9.81;
        current.z = (az_raw / 16384.0) * 9.81;

        printf("X: %.2f m/s² Y: %.2f m/s² Z: %.2f m/s²\n", current.x, current.y, current.z);

        // Adicionar ao buffer
        add_to_buffer(current);

        // Calcular média
        accel_data_t avg;
        calculate_average(&avg);

        if (check_acceleration_change(avg)) {
            // LED pisca
            gpio_set_level(LED, 1);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(LED, 0);
         }
        last_accel = avg;  // Atualiza para próxima iteração
            
        printf("X: %.2f m/s²  Y: %.2f m/s²  Z: %.2f m/s²\n", avg.x, avg.y, avg.z);
        
        ssd1306_clear();
        
        ssd1306_draw_text(0, 0, "Aceleracao", 1);
        
        sprintf(buffer, "X: %.2f", avg.x);
        ssd1306_draw_text(0, 18, buffer, 2);
        
        sprintf(buffer, "Y: %.2f", avg.y);
        ssd1306_draw_text(0, 34, buffer, 2);
        
        sprintf(buffer, "Z: %.2f", avg.z);
        ssd1306_draw_text(0, 50, buffer, 2);
        
        ssd1306_display();
        }
        vTaskDelay(SAMPLE_PERIOD_MS / portTICK_PERIOD_MS);
    }
  }

// ================================ Funções Auxiliares MPU6050 ==================================

// Função para inicializar o I2C
static esp_err_t mpu6050_init() {
    // Dados para enviar
    uint8_t data = 0x00;  // Coloca MPU6050 em modo normal
    
    // Criação de  comando I2C
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Construção de sequência I2C
    i2c_master_start(cmd);                    // Inicia comunicação
    
    // Escrevendo endereço + comando de WRITE
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    
    // Escrevendo o registro que queremos modificar
    i2c_master_write_byte(cmd, 0x6B, true);  // PWR_MGMT_1 register
    
    // Escrevendo o valor
    i2c_master_write_byte(cmd, data, true);  // Valor: 0x00 (normal)
    
    // Parando comunicação
    i2c_master_stop(cmd);
    
    // Executando comando (timeout 1000ms)
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    
    // Liberando memória
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        printf("MPU6050 inicializado com sucesso!\n");
    }
    return ret;
}

// Função para ler dados do mpu6050
static esp_err_t mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t data[6];  // Vamos ler 6 bytes (2 para cada eixo)
    
    // Criando comando para ESCREVER o registro que queremos ler
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true);  // ACCEL_XOUT_H register (primeiro byte)
    
    // Agora RECOMEÇAR e LER (repeated start)
    i2c_master_start(cmd);  // ← Repeated START (não solta o barramento)
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    
    // Ler os 6 bytes
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    //                                 ↑
    //                    Último byte: sem ACK
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    // Combinando  os bytes em int16_t
    if (ret == ESP_OK) {
        *ax = (int16_t)((data[0] << 8) | data[1]);        
        *ay = (int16_t)((data[2] << 8) | data[3]);
        *az = (int16_t)((data[4] << 8) | data[5]);
    }
    return ret;
}

// ==================================== Funções para o SSD1306 ===================================

static esp_err_t ssd1306_init() {
    uint8_t init_cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
    };
    
    // Envia comandos de inicialização
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SSD1306_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);  // Control byte
    
    for (int i = 0; i < sizeof(init_cmds); i++) {
        i2c_master_write_byte(cmd, init_cmds[i], true);
    }
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        printf("SSD1306 inicializado com sucesso!\n");
    }
    
    return ret;
}

// PASSO 9B: Limpar buffer do display
static void ssd1306_clear() {
    memset(ssd1306_buffer, 0, sizeof(ssd1306_buffer));
}

// PASSO 9C: Enviar buffer para display
static void ssd1306_display() {
    for (uint8_t page = 0; page < 8; page++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SSD1306_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x00, true);
        i2c_master_write_byte(cmd, 0xB0 + page, true);
        i2c_master_write_byte(cmd, 0x00, true);
        i2c_master_write_byte(cmd, 0x10, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SSD1306_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x40, true);
        i2c_master_write(cmd, &ssd1306_buffer[SSD1306_WIDTH * page], SSD1306_WIDTH, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    }
}


static void ssd1306_draw_char(uint8_t x, uint8_t y, char c, uint8_t size) {
    int index = -1;
    
    if (c >= '0' && c <= '9') {
        index = (c - '0') + 16;
    } else if (c >= 'A' && c <= 'Z') {
        index = (c - 'A') + 33;
    } else if (c >= 'a' && c <= 'z') {
        index = (c - 'a') + 33;
    } else if (c == ' ') {
        index = 0;
    } else if (c == ':') {
        index = 26;
    } else if (c == '.') {
        index = 14;
    } else if (c == '-') {
        index = 13;
    }
    
    if (index >= 0 && index < sizeof(font8x8) / sizeof(font8x8[0])) {
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                if (font8x8[index][i] & (1 << j)) {
                    for (int sy = 0; sy < size; sy++) {
                        for (int sx = 0; sx < size; sx++) {
                            int px = x + i * size + sx;
                            int py = y + j * size + sy;
                            if (px < SSD1306_WIDTH && py < SSD1306_HEIGHT) {
                                ssd1306_buffer[px + (py / 8) * SSD1306_WIDTH] |= (1 << (py % 8));
                            }
                        }
                    }
                }
            }
        }
    }
}

static void ssd1306_draw_text(uint8_t x, uint8_t y, const char *text, uint8_t size) {
    int pos_x = x;
    for (int i = 0; text[i] != '\0'; i++) {
        ssd1306_draw_char(pos_x, y, text[i], size);
        pos_x += 8 * size;
    }
}

// ================================= Funções Auxiliares ===================================

// Função para adicionar ao buffer
void add_to_buffer(accel_data_t data) {
    accel_buffer[buffer_index] = data;
    
    // Próximo índice (volta a 0 quando chega em 10)
    buffer_index = (buffer_index + 1) % 10;
    
    // Conta até 10
    if (buffer_count < 10) {
        buffer_count++;
    }
}
    
// Função para Calcular média
void calculate_average(accel_data_t *avg) {
    avg->x = 0;
    avg->y = 0;
    avg->z = 0;
    
    // Soma todos os valores
    for (int i = 0; i < buffer_count; i++) {
        avg->x += accel_buffer[i].x;
        avg->y += accel_buffer[i].y;
        avg->z += accel_buffer[i].z;
    }
    
    // Divide pela quantidade
    if (buffer_count > 0) {
        avg->x /= buffer_count;
        avg->y /= buffer_count;
        avg->z /= buffer_count;
    }
}

// Função para detectar mudança brusca de aceleração
bool check_acceleration_change(accel_data_t current) {
    // Calcula diferença em cada eixo
    float delta_x = fabs(current.x - last_accel.x);
    float delta_y = fabs(current.y - last_accel.y);
    float delta_z = fabs(current.z - last_accel.z);
    
    // Se QUALQUER mudança >= 0.5 m/s², é significativa
    return (delta_x >= 0.5 || delta_y >= 0.5 || delta_z >= 0.5);
}

// ================================= Funções de Configuração ========================================

// Função para configurar o I2C
static esp_err_t i2c_initial_config() {
  // Configuração I2C
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,           // ESP32 é MASTER
      .sda_io_num = I2C_SDA,             // GPIO 16
      .scl_io_num = I2C_SCL,             // GPIO 15
      .sda_pullup_en = GPIO_PULLUP_ENABLE,   // Ativa pull-up
      .scl_pullup_en = GPIO_PULLUP_ENABLE,   // Ativa pull-up
      .master.clk_speed = 100000,        // 100 kHz
  };

    // Aplicar configuração
    esp_err_t err = i2c_param_config(I2C_NUM_0, &conf);
    if (err != ESP_OK) {
        printf("Erro ao configurar I2C!\n");
        return err;
    }
    // Instalar driver
    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0); 
}

// Função para configurar os LEDs
void initial_config_led() {
  gpio_reset_pin(LED);
  gpio_set_direction(LED, GPIO_MODE_OUTPUT);
  gpio_set_level(LED, 0);
}