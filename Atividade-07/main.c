#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

// Pinos e configurações I2C
#define LED 38
#define I2C_SCL 15
#define I2C_SDA 16
#define I2C_PORT I2C_NUM_0  
#define I2C_FREQ 100000

// Endereços I2C dos dispositivos
#define MPU6050_ADDR 0x68
#define SSD1306_ADDR 0x3C

// Registradores do MPU6050
#define MPU6050_PWR_MGMT_1 0x6B      // Controle de energia
#define MPU6050_ACCEL_XOUT_H 0x3B    // Início dos dados de aceleração

// Display OLED
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64

// Parâmetros do sistema
#define BUFFER_SIZE 10           // Tamanho do buffer para média móvel
#define ACCEL_THRESHOLD 0.5      // Limite para piscar LED (m/s²)
#define SAMPLE_PERIOD_MS 200     // Período de amostragem

// Estrutura para armazenar dados de aceleração
typedef struct {
    float x;
    float y;
    float z;
} accel_data_t;

// Buffer circular para cálculo da média
static accel_data_t accel_buffer[BUFFER_SIZE];
static int buffer_index = 0;
static int buffer_count = 0;
static accel_data_t last_accel = {0, 0, 0};

// Buffer do display
static uint8_t ssd1306_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Font 8x8 para display
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

// Protótipos
static esp_err_t i2c_initial_config();
static void led_initial_config();
static esp_err_t mpu6050_init();
static esp_err_t mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az);
static esp_err_t ssd1306_init();
static void ssd1306_clear();
static void ssd1306_display();
static void ssd1306_draw_text(uint8_t x, uint8_t y, const char *text, uint8_t size);
static void calculate_average(accel_data_t *avg);
static bool check_acceleration_change(accel_data_t current);

void app_main() {
    printf("Iniciando sistema...\n");
    
    // Inicializar periféricos
    led_initial_config();
    
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
    
    // Loop principal
    while (1) {
        if (mpu6050_read_accel(&ax_raw, &ay_raw, &az_raw) == ESP_OK) {
            // Converter valores raw para m/s²
            // MPU6050 escala padrão: ±2g = 16384 LSB/g
            accel_data_t current;
            current.x = (ax_raw / 16384.0) * 9.81;
            current.y = (ay_raw / 16384.0) * 9.81;
            current.z = (az_raw / 16384.0) * 9.81;
            
            // Adicionar ao buffer circular
            accel_buffer[buffer_index] = current;
            buffer_index = (buffer_index + 1) % BUFFER_SIZE;
            if (buffer_count < BUFFER_SIZE) {
                buffer_count++;
            }
            
            // Calcular média das últimas 10 leituras
            accel_data_t avg;
            calculate_average(&avg);
            
            // LED pisca quando detecta mudança significativa
            if (check_acceleration_change(avg)) {
                gpio_set_level(LED, 1);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                gpio_set_level(LED, 0);
            }
            
            last_accel = avg;
            
            // Enviar para serial
            printf("X: %.2f m/s²  Y: %.2f m/s²  Z: %.2f m/s²\n", avg.x, avg.y, avg.z);
            
            // Atualizar display
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

static esp_err_t i2c_initial_config() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,  // Pull-ups externos na montagem
        .scl_pullup_en = GPIO_PULLUP_DISABLE,  // Pull-ups externos na montagem
        .master.clk_speed = I2C_FREQ,
    };

    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

static void led_initial_config() {
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_level(LED, 0);
}

static esp_err_t mpu6050_init() {
    // Escrever 0 no registrador de power management para acordar o sensor
    uint8_t data = 0x00;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        printf("MPU6050 inicializado\n");
    }
    return ret;
}

static esp_err_t mpu6050_read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t data[6];
    
    // Ler 6 bytes começando do registrador ACCEL_XOUT_H
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        // Combinar bytes high e low de cada eixo
        *ax = (int16_t)((data[0] << 8) | data[1]);
        *ay = (int16_t)((data[2] << 8) | data[3]);
        *az = (int16_t)((data[4] << 8) | data[5]);
    }
    
    return ret;
}

static esp_err_t ssd1306_init() {
    // Sequência de comandos de inicialização do SSD1306
    uint8_t init_cmds[] = {
        0xAE,       // Display OFF
        0xD5, 0x80, // Set clock divide ratio
        0xA8, 0x3F, // Set multiplex ratio
        0xD3, 0x00, // Set display offset
        0x40,       // Set start line
        0x8D, 0x14, // Enable charge pump
        0x20, 0x00, // Memory addressing mode
        0xA1,       // Set segment remap
        0xC8,       // Set COM output scan direction
        0xDA, 0x12, // Set COM pins hardware configuration
        0x81, 0xCF, // Set contrast control
        0xD9, 0xF1, // Set pre-charge period
        0xDB, 0x40, // Set VCOMH deselect level
        0xA4,       // Display follows RAM content
        0xA6,       // Normal display (not inverted)
        0xAF        // Display ON
    };
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SSD1306_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // Co = 0, D/C = 0 (command mode)
    for (int i = 0; i < sizeof(init_cmds); i++) {
        i2c_master_write_byte(cmd, init_cmds[i], true);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        printf("SSD1306 inicializado\n");
        ssd1306_clear();
        ssd1306_display();
    }
    
    return ret;
}

static void ssd1306_clear() {
    memset(ssd1306_buffer, 0, sizeof(ssd1306_buffer));
}

static void ssd1306_display() {
    // Display dividido em 8 páginas de 8 pixels de altura
    for (uint8_t page = 0; page < 8; page++) {
        // Configurar posição
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SSD1306_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x00, true); // Command mode
        i2c_master_write_byte(cmd, 0xB0 + page, true); // Set page
        i2c_master_write_byte(cmd, 0x00, true); // Set lower column
        i2c_master_write_byte(cmd, 0x10, true); // Set higher column
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        // Enviar dados da página
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SSD1306_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x40, true); // Data mode
        i2c_master_write(cmd, &ssd1306_buffer[SSD1306_WIDTH * page], SSD1306_WIDTH, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    }
}

static void ssd1306_draw_char(uint8_t x, uint8_t y, char c, uint8_t size) {
    int index = -1;
    
    // Mapear caractere para índice na tabela de font
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
        // Desenhar cada pixel do caractere com escala
        for (int i = 0; i < 8; i++) {
            for (int j = 0; j < 8; j++) {
                if (font8x8[index][i] & (1 << j)) {
                    // Aplicar escala (size x size pixels)
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

static void calculate_average(accel_data_t *avg) {
    avg->x = 0;
    avg->y = 0;
    avg->z = 0;
    
    int count = (buffer_count < BUFFER_SIZE) ? buffer_count : BUFFER_SIZE;
    for (int i = 0; i < count; i++) {
        avg->x += accel_buffer[i].x;
        avg->y += accel_buffer[i].y;
        avg->z += accel_buffer[i].z;
    }
    
    if (count > 0) {
        avg->x /= count;
        avg->y /= count;
        avg->z /= count;
    }
}

static bool check_acceleration_change(accel_data_t current) {
    // Calcular variação em relação à última leitura
    float delta_x = fabs(current.x - last_accel.x);
    float delta_y = fabs(current.y - last_accel.y);
    float delta_z = fabs(current.z - last_accel.z);
    
    // Retorna true se qualquer eixo teve variação >= threshold
    return (delta_x >= ACCEL_THRESHOLD || delta_y >= ACCEL_THRESHOLD || delta_z >= ACCEL_THRESHOLD);
}