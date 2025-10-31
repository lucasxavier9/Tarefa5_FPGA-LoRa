#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include <string.h>
#include <stdbool.h>
#include "inc/lora_RFM95.h"
#include "inc/ssd1306.h"

// =====================
// Definições de SPI (LoRa)
// =====================
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// =====================
// Definições de I2C (OLED + BH1750)
// =====================
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15

// =====================
// Pinagem do módulo LoRa
// =====================
#define PIN_RST 20
#define PIN_DIO0 8
#define LORA_FREQUENCY 915E6

ssd1306_t disp;
#include "blink.pio.h"

#define SEND_INTERVAL_MS 10000  // Intervalo de envio (10 segundos)

// =====================
// Estrutura de dados recebidos via LoRa
// =====================
typedef struct {
    uint16_t iluminancia; // Lux * 100 (para evitar float)
} bh1750_dados;

// =====================
// Funções auxiliares de texto no OLED
// =====================
void print_texto(char *msg, uint pos_x, uint pos_y, uint scale) {
    ssd1306_draw_string(&disp, pos_x, pos_y, scale, (uint8_t*)msg);
}

static int text_width_px(const char* s, int scale) {
    return (int)strlen(s) * 6 * (scale > 0 ? scale : 1);
}

static int center_x_for(const char* s, int scale) {
    int w = text_width_px(s, scale);
    int x = (128 - w) / 2;
    return x < 0 ? 0 : x;
}

static void print_texto_centered(const char* msg, int y, int scale) {
    int x = center_x_for(msg, scale);
    ssd1306_draw_string(&disp, x, y, (uint)scale, (const uint8_t*)msg);
}

// =====================
// Mostra iluminância (Lux) no display OLED
// =====================
static void show_lux(float lux) {
    char line[32];
    ssd1306_clear(&disp);
    int y_top = (64 - 16) / 2;
    snprintf(line, sizeof(line), "%.1f Lux", lux);
    print_texto_centered(line, y_top, 2);
    ssd1306_show(&disp);
}

// =====================
// Programa principal
// =====================
int main() {
    stdio_init_all();

    // Inicializa I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL);
    gpio_pull_up(I2C_SDA);

    disp.external_vcc = false;
    ssd1306_init(&disp, 128, 64, 0x3C, I2C_PORT);
    ssd1306_clear(&disp);
    print_texto_centered("Inicializando...", (64 - 8) / 2, 1);
    ssd1306_show(&disp);
    sleep_ms(1000);

    // Configura pino CS do SPI
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    // Mensagem inicial no OLED
    ssd1306_clear(&disp);
    print_texto_centered("Iniciando LoRa", 20, 1);
    char msg_freq[32];
    snprintf(msg_freq, sizeof(msg_freq), "Freq: %.0f MHz", LORA_FREQUENCY / 1e6);
    print_texto_centered(msg_freq, 36, 1);
    ssd1306_show(&disp);
    sleep_ms(2000);

    // Inicializa LoRa
    lora_config_t lora_cfg = {
        .spi_instance = SPI_PORT,
        .pin_miso = PIN_MISO,
        .pin_cs = PIN_CS,
        .pin_sck = PIN_SCK,
        .pin_mosi = PIN_MOSI,
        .pin_rst = PIN_RST,
        .pin_dio0 = PIN_DIO0,
        .frequency = LORA_FREQUENCY
    };

    printf("Inicializando módulo LoRa (%.0f Hz)...\n", (float)LORA_FREQUENCY);
    if (!lora_init(lora_cfg)) {
        printf("ERRO - Falha ao inicializar o módulo LoRa.\n");
        ssd1306_clear(&disp);
        print_texto_centered("Falha LoRa!", 20, 2);
        print_texto_centered("Verifique conexoes", 42, 1);
        ssd1306_show(&disp);
        while (true) sleep_ms(1000);
    } else {
        printf("SUCESSO - Módulo LoRa inicializado. RX contínuo.\n");
        ssd1306_clear(&disp);
        print_texto_centered("LoRa BH1750", 20, 1);
        print_texto_centered("Modo RX ativo", 36, 1);
        ssd1306_show(&disp);
        sleep_ms(1500);
        lora_start_rx_continuous();
    }

    uint8_t rxbuf[64];
    bool got_first_data = false;
    uint32_t anim_tick = 0;
    int dots = 1;

    // =====================
    // Loop principal
    // =====================
    while (true) {
        int len = lora_receive_bytes(rxbuf, sizeof(rxbuf));

        if (len == sizeof(bh1750_dados)) {
            bh1750_dados rec;
            memcpy(&rec, rxbuf, sizeof(rec));
            
            // CORREÇÃO: Dividir por 100 para obter o valor real em Lux
            float lux = (float)rec.iluminancia / 100.0f;
            
            show_lux(lux);
            got_first_data = true;
            int rssi = lora_get_rssi();
            printf("Recebido: %.1f Lux | RSSI=%d dBm\n", lux, rssi);
        } 
        else if (len > 0) {
            printf("LoRa recebeu %d bytes (brutos): ", len);
            for (int i = 0; i < len; ++i) printf("%02X ", rxbuf[i]);
            printf("\n");
        } 
        else {
            if (!got_first_data) {
                anim_tick++;
                if ((anim_tick % 3) == 0) {
                    char msg[32];
                    snprintf(msg, sizeof(msg), "Esperando dados%.*s", dots, "...");
                    ssd1306_clear(&disp);
                    print_texto_centered(msg, (64 - 8) / 2, 1);
                    ssd1306_show(&disp);
                    dots++;
                    if (dots > 3) dots = 1;
                }
            }
        }
        sleep_ms(100);
    }
}
