#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <generated/csr.h>
#include <generated/soc.h>
#include <system.h>
#include <irq.h>
#include <uart.h>
#include <console.h>

#include "bh1750.h"   // Agora contém I2C + BH1750
#include "lora_RFM95.h"

// ------------------------------
// Utils de tempo
// ------------------------------
static void busy_wait_ms(unsigned int ms) {
    for (unsigned int i = 0; i < ms; ++i) {
#ifdef CSR_TIMER0_BASE
        busy_wait_us(1000);
#else
        for (volatile int j = 0; j < 1000; j++);
#endif
    }
}

// ------------------------------
// Console (mantenha igual)
// ------------------------------
static char *readstr(void) {
    char c[2];
    static char s[64];
    static int ptr = 0;

    if(readchar_nonblock()) {
        c[0] = readchar();
        c[1] = 0;
        switch(c[0]) {
            case 0x7f: case 0x08:
                if(ptr > 0) { ptr--; putsnonl("\x08 \x08"); }
                break;
            case 0x07:
                break;
            case '\r': case '\n':
                s[ptr] = 0x00; putsnonl("\n"); ptr = 0; return s;
            default:
                if(ptr >= (sizeof(s)-1)) break;
                putsnonl(c); s[ptr++] = c[0];
                break;
        }
    }
    return NULL;
}

static char *get_token(char **str) {
    char *c = strchr(*str, ' ');
    char *d;
    if(c == NULL) { d = *str; *str = *str + strlen(*str); return d; }
    *c = 0; d = *str; *str = c+1; return d;
}

static void prompt(void) { printf("RUNTIME>"); }

static void help(void) {
    puts("Available commands:");
    puts("help        - this command");
    puts("reboot      - reboot CPU");
    puts("led         - led test");
    puts("enviar      - ler BH1750 e enviar via LoRa");
    puts("info_LoRa   - informações do módulo LoRa");
    puts("scan_i2c    - escanear barramento I2C");
}

static void reboot(void) { ctrl_reset_write(1); }

static void toggle_led(void) {
    int i = leds_out_read();
    leds_out_write(!i);
    printf("LED invertido.\n");
}

// ------------------------------
// Enviar dados BH1750 via LoRa
// ------------------------------
static void send_sensor_data(void) {
    bh1750_dados luz;

    printf("Lendo BH1750...\n");
    if(bh1750_get_data(&luz)) {
        printf("Luminosidade: %u.%02u lux\n", luz.luminosidade/100, luz.luminosidade%100);

        if(!lora_send_bytes((uint8_t*)&luz, sizeof(luz))) {
            printf("Falha no envio LoRa.\n");
        } else {
            printf("Dados BH1750 enviados via LoRa.\n");
        }
    } else {
        printf("Falha ao ler BH1750.\n");
    }
}

static void lorainfo(void) {
    uint8_t version = lora_read_reg(0x42);
    printf("LoRa Version: 0x%02X\n", version);
}

// ------------------------------
// Serviço do console
// ------------------------------
static void console_service(void) {
    char *str = readstr();
    if(!str) return;
    char *token = get_token(&str);

    if(strcmp(token, "help") == 0) help();
    else if(strcmp(token, "reboot") == 0) reboot();
    else if(strcmp(token, "led") == 0) toggle_led();
    else if(strcmp(token, "enviar") == 0) send_sensor_data();
    else if(strcmp(token, "info_LoRa") == 0) lorainfo();
    else if(strcmp(token, "scan_i2c") == 0) i2c_scan();
    else puts("Comando desconhecido. Digite 'help'.");

    prompt();
}

// ------------------------------
// main
// ------------------------------
int main(void) {
#ifdef CONFIG_CPU_HAS_INTERRUPT
    irq_setmask(0);
    irq_setie(1);
#endif

    uart_init();
    busy_wait_ms(500);

    printf("Hello World!\n");
    printf("Tarefa – Transmissão de dados BH1750 via LoRa\n");

    // Inicializa I2C UMA VEZ
    i2c_init();
    
    // Inicializa sensores
    if(bh1750_init() != 0) {
        printf("Falha ao inicializar BH1750.\n");
    } else {
        printf("BH1750 inicializado com sucesso.\n");
    }
    
    // Inicializa LoRa UMA VEZ
    if(!lora_init()) {
        printf("Falha ao inicializar LoRa.\n");
    } else {
        printf("LoRa inicializado com sucesso.\n");
    }

    help();
    prompt();

    while(1) console_service();

    return 0;
}
