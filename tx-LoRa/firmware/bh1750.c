#include "bh1750.h"
#include <generated/csr.h>
#include <system.h> // busy_wait_us
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

// ============================================
// === Utils de tempo ===
// ============================================
static void busy_wait_ms(unsigned int ms) {
    for (unsigned int i = 0; i < ms; ++i) {
#ifdef CSR_TIMER0_BASE
        busy_wait_us(1000);
#else
        for (volatile int j = 0; j < 1000; j++);
#endif
    }
}

// ============================================
// === I2C Driver (Bitbang com CSR) ===
// ============================================

static uint32_t i2c_w_reg = 0;

static void i2c_delay(void) { busy_wait_us(5); }

static void i2c_set_scl(int val) {
    if (val) i2c_w_reg |= (1 << CSR_I2C_W_SCL_OFFSET);
    else     i2c_w_reg &= ~(1 << CSR_I2C_W_SCL_OFFSET);
    i2c_w_write(i2c_w_reg);
}

static void i2c_set_sda(int val) {
    if (val) i2c_w_reg |= (1 << CSR_I2C_W_SDA_OFFSET);
    else     i2c_w_reg &= ~(1 << CSR_I2C_W_SDA_OFFSET);
    i2c_w_write(i2c_w_reg);
}

static void i2c_set_oe(int val) {
    if (val) i2c_w_reg |= (1 << CSR_I2C_W_OE_OFFSET);
    else     i2c_w_reg &= ~(1 << CSR_I2C_W_OE_OFFSET);
    i2c_w_write(i2c_w_reg);
}

static int i2c_read_sda(void) {
    return (i2c_r_read() & (1 << CSR_I2C_R_SDA_OFFSET)) != 0;
}

// --- Funções Públicas I2C ---
void i2c_init(void) {
    i2c_set_oe(1); i2c_set_scl(1); i2c_set_sda(1);
    busy_wait_ms(1);
}

// --- Funções Internas I2C (static) ---
static void i2c_start(void) {
    i2c_set_sda(1); i2c_set_oe(1); i2c_set_scl(1); i2c_delay();
    i2c_set_sda(0); i2c_delay();
    i2c_set_scl(0); i2c_delay();
}

static void i2c_stop(void) {
    i2c_set_sda(0); i2c_set_oe(1); i2c_set_scl(0); i2c_delay();
    i2c_set_scl(1); i2c_delay();
    i2c_set_sda(1); i2c_delay();
}

static bool i2c_write_byte(uint8_t byte) {
    int i; bool ack;
    i2c_set_oe(1);
    for (i = 0; i < 8; i++) {
        i2c_set_sda((byte & 0x80) != 0); i2c_delay();
        i2c_set_scl(1); i2c_delay();
        i2c_set_scl(0); i2c_delay();
        byte <<= 1;
    }
    i2c_set_oe(0); i2c_set_sda(1); i2c_delay();
    i2c_set_scl(1); i2c_delay();
    ack = !i2c_read_sda();
    i2c_set_scl(0); i2c_delay();
    return ack;
}

static uint8_t i2c_read_byte(bool send_ack) {
    int i; uint8_t byte = 0;
    i2c_set_oe(0); i2c_set_sda(1); i2c_delay();
    for (i = 0; i < 8; i++) {
        byte <<= 1;
        i2c_set_scl(1); i2c_delay();
        if (i2c_read_sda()) byte |= 1;
        i2c_set_scl(0); i2c_delay();
    }
    i2c_set_oe(1); i2c_set_sda(!send_ack); i2c_delay();
    i2c_set_scl(1); i2c_delay();
    i2c_set_scl(0); i2c_delay();
    return byte;
}

// --- Função Pública I2C Scan ---
void i2c_scan(void) {
    printf("Escaneando barramento I2C...\n");
    for (uint8_t addr = 1; addr < 128; addr++) {
        i2c_start();
        if (i2c_write_byte(addr << 1 | 0)) {
            printf("  Dispositivo encontrado em 0x%02X\n", addr);
        }
        i2c_stop();
        busy_wait_us(100);
    }
    printf("Scan completo.\n");
}

// ======================================================
// BH1750
// ======================================================
#define BH1750_I2C_ADDR       0x23
#define BH1750_POWER_ON       0x01
#define BH1750_CONT_HRES_MODE 0x10
#define BH1750_ONE_TIME_HRES_MODE 0x20

int bh1750_init(void) {
    i2c_start();
    if (!i2c_write_byte(BH1750_I2C_ADDR << 1 | 0)) { i2c_stop(); return -1; }
    if (!i2c_write_byte(BH1750_POWER_ON)) { i2c_stop(); return -1; }
    i2c_stop();
    busy_wait_ms(10);
    
    // Configurar modo contínuo de alta resolução
    i2c_start();
    if (!i2c_write_byte(BH1750_I2C_ADDR << 1 | 0)) { i2c_stop(); return -1; }
    if (!i2c_write_byte(BH1750_CONT_HRES_MODE)) { i2c_stop(); return -1; }
    i2c_stop();
    busy_wait_ms(180); // Esperar primeira medição
    
    return 0;
}

bool bh1750_get_data(bh1750_dados *d) {
    uint8_t data[2];
    uint16_t raw;

    // Ler dados (modo contínuo já está configurado)
    i2c_start();
    if (!i2c_write_byte(BH1750_I2C_ADDR << 1 | 1)) { i2c_stop(); return false; }

    data[0] = i2c_read_byte(true);
    data[1] = i2c_read_byte(false);
    i2c_stop();

    raw = ((uint16_t)data[0] << 8) | data[1];
    if (raw == 0xFFFF || raw == 0x0000) return false;

    // Conversão: lux = raw / 1.2 → multiplicado por 100 (sem float)
    // raw * 100 / 1.2 = raw * 1000 / 12 = raw * 250 / 3
    d->luminosidade = (uint16_t)((raw * 250) / 3);
    
    return true;
}
