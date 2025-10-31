#ifndef BH1750_H_
#define BH1750_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * Estrutura para armazenar dados do sensor BH1750.
 * Valores em lux multiplicados por 100 (para evitar float).
 */
typedef struct {
    uint16_t luminosidade; // iluminância em lux * 100
} bh1750_dados;

// ============================================
// === Protótipos I2C (antes no aht10.h) ===
// ============================================

/**
 * @brief Inicializa o driver I2C bitbang.
 * Deve ser chamada antes de qualquer outra função I2C ou BH1750.
 */
void i2c_init(void);

/**
 * @brief Varre o barramento I2C e imprime endereços de dispositivos encontrados.
 */
void i2c_scan(void);

// ============================================
// === Protótipos BH1750 ===
// ============================================

/**
 * Inicializa o sensor BH1750.
 * Retorna 0 em sucesso, -1 em falha.
 */
int bh1750_init(void);

/**
 * Lê os dados do BH1750 e preenche a estrutura bh1750_dados.
 * Retorna true se a leitura foi bem-sucedida, false caso contrário.
 */
bool bh1750_get_data(bh1750_dados *d);

#endif // BH1750_H_
