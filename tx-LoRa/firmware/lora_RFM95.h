// lora_RFM95.h
#ifndef LORA_RFM95_H_
#define LORA_RFM95_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h> // Para size_t

// ============================
// === Funções Públicas ===
// ============================

/**
 * @brief Inicializa o hardware SPI e o módulo LoRa SX1276/RFM95.
 * Deve ser chamada antes de qualquer outra função LoRa.
 * @return true se a inicialização foi bem-sucedida (versão do chip lida corretamente), false caso contrário.
 */
bool lora_init(void);

/**
 * @brief Envia um buffer de bytes via LoRa.
 * @param data Ponteiro para o buffer de dados a ser enviado.
 * @param len Número de bytes a serem enviados (máximo 255).
 * @return true se o pacote foi enviado com sucesso (TxDone recebido), false em caso de erro ou timeout.
 */
bool lora_send_bytes(const uint8_t *data, size_t len);

/**
 * @brief Coloca o rádio LoRa em um modo de operação específico.
 * (Ex: Sleep, Standby, TX, RX contínuo)
 * Use as macros de modo (MODE_SLEEP, MODE_STDBY, etc.) se definidas publicamente,
 * ou use os valores numéricos diretamente (0x00, 0x01, etc.).
 * @param mode O modo de operação desejado (e.g., 0x01 para Standby).
 */
void lora_set_mode(uint8_t mode);

/**
 * @brief Lê o valor de um registrador do módulo LoRa.
 * (Função de baixo nível, usar com cuidado).
 * @param reg O endereço do registrador (e.g., 0x42 para REG_VERSION).
 * @return O valor de 8 bits lido do registrador.
 */
uint8_t lora_read_reg(uint8_t reg);

/**
 * @brief Escreve um valor em um registrador do módulo LoRa.
 * (Função de baixo nível, usar com cuidado).
 * @param reg O endereço do registrador.
 * @param value O valor de 8 bits a ser escrito.
 */
void lora_write_reg(uint8_t reg, uint8_t value);


#endif // LORA_RFM95_H_