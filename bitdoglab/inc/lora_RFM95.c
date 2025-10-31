// lora_RFM95.c

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "lora_RFM95.h"

// DEFINIÇÕES E REGISTRADORES INTERNOS
// === MAPA DE REGISTRADORES DO MÓDULO RFM95 (MODO LORA) ===

#define REG_FIFO                 0x00 // Usado para ler e escrever dados no buffer (FIFO) de 256 bytes. [cite: 2177, 2407]
#define REG_OP_MODE              0x01 // Configura o modo de operação do rádio (Sleep, Standby, TX, RX) e seleciona entre LoRa ou FSK/OOK. [cite: 2177, 2411]
#define REG_FRF_MSB              0x06 // Byte mais significativo (MSB) da frequência da portadora de RF. [cite: 2177, 2412]
#define REG_FRF_MID              0x07 // Byte intermediário da frequência da portadora de RF. [cite: 2177, 2412]
#define REG_FRF_LSB              0x08 // Byte menos significativo (LSB) da frequência da portadora de RF. A escrita neste registrador ativa a mudança de frequência. [cite: 2177, 2414]
#define REG_PA_CONFIG            0x09 // Configura o amplificador de potência (PA), selecionando a saída (RFO ou PA_BOOST) e o nível de potência. [cite: 2177, 2416]
#define REG_LNA                  0x0C // Configura o amplificador de baixo ruído (LNA), ajustando o ganho e o modo boost. [cite: 2177, 2422]
#define REG_FIFO_ADDR_PTR        0x0D // Ponteiro de endereço para a leitura/escrita via SPI no buffer FIFO. [cite: 2177, 2425]
#define REG_FIFO_TX_BASE_ADDR    0x0E // Define o endereço inicial no FIFO para o buffer de transmissão. [cite: 2177, 2425]
#define REG_FIFO_RX_BASE_ADDR    0x0F // Define o endereço inicial no FIFO para o buffer de recepção. [cite: 2177, 2425]
#define REG_FIFO_RX_CURRENT_ADDR 0x10 // Contém o endereço inicial do último pacote recebido no buffer FIFO. [cite: 2177, 2425]
#define REG_IRQ_FLAGS_MASK       0x11 // Permite mascarar (desativar) interrupções específicas. Se um bit está em 1, a IRQ correspondente é ignorada. [cite: 2177, 2427]
#define REG_IRQ_FLAGS            0x12 // Contém as flags de status das interrupções (TxDone, RxDone, CrcError, etc.). A escrita de '1' em um bit limpa a flag correspondente. [cite: 2177, 2431]
#define REG_RX_NB_BYTES          0x13 // Indica o número de bytes de payload recebidos no último pacote. [cite: 2177, 2431]
#define REG_MODEM_CONFIG_1       0x1D // Configura parâmetros do modem: Largura de Banda (BW), Taxa de Codificação (CR) e Modo de Cabeçalho (Explícito/Implícito). [cite: 2182, 2444]
#define REG_MODEM_CONFIG_2       0x1E // Configura parâmetros do modem: Spreading Factor (SF) e ativa o CRC no payload. [cite: 2182, 2450]
#define REG_PREAMBLE_MSB         0x20 // Byte mais significativo (MSB) do comprimento do preâmbulo. [cite: 2182, 2452]
#define REG_PREAMBLE_LSB         0x21 // Byte menos significativo (LSB) do comprimento do preâmbulo. [cite: 2182, 2452]
#define REG_PAYLOAD_LENGTH       0x22 // Define o comprimento do payload. Usado em modo de cabeçalho implícito e para o pacote a ser transmitido. [cite: 2182, 2453]
#define REG_MODEM_CONFIG_3       0x26 // Configurações adicionais: Otimização para Baixa Taxa de Dados (LDO) e Controle de Ganho Automático (AGC). [cite: 2182, 2458]
#define REG_DIO_MAPPING_1        0x40 // Mapeia as funções dos pinos de interrupção digital DIO0 a DIO3 (ex: TxDone, RxDone). [cite: 2182, 871]
#define REG_VERSION              0x42 // Contém a versão do chip de silício. Útil para verificar a comunicação e identificar o hardware. [cite: 2182, 2313]
#define REG_PA_DAC               0x4D // Configurações do DAC do amplificador de potência, incluindo a ativação do modo de alta potência de +20dBm. [cite: 2187, 1930]
// MODOS
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05

// IRQ FLAGS
#define IRQ_TX_DONE_MASK         0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK         0x40

#define REG_PKT_RSSI_VALUE       0x1A // Contém o valor do RSSI do pacote mais recente.


// VARIÁVEIS PRIVADAS (STATIC)
static lora_config_t lora;
volatile static bool tx_done = false;
volatile static bool rx_done = false;
volatile static bool dio0_event = false;

// PROTÓTIPOS DE FUNÇÕES PRIVADAS
static void lora_reset();
static void lora_write_reg(uint8_t reg, uint8_t value);
static uint8_t lora_read_reg(uint8_t reg);
static void lora_write_fifo(const uint8_t *data, uint8_t len);
static void lora_read_fifo(uint8_t *data, uint8_t len);
static void lora_set_mode(uint8_t mode);
static void cs_select();
static void cs_deselect();
static void dio0_irq_handler(uint gpio, uint32_t events);
static void handle_dio0_events();

// IMPLEMENTAÇÃO DAS FUNÇÕES

// --- Funções Públicas ---

bool lora_init(lora_config_t config) {
    lora = config; // Copia a configuração para a variável estática

    // --- Inicialização do Hardware ---
    spi_init(lora.spi_instance, 5E6);
    gpio_set_function(lora.pin_miso, GPIO_FUNC_SPI);
    gpio_set_function(lora.pin_mosi, GPIO_FUNC_SPI);
    gpio_set_function(lora.pin_sck, GPIO_FUNC_SPI);
    gpio_init(lora.pin_cs); gpio_set_dir(lora.pin_cs, GPIO_OUT); gpio_put(lora.pin_cs, 1);
    gpio_init(lora.pin_rst); gpio_set_dir(lora.pin_rst, GPIO_OUT);
    
    gpio_init(lora.pin_dio0); gpio_set_dir(lora.pin_dio0, GPIO_IN);
    gpio_pull_down(lora.pin_dio0);
    gpio_set_irq_enabled_with_callback(lora.pin_dio0, GPIO_IRQ_EDGE_RISE, true, &dio0_irq_handler);

    lora_reset();
    
    lora_set_mode(MODE_SLEEP);
    lora_set_mode(MODE_STDBY);

    lora_write_reg(REG_IRQ_FLAGS, 0xFF); // Limpa todas as flags de IRQ
    
    uint64_t frf = ((uint64_t)lora.frequency << 19) / 32000000;
    lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));

    // Configurações para longo alcance e robustez
    lora_write_reg(REG_PA_CONFIG, 0xFF); // PaConfig: Max Power (+17dBm on PA_BOOST)
    lora_write_reg(REG_PA_DAC, 0x87); // PaDac: Ativa +20dBm
    lora_write_reg(REG_MODEM_CONFIG_1, 0x78); // ModemConfig1: BW 62.5kHz, CR 4/8
    lora_write_reg(REG_MODEM_CONFIG_2, 0xC4); // ModemConfig2: SF12, CRC on
    lora_write_reg(REG_MODEM_CONFIG_3, 0x0C); // ModemConfig3: LDO on, AGC on
    lora_write_reg(REG_PREAMBLE_MSB, 0x00);
    lora_write_reg(REG_PREAMBLE_LSB, 0x0C);

    lora_write_reg(0x0B, 0x37); // OCP default
    lora_write_reg(0x39, 0x12);

    // Outras configurações
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
    lora_write_reg(REG_LNA, 0x23); // LNA boost para RX
    lora_write_reg(REG_IRQ_FLAGS_MASK, 0x00); // Libera todas as IRQs
    lora_write_reg(REG_IRQ_FLAGS, 0xFF); // Limpa IRQs
    
    //lora_set_mode(MODE_STDBY);
    
    uint8_t version = lora_read_reg(REG_VERSION);
    return (version == 0x12);
}

bool lora_send(const char *msg) {
    if (strlen(msg) > 255) return false;

    lora_set_mode(MODE_STDBY); 
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    lora_write_fifo((const uint8_t*)msg, strlen(msg));
    lora_write_reg(REG_PAYLOAD_LENGTH, strlen(msg));

    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
    lora_write_reg(REG_DIO_MAPPING_1, 0x40); // DIO0 -> TxDone

    tx_done = false;
    lora_set_mode(MODE_TX);

    absolute_time_t start_time = get_absolute_time();
    while (!tx_done) {
        handle_dio0_events();
        if (absolute_time_diff_us(start_time, get_absolute_time()) > (TX_TIMEOUT_MS * 1000)) {
            lora_set_mode(MODE_STDBY); // Aborta TX
            return false; // Timeout
        }
        tight_loop_contents();
    }

    lora_set_mode(MODE_STDBY);
    return true;
}

int lora_receive(char *buf, size_t maxlen) {
    // Tenta tratar evento por interrupção
    handle_dio0_events();
    // Fallback: se DIO0 não estiver ligado, faça polling do registrador de IRQs
    if (!rx_done) {
        uint8_t irq_flags = lora_read_reg(REG_IRQ_FLAGS);
        if (irq_flags) {
            lora_write_reg(REG_IRQ_FLAGS, 0xFF); // limpa todas as flags
            if ((irq_flags & IRQ_RX_DONE_MASK) && !(irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK)) {
                rx_done = true;
            } else if (irq_flags & IRQ_TX_DONE_MASK) {
                tx_done = true;
            }
        }
    }
    if (!rx_done) return 0;
    rx_done = false;

    uint8_t len = lora_read_reg(REG_RX_NB_BYTES);
    if (len > maxlen - 1) {
        printf("[AVISO] Pacote de %u bytes truncado para %u.\n", len, (unsigned)(maxlen - 1));
        len = (uint8_t)(maxlen - 1);
    }

    uint8_t fifo_addr = lora_read_reg(REG_FIFO_RX_CURRENT_ADDR);
    lora_write_reg(REG_FIFO_ADDR_PTR, fifo_addr);

    lora_read_fifo((uint8_t*)buf, len);
    buf[len] = '\0';

    return len;
}

// <<< ADICIONAR IMPLEMENTAÇÃO DAS NOVAS FUNÇÕES >>>
bool lora_send_bytes(const uint8_t *data, size_t len) {
    if (len > 255) return false;

    lora_set_mode(MODE_STDBY);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    lora_write_fifo(data, len); // Usa a função existente de escrita no FIFO
    lora_write_reg(REG_PAYLOAD_LENGTH, len);

    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
    lora_write_reg(REG_DIO_MAPPING_1, 0x40); // DIO0 -> TxDone

    tx_done = false;
    lora_set_mode(MODE_TX);

    absolute_time_t start_time = get_absolute_time();
    while (!tx_done) {
        handle_dio0_events();
        if (absolute_time_diff_us(start_time, get_absolute_time()) > (TX_TIMEOUT_MS * 1000)) {
            lora_set_mode(MODE_STDBY);
            return false;
        }
        tight_loop_contents();
    }

    lora_set_mode(MODE_STDBY);
    return true;
}

int lora_receive_bytes(uint8_t *buf, size_t maxlen) {
    // Tenta tratar evento por interrupção
    handle_dio0_events();
    // Fallback: se DIO0 não estiver ligado, faça polling do registrador de IRQs
    if (!rx_done) {
        uint8_t irq_flags = lora_read_reg(REG_IRQ_FLAGS);
        if (irq_flags) {
            lora_write_reg(REG_IRQ_FLAGS, 0xFF); // limpa todas as flags
            if ((irq_flags & IRQ_RX_DONE_MASK) && !(irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK)) {
                rx_done = true;
            } else if (irq_flags & IRQ_TX_DONE_MASK) {
                tx_done = true;
            }
        }
    }
    if (!rx_done) return 0;
    rx_done = false;

    uint8_t len = lora_read_reg(REG_RX_NB_BYTES);
    if (len > maxlen) {
        printf("[AVISO] Pacote de %u bytes truncado para %u.\n", len, (unsigned)maxlen);
        len = (uint8_t)maxlen;
    }

    uint8_t fifo_addr = lora_read_reg(REG_FIFO_RX_CURRENT_ADDR);
    lora_write_reg(REG_FIFO_ADDR_PTR, fifo_addr);

    lora_read_fifo(buf, len); // Lê os bytes brutos

    return len;
}


void lora_start_rx_continuous(void) {
    lora_write_reg(REG_IRQ_FLAGS, 0xFF);
    lora_write_reg(REG_DIO_MAPPING_1, 0x00); // DIO0 -> RxDone
    lora_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    lora_set_mode(MODE_RX_CONTINUOUS);
}

// --- Funções Privadas ---

static void cs_select() { gpio_put(lora.pin_cs, 0); }
static void cs_deselect() { gpio_put(lora.pin_cs, 1); }

static void lora_reset() {
    gpio_put(lora.pin_rst, 0); sleep_ms(10);
    gpio_put(lora.pin_rst, 1); sleep_ms(10);
}

static void lora_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { (uint8_t)(reg | 0x80), value };
    cs_select();
    spi_write_blocking(lora.spi_instance, buf, 2);
    cs_deselect();
}

static uint8_t lora_read_reg(uint8_t reg) {
    uint8_t buf[2] = { reg & 0x7F, 0x00 };
    uint8_t rx[2];
    cs_select();
    spi_write_read_blocking(lora.spi_instance, buf, rx, 2);
    cs_deselect();
    return rx[1];
}

static void lora_write_fifo(const uint8_t *data, uint8_t len) {
    cs_select();
    uint8_t addr = REG_FIFO | 0x80;
    spi_write_blocking(lora.spi_instance, &addr, 1);
    spi_write_blocking(lora.spi_instance, data, len);
    cs_deselect();
}

static void lora_read_fifo(uint8_t *data, uint8_t len) {
    cs_select();
    uint8_t addr = REG_FIFO & 0x7F;
    spi_write_blocking(lora.spi_instance, &addr, 1);
    spi_read_blocking(lora.spi_instance, 0x00, data, len);
    cs_deselect();
}

static void lora_set_mode(uint8_t mode) {
    lora_write_reg(REG_OP_MODE, (0x80 | mode)); // Bit 7 (LongRangeMode) sempre deve ser 1
}

static void dio0_irq_handler(uint gpio, uint32_t events) {
    (void)gpio; (void)events;
    dio0_event = true;
}

static void handle_dio0_events() {
    if (!dio0_event) return;
    dio0_event = false;

    uint8_t irq_flags = lora_read_reg(REG_IRQ_FLAGS);
    lora_write_reg(REG_IRQ_FLAGS, 0xFF); // Limpa todas as flags escrevendo 1s

    if ((irq_flags & IRQ_RX_DONE_MASK) && !(irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK)) {
        rx_done = true;
    } else if (irq_flags & IRQ_TX_DONE_MASK) {
        tx_done = true;
    } else if (irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK) {
        printf("[LORA_LIB] Erro de CRC no pacote!\n");
    }
}



// <<< ADICIONE A IMPLEMENTAÇÃO DA NOVA FUNÇÃO AQUI >>>
int lora_get_rssi(void) {
    uint8_t rssi_raw = lora_read_reg(REG_PKT_RSSI_VALUE);
    // A fórmula para calcular o RSSI em dBm é RSSI = -157 + Rssi (para o frontend de HF)
    // Veja a seção 5.5.5 do datasheet do SX1276/7/8/9.
    return rssi_raw - 157;
}
