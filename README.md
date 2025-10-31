# Projeto: Transmissão de Dados com FPGA via LoRa – Sensor BH1750

## Descrição Geral

Este projeto implementa a **leitura do sensor de luminosidade BH1750** e a **transmissão dos dados via rádio LoRa (RFM95)** utilizando um **SoC RISC-V embarcado em FPGA** (placa **Colorlight i5**).  
O código foi desenvolvido em **C** e compilado utilizando a **OSS CAD Suite**, com integração ao ambiente **LiteX**.

---

## Requisitos

- FPGA 
- Módulo BH1750 conectado via I2C
- Módulo LoRa RFM95
- Toolchain RISC-V 
- OSS CAD Suite

---

## Estrutura do Projeto


---

## Funcionalidades

- Leitura contínua da **luminosidade ambiente (em lux)** via sensor BH1750.  
- Comunicação **I2C por bit-banging** utilizando os registradores CSR do LiteX.  
- Envio dos dados lidos via **LoRa RFM95**.  
- Interface de console via UART com comandos simples para controle e debug.

---

##  Comandos Disponíveis no Console

| Comando        | Descrição                                  |
|----------------|---------------------------------------------|
| `help`         | Exibe a lista de comandos disponíveis       |
| `enviar`       | Lê o sensor BH1750 e envia os dados via LoRa |
| `led`          | Alterna o estado do LED onboard             |
| `reboot`       | Reinicia o sistema                          |
| `info_LoRa`    | Mostra informações do módulo LoRa conectado |
| `scan_i2c`    | Varre o barramento I2C e imprime os endereços de dispositivos |

---

## Compilação e Geração do Binário

O projeto utiliza o **toolchain RISC-V** integrado à **OSS CAD Suite**.

### Etapas de compilação

```bash
python3 litex/colorlight_i5.py --board i9 --revision 7.2 --build 
```

O bitstream resultante será salvo em:

```
build/colorlight_i5/gateware/colorlight_i5.bit
```

## Gravação na FPGA

Para programar a FPGA Colorlight i5 com o bitstream gerado:

```bash
which openFPGALoader #para descobrir o caminho
```

```bash
/caminho/do/openFPGALoader -b colorlight-i5 build/colorlight_i5/gateware/colorlight_i5.bit
```

(O caminho do `openFPGALoader` deve apontar para o executável dentro da sua instalação do OSS CAD Suite.)

---

## Execução do Firmware

Com a FPGA já configurada com o bitstream, compile e embarque o firmware LiteX:

```bash
cd firmware/
make clean
make
litex_term /dev/ttyACM0 --kernel main.bin
```
(O caminho  `/dev/ttyACM0` deve apontar para o dispositivo serial (porta de comunicação) que o Linux cria)

dentro do terminal: litex> utilize o comando: `reboot` para inicializar com seu firmware

Durante a execução, o firmware:
1. Inicializa o barramento I2C e o sensor BH1750;
2. Lê os valores de luminosidade do sensor (luminosidade);
3. Envia os dados via LoRa RFM95 para outro nó/receptor;
4. Permite monitoramento e debug através do console (scan_i2c, info_LoRa);
5. Repete a leitura e envio periodicamente ou sob comando do usuário (enviar).
