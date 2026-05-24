/**
  ******************************************************************************
  * @file    lora.c
  * @brief   SX1276/SX1278 LoRa PHY driver (register-level).
  *
  * References: Semtech SX1276/77/78/79 datasheet rev 7, §6 (LoRa registers).
  ******************************************************************************
  */
#include "lora.h"
#include <string.h>

/* ---- SX127x register map (LoRa mode) ------------------------------------ */
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0B
#define REG_LNA                  0x0C
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1A
#define REG_PKT_SNR_VALUE        0x1B
#define REG_MODEM_CONFIG_1       0x1D
#define REG_MODEM_CONFIG_2       0x1E
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_DETECT_OPTIMIZE      0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4D

#define SX127X_VERSION           0x12

/* OP_MODE bits */
#define OPMODE_LONG_RANGE        0x80   /* LoRa mode (write only in sleep)  */
#define OPMODE_SLEEP             0x00
#define OPMODE_STANDBY           0x01
#define OPMODE_TX                0x03
#define OPMODE_RX_CONTINUOUS     0x05

/* IRQ flags (REG_IRQ_FLAGS, write 1 to clear) */
#define IRQ_RX_DONE              0x40
#define IRQ_PAYLOAD_CRC_ERROR    0x20
#define IRQ_TX_DONE              0x08

#define FXOSC_HZ                 32000000u

/* ---- module state -------------------------------------------------------- */

static lora_config_t s_cfg;
static volatile uint8_t s_tx_done = 0;
static volatile uint8_t s_rx_done = 0;

/* ---- low-level SPI helpers ---------------------------------------------- */

static inline void cs_low (void) { HAL_GPIO_WritePin(s_cfg.nss_port, s_cfg.nss_pin, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(s_cfg.nss_port, s_cfg.nss_pin, GPIO_PIN_SET);   }

static lora_status_t reg_write(uint8_t addr, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(addr | 0x80), val };
    cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(s_cfg.hspi, tx, 2, HAL_MAX_DELAY);
    cs_high();
    return (st == HAL_OK) ? LORA_OK : LORA_ERR_SPI;
}

static uint8_t reg_read(uint8_t addr)
{
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0x00 };
    uint8_t rx[2] = { 0 };
    cs_low();
    HAL_SPI_TransmitReceive(s_cfg.hspi, tx, rx, 2, HAL_MAX_DELAY);
    cs_high();
    return rx[1];
}

static void burst_write(uint8_t addr, const uint8_t *buf, size_t len)
{
    uint8_t a = (uint8_t)(addr | 0x80);
    cs_low();
    HAL_SPI_Transmit(s_cfg.hspi, &a, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(s_cfg.hspi, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
    cs_high();
}

static void burst_read(uint8_t addr, uint8_t *buf, size_t len)
{
    uint8_t a = (uint8_t)(addr & 0x7F);
    cs_low();
    HAL_SPI_Transmit(s_cfg.hspi, &a, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(s_cfg.hspi, buf, (uint16_t)len, HAL_MAX_DELAY);
    cs_high();
}

/* ---- mode helpers -------------------------------------------------------- */

static void set_mode(uint8_t mode_bits)
{
    reg_write(REG_OP_MODE, OPMODE_LONG_RANGE | mode_bits);
}

static void hw_reset(void)
{
    HAL_GPIO_WritePin(s_cfg.rst_port, s_cfg.rst_pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(s_cfg.rst_port, s_cfg.rst_pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

/* ---- public API ---------------------------------------------------------- */

lora_status_t lora_init(const lora_config_t *cfg)
{
    if (!cfg || !cfg->hspi) return LORA_ERR_PARAM;
    s_cfg = *cfg;
    cs_high();

    hw_reset();

    /* Enter sleep so we can flip the LongRangeMode bit, then standby. */
    reg_write(REG_OP_MODE, OPMODE_SLEEP);
    HAL_Delay(1);
    reg_write(REG_OP_MODE, OPMODE_LONG_RANGE | OPMODE_SLEEP);
    HAL_Delay(1);

    if (reg_read(REG_VERSION) != SX127X_VERSION) {
        return LORA_ERR_NODEV;
    }

    /* Reasonable defaults — caller will tune as needed. */
    reg_write(REG_FIFO_TX_BASE_ADDR, 0x00);
    reg_write(REG_FIFO_RX_BASE_ADDR, 0x00);

    /* LNA: max gain, boost on HF (>525MHz). For 433MHz this still works. */
    reg_write(REG_LNA, (reg_read(REG_LNA) & 0x03) | 0x20);

    /* AGC auto-on (ModemConfig3 bit 2). LowDataRateOptimize off by default. */
    reg_write(REG_MODEM_CONFIG_3, 0x04);

    /* 8-symbol preamble (default is 8, set explicitly for clarity). */
    reg_write(REG_PREAMBLE_MSB, 0x00);
    reg_write(REG_PREAMBLE_LSB, 0x08);

    set_mode(OPMODE_STANDBY);
    return LORA_OK;
}

lora_status_t lora_set_frequency(uint32_t hz)
{
    /* Frf = (hz * 2^19) / 32_000_000. Compute carefully to avoid 32-bit overflow. */
    uint64_t frf = ((uint64_t)hz << 19) / FXOSC_HZ;
    reg_write(REG_FRF_MSB, (uint8_t)(frf >> 16));
    reg_write(REG_FRF_MID, (uint8_t)(frf >> 8));
    reg_write(REG_FRF_LSB, (uint8_t)(frf));
    return LORA_OK;
}

lora_status_t lora_set_spreading_factor(uint8_t sf)
{
    if (sf < 6 || sf > 12) return LORA_ERR_PARAM;

    /* SF6 needs implicit-header mode and special detection thresholds. */
    if (sf == 6) {
        reg_write(REG_DETECT_OPTIMIZE,     0xC5);
        reg_write(REG_DETECTION_THRESHOLD, 0x0C);
    } else {
        reg_write(REG_DETECT_OPTIMIZE,     0xC3);
        reg_write(REG_DETECTION_THRESHOLD, 0x0A);
    }

    uint8_t mc2 = reg_read(REG_MODEM_CONFIG_2);
    mc2 = (uint8_t)((mc2 & 0x0F) | ((sf & 0x0F) << 4));
    mc2 |= 0x04;   /* RxPayloadCrcOn = 1 */
    reg_write(REG_MODEM_CONFIG_2, mc2);
    return LORA_OK;
}

lora_status_t lora_set_bandwidth_125khz(void)
{
    /* ModemConfig1: bw[7:4]=0111 (125kHz), cr[3:1] unchanged, implicit[0]=0 */
    uint8_t mc1 = reg_read(REG_MODEM_CONFIG_1);
    mc1 = (uint8_t)((mc1 & 0x0F) | (0x07 << 4));
    mc1 &= (uint8_t)~0x01;   /* explicit header */
    reg_write(REG_MODEM_CONFIG_1, mc1);
    return LORA_OK;
}

lora_status_t lora_set_coding_rate_4_5(void)
{
    uint8_t mc1 = reg_read(REG_MODEM_CONFIG_1);
    mc1 = (uint8_t)((mc1 & 0xF1) | (0x01 << 1));   /* 001 = 4/5 */
    reg_write(REG_MODEM_CONFIG_1, mc1);
    return LORA_OK;
}

lora_status_t lora_set_tx_power(int8_t dbm)
{
    /* Use PA_BOOST pin (most Ai-Thinker SX1278 modules have only PA_BOOST wired).
     * With PaDac default (0x84): Pout = 17 - (15 - OutputPower), range 2..17 dBm. */
    if (dbm < 2)  dbm = 2;
    if (dbm > 17) dbm = 17;
    reg_write(REG_PA_DAC,    0x84);
    reg_write(REG_PA_CONFIG, (uint8_t)(0x80 | (dbm - 2)));   /* PaSelect=1, OutputPower */
    return LORA_OK;
}

lora_status_t lora_set_sync_word(uint8_t sw)
{
    reg_write(REG_SYNC_WORD, sw);
    return LORA_OK;
}

lora_status_t lora_send(const uint8_t *buf, size_t len)
{
    if (!buf || len == 0 || len > 255) return LORA_ERR_PARAM;

    set_mode(OPMODE_STANDBY);

    /* Map DIO0 -> TxDone (bits 7:6 = 01). */
    reg_write(REG_DIO_MAPPING_1, 0x40);

    /* Clear any pending IRQs. */
    reg_write(REG_IRQ_FLAGS, 0xFF);

    reg_write(REG_FIFO_TX_BASE_ADDR, 0x00);
    reg_write(REG_FIFO_ADDR_PTR,     0x00);
    burst_write(REG_FIFO, buf, len);
    reg_write(REG_PAYLOAD_LENGTH, (uint8_t)len);

    s_tx_done = 0;
    set_mode(OPMODE_TX);

    /* Wait for TxDone. ~100ms is plenty for a 24-byte payload at SF7/125kHz
     * (airtime ≈ 56ms). Fall back to polling IRQ flags in case the EXTI line
     * is not wired or the ISR is missed. */
    uint32_t start = HAL_GetTick();
    while (!s_tx_done) {
        if ((reg_read(REG_IRQ_FLAGS) & IRQ_TX_DONE) != 0) {
            reg_write(REG_IRQ_FLAGS, IRQ_TX_DONE);
            s_tx_done = 1;
            break;
        }
        if ((HAL_GetTick() - start) > 1000) {
            set_mode(OPMODE_STANDBY);
            return LORA_ERR_TIMEOUT;
        }
    }

    set_mode(OPMODE_STANDBY);
    return LORA_OK;
}

lora_status_t lora_receive_continuous(void)
{
    set_mode(OPMODE_STANDBY);
    /* Map DIO0 -> RxDone (bits 7:6 = 00). */
    reg_write(REG_DIO_MAPPING_1, 0x00);
    reg_write(REG_IRQ_FLAGS, 0xFF);
    reg_write(REG_FIFO_RX_BASE_ADDR, 0x00);
    reg_write(REG_FIFO_ADDR_PTR,     0x00);
    s_rx_done = 0;
    set_mode(OPMODE_RX_CONTINUOUS);
    return LORA_OK;
}

int lora_read_packet(uint8_t *out, size_t out_max)
{
    uint8_t flags = reg_read(REG_IRQ_FLAGS);
    reg_write(REG_IRQ_FLAGS, flags);   /* clear */

    if ((flags & IRQ_PAYLOAD_CRC_ERROR) != 0) return LORA_ERR_SPI;
    if ((flags & IRQ_RX_DONE)           == 0) return 0;

    uint8_t n  = reg_read(REG_RX_NB_BYTES);
    uint8_t rx = reg_read(REG_FIFO_RX_CURRENT_ADDR);
    reg_write(REG_FIFO_ADDR_PTR, rx);
    if (n > out_max) n = (uint8_t)out_max;
    burst_read(REG_FIFO, out, n);
    s_rx_done = 0;
    return n;
}

void lora_handle_dio0_irq(void)
{
    /* ISR context — keep it short. Flag is cleared by the foreground handler
     * (lora_send polls / lora_read_packet drains). */
    uint8_t f = reg_read(REG_IRQ_FLAGS);
    if (f & IRQ_TX_DONE) {
        reg_write(REG_IRQ_FLAGS, IRQ_TX_DONE);
        s_tx_done = 1;
    }
    if (f & IRQ_RX_DONE) {
        s_rx_done = 1;   /* foreground will drain the FIFO and clear the flag */
    }
}
