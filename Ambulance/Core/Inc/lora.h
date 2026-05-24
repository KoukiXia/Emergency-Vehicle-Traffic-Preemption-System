/**
  ******************************************************************************
  * @file    lora.h
  * @brief   Minimal SX1276/SX1278 LoRa driver for the ambulance preemption link.
  *
  * Register-level driver — no LoRaWAN, no MAC. We only need raw LoRa PHY for
  * a one-way short-burst V2I link. The ambulance side is TX-only; the RX
  * path is included so the same driver can be reused on the intersection node.
  ******************************************************************************
  */
#ifndef __LORA_H
#define __LORA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

typedef enum {
    LORA_OK         = 0,
    LORA_ERR_SPI    = -1,
    LORA_ERR_NODEV  = -2,   /* RegVersion did not return 0x12 */
    LORA_ERR_PARAM  = -3,
    LORA_ERR_BUSY   = -4,
    LORA_ERR_TIMEOUT= -5
} lora_status_t;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *nss_port;   uint16_t nss_pin;
    GPIO_TypeDef      *rst_port;   uint16_t rst_pin;
} lora_config_t;

/* ---- bring-up & configuration -------------------------------------------- */

lora_status_t lora_init(const lora_config_t *cfg);

lora_status_t lora_set_frequency(uint32_t hz);            /* e.g. 433000000 */
lora_status_t lora_set_spreading_factor(uint8_t sf);      /* 6..12 */
lora_status_t lora_set_bandwidth_125khz(void);            /* hard-coded helper */
lora_status_t lora_set_coding_rate_4_5(void);             /* hard-coded helper */
lora_status_t lora_set_tx_power(int8_t dbm);              /* -1..+20 (PA_BOOST) */
lora_status_t lora_set_sync_word(uint8_t sw);             /* private network filter */

/* ---- TX / RX ------------------------------------------------------------- */

/**
  * @brief  Blocking transmit. Returns once TxDone is asserted (or timeout).
  *         Safe to call from main-loop context.
  */
lora_status_t lora_send(const uint8_t *buf, size_t len);

/**
  * @brief  Place the radio in continuous-RX mode.
  *         A subsequent DIO0 IRQ signals RxDone — call lora_read_packet().
  */
lora_status_t lora_receive_continuous(void);

/**
  * @brief  Pull the most recently received packet from the FIFO.
  * @retval number of bytes copied, or negative lora_status_t on error.
  */
int lora_read_packet(uint8_t *out, size_t out_max);

/* ---- IRQ glue ------------------------------------------------------------ */

/**
  * @brief  Call from HAL_GPIO_EXTI_Callback when DIO0 fires. Clears IRQ flags
  *         and updates internal TX/RX-done state.
  */
void lora_handle_dio0_irq(void);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_H */
