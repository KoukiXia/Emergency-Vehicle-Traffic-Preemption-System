/**
  ******************************************************************************
  * @file    packet.h
  * @brief   Emergency Vehicle Traffic Preemption — packet format & HMAC API.
  *
  * Wire format is a fixed 24-byte struct, little-endian (Cortex-M4 native).
  * The trailing 8 bytes are a truncated HMAC-SHA256 over the preceding 16
  * bytes of the packet, keyed with a shared secret. The intersection node
  * recomputes the tag and silently drops mismatches (FR7).
  ******************************************************************************
  */
#ifndef __PACKET_H
#define __PACKET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* Magic distinguishes our frames from random LoRa traffic that survives the
 * sync-word filter. ASCII "GW18" → 0x47573138, little-endian on the wire. */
#define EMERGENCY_PACKET_MAGIC   0x47573138u

#define PACKET_HMAC_LEN          8u   /* truncated HMAC-SHA256 */

typedef enum {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
} approach_direction_t;

#if defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM) || defined(__ARMCC_VERSION)
#  define PACKET_PACKED __attribute__((packed))
#else
#  define PACKET_PACKED
#endif

typedef struct PACKET_PACKED {
    uint32_t magic;            /* EMERGENCY_PACKET_MAGIC                      */
    uint8_t  ambulance_id;     /* per-vehicle ID, provisioned at flash time   */
    uint8_t  direction;        /* approach_direction_t                        */
    uint8_t  flags;            /* bit 0: button currently pressed             */
    uint8_t  reserved;         /* keep struct 4-byte aligned, must be 0       */
    uint32_t sequence;         /* monotonic, replay-protection                */
    uint32_t timestamp_ms;     /* HAL_GetTick() at TX, for latency telemetry  */
    uint8_t  hmac[PACKET_HMAC_LEN];
} emergency_packet_t;

/* sizeof(emergency_packet_t) == 24, checked at compile time */
typedef char _packet_size_check[(sizeof(emergency_packet_t) == 24) ? 1 : -1];

/**
  * @brief  Compute and write the HMAC-SHA256 tag into pkt->hmac.
  *
  * The tag covers every byte of pkt except the hmac field itself. The caller
  * must populate all other fields (magic, ambulance_id, direction, flags,
  * reserved, sequence, timestamp_ms) before calling this function.
  *
  * @param  pkt      packet to sign (modified in place)
  * @param  key      shared secret
  * @param  key_len  shared-secret length in bytes
  */
void packet_sign(emergency_packet_t *pkt, const uint8_t *key, size_t key_len);

/**
  * @brief  Recompute the tag and compare in constant time.
  * @retval 1 if tag matches, 0 otherwise.
  */
int packet_verify(const emergency_packet_t *pkt, const uint8_t *key, size_t key_len);

#ifdef __cplusplus
}
#endif

#endif /* __PACKET_H */
