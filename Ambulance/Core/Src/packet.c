/**
  ******************************************************************************
  * @file    packet.c
  * @brief   HMAC-SHA256 packet authentication for the ambulance preemption link.
  *
  * Self-contained: bundles a small SHA-256 implementation so we don't pull in
  * mbedTLS for one tag. Tag is truncated to PACKET_HMAC_LEN bytes — the
  * sequence number and short link-time window keep the truncation safe.
  ******************************************************************************
  */
#include "packet.h"
#include <string.h>

/* ---------------- SHA-256 (FIPS 180-4) -------------------------------------*/

#define SHA256_BLOCK_BYTES  64
#define SHA256_DIGEST_BYTES 32

typedef struct {
    uint32_t state[8];
    uint64_t bitlen;
    uint32_t buflen;
    uint8_t  buf[SHA256_BLOCK_BYTES];
} sha256_ctx_t;

static const uint32_t K[64] = {
    0x428a2f98u,0x71374491u,0xb5c0fbcfu,0xe9b5dba5u,0x3956c25bu,0x59f111f1u,
    0x923f82a4u,0xab1c5ed5u,0xd807aa98u,0x12835b01u,0x243185beu,0x550c7dc3u,
    0x72be5d74u,0x80deb1feu,0x9bdc06a7u,0xc19bf174u,0xe49b69c1u,0xefbe4786u,
    0x0fc19dc6u,0x240ca1ccu,0x2de92c6fu,0x4a7484aau,0x5cb0a9dcu,0x76f988dau,
    0x983e5152u,0xa831c66du,0xb00327c8u,0xbf597fc7u,0xc6e00bf3u,0xd5a79147u,
    0x06ca6351u,0x14292967u,0x27b70a85u,0x2e1b2138u,0x4d2c6dfcu,0x53380d13u,
    0x650a7354u,0x766a0abbu,0x81c2c92eu,0x92722c85u,0xa2bfe8a1u,0xa81a664bu,
    0xc24b8b70u,0xc76c51a3u,0xd192e819u,0xd6990624u,0xf40e3585u,0x106aa070u,
    0x19a4c116u,0x1e376c08u,0x2748774cu,0x34b0bcb5u,0x391c0cb3u,0x4ed8aa4au,
    0x5b9cca4fu,0x682e6ff3u,0x748f82eeu,0x78a5636fu,0x84c87814u,0x8cc70208u,
    0x90befffau,0xa4506cebu,0xbef9a3f7u,0xc67178f2u
};

static inline uint32_t ror32(uint32_t x, unsigned n) { return (x >> n) | (x << (32 - n)); }

static void sha256_compress(sha256_ctx_t *c, const uint8_t block[64])
{
    uint32_t w[64];
    for (unsigned i = 0; i < 16; i++) {
        w[i] = ((uint32_t)block[i*4] << 24) | ((uint32_t)block[i*4+1] << 16) |
               ((uint32_t)block[i*4+2] << 8) |  (uint32_t)block[i*4+3];
    }
    for (unsigned i = 16; i < 64; i++) {
        uint32_t s0 = ror32(w[i-15], 7) ^ ror32(w[i-15], 18) ^ (w[i-15] >> 3);
        uint32_t s1 = ror32(w[i-2], 17) ^ ror32(w[i-2], 19) ^ (w[i-2] >> 10);
        w[i] = w[i-16] + s0 + w[i-7] + s1;
    }

    uint32_t a=c->state[0], b=c->state[1], cc=c->state[2], d=c->state[3];
    uint32_t e=c->state[4], f=c->state[5], g=c->state[6], h=c->state[7];

    for (unsigned i = 0; i < 64; i++) {
        uint32_t S1 = ror32(e, 6) ^ ror32(e, 11) ^ ror32(e, 25);
        uint32_t ch = (e & f) ^ (~e & g);
        uint32_t t1 = h + S1 + ch + K[i] + w[i];
        uint32_t S0 = ror32(a, 2) ^ ror32(a, 13) ^ ror32(a, 22);
        uint32_t mj = (a & b) ^ (a & cc) ^ (b & cc);
        uint32_t t2 = S0 + mj;
        h = g; g = f; f = e; e = d + t1;
        d = cc; cc = b; b = a; a = t1 + t2;
    }
    c->state[0]+=a; c->state[1]+=b; c->state[2]+=cc; c->state[3]+=d;
    c->state[4]+=e; c->state[5]+=f; c->state[6]+=g; c->state[7]+=h;
}

static void sha256_init(sha256_ctx_t *c)
{
    static const uint32_t H0[8] = {
        0x6a09e667u,0xbb67ae85u,0x3c6ef372u,0xa54ff53au,
        0x510e527fu,0x9b05688cu,0x1f83d9abu,0x5be0cd19u
    };
    memcpy(c->state, H0, sizeof(H0));
    c->bitlen = 0;
    c->buflen = 0;
}

static void sha256_update(sha256_ctx_t *c, const uint8_t *data, size_t len)
{
    c->bitlen += (uint64_t)len * 8u;
    while (len > 0) {
        uint32_t take = SHA256_BLOCK_BYTES - c->buflen;
        if (take > len) take = (uint32_t)len;
        memcpy(c->buf + c->buflen, data, take);
        c->buflen += take;
        data      += take;
        len       -= take;
        if (c->buflen == SHA256_BLOCK_BYTES) {
            sha256_compress(c, c->buf);
            c->buflen = 0;
        }
    }
}

static void sha256_final(sha256_ctx_t *c, uint8_t out[SHA256_DIGEST_BYTES])
{
    /* Padding: 0x80, then zeros, then 64-bit big-endian length */
    c->buf[c->buflen++] = 0x80;
    if (c->buflen > 56) {
        while (c->buflen < SHA256_BLOCK_BYTES) c->buf[c->buflen++] = 0;
        sha256_compress(c, c->buf);
        c->buflen = 0;
    }
    while (c->buflen < 56) c->buf[c->buflen++] = 0;

    uint64_t bl = c->bitlen;
    for (int i = 7; i >= 0; i--) c->buf[c->buflen++] = (uint8_t)(bl >> (i * 8));
    sha256_compress(c, c->buf);

    for (unsigned i = 0; i < 8; i++) {
        out[i*4]   = (uint8_t)(c->state[i] >> 24);
        out[i*4+1] = (uint8_t)(c->state[i] >> 16);
        out[i*4+2] = (uint8_t)(c->state[i] >> 8);
        out[i*4+3] = (uint8_t)(c->state[i]);
    }
}

/* ---------------- HMAC-SHA256 (RFC 2104) -----------------------------------*/

static void hmac_sha256(const uint8_t *key, size_t key_len,
                        const uint8_t *msg, size_t msg_len,
                        uint8_t out[SHA256_DIGEST_BYTES])
{
    uint8_t k_block[SHA256_BLOCK_BYTES];
    uint8_t k_ipad [SHA256_BLOCK_BYTES];
    uint8_t k_opad [SHA256_BLOCK_BYTES];
    sha256_ctx_t ctx;

    /* If key is longer than the block, hash it down first */
    if (key_len > SHA256_BLOCK_BYTES) {
        sha256_init(&ctx);
        sha256_update(&ctx, key, key_len);
        sha256_final(&ctx, k_block);
        memset(k_block + SHA256_DIGEST_BYTES, 0, SHA256_BLOCK_BYTES - SHA256_DIGEST_BYTES);
    } else {
        memcpy(k_block, key, key_len);
        memset(k_block + key_len, 0, SHA256_BLOCK_BYTES - key_len);
    }

    for (unsigned i = 0; i < SHA256_BLOCK_BYTES; i++) {
        k_ipad[i] = k_block[i] ^ 0x36;
        k_opad[i] = k_block[i] ^ 0x5C;
    }

    uint8_t inner[SHA256_DIGEST_BYTES];
    sha256_init(&ctx);
    sha256_update(&ctx, k_ipad, SHA256_BLOCK_BYTES);
    sha256_update(&ctx, msg, msg_len);
    sha256_final(&ctx, inner);

    sha256_init(&ctx);
    sha256_update(&ctx, k_opad, SHA256_BLOCK_BYTES);
    sha256_update(&ctx, inner, SHA256_DIGEST_BYTES);
    sha256_final(&ctx, out);
}

/* ---------------- Public API -----------------------------------------------*/

/* The signed region is everything before the trailing hmac[] field. */
#define PACKET_SIGNED_BYTES (sizeof(emergency_packet_t) - PACKET_HMAC_LEN)

void packet_sign(emergency_packet_t *pkt, const uint8_t *key, size_t key_len)
{
    uint8_t tag[SHA256_DIGEST_BYTES];
    hmac_sha256(key, key_len, (const uint8_t *)pkt, PACKET_SIGNED_BYTES, tag);
    memcpy(pkt->hmac, tag, PACKET_HMAC_LEN);
}

int packet_verify(const emergency_packet_t *pkt, const uint8_t *key, size_t key_len)
{
    uint8_t tag[SHA256_DIGEST_BYTES];
    hmac_sha256(key, key_len, (const uint8_t *)pkt, PACKET_SIGNED_BYTES, tag);

    /* Constant-time comparison — never short-circuit on first mismatch. */
    uint8_t diff = 0;
    for (unsigned i = 0; i < PACKET_HMAC_LEN; i++) diff |= (uint8_t)(tag[i] ^ pkt->hmac[i]);
    return diff == 0;
}
