#ifndef STOCK_RECORD_HASH_H
#define STOCK_RECORD_HASH_H
#include "stm32u5xx_hal_hash.h"
#include "stm32u5xx_hal_hash_ex.h"
#include <stdint.h>

/* One low-priority owner of the peripheral; independent context per stream.
 * The caller must exclude Wi-Fi's stopped transfer hasher. No acquisition
 * pointer is retained. Arbitrary byte tails are copied into this context. */
typedef struct {
  HASH_HandleTypeDef handle;
  uint32_t registers[sizeof(HASH->CSR)/sizeof(uint32_t)+3U];
  uint64_t bytes,fed_bytes;
  uint32_t pending_words[17];
  uint32_t pending_bytes,initialized,saved,complete;
} rh_context;
void rh_open(void);
void rh_shutdown(void);
int rh_update(rh_context *c,const uint8_t *data,uint32_t n,uint32_t last,uint8_t digest[32]);
#endif
