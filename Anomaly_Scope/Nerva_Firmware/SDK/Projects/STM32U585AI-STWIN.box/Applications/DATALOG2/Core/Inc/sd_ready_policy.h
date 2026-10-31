#ifndef SD_READY_POLICY_H
#define SD_READY_POLICY_H
#include <stdint.h>
/* Every data-ready queue message authorizes one full block. Control messages
 * still follow the native dispatcher. Each stream is consumed only at its tail.
 * Rates have a common observation interval, so it cancels in the comparison. */
typedef struct {
  uint32_t ready, reserve_bytes;
  uint64_t produced_bytes;
} sd_ready_view_t;
uint32_t sd_ready_select(const sd_ready_view_t *streams, uint32_t count, uint32_t fallback);
#endif
