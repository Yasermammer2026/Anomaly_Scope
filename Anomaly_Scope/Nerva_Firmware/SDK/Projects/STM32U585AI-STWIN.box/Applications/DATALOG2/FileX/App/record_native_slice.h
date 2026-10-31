#ifndef STOCK_RECORD_NATIVE_SLICE_H
#define STOCK_RECORD_NATIVE_SLICE_H
#include <stdint.h>
/* Bound one vendor fill call to the current native item. The collector copies
 * and releases that completed item before any following fill can reuse it.
 * Source events/counters/timestamps are not altered. No wait or source storage. */
static inline uint32_t stock_native_slice(uint32_t item,uint32_t index,
                                          uint32_t has_current,uint32_t remaining) {
  if(item<=4U || !remaining || (has_current?(index<4U || index>=item):index!=0U))return 0;
  uint32_t room=item-(has_current?index:4U);
  return remaining<room?remaining:room;
}
#endif
