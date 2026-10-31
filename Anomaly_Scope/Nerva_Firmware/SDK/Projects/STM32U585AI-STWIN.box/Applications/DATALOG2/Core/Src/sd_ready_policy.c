#include "sd_ready_policy.h"
uint32_t sd_ready_select(const sd_ready_view_t *v,uint32_t n,uint32_t fallback) {
  uint32_t best=fallback;
  /* Preserve FIFO ties and use the original stream if rates are unavailable. */
  uint32_t found=best<n && v[best].ready && v[best].produced_bytes;
  for(uint32_t i=0;i<n;i++) {
    if(!v[i].ready || !v[i].produced_bytes)continue;
    if(!found || (uint64_t)v[i].reserve_bytes*v[best].produced_bytes <
                 (uint64_t)v[best].reserve_bytes*v[i].produced_bytes) {
      best=i;found=1;
    }
  }
  return best;
}
