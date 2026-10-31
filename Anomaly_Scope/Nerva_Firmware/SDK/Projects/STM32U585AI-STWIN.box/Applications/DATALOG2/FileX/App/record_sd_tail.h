#ifndef STOCK_RECORD_SD_TAIL_H
#define STOCK_RECORD_SD_TAIL_H
#include <stdint.h>
#define RSD_STREAMS 16U
#define RSD_SECTOR 512U
/* Single SD-writer owner; no acquisition/output pointer is stored. V4 records
 * and backing addresses are word aligned. Only stopped finalization flushes
 * partial tails. Append success means independent writer acceptance; final
 * on-card qualification additionally requires flush, close and reconstruction. */
typedef struct {
  uint32_t used[RSD_STREAMS];
  uint8_t data[RSD_STREAMS][RSD_SECTOR];
} rsd_tails;
typedef int (*rsd_write)(void *,uint32_t,const uint8_t *,uint32_t);
int rsd_append(rsd_tails *,uint32_t,const uint8_t *,uint32_t,rsd_write,void *);
int rsd_flush(rsd_tails *,uint32_t,rsd_write,void *);
#endif
