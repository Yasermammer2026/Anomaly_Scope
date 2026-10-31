#ifndef STOCK_WALL_CLOCK_H
#define STOCK_WALL_CLOCK_H
#include <stdint.h>
/* RTC continues through debugger halts. Its LSI rate must be calibrated on the
 * same boot; raw stamps alone are not measurements in host seconds. */
typedef struct {
  uint32_t ssr,tr,dr,prer,freeze,valid;
} stock_wall_stamp;
extern volatile uint32_t stock_wall_enabled,stock_wall_stop_kind;
/* Volatile host challenge: reset clears it, preventing cross-boot calibration. */
extern volatile uint32_t stock_wall_boot_nonce;
extern volatile stock_wall_stamp stock_wall_stop,stock_wall_ack,stock_wall_persisted;
void stock_wall_begin(void);
void stock_wall_stop_request(uint32_t kind);
void stock_wall_snapshot(volatile stock_wall_stamp *stamp);
#endif
