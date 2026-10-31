#include "stock_wall_clock.h"
#include "rtc.h"
volatile uint32_t stock_wall_enabled,stock_wall_stop_kind;
volatile uint32_t stock_wall_boot_nonce;
volatile stock_wall_stamp stock_wall_stop,stock_wall_ack,stock_wall_persisted;
void stock_wall_begin(void) {
  if(!stock_wall_enabled)return;
  stock_wall_stop.valid=stock_wall_ack.valid=stock_wall_persisted.valid=0;
  stock_wall_stop_kind=0;
  __HAL_DBGMCU_UNFREEZE_RTC();
}
void stock_wall_snapshot(volatile stock_wall_stamp *stamp) {
  /* The SDK initializes year 00. INITS is clear on this valid, advancing
   * calendar; require HAL readiness and synchronized, non-init shadow state. */
  if(!stock_wall_enabled || !stock_wall_boot_nonce || hrtc.Instance!=RTC ||
     hrtc.State!=HAL_RTC_STATE_READY || !(RTC->ICSR&RTC_ICSR_RSF) ||
     (RTC->ICSR&(RTC_ICSR_INIT|RTC_ICSR_INITF)))return;
  uint32_t irq=__get_PRIMASK();__disable_irq();
  /* Reading SSR/TR locks the shadow calendar; DR releases it. */
  stamp->ssr=RTC->SSR;stamp->tr=RTC->TR;stamp->dr=RTC->DR;
  stamp->prer=RTC->PRER;
  stamp->freeze=DBGMCU->APB3FZR&DBGMCU_APB3FZR_DBG_RTC_STOP;
  stamp->valid=1;
  __set_PRIMASK(irq);
}
void stock_wall_stop_request(uint32_t kind) {
  if(!stock_wall_enabled || stock_wall_stop.valid)return;
  stock_wall_snapshot(&stock_wall_stop);
  if(stock_wall_stop.valid)stock_wall_stop_kind=kind;
}
