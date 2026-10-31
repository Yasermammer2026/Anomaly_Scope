/* Diagnostic-only instrumentation. Original recorder policy and API results remain intact. */
#include "sd_diag.h"
#include "sd_ready_policy.h"
#include "sdmmc.h"
#include "mdf.h"
#include "services/sysmem.h"
#include <string.h>
#pragma GCC optimize ("O2")
extern TX_THREAD *_tx_thread_current_ptr;
TX_BYTE_POOL *sd_diag_pool;
extern VOID fx_stm32_sd_driver_original(FX_MEDIA *media);
volatile sd_diag_state_t sd_diag;
static filex_dctrl_class_t *owner;
static sd_diag_event_t *trace_memory;
static CBItem *write_item[SD_DIAG_STREAMS];
static uint32_t write_offset[SD_DIAG_STREAMS];
static uint32_t native_item_size[SD_DIAG_STREAMS];
extern TX_BYTE_POOL *byte_pool; /* NetX reservation; acquisition system pool is untouched. */

static uint32_t lock(void) { uint32_t p=__get_PRIMASK(); __disable_irq(); return p; }
static void unlock(uint32_t p) { __set_PRIMASK(p); }
static uint32_t duration(uint32_t tick, uint32_t cycle) {
  uint32_t dt=tx_time_get()-tick;
  if (dt > 20000U) return dt*1000000U/TX_TIMER_TICKS_PER_SECOND;
  return (uint32_t)((uint64_t)(DWT->CYCCNT-cycle)*1000000U/sd_diag.clock_hz);
}
void sd_diag_event(uint32_t ev,uint32_t a,uint32_t b,uint32_t c) {
  if (!sd_diag.active || sd_diag.frozen || !sd_diag.trace) return;
  uint32_t begin=DWT->CYCCNT, p=lock(), i=sd_diag.trace_next;
  if (!sd_diag.frozen) {
    sd_diag.trace[i]=(sd_diag_event_t){tx_time_get(),DWT->CYCCNT,ev,a,b,c};
    sd_diag.trace_next=(i+1U)%SD_DIAG_EVENTS; sd_diag.trace_count++;
  }
  unlock(p);
  uint32_t cost=DWT->CYCCNT-begin;
  sd_diag.trace_cost_cycles+=cost;
  if(cost>sd_diag.trace_cost_max_cycles) sd_diag.trace_cost_max_cycles=cost;
}
static uint64_t produced(void) {
  uint64_t n=0; for(uint32_t i=0;i<SD_DIAG_STREAMS;i++) n+=sd_diag.streams[i].payload_bytes; return n;
}
static uint32_t buffers(uint32_t which) {
  uint32_t n=0;
  if(!owner) return 0;
  for(uint32_t i=0;i<SD_DIAG_STREAMS;i++) {
    CircularBufferDL2 *cb=owner->cbdl2[i];
    if(!cb) continue;
    for(uint32_t j=0;j<cb->cb.item_count;j++)
      if(cb->cb.p_items[j].status.status==which) n++;
  }
  return n;
}
static uint32_t depth(void) { return owner ? owner->fx_app_queue.tx_queue_enqueued : 0; }
static uint32_t stream_of(FX_FILE *f) {
  if(owner) for(uint32_t i=0;i<SD_DIAG_STREAMS;i++) if(f==&owner->file_dat[i]) return i;
  return SD_DIAG_UNKNOWN;
}
void sd_diag_begin(filex_dctrl_class_t *obj) {
  if(!trace_memory && byte_pool)
    (void)tx_byte_allocate(byte_pool,(VOID **)&trace_memory,SD_DIAG_EVENTS*sizeof(sd_diag_event_t),TX_NO_WAIT);
  uint32_t halt=sd_diag.halt_on_failure;
  uint32_t urgency=sd_diag.urgency_enabled;
  uint32_t chunk=sd_diag.chunk_bytes,tail=sd_diag.tail_fixed;
  uint32_t split=sd_diag.split_ring;
  memset((void *)&sd_diag,0,sizeof(sd_diag)); owner=obj;
  memset(write_item,0,sizeof(write_item));memset(write_offset,0,sizeof(write_offset));
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  sd_diag.magic=0x53444447;sd_diag.version=11;sd_diag.clock_hz=SystemCoreClock;
  sd_diag.trace=trace_memory;sd_diag.trace_capacity=SD_DIAG_EVENTS;
  sd_diag.trace_alloc_failed=!trace_memory;sd_diag.halt_on_failure=halt;
  sd_diag.urgency_enabled=urgency;
  sd_diag.chunk_bytes=chunk;sd_diag.tail_fixed=tail;
  sd_diag.split_ring=split;
  sd_diag.start_tick=tx_time_get();sd_diag.active=1;
  sd_diag.filex_status=sd_diag.flush_status=sd_diag.close_status=sd_diag.disk_status=SD_DIAG_UNKNOWN;
  sd_diag.semaphore_status=SD_DIAG_UNKNOWN;
  sd_diag.card_state=SD_DIAG_UNKNOWN;
  sd_diag.min_heap_available=SD_DIAG_UNKNOWN;
  sd_diag.write.min_us=sd_diag.flush.min_us=sd_diag.close.min_us=SD_DIAG_UNKNOWN;
  for(uint32_t i=0;i<SD_DIAG_STREAMS;i++) sd_diag.streams[i].min_free=SD_DIAG_UNKNOWN;
  sd_diag_event(SD_EV_START,sd_diag.clock_hz,sizeof(sd_diag),SD_DIAG_EVENTS*sizeof(sd_diag_event_t));
}
__attribute__((noinline)) void sd_diag_failure_marker(void) { __asm volatile("nop"); }
__attribute__((noinline)) void sd_diag_finished_marker(void) { __asm volatile("nop"); }
void sd_diag_fail(uint32_t reason,uint32_t stream,uint32_t detail) {
  if(!sd_diag.active || sd_diag.first.valid) return;
  sd_diag_event(SD_EV_FIRST_FAILURE,reason,stream,detail);
  uint32_t p=lock();
  if(sd_diag.first.valid) {unlock(p);return;}
  sd_diag.frozen=1;
  volatile sd_diag_failure_t *f=&sd_diag.first;
  f->tick=tx_time_get();f->cycle=DWT->CYCCNT;f->elapsed_ticks=f->tick-sd_diag.start_tick;
  f->reason=reason;f->stream=stream;f->detail=detail;f->thread=(uint32_t)_tx_thread_current_ptr;
  f->queue_depth=depth();f->queue_capacity=owner?owner->fx_app_queue.tx_queue_capacity:0;
  f->queue_high=sd_diag.queue_high;f->free_items=buffers(0);f->ready_items=buffers(2);
  f->op=sd_diag.current_op;f->op_start_tick=sd_diag.op_tick;f->op_start_cycle=sd_diag.op_cycle;
  f->requested=sd_diag.op_requested;f->actual_offset_delta=SD_DIAG_UNKNOWN;
  f->filex_status=sd_diag.filex_status;f->flush_status=sd_diag.flush_status;f->close_status=sd_diag.close_status;
  f->disk_status=sd_diag.disk_status;f->semaphore_status=sd_diag.semaphore_status;
  f->hal_state=hsd1.State;f->hal_error=hsd1.ErrorCode;f->hal_context=hsd1.Context;
  f->sdmmc_sta=hsd1.Instance?hsd1.Instance->STA:0;
  f->sdmmc_dctrl=hsd1.Instance?hsd1.Instance->DCTRL:0;
  f->sdmmc_dcount=hsd1.Instance?hsd1.Instance->DCOUNT:0;
  f->idmactrl=hsd1.Instance?hsd1.Instance->IDMACTRL:0;
  f->tx_ptr=(uint32_t)hsd1.pTxBuffPtr;f->rx_ptr=(uint32_t)hsd1.pRxBuffPtr;
  /* U585 SD driver uses integrated DMA, not separate DMA handles. */
  f->dma_tx_error=f->dma_rx_error=SD_DIAG_UNKNOWN;
  f->heap_available=sd_diag_pool?sd_diag_pool->tx_byte_pool_available:SD_DIAG_UNKNOWN;
  uint32_t s=sd_diag.op_stream;
  if(owner) {
    if(s<SD_DIAG_STREAMS) {
      f->file_offset=owner->file_dat[s].fx_file_current_file_offset;
      f->file_size=owner->file_dat[s].fx_file_current_file_size;
    }
    f->sd_free_bytes=(uint64_t)owner->sdio_disk.fx_media_available_clusters*
       owner->sdio_disk.fx_media_sectors_per_cluster*owner->sdio_disk.fx_media_bytes_per_sector;
  }
  f->last_write_us=sd_diag.last_write_us;f->max_write_us=sd_diag.write.max_us;
  f->card_state=sd_diag.card_state;f->card_wait_active=sd_diag.card_wait_active;
  f->card_wait_start_tick=sd_diag.card_wait_tick;f->card_wait_start_cycle=sd_diag.card_wait_cycle;
  for(uint32_t i=0;i<SD_DIAG_STREAMS;i++) f->streams[i]=sd_diag.streams[i];
  f->valid=1;unlock(p);
  if(sd_diag.halt_on_failure) sd_diag_failure_marker();
}
void sd_diag_trigger(void) {
  if(!sd_diag.first.valid) sd_diag_fail(SD_FAIL_UNKNOWN,SD_DIAG_UNKNOWN,0);
  sd_diag.trigger_count++;sd_diag.trigger_reason=sd_diag.first.reason;sd_diag.trigger_tick=tx_time_get();
}
void sd_diag_end(void) {
  if(!sd_diag.active) return;
  sd_diag_event(SD_EV_STOP,0,0,0);sd_diag.stop_tick=tx_time_get();sd_diag.active=0;
  sd_diag_finished_marker();
}
void sd_diag_producer(uint32_t s,uint32_t sensor,uint32_t samples,uint32_t bps) {
  if(!sd_diag.active || sd_diag.frozen || s>=SD_DIAG_STREAMS) return;
  volatile sd_diag_stream_t *v=&sd_diag.streams[s];
  v->enabled=1;v->sensor_id=sensor;v->bytes_per_sample=bps;
  if(!v->producer_calls) v->first_tick=tx_time_get();
  v->last_tick=tx_time_get();v->producer_calls++;v->samples_delivered+=samples;v->payload_bytes+=(uint64_t)samples*bps;
  if(samples>v->max_burst_samples) {
    v->max_burst_samples=samples;sd_diag_event(SD_EV_PRODUCER,s,samples,bps);
  }
}
void sd_diag_buffers(uint32_t s,uint32_t ev) {
  if(!sd_diag.active || sd_diag.frozen || !owner || s>=SD_DIAG_STREAMS || !owner->cbdl2[s]) return;
  CircularBufferDL2 *c=owner->cbdl2[s];uint32_t free=0,ready=0;
  for(uint32_t i=0;i<c->cb.item_count;i++) {free+=c->cb.p_items[i].status.status==0;ready+=c->cb.p_items[i].status.status==2;}
  volatile sd_diag_stream_t *v=&sd_diag.streams[s];v->item_size=c->cb.item_size;v->items=c->cb.item_count;
  uint32_t changed=free!=v->last_free || ready!=v->last_ready;
  v->last_free=free;v->last_ready=ready;if(free<v->min_free) v->min_free=free;
  if(changed)sd_diag_event(ev,s,free,ready);
}
void sd_diag_queue(uint32_t ev,TX_QUEUE *q,uint32_t msg) {
  if(!sd_diag.active || sd_diag.frozen) return;
  uint32_t n=q->tx_queue_enqueued;
  if(ev==SD_EV_QUEUE_HIGH && n<=sd_diag.queue_high)return;
  if(n>sd_diag.queue_high) sd_diag.queue_high=n;
  if(ev==SD_EV_QUEUE_PUSH) sd_diag.queue_pushes++;
  if(ev==SD_EV_QUEUE_POP) sd_diag.queue_pops++;
  sd_diag_event(ev,n,msg,q->tx_queue_available_storage);
}
UINT sd_diag_queue_send(TX_QUEUE *q,VOID *msg,ULONG wait) {
  UINT r=tx_queue_send(q,msg,wait);
  if(sd_diag.active && !sd_diag.frozen) {
    if(r==TX_SUCCESS)sd_diag_queue(SD_EV_QUEUE_PUSH,q,*(ULONG *)msg);
    else {sd_diag.queue_send_errors++;sd_diag_fail(SD_FAIL_QUEUE_SEND,*(ULONG *)msg & 255U,r);}
  }
  return r;
}
uint16_t sd_diag_release(CircularBuffer *cb,CBItem *item) {
  uint16_t r=CB_ReleaseItem(cb,item);uint32_t s=SD_DIAG_UNKNOWN;
  if(owner)for(uint32_t i=0;i<SD_DIAG_STREAMS;i++)if((CircularBuffer *)owner->cbdl2[i]==cb)s=i;
  sd_diag_buffers(s,SD_EV_BUFFER_RELEASE);
  if(r)sd_diag_fail(SD_FAIL_BUFFER_OWNERSHIP,s,r);
  return r;
}
static uint32_t bucket(uint32_t us) {
  if(us<=100000U) return (us+999U)/1000U;
  if(us<=1000000U) return 100U+(us-100000U+9999U)/10000U;
  if(us<=10000000U) return 190U+(us-1000000U+99999U)/100000U;
  return 281U;
}
static void latency(volatile sd_diag_latency_t *v,uint32_t us,uint32_t bytes,uint32_t status) {
  static const uint32_t bounds[6]={10000,50000,100000,250000,500000,1000000};
  v->count++;v->total_us+=us;v->bytes+=bytes;v->errors+=status!=FX_SUCCESS;
  if(us<v->min_us)v->min_us=us;
  if(us>v->max_us)v->max_us=us;
  v->hist[bucket(us)]++;for(uint32_t i=0;i<6;i++) if(us>bounds[i])v->over_ms[i]++;
}
UINT sd_diag_file_write(FX_FILE *f,VOID *buf,ULONG count) {
  if(!sd_diag.active || sd_diag.frozen) return fx_file_write(f,buf,count);
  uint32_t s=stream_of(f),t=tx_time_get(),cy=DWT->CYCCNT;
  uint64_t off=f->fx_file_current_file_offset;
  sd_diag.current_op=SD_EV_WRITE_BEGIN;sd_diag.op_tick=t;sd_diag.op_cycle=cy;
  sd_diag.op_requested=count;sd_diag.op_stream=s;sd_diag.op_queue_before=depth();sd_diag.op_free_before=buffers(0);
  sd_diag.op_produced_before=produced();
  sd_diag_event(SD_EV_WRITE_BEGIN,s,count,sd_diag.op_queue_before);
  UINT result=fx_file_write(f,buf,count);
  uint32_t us=duration(t,cy);uint64_t delta=f->fx_file_current_file_offset-off;
  if(sd_diag.frozen) {
    sd_diag.post_failure_write_us=us;sd_diag.post_failure_write_status=result;
    return result;
  }
  sd_diag.filex_status=result;sd_diag.last_write_us=us;sd_diag.last_queue_after=depth();sd_diag.last_free_after=buffers(0);
  sd_diag.produced_during_last_write=produced()-sd_diag.op_produced_before;
  latency(&sd_diag.write,us,result==FX_SUCCESS?count:0,result);
  if(s<SD_DIAG_STREAMS && result==FX_SUCCESS) sd_diag.streams[s].filex_bytes+=count;
  uint32_t i=sd_diag.history_next;
  sd_diag.history[i].tick=t;sd_diag.history[i].stream=s;sd_diag.history[i].bytes=count;sd_diag.history[i].us=us;
  sd_diag.history[i].queue_before=sd_diag.op_queue_before;sd_diag.history[i].queue_after=sd_diag.last_queue_after;
  sd_diag.history[i].free_before=sd_diag.op_free_before;sd_diag.history[i].free_after=sd_diag.last_free_after;
  sd_diag.history[i].produced_during=sd_diag.produced_during_last_write;
  sd_diag.history_next=(i+1U)%SD_DIAG_HISTORY;sd_diag.history_count++;
  sd_diag_event(SD_EV_WRITE_END,s,us,result);
  if(result!=FX_SUCCESS) sd_diag_fail(SD_FAIL_FILEX_WRITE,s,result);
  else if(delta!=count) {sd_diag.write.partials++;sd_diag_fail(SD_FAIL_PARTIAL_OFFSET,s,(uint32_t)delta);}
  if(sd_diag.first.valid) sd_diag.first.actual_offset_delta=(uint32_t)delta;
  sd_diag.current_op=0;return result;
}
UINT sd_diag_media_flush(FX_MEDIA *m) {
  if(!sd_diag.active || sd_diag.frozen) return fx_media_flush(m);
  uint32_t t=tx_time_get(),cy=DWT->CYCCNT;sd_diag.current_op=SD_EV_FLUSH_BEGIN;
  sd_diag.op_tick=t;sd_diag.op_cycle=cy;sd_diag.op_requested=0;sd_diag.op_stream=SD_DIAG_UNKNOWN;
  sd_diag_event(SD_EV_FLUSH_BEGIN,depth(),buffers(0),0);
  UINT r=fx_media_flush(m);uint32_t us=duration(t,cy);
  if(sd_diag.frozen)return r;
  sd_diag.flush_status=r;latency(&sd_diag.flush,us,0,r);sd_diag_event(SD_EV_FLUSH_END,us,r,0);
  if(r!=FX_SUCCESS)sd_diag_fail(SD_FAIL_FILEX_FLUSH,SD_DIAG_UNKNOWN,r);
  sd_diag.current_op=0;return r;
}
UINT sd_diag_file_close(FX_FILE *f) {
  if(!sd_diag.active || sd_diag.frozen)return fx_file_close(f);
  uint32_t t=tx_time_get(),cy=DWT->CYCCNT;sd_diag.current_op=SD_EV_CLOSE_BEGIN;
  sd_diag.op_tick=t;sd_diag.op_cycle=cy;sd_diag.op_requested=0;sd_diag.op_stream=stream_of(f);
  sd_diag_event(SD_EV_CLOSE_BEGIN,sd_diag.op_stream,0,0);
  UINT r=fx_file_close(f);uint32_t us=duration(t,cy);
  if(sd_diag.frozen)return r;
  sd_diag.close_status=r;latency(&sd_diag.close,us,0,r);sd_diag_event(SD_EV_CLOSE_END,us,r,0);
  if(r!=FX_SUCCESS)sd_diag_fail(SD_FAIL_FILEX_CLOSE,sd_diag.op_stream,r);
  sd_diag.current_op=0;return r;
}
VOID sd_diag_disk_driver(FX_MEDIA *m) {
  sd_diag_event(SD_EV_DISK_BEGIN,m->fx_media_driver_request,m->fx_media_driver_logical_sector,m->fx_media_driver_sectors);
  fx_stm32_sd_driver_original(m);
  if(!sd_diag.active || sd_diag.frozen)return;
  sd_diag.disk_status=m->fx_media_driver_status;
  sd_diag_event(SD_EV_DISK_END,m->fx_media_driver_request,m->fx_media_driver_status,m->fx_media_driver_sectors);
  if(m->fx_media_driver_status!=FX_SUCCESS)sd_diag_fail(SD_FAIL_DISKIO,sd_diag.op_stream,m->fx_media_driver_status);
}
UINT sd_diag_semaphore_get(TX_SEMAPHORE *s,ULONG wait,uint32_t site) {
  sd_diag_event(SD_EV_SEM_WAIT,site,s->tx_semaphore_count,wait);
  UINT r=tx_semaphore_get(s,wait);
  if(sd_diag.active && !sd_diag.frozen) {
    sd_diag.semaphore_status=r;sd_diag_event(SD_EV_SEM_END,site,r,s->tx_semaphore_count);
    if(r!=TX_SUCCESS){sd_diag.timeouts++;sd_diag_fail(SD_FAIL_TIMEOUT,sd_diag.op_stream,site);}
  }
  return r;
}
void sd_diag_hal_begin(uint32_t sector,uint32_t blocks,uint32_t write) {
  if(sd_diag.active && !sd_diag.frozen) sd_diag.hal_starts++;
  sd_diag_event(SD_EV_DMA_BEGIN,sector,blocks,write);
}
void sd_diag_hal_end(uint32_t status) {
  if(status!=HAL_OK && sd_diag.active && !sd_diag.frozen){sd_diag.hal_errors++;sd_diag_fail(SD_FAIL_HAL_START,sd_diag.op_stream,status);}
}
void sd_diag_hal_callback(uint32_t error) {
  if(!sd_diag.active || sd_diag.frozen)return;
  if(error){sd_diag.hal_errors++;sd_diag_event(SD_EV_HAL_ERROR,hsd1.ErrorCode,hsd1.State,hsd1.Instance->STA);sd_diag_fail(SD_FAIL_HAL_CALLBACK,sd_diag.op_stream,hsd1.ErrorCode);}
  else {sd_diag.hal_completions++;sd_diag_event(SD_EV_DMA_END,hsd1.State,hsd1.Context,hsd1.Instance->DCOUNT);}
}
void sd_diag_card_begin(void) {
  if(!sd_diag.active || sd_diag.frozen)return;
  sd_diag.card_wait_active=1;sd_diag.card_wait_tick=tx_time_get();sd_diag.card_wait_cycle=DWT->CYCCNT;
  sd_diag.card_wait_calls++;
}
void sd_diag_card_poll(uint32_t state) {
  if(!sd_diag.active || sd_diag.frozen)return;
  sd_diag.card_polls++;
  if(state!=HAL_SD_CARD_TRANSFER)sd_diag.card_busy_polls++;
  if(state!=sd_diag.card_state)sd_diag_event(SD_EV_CARD_STATE,state,hsd1.ErrorCode,sd_diag.op_stream);
  sd_diag.card_state=state;
}
void sd_diag_card_end(void) {
  if(!sd_diag.active || sd_diag.frozen)return;
  uint32_t us=duration(sd_diag.card_wait_tick,sd_diag.card_wait_cycle);
  sd_diag.card_wait_last_us=us;
  if(us>sd_diag.card_wait_max_us)sd_diag.card_wait_max_us=us;
  if(us>1000U){sd_diag.card_busy_waits++;sd_diag_event(SD_EV_CARD_WAIT,us,sd_diag.card_state,sd_diag.op_stream);}
  sd_diag.card_wait_active=0;
}
uint32_t sd_diag_choose(uint32_t message) {
  if(!sd_diag.active || sd_diag.frozen || !owner)return message;
  if(!sd_diag.urgency_enabled)return message;
  sd_ready_view_t view[SD_DIAG_STREAMS]={0};
  uint32_t p=lock();
  for(uint32_t i=0;i<SD_DIAG_STREAMS;i++) {
    CircularBufferDL2 *c=owner->cbdl2[i];
    if(!c || !c->cb.item_count)continue;
    view[i].ready=write_item[i] || c->cb.p_items[c->cb.tail_idx].status.status==2U;
    view[i].produced_bytes=sd_diag.streams[i].stream_input_bytes;
    for(uint32_t j=0;j<c->cb.item_count;j++)
      if(c->cb.p_items[j].status.status==0U)view[i].reserve_bytes+=c->cb.item_size-4U;
    if(c->p_current_item && c->item_idx<c->cb.item_size)
      view[i].reserve_bytes+=c->cb.item_size-c->item_idx;
  }
  unlock(p);
  uint32_t original=message & FILEX_DCTRL_DATA_SENSOR_ID_MASK;
  uint32_t selected=sd_ready_select(view,SD_DIAG_STREAMS,original);
  if(selected!=original) {
    sd_diag.urgency_selections++;
    sd_diag_event(SD_EV_SELECT,original,selected,view[selected].reserve_bytes);
  }
  return (message & ~FILEX_DCTRL_DATA_SENSOR_ID_MASK) | selected;
}
void sd_diag_ready_result(uint32_t consumed) {
  if(!sd_diag.active || sd_diag.frozen)return;
  if(consumed)sd_diag.blocks_completed++;
  else sd_diag.empty_ready_messages++;
}
void sd_diag_ready_message(void) {
  if(sd_diag.active && !sd_diag.frozen)sd_diag.ready_messages++;
}
static UINT write_piece(uint32_t s,uint32_t limit,uint32_t *complete) {
  CircularBufferDL2 *c=owner->cbdl2[s];*complete=0;
  if(!write_item[s]) {
    if(CB_GetReadyItemFromTail((CircularBuffer *)c,&write_item[s])!=SYS_NO_ERROR_CODE)return FX_NOT_FOUND;
    write_offset[s]=0;
  }
  uint32_t count=c->cb.item_size-write_offset[s];if(count>limit)count=limit;
  UINT r=sd_diag_file_write(&owner->file_dat[s],(uint8_t *)CB_GetItemData(write_item[s])+write_offset[s],count);
  if(sd_diag.active && !sd_diag.frozen)sd_diag.chunk_calls++;
  if(r!=FX_SUCCESS)return r;
  write_offset[s]+=count;
  if(write_offset[s]==c->cb.item_size) {
    sd_diag_release((CircularBuffer *)c,write_item[s]);write_item[s]=NULL;write_offset[s]=0;*complete=1;
  }
  return FX_SUCCESS;
}
UINT sd_diag_chunk_ready(uint32_t message) {
  /* One queue message is one completion credit, even when writes interleave. */
  for(;;) {
    uint32_t chosen=sd_diag_choose(message)&FILEX_DCTRL_DATA_SENSOR_ID_MASK;
    if(chosen>=SD_DIAG_STREAMS || !owner->cbdl2[chosen])return FX_NOT_FOUND;
    uint32_t complete;UINT r=write_piece(chosen,sd_diag.chunk_bytes,&complete);
    if(r==FX_NOT_FOUND){sd_diag_ready_result(0);return FX_SUCCESS;}
    if(r!=FX_SUCCESS)return r;
    if(complete){sd_diag_ready_result(1);return FX_SUCCESS;}
  }
}
UINT sd_diag_drain(uint32_t s) {
  CircularBufferDL2 *c=owner->cbdl2[s];if(!c)return FX_SUCCESS;
  for(;;) {
    uint32_t done;UINT r=write_piece(s,c->cb.item_size,&done);
    if(r==FX_NOT_FOUND)break;
    if(r!=FX_SUCCESS)return r;
  }
  /* item_idx includes the header; byte_counter is cumulative across the file. */
  if(c->p_current_item && c->item_idx>4U)
    return sd_diag_file_write(&owner->file_dat[s],CB_GetItemData(c->p_current_item),c->item_idx);
  return FX_SUCCESS;
}
uint32_t sd_diag_item_size(uint32_t s,uint32_t original) {
  if(s>=SD_DIAG_STREAMS)return original;
  if(!native_item_size[s])native_item_size[s]=original;
  if(!sd_diag.split_ring)return native_item_size[s];
  original=native_item_size[s];
  if(original<1024U)return original;
  uint32_t smaller=(original/2U)&~511U;
  /* The observed 512-byte IIS2ICLX burst must fit one payload item. */
  return smaller<1024U?1024U:smaller;
}
static volatile sd_diag_stream_t *sensor(uint32_t id) {
  if(!sd_diag.active || sd_diag.frozen)return NULL;
  for(uint32_t i=0;i<SD_DIAG_STREAMS;i++)
    if(sd_diag.streams[i].enabled && sd_diag.streams[i].sensor_id==id)return &sd_diag.streams[i];
  return NULL;
}
void sd_diag_config(uint32_t s,uint32_t id,uint32_t bps,uint32_t spts,uint32_t rate) {
  if(!sd_diag.active || sd_diag.frozen || s>=SD_DIAG_STREAMS)return;
  volatile sd_diag_stream_t *v=&sd_diag.streams[s];
  v->enabled=1;v->sensor_id=id;v->bytes_per_sample=bps;v->samples_per_ts=spts;v->nominal_rate_millihz=rate;
  sd_diag_buffers(s,SD_EV_RESOURCE);
}
void sd_diag_sensor_fifo(uint32_t id,uint32_t level,uint32_t overflow,uint32_t status) {
  volatile sd_diag_stream_t *v=sensor(id);if(!v)return;
  if(status){v->sensor_io_errors++;sd_diag_event(SD_EV_SENSOR_IO_ERROR,id,status,0);return;}
  v->fifo_observed++;v->fifo_last_level=level;
  if(level>v->fifo_max_level)v->fifo_max_level=level;
  if(overflow){v->fifo_overflows++;sd_diag_event(SD_EV_FIFO_OVERFLOW,id,level,overflow);}
}
void sd_diag_sensor_read(uint32_t id,uint32_t samples,uint32_t warmup) {
  volatile sd_diag_stream_t *v=sensor(id);if(!v)return;
  v->driver_samples_read+=samples;
  if(warmup)v->warmup_samples+=samples;
}
uint16_t sd_diag_sensor_dma(uint32_t id,uint32_t samples) {
  volatile sd_diag_stream_t *v=sensor(id);if(!v)return 0;
  v->dma_callbacks++;v->dma_generated_samples+=samples;v->dma_last_seq++;
  return (uint16_t)v->dma_last_seq;
}
void sd_diag_sensor_dma_check(uint32_t id,uint16_t seq,uint32_t consumed) {
  volatile sd_diag_stream_t *v=sensor(id);if(!v)return;
  if(consumed!=1U && seq!=(uint16_t)v->dma_last_seq) {
    if(!v->dma_first_suspect_tick){v->dma_first_suspect_tick=tx_time_get();v->dma_first_suspect_phase=consumed;}
    if(consumed==0U)v->dma_stale_accesses++;
    else if(consumed==3U)v->dma_warmup_late_callbacks++;
    else v->dma_late_returns++;
    sd_diag_event(SD_EV_ACQ_DMA_OVERRUN,id,seq,(v->dma_last_seq<<2)|consumed);
  }
  if(consumed==1U) {
    if(seq!=(uint16_t)(v->dma_last_processed+1U))v->dma_sequence_gaps++;
    v->dma_last_processed=seq;
  }
}
void sd_diag_sensor_queue_error(uint32_t id) {
  volatile sd_diag_stream_t *v=sensor(id);if(v)v->acquisition_queue_errors++;
}
void sd_diag_sensor_io(uint32_t id,int32_t status) {
  volatile sd_diag_stream_t *v=sensor(id);if(!v || !status)return;
  v->sensor_io_errors++;sd_diag_event(SD_EV_SENSOR_IO_ERROR,id,(uint32_t)status,1);
}
void sd_diag_sensor_register(uint32_t id,uint32_t overrun,int32_t status,uint32_t warmup) {
  volatile sd_diag_stream_t *v=sensor(id);if(!v)return;
  if(status){sd_diag_sensor_io(id,status);return;}
  v->register_status_reads++;
  if(overrun) {
    if(warmup)v->register_warmup_overruns++;
    else v->register_overruns++;
    sd_diag_event(SD_EV_REGISTER_OVERRUN,id,overrun,warmup);
  }
}
void sd_diag_sensor_irq(uint32_t id,uint32_t samples) {
  volatile sd_diag_stream_t *v=sensor(id);if(!v)return;
  v->irq_callbacks++;v->irq_generated_samples+=samples;
}
void sd_diag_tick(void) {
  if(!sd_diag.active || sd_diag.frozen)return;
  TX_THREAD *t=_tx_thread_current_ptr;sd_diag.cpu_samples++;
  sd_diag.adf_error_flags|=AdfHandle0.ErrorCode;sd_diag.mdf_error_flags|=MdfHandle0.ErrorCode;
  if(AdfHandle0.hdma)sd_diag.adf_dma_error_flags|=AdfHandle0.hdma->ErrorCode;
  if(MdfHandle0.hdma)sd_diag.mdf_dma_error_flags|=MdfHandle0.hdma->ErrorCode;
  if(sd_diag_pool && sd_diag_pool->tx_byte_pool_available<sd_diag.min_heap_available)
    sd_diag.min_heap_available=sd_diag_pool->tx_byte_pool_available;
  if(!t)sd_diag.idle_samples++;
  else if(t->tx_thread_priority<32U)sd_diag.cpu_thread[t->tx_thread_priority]++;
  if(owner) {
    if(t==&owner->fx_app_thread)sd_diag.writer_samples++;
    if(owner->fx_app_thread.tx_thread_state==TX_READY && t!=&owner->fx_app_thread) {
      if(!sd_diag.writer_ready_since)sd_diag.writer_ready_since=tx_time_get();
      uint32_t d=tx_time_get()-sd_diag.writer_ready_since;
      if(d>sd_diag.writer_ready_max_ticks)sd_diag.writer_ready_max_ticks=d;
    } else sd_diag.writer_ready_since=0;
  }
}
void HAL_MDF_ErrorCallback(MDF_HandleTypeDef *h) {
  if(!sd_diag.active || sd_diag.frozen)return;
  if(h==&AdfHandle0)sd_diag.adf_error_flags|=h->ErrorCode;
  if(h==&MdfHandle0)sd_diag.mdf_error_flags|=h->ErrorCode;
  sd_diag_event(SD_EV_ACQ_DMA_OVERRUN,(uint32_t)h,h->ErrorCode,h->hdma?h->hdma->ErrorCode:0);
}

/* Release inactive recording trace from the reserved NetX pool before Wi-Fi. */
void cycle30_release_sd_trace(void) {
  if(!sd_diag.active && trace_memory) {
    if(tx_byte_release(trace_memory)==TX_SUCCESS) {
      trace_memory=NULL;sd_diag.trace=NULL;sd_diag.trace_capacity=0;
    }
  }
}
