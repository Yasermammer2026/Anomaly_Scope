/* Opt-in qualified-codec integration. Included only in the working diagnostic
 * FileX TU. Original SDK and rollback image stay separate. */
#include "record_pipeline.h"
#include "record_native_slice.h"
#include "record_sd_tail.h"
#include "../../NetX/app/record_hash.h"
#include "stock_record_stage.inc"
#include "stock_record_chunk.inc"
#include "stock_record_pipeline.inc"
#include "stock_record_sd_tail.inc"
#define STOCK_WRITER_MESSAGE 0x80000000UL
#define STOCK_WRITER_FLUSH 0x80000001UL
#define STOCK_WORKER_STACK 4096U
#define STOCK_DRAIN_TICKS (30U*TX_TIMER_TICKS_PER_SECOND)
typedef struct {
  rp_pipeline pipeline;
  rh_context hashes[RS_STREAMS];
  uint8_t sha256[RS_STREAMS][32];
  uint32_t complete[RS_STREAMS];
  rsd_tails sd_tails;
  TX_THREAD thread;
  TX_EVENT_FLAGS_GROUP event;
  filex_dctrl_class_t *owner;
  uint8_t *stack;
  uint32_t bytes,event_created,thread_created,allocation_failed;
  volatile uint32_t accepting,finalize_requested,finalized,status;
  volatile uint32_t tail_flush_requested,tail_flush_done;
  uint32_t stop_tick,finalize_tick,max_worker_cycles,worker_blocks;
  uint64_t worker_cycles;
} stock_runtime;
volatile uint32_t stock_record_enabled,stock_record_allocation_failed;
volatile uint32_t stock_record_release_requested,stock_record_release_done,stock_record_release_error;
stock_runtime *stock_record;
static uint32_t stock_lock(void *arg){(void)arg;uint32_t key=__get_PRIMASK();__disable_irq();return key;}
static void stock_unlock(void *arg,uint32_t key){(void)arg;__set_PRIMASK(key);}
static int stock_hash(void *arg,uint32_t sid,const uint8_t *p,uint32_t n) {
  stock_runtime *r=arg;return rh_update(&r->hashes[sid],p,n,0,NULL);
}
static int stock_sector_write(void *arg,uint32_t sid,const uint8_t *p,uint32_t n) {
  stock_runtime *r=arg;return (int)sd_diag_file_write(&r->owner->file_dat[sid],(VOID *)p,n);
}
static int stock_write(void *arg,uint32_t sid,const uint8_t *p,uint32_t n) {
  stock_runtime *r=arg;return rsd_append(&r->sd_tails,sid,p,n,stock_sector_write,r);
}
static int stock_wake_worker(void *arg) {
  stock_runtime *r=arg;return (int)tx_event_flags_set(&r->event,1UL,TX_OR);
}
static int stock_wake_writer(void *arg) {
  stock_runtime *r=arg;ULONG message=STOCK_WRITER_MESSAGE;
  return (int)sd_diag_queue_send(&r->owner->fx_app_queue,&message,TX_NO_WAIT);
}
static void stock_failed(void *arg,uint32_t reason,uint32_t sid,int detail) {
  stock_runtime *r=arg;r->status=reason;
  sd_diag_fail(reason==RP_ADMISSION?SD_FAIL_BUFFER_EXHAUSTED:
               (reason==RP_SD_WRITE?SD_FAIL_FILEX_WRITE:SD_FAIL_UNKNOWN),sid,(uint32_t)detail);
  if(r->owner->filex_msg_queue_not_send_cb)r->owner->filex_msg_queue_not_send_cb();
}
static int stock_error(stock_runtime *r,uint32_t reason,uint32_t sid,int detail) {
  uint32_t key=stock_lock(r);uint32_t first=!r->pipeline.error;
  if(first){r->pipeline.error=reason;r->pipeline.failed_stream=sid;r->pipeline.failure_detail=detail;}
  stock_unlock(r,key);if(first)stock_failed(r,reason,sid,detail);return -1;
}
static int stock_output(stock_runtime *r,uint32_t sid,uint32_t footer) {
  uint32_t start=tx_time_get(),index;uint64_t ticket;int st;
  do {
    uint32_t key=stock_lock(r);st=rs_output_reserve(&r->pipeline.writer,&index,&ticket);stock_unlock(r,key);
    if(st!=1)break;
    if(r->pipeline.error)return -1;
    if((uint32_t)(tx_time_get()-start)>STOCK_DRAIN_TICKS)return stock_error(r,RP_OUTPUT_RESERVE,sid,TX_NOT_AVAILABLE);
    tx_thread_sleep(1);
  } while(1);
  if(st)return stock_error(r,RP_OUTPUT_RESERVE,sid,st);
  rc_stream *s=&r->pipeline.streams[sid];rc_result result;
  uint8_t *out=r->pipeline.output[index];
  if(footer) {
    st=rh_update(&r->hashes[sid],NULL,0,1,r->sha256[sid]);
    if(st)return stock_error(r,RP_HASH,sid,st);
    st=rc_footer(s,sid,r->sha256[sid],out,RC_OUTPUT_BYTES,&result);
  } else st=rc_begin(s,sid,out,RC_OUTPUT_BYTES,&result);
  if(st)return stock_error(r,RP_FRAME,sid,st);
  uint32_t key=stock_lock(r);
  st=rs_output_publish(&r->pipeline.writer,index,ticket,sid,result.physical_bytes,0);
  stock_unlock(r,key);
  if(st)return stock_error(r,RP_OUTPUT_PUBLISH,sid,st);
  st=footer?rc_close(s):rc_start(s);
  if(st)return stock_error(r,RP_SOURCE_ACCEPT,sid,st);
  st=stock_wake_writer(r);
  return st?stock_error(r,RP_WRITER_WAKE,sid,st):0;
}
static void stock_worker(ULONG arg) {
  stock_runtime *r=(stock_runtime *)arg;
  while(!r->pipeline.error) {
    ULONG flags;
    if(tx_event_flags_get(&r->event,1UL,TX_OR_CLEAR,&flags,TX_WAIT_FOREVER)!=TX_SUCCESS) {
      stock_error(r,RP_WORKER_WAKE,RS_STREAMS,TX_NOT_AVAILABLE);break;
    }
    for(;;) {
      uint32_t begin=DWT->CYCCNT;int st=rp_worker_step(&r->pipeline);
      uint32_t cycles=DWT->CYCCNT-begin;
      if(st)break;
      r->worker_cycles+=cycles;++r->worker_blocks;
      if(cycles>r->max_worker_cycles)r->max_worker_cycles=cycles;
    }
    if(r->pipeline.error)break;
    if(r->finalize_requested && rp_drained(&r->pipeline)) {
      for(uint32_t sid=0;sid<RS_STREAMS && !r->pipeline.error;++sid)
        if(r->pipeline.streams[sid].schema.item_size && stock_output(r,sid,1))break;
      uint32_t start=tx_time_get();
      while(!r->pipeline.error && !rp_drained(&r->pipeline)) {
        if((uint32_t)(tx_time_get()-start)>STOCK_DRAIN_TICKS){stock_error(r,RP_SD_WRITE,RS_STREAMS,TX_NOT_AVAILABLE);break;}
        tx_thread_sleep(1);
      }
      if(!r->pipeline.error) {
        ULONG message=STOCK_WRITER_FLUSH;r->tail_flush_requested=1;
        int st=(int)sd_diag_queue_send(&r->owner->fx_app_queue,&message,TX_NO_WAIT);
        if(st)stock_error(r,RP_WRITER_WAKE,RS_STREAMS,st);
        start=tx_time_get();
        while(!r->pipeline.error && !r->tail_flush_done) {
          if((uint32_t)(tx_time_get()-start)>STOCK_DRAIN_TICKS){stock_error(r,RP_SD_WRITE,RS_STREAMS,TX_NOT_AVAILABLE);break;}
          tx_thread_sleep(1);
        }
      }
      if(!r->pipeline.error) {
        for(uint32_t sid=0;sid<RS_STREAMS;++sid)
          r->complete[sid]=r->pipeline.streams[sid].closed;
        rh_shutdown();r->finalize_tick=tx_time_get();r->finalized=1;
      }
      break;
    }
  }
}
static int stock_record_prepare(filex_dctrl_class_t *obj) {
  if(!stock_record_enabled)return 0;
  if(stock_record)return -1; /* Qualification starts from a reset image. */
  uint32_t bytes=sizeof(stock_runtime)+RS_ARENA_BYTES+3U*RC_OUTPUT_BYTES+STOCK_WORKER_STACK+63U;
  stock_runtime *r=SysAlloc(bytes);stock_record_allocation_failed=!r;
  if(!r){sd_diag_fail(SD_FAIL_UNKNOWN,RS_STREAMS,SYS_OUT_OF_MEMORY_ERROR_CODE);return -1;}
  memset(r,0,sizeof(*r));stock_record=r;r->owner=obj;r->bytes=bytes;
  uint8_t *arena=(uint8_t *)(((uintptr_t)(r+1)+31U)&~(uintptr_t)31U);
  uint8_t *out0=arena+RS_ARENA_BYTES,*out1=out0+RC_OUTPUT_BYTES;
  uint8_t *scratch=out1+RC_OUTPUT_BYTES;r->stack=scratch+RC_OUTPUT_BYTES;
  memset(r->stack,0xef,STOCK_WORKER_STACK);
  static const rc_stream blank[RS_STREAMS]={0};
  rp_hooks hooks={r,stock_lock,stock_unlock,stock_hash,stock_write,stock_wake_worker,stock_wake_writer,stock_failed};
  if(rp_init(&r->pipeline,&hooks,arena,out0,out1,blank))return stock_error(r,RP_ADMISSION,RS_STREAMS,-1);
  if(rp_enable_packing(&r->pipeline,scratch,RC_OUTPUT_BYTES))return stock_error(r,RP_ADMISSION,RS_STREAMS,-1);
  uint32_t configured=0;
  for(uint32_t sid=0;sid<RS_STREAMS;++sid) {
    CircularBufferDL2 *c=obj->cbdl2[sid];
    if(!c) {
      if(sd_diag.streams[sid].enabled)return stock_error(r,RP_ADMISSION,sid,-1);
      continue;
    }
    ++configured;
    rc_stream *s=&r->pipeline.streams[sid];const char *name=obj->file_dat_name[sid];
    uint32_t bps=sd_diag.streams[sid].bytes_per_sample,dim=1U,width=2U;
    uint32_t group=256U,order=1U;
    if(!strcmp(name,"iis3dwb_acc.slc")){dim=3;order=10;}
    else if(!strcmp(name,"imp34dt05_mic.slc"))order=6;
    else if(!strcmp(name,"ism330dhcx_gyro.slc")){dim=3;order=4;}
    else if(!strcmp(name,"ism330dhcx_acc.slc") || !strcmp(name,"iis2dlpc_acc.slc")){dim=3;order=9;}
    else if(!strcmp(name,"iis2iclx_acc.slc"))dim=2;
    else if(!strcmp(name,"iis2mdc_mag.slc")){dim=3;order=10;}
    else if(!strcmp(name,"ilps22qs_press.slc")){width=4;group=128;order=9;}
    else if(!strcmp(name,"stts22h_temp.slc")){width=4;group=128;order=10;}
    else if(strcmp(name,"imp23absu_mic.slc")){dim=1;width=bps;order=0;}
    if(dim*width!=bps){dim=1;width=bps;order=0;}
    if(!bps || bps>255U || sd_diag.streams[sid].samples_per_ts>65535U || c->cb.item_size>2U*RS_CHUNK_BYTES)
      return stock_error(r,RP_ADMISSION,sid,-1);
    s->schema=(sl_schema){c->cb.item_size,dim,sd_diag.streams[sid].samples_per_ts,width};
    s->normal=(rc_profile){group,order,1};s->pressure=(rc_profile){128,order==4U?4U:1U,1};s->version=4;
  }
  if(configured!=10U)return stock_error(r,RP_ADMISSION,RS_STREAMS,(int)configured);
  UINT st=tx_event_flags_create(&r->event,"stock codec worker");
  if(st)return stock_error(r,RP_WORKER_WAKE,RS_STREAMS,(int)st);
  r->event_created=1;
  st=tx_thread_create(&r->thread,"stock independent codec",stock_worker,(ULONG)r,r->stack,
                      STOCK_WORKER_STACK,30U,30U,TX_NO_TIME_SLICE,TX_DONT_START);
  if(st)return stock_error(r,RP_WORKER_WAKE,RS_STREAMS,(int)st);
  r->thread_created=1;rh_open();
  for(uint32_t sid=0;sid<RS_STREAMS;++sid)
    if(r->pipeline.streams[sid].schema.item_size && stock_output(r,sid,0))return -1;
  uint32_t start=tx_time_get();
  while(!r->pipeline.error && !rp_drained(&r->pipeline)) {
    if((uint32_t)(tx_time_get()-start)>STOCK_DRAIN_TICKS)return stock_error(r,RP_SD_WRITE,RS_STREAMS,TX_NOT_AVAILABLE);
    tx_thread_sleep(1);
  }
  if(r->pipeline.error)return -1;
  r->accepting=1;st=tx_thread_resume(&r->thread);
  return st?stock_error(r,RP_WORKER_WAKE,RS_STREAMS,(int)st):0;
}
typedef struct {CircularBufferDL2 *cb;CBItem *item;uint32_t partial;} stock_release_ref;
static int stock_release(void *arg) {
  stock_release_ref *ref=arg;
  int st=sd_diag_release((CircularBuffer *)ref->cb,ref->item);
  if(!st && ref->partial){ref->cb->p_current_item=NULL;ref->cb->item_idx=0;}
  return st;
}
static int stock_copy_ready(uint32_t sid,uint32_t stopped) {
  stock_runtime *r=stock_record;CircularBufferDL2 *c=r->owner->cbdl2[sid];if(!c)return 0;
  for(;;) {
    if(r->pipeline.error)return -1;
    if(stopped) {
      uint32_t start=tx_time_get();
      for(;;) {
        uint32_t key=stock_lock(r);int room=rs_can_reserve(&r->pipeline.stage,sid,c->cb.item_size);stock_unlock(r,key);
        if(!room)break;
        if(room<0 || r->pipeline.error || (uint32_t)(tx_time_get()-start)>STOCK_DRAIN_TICKS)
          return stock_error(r,RP_ADMISSION,sid,room);
        tx_thread_sleep(1);
      }
    }
    CBItem *item;
    if(CB_GetReadyItemFromTail((CircularBuffer *)c,&item)!=SYS_NO_ERROR_CODE)return 0;
    stock_release_ref ref={c,item,0};
    if(rp_collect(&r->pipeline,sid,CB_GetItemData(item),c->cb.item_size,stock_release,&ref))return -1;
  }
}
static int stock_record_stop(void) {
  stock_runtime *r=stock_record;if(!stock_record_enabled || !r)return 0;
  r->stop_tick=tx_time_get();r->accepting=0;
  if(r->pipeline.error)return -1;
  for(uint32_t sid=0;sid<RS_STREAMS;++sid) {
    CircularBufferDL2 *c=r->owner->cbdl2[sid];if(!c)continue;
    if(stock_copy_ready(sid,1))return -1;
    if(c->p_current_item && c->item_idx>4U) {
      uint32_t start=tx_time_get();
      for(;;) {
        uint32_t key=stock_lock(r);int room=rs_can_reserve(&r->pipeline.stage,sid,c->item_idx);stock_unlock(r,key);
        if(!room)break;
        if(room<0 || r->pipeline.error || (uint32_t)(tx_time_get()-start)>STOCK_DRAIN_TICKS)
          return stock_error(r,RP_ADMISSION,sid,room);
        tx_thread_sleep(1);
      }
      CBItem *item=c->p_current_item;uint32_t n=c->item_idx;
      if(CB_SetItemReady((CircularBuffer *)c,item)!=SYS_NO_ERROR_CODE)return stock_error(r,RP_ACQUISITION_RELEASE,sid,-1);
      CBItem *ready;
      if(CB_GetReadyItemFromTail((CircularBuffer *)c,&ready)!=SYS_NO_ERROR_CODE || ready!=item)
        return stock_error(r,RP_ACQUISITION_RELEASE,sid,-1);
      stock_release_ref ref={c,item,1};
      if(rp_collect(&r->pipeline,sid,CB_GetItemData(item),n,stock_release,&ref))return -1;
    }
  }
  uint32_t start=tx_time_get();
  while(!r->pipeline.error && !rp_drained(&r->pipeline)) {
    if((uint32_t)(tx_time_get()-start)>STOCK_DRAIN_TICKS)return stock_error(r,RP_SD_WRITE,RS_STREAMS,TX_NOT_AVAILABLE);
    tx_thread_sleep(1);
  }
  if(r->pipeline.error)return -1;
  r->finalize_requested=1;if(stock_wake_worker(r))return stock_error(r,RP_WORKER_WAKE,RS_STREAMS,-1);
  start=tx_time_get();
  while(!r->pipeline.error && !r->finalized) {
    if((uint32_t)(tx_time_get()-start)>STOCK_DRAIN_TICKS)return stock_error(r,RP_HASH,RS_STREAMS,TX_NOT_AVAILABLE);
    tx_thread_sleep(1);
  }
  return r->pipeline.error?-1:0;
}
/* Public startup gate, invoked after all stream allocations but before sensor
 * power-mode activation. An allocation failure must not start acquisition. */
int stock_recording_prepare(void *obj){return stock_record_prepare(obj);}
static void stock_flush_sd_tails(void) {
  stock_runtime *r=stock_record;
  if(!r || r->accepting || !r->finalize_requested || !r->tail_flush_requested ||
     r->pipeline.error || !rp_drained(&r->pipeline))return;
  for(uint32_t sid=0;sid<RS_STREAMS;++sid) {
    int st=rsd_flush(&r->sd_tails,sid,stock_sector_write,r);
    if(st){stock_error(r,RP_SD_WRITE,sid,st);return;}
  }
  r->tail_flush_done=1;r->tail_flush_requested=0;
}
static void stock_record_release_memory(void) {
  if(!stock_record_release_requested)return;
  stock_runtime *r=stock_record;
  if(!r){stock_record_release_done=1;stock_record_release_requested=0;return;}
  if(r->accepting || !r->finalized || !r->tail_flush_done || r->pipeline.error || !rp_drained(&r->pipeline)) {
    stock_record_release_error=TX_NOT_AVAILABLE;stock_record_release_requested=0;return;
  }
  if(r->thread.tx_thread_state!=TX_COMPLETED)return;
  UINT st=tx_thread_delete(&r->thread);
  if(!st)st=tx_event_flags_delete(&r->event);
  if(st){stock_record_release_error=st;stock_record_release_requested=0;return;}
  stock_record=NULL;SysFree(r);stock_record_enabled=0;
  st=sd_diag_release_memory();
  if(st){stock_record_release_error=st;stock_record_release_requested=0;return;}
  stock_record_release_done=1;stock_record_release_requested=0;
}
