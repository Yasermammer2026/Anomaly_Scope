/* Disabled acquisition-time experiment. Included by sd_diag.c after its helpers.
 * One bounded output buffer. Input remains owned by its original CB item until
 * the framed SD write finishes. FileX queue remains the only bounded queue.
 * No allocation, printf or additional producer wait occurs in the hot path. */
#include "sensor_stream_codec.h"
#define _NX_CRYPTO_INITIALIZE_
#include "nx_crypto_sha2.h"
#undef _NX_CRYPTO_INITIALIZE_
/* Use the bundled implementation without changing its source/license. */
#include "../../../../../../Middlewares/ST/netxduo/crypto_libraries/src/nx_crypto_sha2.c"

#define SRC_BLOCK 32768U
#define SRC_OUTPUT 16384U
typedef struct {
  uint32_t magic; uint16_t version,codec; uint32_t stream,sequence;
  uint64_t offset; uint32_t raw,encoded,raw_crc,item_size,layout,header_crc;
} src_header_t;
_Static_assert(sizeof(src_header_t)==48,"record chunk ABI");
typedef struct {
  uint64_t raw,physical,codec_cycles;
  uint32_t blocks,compressed,raw_fallbacks,pressure_fallbacks,max_codec_cycles;
  uint32_t item_size,dim,scalar_bytes,samples_per_ts,complete;
  uint8_t sha256[32];
} src_stream_t;
typedef struct {
  uint32_t enabled,active,allocation_failed,status,priority_errors;
  uint32_t output_bytes,context_bytes,queue_high,backlog_high,min_pool;
  uint32_t order,coding,worker_priority;
  src_stream_t streams[SD_DIAG_STREAMS];
} src_state_t;
volatile src_state_t record_codec __attribute__((section(".transfer_bss")));
static uint8_t *src_output;
static NX_CRYPTO_SHA256 *src_hashes;
static uint32_t src_initialized[SD_DIAG_STREAMS],src_last_physical;
extern uint32_t transfer_bench_crc_update(uint32_t,const uint8_t *,uint32_t);
static uint32_t src_crc(const void *p,uint32_t n) {
  return transfer_bench_crc_update(UINT32_MAX,p,n)^UINT32_MAX;
}
static void src_begin(void) {
  uint32_t enabled=record_codec.enabled,order=record_codec.order,coding=record_codec.coding;
  memset((void *)&record_codec,0,sizeof(record_codec));
  record_codec.enabled=enabled;record_codec.order=order?order:1U;
  record_codec.coding=coding;record_codec.worker_priority=13U;
  record_codec.min_pool=UINT32_MAX;memset(src_initialized,0,sizeof(src_initialized));
  if(!enabled)return;
  src_output=SysAlloc(SRC_OUTPUT);
  src_hashes=SysAlloc(sizeof(NX_CRYPTO_SHA256)*SD_DIAG_STREAMS);
  if(!src_output || !src_hashes){
    if(src_output)SysFree(src_output);
    if(src_hashes)SysFree(src_hashes);
    src_output=NULL;src_hashes=NULL;record_codec.allocation_failed=1;
    return; /* Stay on the original .dat path if start resources are absent. */
  }
  record_codec.output_bytes=SRC_OUTPUT;
  record_codec.context_bytes=sizeof(NX_CRYPTO_SHA256)*SD_DIAG_STREAMS;
  record_codec.active=1;
}
static void src_sample(void) {
  if(!record_codec.active || !owner)return;
  uint32_t q=depth(),backlog=0;
  if(q>record_codec.queue_high)record_codec.queue_high=q;
  for(uint32_t s=0;s<SD_DIAG_STREAMS;++s){
    CircularBufferDL2 *c=owner->cbdl2[s];if(!c)continue;
    for(uint32_t i=0;i<c->cb.item_count;++i)
      if(c->cb.p_items[i].status.status==2U)backlog+=c->cb.item_size;
  }
  if(backlog>record_codec.backlog_high)record_codec.backlog_high=backlog;
  if(sd_diag_pool && sd_diag_pool->tx_byte_pool_available<record_codec.min_pool)
    record_codec.min_pool=sd_diag_pool->tx_byte_pool_available;
}
static void src_schema(uint32_t s) {
  volatile src_stream_t *v=&record_codec.streams[s];
  v->item_size=owner->cbdl2[s]->cb.item_size;
  /* Known int16 streams use the actual recorded item/TS configuration.
   * Unknown or changed sample types select RAW. Every block stores layout. */
  const char *name=owner->file_dat_name[s];
  if(!strcmp(name,"imp23absu_mic.slc") || !strcmp(name,"imp34dt05_mic.slc"))v->dim=1;
  else if(!strcmp(name,"iis3dwb_acc.slc") || !strcmp(name,"ism330dhcx_acc.slc") ||
          !strcmp(name,"ism330dhcx_gyro.slc") || !strcmp(name,"iis2dlpc_acc.slc"))v->dim=3;
  else if(!strcmp(name,"iis2iclx_acc.slc"))v->dim=2;
  if(v->dim && sd_diag.streams[s].bytes_per_sample==v->dim*2U &&
     sd_diag.streams[s].samples_per_ts<=65535U){
    v->scalar_bytes=2;v->samples_per_ts=sd_diag.streams[s].samples_per_ts;
  } else v->dim=0;
}
static UINT src_write(FX_FILE *f,uint32_t s,const uint8_t *p,uint32_t count) {
  volatile src_stream_t *v=&record_codec.streams[s];src_last_physical=0;
  if(!src_initialized[s]){
    UINT st=_nx_crypto_sha256_initialize(&src_hashes[s],NX_CRYPTO_HASH_SHA256);
    if(st)return FX_IO_ERROR;
    src_schema(s);src_initialized[s]=1;
  }
  for(uint32_t pos=0;pos<count;){
    uint32_t n=count-pos>SRC_BLOCK?SRC_BLOCK:count-pos;
    uint32_t q=depth(),pressure=sd_diag.frozen || q>=owner->fx_app_queue.tx_queue_capacity/2U;
    CircularBufferDL2 *c=owner->cbdl2[s];uint32_t free=0;
    for(uint32_t i=0;i<c->cb.item_count;++i)free+=c->cb.p_items[i].status.status==0U;
    pressure|=free<2U;src_sample();
    UINT original=0;
    TX_THREAD *worker=_tx_thread_current_ptr;
    UINT saved_threshold=worker->tx_thread_user_preempt_threshold;
    UINT target=worker->tx_thread_user_priority>13U?worker->tx_thread_user_priority:13U;
    UINT st=tx_thread_priority_change(worker,target,&original);
    if(st){record_codec.priority_errors++;return FX_IO_ERROR;}
    size_t encoded=n;uint32_t codec=0;uint32_t begin=DWT->CYCCNT;
    if(!pressure && v->dim){
      sl_schema schema={v->item_size,v->dim,v->samples_per_ts,v->scalar_bytes};
      int result=ssc_encode(p+pos,n,v->raw,&schema,record_codec.order,record_codec.coding,src_output,SRC_OUTPUT,&encoded);
      if(!result)codec=6;else encoded=n; /* RAW selection on capacity/invalid input. */
    }
    uint32_t cycles=DWT->CYCCNT-begin;v->codec_cycles+=cycles;
    if(cycles>v->max_codec_cycles)v->max_codec_cycles=cycles;
    UINT hash_status=_nx_crypto_sha256_update(&src_hashes[s],(UCHAR *)(p+pos),n);
    uint32_t crc=src_crc(p+pos,n);UINT discarded;
    UINT restored=tx_thread_priority_change(worker,original,&discarded);
    if(!restored)restored=tx_thread_preemption_change(worker,saved_threshold,&discarded);
    if(restored){record_codec.priority_errors++;return FX_IO_ERROR;}
    if(hash_status)return FX_IO_ERROR;
    src_header_t h={0x31524c53U,1,(uint16_t)codec,s,v->blocks,v->raw,n,(uint32_t)encoded,crc,
                    v->item_size,v->dim|(v->scalar_bytes<<8)|(v->samples_per_ts<<16),0};
    h.header_crc=src_crc(&h,44);
    st=fx_file_write(f,&h,sizeof(h));
    if(!st)st=fx_file_write(f,codec?src_output:(VOID *)(p+pos),encoded);
    if(st){record_codec.status=st;return st;}
    v->raw+=n;v->physical+=sizeof(h)+encoded;src_last_physical+=sizeof(h)+encoded;
    v->blocks++;v->compressed+=codec!=0;v->raw_fallbacks+=codec==0;
    v->pressure_fallbacks+=pressure;pos+=n;
  }
  return FX_SUCCESS;
}
static UINT src_close(FX_FILE *f,uint32_t s) {
  if(!record_codec.active || s>=SD_DIAG_STREAMS || !src_initialized[s])return FX_SUCCESS;
  volatile src_stream_t *v=&record_codec.streams[s];if(v->complete)return FX_SUCCESS;
  uint8_t digest[32];UINT st=_nx_crypto_sha256_digest_calculate(&src_hashes[s],digest,NX_CRYPTO_HASH_SHA256);
  if(st)return FX_IO_ERROR;
  src_header_t h={0x31524c53U,1,65535,s,v->blocks,v->raw,0,32,src_crc(digest,32),
                  v->item_size,v->dim|(v->scalar_bytes<<8)|(v->samples_per_ts<<16),0};
  h.header_crc=src_crc(&h,44);st=fx_file_write(f,&h,sizeof(h));
  if(!st)st=fx_file_write(f,digest,32);
  if(!st){memcpy((void *)v->sha256,digest,32);v->physical+=80;v->complete=1;}
  else record_codec.status=st;
  return st;
}
static void src_end(void) {
  if(src_output)SysFree(src_output);
    if(src_hashes)SysFree(src_hashes);
  src_output=NULL;src_hashes=NULL;record_codec.active=0;
}
