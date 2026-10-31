#ifndef STOCK_RECORD_PIPELINE_H
#define STOCK_RECORD_PIPELINE_H
#include "record_chunk.h"

/* Prepared portable ownership engine; not linked into the recorder.
 * Collector: copy/release/publish only. Worker: CRC/codec/source hash.
 * Writer: SD callback only. Caller supplies three distinct backing areas and
 * short IRQ locks. All waits/scheduling belong to low-priority integration. */
typedef struct {
  void *arg;
  uint32_t (*lock)(void *);
  void (*unlock)(void *,uint32_t);
  int (*hash)(void *,uint32_t,const uint8_t *,uint32_t);
  int (*write)(void *,uint32_t,const uint8_t *,uint32_t);
  int (*wake_worker)(void *);
  int (*wake_writer)(void *);
  void (*failed)(void *,uint32_t,uint32_t,int);
} rp_hooks;
enum rp_error { RP_ADMISSION=1,RP_ACQUISITION_RELEASE,RP_STAGE_PUBLISH,
                RP_WORKER_WAKE,RP_STAGE_TAKE,RP_OUTPUT_RESERVE,RP_FRAME,
                RP_HASH,RP_SOURCE_ACCEPT,RP_OUTPUT_PUBLISH,RP_WRITER_WAKE,
                RP_STAGE_RELEASE,RP_OUTPUT_TAKE,RP_SD_WRITE,RP_OUTPUT_RELEASE };
typedef struct {
  rs_stage stage;
  rs_writer writer;
  rc_stream streams[RS_STREAMS];
  rp_hooks hooks;
  uint8_t *arena,*output[2];
  uint64_t copied[RS_STREAMS],hashed[RS_STREAMS],persisted[RS_STREAMS];
  uint64_t physical[RS_STREAMS];
  uint32_t fallback[5];
  volatile uint32_t error;
  uint32_t failed_stream;
  int failure_detail;
  uint32_t worker_held,worker_index,collector_active;
  uint8_t *scratch;
  rc_result packed[RS_DESCRIPTORS];
  uint8_t packed_header[RS_DESCRIPTORS][RC_HEADER_BYTES];
  uint32_t packed_header_bytes[RS_DESCRIPTORS];
  uint32_t packed_blocks,packed_saved_units;
} rp_pipeline;
int rp_init(rp_pipeline *p,const rp_hooks *hooks,uint8_t *arena,
            uint8_t *out0,uint8_t *out1,const rc_stream streams[RS_STREAMS]);
/* Optional independent scratch permits preparing encoded staging while both
 * SD output slots are busy. Configure before accepting any producer. */
int rp_enable_packing(rp_pipeline *,uint8_t *,uint32_t);
/* release called exactly once after all bytes are copied, before publication.
 * Callback argument exists only on this call's stack; it is never retained.
 * Nonzero return is a first failure, never a silent drop or producer wait. */
int rp_collect(rp_pipeline *p,uint32_t stream,const uint8_t *acquisition,uint32_t n,
               int (*release)(void *),void *release_arg);
/* 0 made progress; 1 no available input/output; negative first failure.
 * Only the dedicated worker calls worker_step, only SD task calls writer_step. */
int rp_worker_step(rp_pipeline *p);
int rp_writer_step(rp_pipeline *p);
int rp_drained(rp_pipeline *p);
#endif
