#ifndef LOSSLESS_CLIENT_H
#define LOSSLESS_CLIENT_H
#include "sensor_lossless.h"
#define LR_STREAMS 16
#define LR_BLOCK 65536U
typedef struct { char path[128]; uint32_t size; sl_schema schema; } lr_stream_t;
typedef struct {
  uint32_t status, start_tick, send_tick, ack_tick, persisted_tick, finish_tick;
  uint32_t chunks, raw_fallbacks, resumed_streams, completed_mask, committed, persisted;
  uint32_t ram_bytes, producer_status, cleanup_status, min_free_ram, retransmits;
  uint64_t raw_bytes, wire_bytes, read_bytes, encoded_bytes;
  uint64_t read_cycles, hash_cycles, codec_cycles, prefix_bytes, resume_bytes;
} lr_profile_t;
extern lr_stream_t lr_streams[LR_STREAMS];
extern volatile lr_profile_t lr_profile;
extern volatile uint32_t lr_count, lr_raw_only;
extern volatile uint64_t lr_acquisition;
extern uint8_t lr_manifest_sha[32];
extern char lr_receipt_path[128];
#endif
