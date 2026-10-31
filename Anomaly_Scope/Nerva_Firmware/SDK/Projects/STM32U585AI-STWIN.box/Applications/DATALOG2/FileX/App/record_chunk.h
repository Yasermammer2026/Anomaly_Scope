#ifndef STOCK_RECORD_CHUNK_H
#define STOCK_RECORD_CHUNK_H
#include "record_stage.h"
#include "sensor_lossless.h"
#include <stddef.h>

/* Preparation only. Called by the low-priority worker on an independent
 * staging copy and independent output slot. Never called by acquisition.
 * No allocation, wait, hash peripheral, SD access or saved source pointer. */
#define RC_HEADER_BYTES 48U
#define RC_OUTPUT_BYTES (RS_CHUNK_BYTES+RC_HEADER_BYTES)
#define RC_FOOTER_BYTES 80U
#define RC_BLOCK3_BYTES 28U
enum rc_fallback { RC_ENCODED, RC_EXPANSION, RC_PRESSURE_RAW,
                   RC_CRITICAL_RAW, RC_LAYOUT_RAW };
typedef struct { uint32_t group,order,coding; } rc_profile;
typedef struct {
  sl_schema schema;
  rc_profile normal,pressure;
  uint64_t next_offset;
  uint32_t next_sequence,closed,version,started;
} rc_stream;
typedef struct {
  uint32_t physical_bytes,native_bytes,codec,fallback;
} rc_result;
/* CRC uses the separately verified table implementation. Frame layout is
 * explicitly little-endian, independent of native struct alignment. */
int rc_frame(const rc_stream *s,const rs_chunk *chunk,const uint8_t *raw,
             uint32_t pressure,uint8_t *output,size_t capacity,rc_result *r);
/* Only advance after independent writer publication and source-hash success. */
int rc_accept(rc_stream *s,const rs_chunk *chunk);
/* Versions 3/4 store immutable source layout once, retaining per-block header
 * CRC, sequence, sizes, source CRC and final independently accumulated SHA.
 * Version 4 adds checked zero padding after payloads to preserve word-aligned
 * FileX direct-sector pointers; payload length/source/codec remain unchanged. */
int rc_begin(const rc_stream *s,uint32_t stream,uint8_t *output,size_t capacity,rc_result *r);
int rc_start(rc_stream *s);
/* Caller supplies the independently accumulated native digest after drain. */
int rc_footer(const rc_stream *s,uint32_t stream,const uint8_t digest[32],
              uint8_t *output,size_t capacity,rc_result *r);
int rc_close(rc_stream *s);
#endif
