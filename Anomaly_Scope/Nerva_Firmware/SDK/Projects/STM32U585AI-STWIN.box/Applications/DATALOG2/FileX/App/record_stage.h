#ifndef STOCK_RECORD_STAGE_H
#define STOCK_RECORD_STAGE_H
#include <stdint.h>

/* Isolated preparation, not integrated into the recorder. Mutating calls must
 * be protected by the caller's short IRQ lock. Copy/codec/SD run outside it.
 * No acquisition pointer is stored. A reservation is all-or-nothing for the
 * completed item. The caller copies, releases the acquisition item, publishes,
 * then wakes a lower-priority worker. Failure is a qualification failure.
 */
#define RS_UNIT_BYTES 1024U
#define RS_UNITS 256U
#define RS_BITMAP_WORDS 4U
#define RS_ARENA_BYTES (RS_UNIT_BYTES * RS_UNITS)
#define RS_CHUNK_BYTES 32768U
#define RS_DESCRIPTORS 128U
#define RS_MAX_ITEM_CHUNKS ((2U*RS_CHUNK_BYTES+RS_UNIT_BYTES-1U)/RS_UNIT_BYTES)
#define RS_STREAMS 16U
enum rs_state { RS_FREE, RS_COPY, RS_READY, RS_ENCODING, RS_ENCODED };
enum rs_pressure { RS_NORMAL, RS_PRESSURE, RS_CRITICAL };
typedef struct {
  uint8_t first_unit, units, state, stream;
  uint32_t bytes, sequence;
  uint64_t offset, ticket;
} rs_chunk;
typedef struct { uint32_t first, count; uint64_t ticket; } rs_reservation;
typedef struct {
  uint64_t occupied[RS_BITMAP_WORDS], next_ticket;
  rs_chunk chunks[RS_DESCRIPTORS];
  uint64_t next_offset[RS_STREAMS];
  uint32_t next_sequence[RS_STREAMS];
  uint32_t head, tail, count, used_units, high_units, high_descriptors;
  uint32_t failed_reservations, invalid_transitions;
  uint64_t first_failed_occupied[RS_BITMAP_WORDS];
  uint32_t first_failed_bytes,first_failed_used_units,first_failed_descriptors;
} rs_stage;
void rs_init(rs_stage *s);
uint32_t rs_level(const rs_stage *s);
/* 0 reserved; 1 finite capacity exhausted; -1 invalid. No wait/allocation.
 * Admission may split across free contiguous spans. Separate finite byte and descriptor
 * capacities are checked before any mutation; neither permits producer waits.
 * The whole native item is admitted atomically before any source copy. */
int rs_reserve(rs_stage *s,uint32_t stream,uint32_t bytes,rs_reservation *r);
/* Read-only admission probe under the same short lock. Stop finalization may
 * wait after fencing acquisition; live producers must never wait on this. */
int rs_can_reserve(const rs_stage *s,uint32_t stream,uint32_t bytes);
int rs_publish(rs_stage *s,const rs_reservation *r);
/* 0 ready head acquired; 1 no published FIFO head. */
int rs_take(rs_stage *s,uint32_t *index);
/* Encode the oldest remaining RAW descriptor without overtaking an
 * unpublished producer. Earlier encoded descriptors may await SD output. */
int rs_take_encode(rs_stage *s,uint32_t *index);
/* After source hash, independent encoding and copy-back, shrink this owned
 * descriptor and publish its encoded payload under the caller's short lock. */
int rs_pack(rs_stage *s,uint32_t index,uint64_t ticket,uint32_t payload_bytes);
/* Only after all source reads and an independent output publication succeed. */
int rs_release(rs_stage *s,uint32_t index,uint64_t ticket);
uint8_t *rs_data(const rs_stage *s,uint8_t *independent_arena,uint32_t index);

enum rs_output_state { RS_OUT_FREE, RS_OUT_FILL, RS_OUT_READY, RS_OUT_WRITE };
typedef struct {
  uint64_t ticket;
  uint32_t state, stream, physical_bytes, native_bytes;
} rs_output;
typedef struct {
  rs_output slots[2];
  uint64_t next_ticket;
  uint32_t head, tail, count, high, invalid_transitions;
} rs_writer;
void rs_writer_init(rs_writer *w);
int rs_output_reserve(rs_writer *w,uint32_t *index,uint64_t *ticket);
int rs_output_publish(rs_writer *w,uint32_t index,uint64_t ticket,uint32_t stream,
                      uint32_t physical_bytes,uint32_t native_bytes);
int rs_output_take(rs_writer *w,uint32_t *index,uint64_t *ticket);
/* A failed SD write must retain ownership for first-failure evidence. */
int rs_output_release(rs_writer *w,uint32_t index,uint64_t ticket);
#endif
