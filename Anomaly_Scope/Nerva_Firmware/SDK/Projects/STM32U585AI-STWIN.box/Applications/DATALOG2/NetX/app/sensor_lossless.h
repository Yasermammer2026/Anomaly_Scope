#ifndef SENSOR_LOSSLESS_H
#define SENSOR_LOSSLESS_H
#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t item_size;       /* Native SD item, including its four-byte counter. */
    uint32_t dim;
    uint32_t samples_per_ts;  /* Zero means no timestamp words. */
    uint32_t scalar_bytes;    /* Two or four. Float samples use their exact bits. */
} sl_schema;

/* Independent blocks, modular sample delta, ZigZag and 128-value bit groups.
 * Counters and timestamps are literal. Wire format matches compression_corpus.py.
 * work_size >= raw_size; output_size >= raw_size + raw_size/256 + 1024.
 * On expansion, return the original bytes (codec=0). Otherwise codec=3.
 * Returns zero on success, -1 on invalid schema, -2 on insufficient capacity.
 * Output must not alias raw or work. No allocation and no global mutable state. */
int sl_encode(const uint8_t *raw, size_t raw_size, uint64_t raw_offset,
              const sl_schema *schema, uint8_t *work, size_t work_size,
              uint8_t *output, size_t output_size, size_t *encoded_size,
              uint32_t *codec);
/* Host decoder: codec 0 or 3, exact raw layout. Work/output >= raw_size.
 * -3 indicates malformed encoded data. No allocation. */
int sl_decode(const uint8_t *encoded, size_t encoded_size, size_t raw_size,
              uint64_t raw_offset, const sl_schema *schema, uint32_t codec,
              uint8_t *work, size_t work_size, uint8_t *output, size_t output_size);
/* Host experiments may LZ4-compress the bit-packed intermediate before deciding
 * whether to fall back to raw. This function retains an expanded intermediate. */
int sl_encode_packed(const uint8_t *raw,size_t raw_size,uint64_t raw_offset,
                    const sl_schema *schema,uint8_t *work,size_t work_size,
                    uint8_t *output,size_t output_size,size_t *encoded_size,uint32_t *codec);
int sl_transform(const uint8_t *raw,size_t size,uint64_t offset,const sl_schema *schema,
                 uint32_t order,uint32_t inverse,uint8_t *output,size_t output_size);
#endif
