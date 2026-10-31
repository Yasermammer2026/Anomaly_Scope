#ifndef SENSOR_STREAM_CODEC_H
#define SENSOR_STREAM_CODEC_H
#include "sensor_lossless.h"

/* Experimental independent-block codec. No input copy, heap allocation or
 * mutable globals. Predictor history + residual array require 1,344 bytes, plus locals/call frames.
 * Output cannot alias input. If capacity is exhausted or compression expands,
 * return 1: the caller writes its independent staged raw bytes as codec RAW.
 * Return 0 for compressed output and -1 for invalid arguments.
 * order 1/2/3: fixed modular predictors; 4..10: experimental stationary predictors. coding 0=bitpack, 1=Rice.
 * This format is separate from qualified codec 3; not enabled in firmware. */
int ssc_encode(const uint8_t *raw,size_t size,uint64_t offset,const sl_schema *schema,
               uint32_t order,uint32_t coding,uint8_t *out,size_t capacity,size_t *used);
/* Strict host decoder. Return 0 for exact reconstruction, -1 invalid args,
 * -3 malformed input. Compressed streams have a 12-byte format header. */
int ssc_encode_grouped(const uint8_t *raw,size_t size,uint64_t offset,const sl_schema *schema,
               uint32_t order,uint32_t coding,uint32_t group,uint8_t *out,size_t capacity,size_t *used);
int ssc_decode(const uint8_t *encoded,size_t encoded_size,size_t raw_size,
               uint64_t offset,const sl_schema *schema,uint8_t *raw,size_t capacity);
#endif
