#include "sensor_lossless.h"
#include <string.h>
#include <limits.h>

#define SL_GROUP 128U
#ifndef SL_MARK
#define SL_MARK(stage) ((void)0)
#endif
static uint32_t load_word(const uint8_t *p, uint32_t width) {
    uint32_t x = (uint32_t)p[0] | ((uint32_t)p[1] << 8);
    if (width == 4) x |= ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return x;
}
static void store_word(uint8_t *p, uint32_t x, uint32_t width) {
    p[0] = (uint8_t)x; p[1] = (uint8_t)(x >> 8);
    if (width == 4) { p[2] = (uint8_t)(x >> 16); p[3] = (uint8_t)(x >> 24); }
}
static uint32_t bit_width(uint32_t value) {
    uint32_t n = 0;
    while (value) { ++n; value >>= 1; }
    return n;
}

static int encode_block(const uint8_t *raw, size_t raw_size, uint64_t raw_offset,
              const sl_schema *s, uint8_t *work, size_t work_size,
              uint8_t *output, size_t output_size, size_t *encoded_size,
              uint32_t *codec, int fallback) {
    uint32_t prior[16] = {0};
    if (!raw || !s || !work || !output || !encoded_size || !codec ||
        (s->scalar_bytes != 2 && s->scalar_bytes != 4) ||
        !s->dim || s->dim > 16 || s->item_size < 8 ||
        s->item_size % s->scalar_bytes || raw_size % s->scalar_bytes ||
        raw_offset % s->scalar_bytes || raw_size > UINT32_MAX ||
        s->samples_per_ts > (UINT32_MAX - 4U) / s->dim) return -1;
    if (work_size < raw_size || raw_size > SIZE_MAX - raw_size/256U - 1024U ||
        output_size < raw_size + raw_size/256U + 1024U) return -2;
    SL_MARK(0);
    const uint32_t width = s->scalar_bytes, bits = width * 8U;
    const uint32_t mask = width == 2 ? 65535U : UINT32_MAX;
    uint32_t within = (uint32_t)(raw_offset % s->item_size);
    uint64_t payload = (raw_offset / s->item_size) * (s->item_size - 4U);
    if (within >= 4) payload += within - 4U;
    payload /= width;
    uint32_t span = s->samples_per_ts * s->dim;
    uint32_t group = s->samples_per_ts ? span + 8U/width : 0;
    uint32_t relative = group ? (uint32_t)(payload % group) : 0;
    uint32_t lane = group ? relative % s->dim : (uint32_t)(payload % s->dim);
    size_t n = 0, literals = 0;
    for (size_t pos = 0; pos < raw_size; pos += width) {
        int is_payload = within >= 4U;
        int is_sample = is_payload && (!group || relative < span);
        if (is_sample) {
            uint32_t value = load_word(raw + pos, width);
            uint32_t delta = (value - prior[lane]) & mask;
            prior[lane] = value;
            uint32_t z = ((delta << 1) ^ (0U - (delta >> (bits - 1U)))) & mask;
            store_word(work + n * width, z, width); ++n;
            if (++lane == s->dim) lane = 0;
        } else {
            memcpy(output + literals, raw + pos, width); literals += width;
        }
        if (is_payload && group) {
            if (++relative == group) { relative = 0; lane = 0; }
        }
        within += width;
        if (within == s->item_size) within = 0;
    }
    SL_MARK(1);
    store_word(output + literals, (uint32_t)n, 4);
    size_t groups = (n + SL_GROUP - 1U) / SL_GROUP;
    uint8_t *widths = output + literals + 4U;
    for (size_t g = 0; g < groups; ++g) {
        uint32_t maximum = 0;
        size_t end = (g + 1U) * SL_GROUP < n ? (g + 1U) * SL_GROUP : n;
        for (size_t i = g * SL_GROUP; i < end; ++i) {
            uint32_t value = load_word(work + i * width, width);
            if (value > maximum) maximum = value;
        }
        widths[g] = (uint8_t)bit_width(maximum);
    }
    SL_MARK(2);
    uint8_t *cursor = widths + groups;
    /* Grouping by width is part of the wire format, not an MCU assumption. */
    for (uint32_t w = 1; w <= bits; ++w) {
        for (size_t g = 0; g < groups; ++g) {
            if (widths[g] != w) continue;
            /* Most 16-bit sensor residuals fit <=16 bits. Avoid software
             * 64-bit shifts on Cortex-M33 when <=31 accumulator bits suffice. */
            if (w <= 24U) {
                uint32_t accumulator = 0, available = 0;
                for (size_t j = 0; j < SL_GROUP; ++j) {
                    size_t i = g * SL_GROUP + j;
                    uint32_t value = i < n ? load_word(work + i * width, width) : 0;
                    accumulator |= value << available; available += w;
                    while (available >= 8) {
                        *cursor++ = (uint8_t)accumulator;
                        accumulator >>= 8; available -= 8;
                    }
                }
                continue;
            }
            uint64_t accumulator = 0; uint32_t available = 0;
            for (size_t j = 0; j < SL_GROUP; ++j) {
                size_t i = g * SL_GROUP + j;
                uint32_t value = i < n ? load_word(work + i * width, width) : 0;
                accumulator |= (uint64_t)value << available; available += w;
                while (available >= 8) {
                    *cursor++ = (uint8_t)accumulator;
                    accumulator >>= 8; available -= 8;
                }
            }
        }
    }
    size_t size = (size_t)(cursor - output);
    if (fallback && size >= raw_size) { memcpy(output, raw, raw_size); size = raw_size; *codec = 0; }
    else *codec = 3;
    *encoded_size = size;
    SL_MARK(3);
    return 0;
}

int sl_encode(const uint8_t *raw, size_t raw_size, uint64_t raw_offset,
              const sl_schema *schema, uint8_t *work, size_t work_size,
              uint8_t *output, size_t output_size, size_t *encoded_size, uint32_t *codec) {
    return encode_block(raw,raw_size,raw_offset,schema,work,work_size,output,output_size,encoded_size,codec,1);
}
int sl_encode_packed(const uint8_t *raw, size_t raw_size, uint64_t raw_offset,
              const sl_schema *schema, uint8_t *work, size_t work_size,
              uint8_t *output, size_t output_size, size_t *encoded_size, uint32_t *codec) {
    return encode_block(raw,raw_size,raw_offset,schema,work,work_size,output,output_size,encoded_size,codec,0);
}
