#include "record_chunk.h"
#include "crc32_fast.h"
#include <string.h>
#include <limits.h>

extern int ssc_encode_grouped(const uint8_t *,size_t,uint64_t,const sl_schema *,
                             uint32_t,uint32_t,uint32_t,uint8_t *,size_t,size_t *);
static uint32_t crc(const void *p,uint32_t n) {
  return stock_crc32_update(UINT32_MAX,p,n)^UINT32_MAX;
}
static void put16(uint8_t *p,uint32_t v) {p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void put32(uint8_t *p,uint32_t v) {
  put16(p,v);put16(p+2,v>>16);
}
static void put64(uint8_t *p,uint64_t v) {put32(p,(uint32_t)v);put32(p+4,(uint32_t)(v>>32));}
static int overlaps(const void *a,size_t an,const void *b,size_t bn) {
  uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;
  if(x>UINTPTR_MAX-an || y>UINTPTR_MAX-bn)return 1;
  return x<y+bn && y<x+an;
}
static int layout_valid(const sl_schema *s) {
  return s->item_size>=8U && s->item_size<=131072U && s->dim && s->dim<=16U &&
         (s->scalar_bytes==2U || s->scalar_bytes==4U) &&
         !(s->item_size%s->scalar_bytes) && s->samples_per_ts<=65535U;
}
static int profile_valid(rc_profile p) {
  return p.order>=1U && p.order<=10U && p.coding<=1U &&
         (p.group==16U || p.group==32U || p.group==64U || p.group==128U || p.group==256U);
}
static int stream_valid(const rc_stream *s) {
  return s && !s->closed && s->schema.item_size>4U && s->schema.item_size<=131072U &&
         s->schema.dim<=255U && s->schema.scalar_bytes<=255U &&
         s->schema.samples_per_ts<=65535U && (s->version==0U || s->version==2U || s->version==3U || s->version==4U);
}
static uint32_t header_bytes(const rc_stream *s){return s->version>=3U?RC_BLOCK3_BYTES:RC_HEADER_BYTES;}
static uint32_t padded_bytes(const rc_stream *s,uint32_t n){return s->version==4U?(n+3U)&~3U:n;}
static void header(uint8_t *p,const rc_stream *s,uint32_t stream,uint32_t codec,
                   uint32_t n,uint32_t enc,uint32_t raw_crc) {
  if(s->version>=3U && codec!=65534U) {
    memcpy(p,s->version==4U?"SLB4":"SLB3",4);put16(p+4,s->version);put16(p+6,codec);
    put32(p+8,s->next_sequence);put32(p+12,n);put32(p+16,enc);
    put32(p+20,raw_crc);put32(p+24,crc(p,24U));return;
  }
  memcpy(p,s->version==4U?"SLF4":(s->version==3U?"SLF3":"SLR1"),4);
  put16(p+4,s->version>=3U?s->version:2U);put16(p+6,codec==65534U?0U:codec);
  put32(p+8,stream);put32(p+12,s->next_sequence);put64(p+16,s->next_offset);
  put32(p+24,n);put32(p+28,enc);put32(p+32,raw_crc);
  put32(p+36,s->schema.item_size);
  put32(p+40,s->schema.dim|(s->schema.scalar_bytes<<8)|(s->schema.samples_per_ts<<16));
  put32(p+44,crc(p,44U));
}
int rc_frame(const rc_stream *s,const rs_chunk *c,const uint8_t *raw,
             uint32_t pressure,uint8_t *out,size_t cap,rc_result *r) {
  if(!stream_valid(s) || !c || !raw || !out || !r || c->stream>=RS_STREAMS ||
     c->state!=RS_ENCODING || !c->bytes || c->bytes>RS_CHUNK_BYTES || pressure>RS_CRITICAL ||
     c->offset!=s->next_offset || c->sequence!=s->next_sequence ||
     c->sequence==UINT32_MAX || c->offset>UINT64_MAX-c->bytes ||
     (s->version>=3U && !s->started) ||
     cap<header_bytes(s)+padded_bytes(s,c->bytes) || overlaps(raw,c->bytes,out,cap))return -1;
  size_t encoded=c->bytes;uint32_t codec=0U,fallback;
  uint8_t *payload=out+header_bytes(s);
  if(pressure==RS_CRITICAL)fallback=RC_CRITICAL_RAW;
  else if(!layout_valid(&s->schema) || c->bytes%s->schema.scalar_bytes ||
          c->offset%s->schema.scalar_bytes)fallback=RC_LAYOUT_RAW;
  else {
    rc_profile p=pressure==RS_PRESSURE?s->pressure:s->normal;
    if(!p.order)fallback=pressure==RS_PRESSURE?RC_PRESSURE_RAW:RC_LAYOUT_RAW;
    else {
      if(!profile_valid(p))return -1;
      int st=ssc_encode_grouped(raw,c->bytes,c->offset,&s->schema,p.order,p.coding,
                                p.group,payload,c->bytes,&encoded);
      if(st<0)return st;
      if(st==0) {
        if(!encoded || encoded>=c->bytes)return -2;
        codec=6U;fallback=RC_ENCODED;
      } else if(st==1){encoded=c->bytes;fallback=RC_EXPANSION;}
      else return -2;
    }
  }
  if(!codec){encoded=c->bytes;memcpy(payload,raw,c->bytes);}
  header(out,s,c->stream,codec,c->bytes,(uint32_t)encoded,crc(raw,c->bytes));
  uint32_t physical=padded_bytes(s,(uint32_t)encoded);
  memset(payload+encoded,0,physical-(uint32_t)encoded);
  *r=(rc_result){header_bytes(s)+physical,c->bytes,codec,fallback};
  return 0;
}
int rc_accept(rc_stream *s,const rs_chunk *c) {
  if(!stream_valid(s) || !c || c->state!=RS_ENCODING || !c->bytes || c->bytes>RS_CHUNK_BYTES ||
     c->stream>=RS_STREAMS || c->offset!=s->next_offset || c->sequence!=s->next_sequence ||
     c->sequence==UINT32_MAX || c->offset>UINT64_MAX-c->bytes ||
     (s->version>=3U && !s->started))return -1;
  s->next_offset+=c->bytes;++s->next_sequence;return 0;
}
int rc_footer(const rc_stream *s,uint32_t stream,const uint8_t digest[32],
              uint8_t *out,size_t cap,rc_result *r) {
  if(!stream_valid(s) || stream>=RS_STREAMS || !digest || !out || !r ||
     (s->version>=3U && !s->started) ||
     cap<header_bytes(s)+32U || overlaps(digest,32U,out,cap))return -1;
  memcpy(out+header_bytes(s),digest,32U);
  header(out,s,stream,65535U,0U,32U,crc(digest,32U));
  *r=(rc_result){header_bytes(s)+32U,0U,65535U,0U};return 0;
}
int rc_begin(const rc_stream *s,uint32_t stream,uint8_t *out,size_t cap,rc_result *r) {
  if(!stream_valid(s) || s->version<3U || s->started || s->next_offset ||
     s->next_sequence || stream>=RS_STREAMS || !out || !r || cap<RC_HEADER_BYTES)return -1;
  header(out,s,stream,65534U,0U,0U,0U);
  *r=(rc_result){RC_HEADER_BYTES,0U,65534U,0U};return 0;
}
int rc_start(rc_stream *s) {
  if(!stream_valid(s) || s->version<3U || s->started || s->next_offset || s->next_sequence)return -1;
  s->started=1U;return 0;
}
int rc_close(rc_stream *s) {
  if(!stream_valid(s))return -1;
  s->closed=1U;return 0;
}
