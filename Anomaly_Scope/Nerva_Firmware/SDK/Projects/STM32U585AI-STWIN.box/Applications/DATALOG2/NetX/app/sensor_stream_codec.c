#include "sensor_stream_codec.h"
#include <string.h>
#include <limits.h>

#ifndef SSC_GROUP
#define SSC_GROUP 128U
#endif
#if SSC_GROUP != 16 && SSC_GROUP != 32 && SSC_GROUP != 64 && SSC_GROUP != 128 && SSC_GROUP != 256
#error Unsupported residual group size
#endif
typedef struct { const sl_schema *s; uint32_t within,relative,lane,span,group; } ssc_layout;
typedef struct { uint8_t *data; size_t cap,pos; uint32_t available,bits; } ssc_writer;
typedef struct { const uint8_t *data; size_t size,pos; uint32_t available; uint64_t bits; } ssc_reader;
static uint32_t ssc_load(const uint8_t *p,uint32_t w) {
  uint32_t v=(uint32_t)p[0]|((uint32_t)p[1]<<8);
  if(w==4)v|=((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
  return v;
}
static void ssc_store(uint8_t *p,uint32_t v,uint32_t w) {
  p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);
  if(w==4){p[2]=(uint8_t)(v>>16);p[3]=(uint8_t)(v>>24);}
}
static int ssc_valid(size_t size,uint64_t offset,const sl_schema *s) {
  return s && size && size<=UINT32_MAX && (s->scalar_bytes==2 || s->scalar_bytes==4)
    && s->dim && s->dim<=16 && s->item_size>=8 && !(s->item_size%s->scalar_bytes)
    && !(size%s->scalar_bytes) && !(offset%s->scalar_bytes)
    && s->samples_per_ts<=(UINT32_MAX-4U)/s->dim;
}
static ssc_layout ssc_begin(uint64_t offset,const sl_schema *s) {
  ssc_layout l={0};l.s=s;l.within=(uint32_t)(offset%s->item_size);
  uint64_t payload=(offset/s->item_size)*(s->item_size-4U);
  if(l.within>=4)payload+=l.within-4U;
  payload/=s->scalar_bytes;l.span=s->samples_per_ts*s->dim;
  l.group=s->samples_per_ts?l.span+8U/s->scalar_bytes:0;
  l.relative=l.group?(uint32_t)(payload%l.group):0;
  l.lane=l.group?l.relative%s->dim:(uint32_t)(payload%s->dim);
  return l;
}
static inline __attribute__((always_inline)) int ssc_sample(ssc_layout *l) {return l->within>=4 && (!l->group || l->relative<l->span);}
static inline __attribute__((always_inline)) void ssc_advance(ssc_layout *l,int is_sample) {
  if(is_sample && ++l->lane==l->s->dim)l->lane=0;
  if(l->within>=4 && l->group && ++l->relative==l->group){l->relative=0;l->lane=0;}
  l->within+=l->s->scalar_bytes;if(l->within==l->s->item_size)l->within=0;
}
static inline __attribute__((always_inline)) uint32_t ssc_predict(uint32_t history[16][5],uint32_t lane,uint32_t order,uint32_t bits) {
  uint32_t *p=history[lane];
  if(order==4U)return 0U;
  if(order==5U)return p[0];
  if(order==8U) {
    int64_t sum=0;
    for(uint32_t i=0;i<4;++i)sum+=bits==16U?(int16_t)p[i]:(int32_t)p[i];
    return (uint32_t)(sum/4);
  }
  if(order>=9U)return p[0];
  if(order>=6U) {
    int64_t a=bits==16U?(int16_t)p[0]:(int32_t)p[0];
    int64_t b=bits==16U?(int16_t)p[1]:(int32_t)p[1];
    return (uint32_t)((a+(order==6U?b:0))/2);
  }
  return order==1?p[0]:(order==2?2U*p[0]-p[1]:3U*p[0]-3U*p[1]+p[2]);
}
static inline __attribute__((always_inline)) void ssc_remember(uint32_t history[16][5],uint32_t lane,uint32_t value,uint32_t order,uint32_t bits) {
  uint32_t *p=history[lane];
  if(order==5U){if(!p[2]){p[0]=value;p[2]=1U;}return;}
  if(order>=9U) {
    if(!p[2]){p[0]=value;p[2]=1U;return;}
    uint32_t delta=value-p[0];
    int32_t signed_delta=bits==16U?(int16_t)delta:(int32_t)delta;
    p[0]=(p[0]+(uint32_t)(order==9U?signed_delta/4:signed_delta/8))&(bits==16U?65535U:UINT32_MAX);
    return;
  }
  if(order==4U)return;
  if(order==1U || order==7U){p[0]=value;return;}
  if(order==2U || order==6U){p[1]=p[0];p[0]=value;return;}
  if(order==3U){p[2]=p[1];p[1]=p[0];p[0]=value;return;}
  p[3]=p[2];p[2]=p[1];p[1]=p[0];p[0]=value;
}
static inline __attribute__((always_inline)) int ssc_put(ssc_writer *w,uint32_t value,uint32_t count) {
  /* At entry available<8. A <=24-bit part keeps every shift within uint32,
   * avoiding the Cortex-M runtime's variable 64-bit shift in the hot path. */
  while(count){
    uint32_t part=count>24U?24U:count;
    w->bits|=(value&((1U<<part)-1U))<<w->available;w->available+=part;
    value>>=part;count-=part;
    while(w->available>=8){
      if(w->pos==w->cap)return 1;
      w->data[w->pos++]=(uint8_t)w->bits;w->bits>>=8;w->available-=8;
    }
  }
  return 0;
}
static int ssc_finish(ssc_writer *w) {
  if(w->available){if(w->pos==w->cap)return 1;w->data[w->pos++]=(uint8_t)w->bits;}
  w->bits=0;w->available=0;return 0;
}
static int ssc_get(ssc_reader *r,uint32_t count,uint32_t *value) {
  if(!count){*value=0;return 0;}
  while(r->available<count){
    if(r->pos==r->size)return -3;
    r->bits|=(uint64_t)r->data[r->pos++]<<r->available;r->available+=8;
  }
  *value=(uint32_t)r->bits & (count==32?UINT32_MAX:((1U<<count)-1U));
  r->bits>>=count;r->available-=count;return 0;
}
static uint32_t ssc_width(uint32_t v){uint32_t n=0;while(v){++n;v>>=1;}return n;}
/* Exact convex Rice cost minimization, seeded near log2(mean). The cost
 * remains identical to the reference; equal costs select the earliest k. */
static uint64_t ssc_rice_cost(const uint32_t *v,uint32_t n,uint32_t k,uint32_t bits) {
  if(bits==16U) {
    uint32_t cost=n*(k+1U);
    for(uint32_t i=0;i<n;++i)cost+=v[i]>>k;
    return cost;
  }
  uint64_t cost=(uint64_t)n*(k+1U);
  for(uint32_t i=0;i<n;++i)cost+=v[i]>>k;
  return cost;
}
static uint32_t ssc_rice_k(const uint32_t *v,uint32_t n,uint32_t bits) {
  uint64_t sum=0;for(uint32_t i=0;i<n;++i)sum+=v[i];
  uint32_t mean=(uint32_t)(sum/n),k=mean?31U-(uint32_t)__builtin_clz(mean):0;
  if(k>=bits)k=bits-1U;
  uint64_t best=ssc_rice_cost(v,n,k,bits);
  while(k) {
    uint64_t lower=ssc_rice_cost(v,n,k-1U,bits);
    if(lower>best)break;
    --k;best=lower;
  }
  while(k+1U<bits) {
    uint64_t higher=ssc_rice_cost(v,n,k+1U,bits);
    if(higher>=best)break;
    ++k;best=higher;
  }
  return k;
}
static int ssc_encode_group(ssc_writer *w,const uint32_t *v,uint32_t n,uint32_t bits,uint32_t coding) {
  uint32_t maximum=0;for(uint32_t i=0;i<n;++i)if(v[i]>maximum)maximum=v[i];
  uint32_t k=coding?ssc_rice_k(v,n,bits):ssc_width(maximum);
  if(w->pos==w->cap)return 1;
  w->data[w->pos++]=(uint8_t)k;
  for(uint32_t i=0;i<n;++i){
    if(coding){
      uint32_t q=v[i]>>k;
      /* Bound unary work by remaining output capacity, before looping. */
      if((uint64_t)q+k+1U>(uint64_t)(w->cap-w->pos)*8U+7U-w->available)return 1;
      if(q+k+1U<=24U) {
        uint32_t low=v[i]&((1U<<k)-1U);
        if(ssc_put(w,(1U<<q)|(low<<(q+1U)),q+k+1U))return 1;
      } else {
        while(q>=32){if(ssc_put(w,0,32))return 1;q-=32;}
        if(ssc_put(w,0,q) || ssc_put(w,1,1) || ssc_put(w,v[i]&((1U<<k)-1U),k))return 1;
      }
    } else if(ssc_put(w,v[i],k))return 1;
  }
  return ssc_finish(w);
}

int ssc_encode_grouped(const uint8_t *raw,size_t size,uint64_t offset,const sl_schema *s,
               uint32_t order,uint32_t coding,uint32_t residual_group,uint8_t *out,size_t cap,size_t *used) {
  if(!raw || !out || !used || !ssc_valid(size,offset,s) || order<1 || order>10 || coding>1)return -1;
  if(residual_group!=16U && residual_group!=32U && residual_group!=64U && residual_group!=128U && residual_group!=256U)return -1;
  *used=size;if(cap<12)return 1;
  ssc_layout l=ssc_begin(offset,s);uint32_t count=0,literals=0;
  for(size_t p=0;p<size;p+=s->scalar_bytes){int a=ssc_sample(&l);if(a)++count;else literals+=s->scalar_bytes;ssc_advance(&l,a);}
  if((size_t)literals+12U>cap)return 1;
  out[0]=order>=4U?3U:(residual_group==128U?1U:2U);out[1]=(uint8_t)order;out[2]=(uint8_t)coding;
  out[3]=(order<4U && residual_group==128U)?0U:(uint8_t)(31U-__builtin_clz(residual_group));
  ssc_store(out+4,literals,4);ssc_store(out+8,count,4);
  ssc_writer w={out,cap,12U+literals,0,0};l=ssc_begin(offset,s);
  uint32_t history[16][5]={{0}},group[256],n=0,literal_pos=12;
  uint32_t bits=s->scalar_bytes*8U,mask=bits==16?65535U:UINT32_MAX;
  for(size_t p=0;p<size;p+=s->scalar_bytes){
    int a=ssc_sample(&l);
    if(a){
      uint32_t value=ssc_load(raw+p,s->scalar_bytes),delta=(value-ssc_predict(history,l.lane,order,bits))&mask;
      ssc_remember(history,l.lane,value,order,bits);group[n++]=((delta<<1)^(0U-(delta>>(bits-1U))))&mask;
      if(n==residual_group){if(ssc_encode_group(&w,group,n,bits,coding))return 1;n=0;}
    } else {memcpy(out+literal_pos,raw+p,s->scalar_bytes);literal_pos+=s->scalar_bytes;}
    ssc_advance(&l,a);
  }
  if(n && ssc_encode_group(&w,group,n,bits,coding))return 1;
  if(w.pos>=size)return 1;
  *used=w.pos;return 0;
}

int ssc_encode(const uint8_t *raw,size_t size,uint64_t offset,const sl_schema *s,
               uint32_t order,uint32_t coding,uint8_t *out,size_t cap,size_t *used) {
  return ssc_encode_grouped(raw,size,offset,s,order,coding,SSC_GROUP,out,cap,used);
}

int ssc_decode(const uint8_t *encoded,size_t encoded_size,size_t size,
               uint64_t offset,const sl_schema *s,uint8_t *raw,size_t cap) {
  if(!encoded || !raw || cap<size || !ssc_valid(size,offset,s))return -1;
  if(encoded_size<12 || (encoded[0]!=1 && encoded[0]!=2 && encoded[0]!=3) || encoded[1]<1 || encoded[1]>10 || encoded[2]>1)return -3;
  if((encoded[0]==1 && encoded[3]) || (encoded[0]>=2 && (encoded[3]<4 || encoded[3]>8)) || (encoded[0]<3 && encoded[1]>3) || (encoded[0]==3 && encoded[1]<4))return -3;
  uint32_t residual_group=encoded[0]==1?128U:(1U<<encoded[3]);
  uint32_t literals=ssc_load(encoded+4,4),count=ssc_load(encoded+8,4),order=encoded[1],coding=encoded[2];
  if(literals>size || (size_t)literals+12U>encoded_size)return -3;
  ssc_layout l=ssc_begin(offset,s);uint32_t expected=0,expected_literals=0;
  for(size_t p=0;p<size;p+=s->scalar_bytes){int a=ssc_sample(&l);if(a)++expected;else expected_literals+=s->scalar_bytes;ssc_advance(&l,a);}
  if(expected!=count || expected_literals!=literals)return -3;
  ssc_reader r={encoded,encoded_size,12U+literals,0,0};l=ssc_begin(offset,s);
  uint32_t history[16][5]={{0}},remaining=count,in_group=0,k=0,literal_pos=12;
  uint32_t bits=s->scalar_bytes*8U,mask=bits==16?65535U:UINT32_MAX;
  for(size_t p=0;p<size;p+=s->scalar_bytes){
    int a=ssc_sample(&l);
    if(a){
      if(!in_group){
        if(r.available && r.bits)return -3;
        r.available=0;r.bits=0;
        if(r.pos==r.size)return -3;
        k=r.data[r.pos++];
        if(k>bits || (coding && k==bits))return -3;
        in_group=remaining<residual_group?remaining:residual_group;
      }
      uint32_t z=0;
      if(coding){
        uint32_t bit=0,q=0;
        do {if(ssc_get(&r,1,&bit))return -3;if(!bit && ++q>(mask>>k))return -3;} while(!bit);
        uint32_t low=0;if(ssc_get(&r,k,&low))return -3;z=(q<<k)|low;
      } else if(ssc_get(&r,k,&z))return -3;
      if(z>mask)return -3;
      uint32_t delta=(z>>1)^(0U-(z&1U)),value=(ssc_predict(history,l.lane,order,bits)+delta)&mask;
      ssc_remember(history,l.lane,value,order,bits);ssc_store(raw+p,value,s->scalar_bytes);--in_group;--remaining;
    } else {memcpy(raw+p,encoded+literal_pos,s->scalar_bytes);literal_pos+=s->scalar_bytes;}
    ssc_advance(&l,a);
  }
  if(remaining || r.pos!=r.size || (r.available && r.bits))return -3;
  return 0;
}
