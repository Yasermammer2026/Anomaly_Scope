/* Host reconstruction of the bounded block codec. No floating point maths. */
#include "sensor_lossless.h"
#include <string.h>
#include <limits.h>
static uint32_t getword(const uint8_t *p, uint32_t w) {
  uint32_t v=(uint32_t)p[0]|((uint32_t)p[1]<<8);
  if(w==4)v|=((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
  return v;
}
static void putword(uint8_t *p,uint32_t v,uint32_t w) {
  p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);
  if(w==4){p[2]=(uint8_t)(v>>16);p[3]=(uint8_t)(v>>24);}
}
typedef struct {uint32_t within,relative,lane,span,group,width,item,dim;} walker;
static walker start(uint64_t offset,const sl_schema *s) {
  walker x={0};x.width=s->scalar_bytes;x.item=s->item_size;x.dim=s->dim;
  x.within=(uint32_t)(offset%x.item);
  uint64_t payload=(offset/x.item)*(x.item-4)+(x.within>=4?x.within-4:0);
  payload/=x.width;x.span=s->samples_per_ts*x.dim;
  x.group=s->samples_per_ts?x.span+8/x.width:0;
  x.relative=x.group?(uint32_t)(payload%x.group):0;
  x.lane=x.group?x.relative%x.dim:(uint32_t)(payload%x.dim);
  return x;
}
static int sample(const walker *x){return x->within>=4&&(!x->group||x->relative<x->span);}
static void advance(walker *x,int is_sample) {
  if(is_sample&&++x->lane==x->dim)x->lane=0;
  if(x->within>=4&&x->group&&++x->relative==x->group){x->relative=0;x->lane=0;}
  x->within+=x->width;if(x->within==x->item)x->within=0;
}

int sl_transform(const uint8_t *raw,size_t size,uint64_t offset,const sl_schema *s,
                 uint32_t order,uint32_t inverse,uint8_t *output,size_t output_size) {
  if(!raw||!s||!output||(s->scalar_bytes!=2&&s->scalar_bytes!=4)||
     !s->dim||s->dim>16||s->item_size<8||s->item_size%s->scalar_bytes||
     size%s->scalar_bytes||offset%s->scalar_bytes||size>UINT32_MAX||
     s->samples_per_ts>(UINT32_MAX-4U)/s->dim||!order||order>2||inverse>1)return -1;
  if(output_size<size)return -2;
  uint32_t prior[16]={0},second[16]={0},mask=s->scalar_bytes==2?65535U:UINT32_MAX;
  walker x=start(offset,s);
  for(size_t p=0;p<size;p+=x.width){
    int a=sample(&x);
    if(a){
      uint32_t value=getword(raw+p,x.width),result;
      if(inverse){
        uint32_t delta=order==2?(value+second[x.lane])&mask:value;
        result=(delta+prior[x.lane])&mask;second[x.lane]=delta;prior[x.lane]=result;
      }else{
        uint32_t delta=(value-prior[x.lane])&mask;
        result=order==2?(delta-second[x.lane])&mask:delta;
        prior[x.lane]=value;second[x.lane]=delta;
      }
      putword(output+p,result,x.width);
    }else memcpy(output+p,raw+p,x.width);
    advance(&x,a);
  }
  return 0;
}

int sl_decode(const uint8_t *encoded,size_t encoded_size,size_t raw_size,uint64_t offset,
              const sl_schema *s,uint32_t codec,uint8_t *work,size_t work_size,
              uint8_t *output,size_t output_size) {
  if(!encoded||!s||!work||!output||(s->scalar_bytes!=2&&s->scalar_bytes!=4)||
     !s->dim||s->dim>16||s->item_size<8||s->item_size%s->scalar_bytes||
     raw_size%s->scalar_bytes||offset%s->scalar_bytes||raw_size>UINT32_MAX||
     s->samples_per_ts>(UINT32_MAX-4U)/s->dim)return -1;
  if(work_size<raw_size||output_size<raw_size)return -2;
  if(codec==0){if(encoded_size!=raw_size)return -3;memcpy(output,encoded,raw_size);return 0;}
  if(codec!=3)return -3;
  walker x=start(offset,s);size_t n=0;
  for(size_t p=0;p<raw_size;p+=x.width){int a=sample(&x);n+=a;advance(&x,a);}
  size_t literals=raw_size-n*s->scalar_bytes,groups=(n+127)/128;
  if(encoded_size<literals+4+groups||getword(encoded+literals,4)!=n)return -3;
  const uint8_t *widths=encoded+literals+4;size_t positions[33]={0},cursor=literals+4+groups;
  for(size_t g=0;g<groups;++g)if(widths[g]>s->scalar_bytes*8)return -3;
  for(uint32_t w=1;w<=s->scalar_bytes*8;++w){
    positions[w]=cursor;
    for(size_t g=0;g<groups;++g)if(widths[g]==w){
      size_t size=16U*w;if(size>encoded_size-cursor)return -3;cursor+=size;
    }
  }
  if(cursor!=encoded_size)return -3;
  for(size_t g=0;g<groups;++g){
    uint32_t w=widths[g],available=0;uint64_t accumulator=0;
    size_t at=positions[w];
    for(size_t j=0;j<128;++j){
      uint32_t value=0;
      if(w){
        while(available<w){accumulator|=(uint64_t)encoded[at++]<<available;available+=8;}
        value=(uint32_t)(accumulator&((UINT64_C(1)<<w)-1));
        accumulator>>=w;available-=w;
      }
      size_t i=g*128+j;
      if(i<n)putword(work+i*s->scalar_bytes,value,s->scalar_bytes);
      else if(value)return -3;
    }
    if(w)positions[w]=at;
  }
  uint32_t prior[16]={0},mask=s->scalar_bytes==2?65535U:UINT32_MAX;
  x=start(offset,s);size_t literal_at=0,sample_at=0;
  for(size_t p=0;p<raw_size;p+=x.width){
    int a=sample(&x);
    if(a){
      uint32_t z=getword(work+sample_at++*x.width,x.width);
      uint32_t delta=(z>>1)^(0U-(z&1));
      uint32_t value=(prior[x.lane]+delta)&mask;
      prior[x.lane]=value;putword(output+p,value,x.width);
    }else{memcpy(output+p,encoded+literal_at,x.width);literal_at+=x.width;}
    advance(&x,a);
  }
  return 0;
}
