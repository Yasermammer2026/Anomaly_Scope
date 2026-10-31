#include "record_stage.h"
#include <string.h>

void rs_init(rs_stage *s) { if(s)memset(s,0,sizeof(*s)); }
uint32_t rs_level(const rs_stage *s) {
  return s->used_units>=RS_UNITS*7U/8U?RS_CRITICAL:(s->used_units>=RS_UNITS*5U/8U?RS_PRESSURE:RS_NORMAL);
}
static int occupied_unit(const uint64_t occupied[RS_BITMAP_WORDS],uint32_t unit) {
  return (int)((occupied[unit/64U]>>(unit%64U))&1U);
}
static void mark(uint64_t occupied[RS_BITMAP_WORDS],uint32_t first,uint32_t units,int owned) {
  for(uint32_t i=first;i<first+units;++i) {
    uint64_t bit=UINT64_C(1)<<(i%64U);
    if(owned)occupied[i/64U]|=bit;else occupied[i/64U]&=~bit;
  }
}
static int span(const uint64_t occupied[RS_BITMAP_WORDS],uint32_t remaining,uint32_t *first,uint32_t *bytes) {
  uint32_t wanted=(remaining+RS_UNIT_BYTES-1U)/RS_UNIT_BYTES;
  if(wanted>RS_CHUNK_BYTES/RS_UNIT_BYTES)wanted=RS_CHUNK_BYTES/RS_UNIT_BYTES;
  uint32_t best=0,at=0;
  for(uint32_t i=0;i<RS_UNITS;) {
    if(occupied_unit(occupied,i)){++i;continue;}
    uint32_t begin=i;
    while(i<RS_UNITS && !(occupied_unit(occupied,i)))++i;
    uint32_t units=i-begin;
    if(units>=wanted){*first=begin;*bytes=remaining<wanted*RS_UNIT_BYTES?remaining:wanted*RS_UNIT_BYTES;return 0;}
    if(units>best){best=units;at=begin;}
  }
  if(!best)return 1;
  *first=at;*bytes=remaining<best*RS_UNIT_BYTES?remaining:best*RS_UNIT_BYTES;return 0;
}
static int admission(const rs_stage *s,uint32_t stream,uint32_t bytes,uint32_t *count) {
  uint64_t occupied[RS_BITMAP_WORDS];memcpy(occupied,s->occupied,sizeof(occupied));
  uint32_t remaining=bytes,nchunks=0;
  while(remaining) {
    if(s->count+nchunks>=RS_DESCRIPTORS)return 1;
    if(s->chunks[(s->tail+nchunks)%RS_DESCRIPTORS].state!=RS_FREE)return -1;
    uint32_t first,n;
    if(span(occupied,remaining,&first,&n))return 1;
    mark(occupied,first,(n+RS_UNIT_BYTES-1U)/RS_UNIT_BYTES,1);
    remaining-=n;++nchunks;
  }
  if(s->next_ticket>UINT64_MAX-nchunks || s->next_sequence[stream]>UINT32_MAX-nchunks ||
     s->next_offset[stream]>UINT64_MAX-bytes)return -1;
  *count=nchunks;
  return 0;
}
int rs_can_reserve(const rs_stage *s,uint32_t stream,uint32_t bytes) {
  if(!s || stream>=RS_STREAMS || !bytes || bytes>2U*RS_CHUNK_BYTES)return -1;
  uint32_t count;return admission(s,stream,bytes,&count);
}
int rs_reserve(rs_stage *s,uint32_t stream,uint32_t bytes,rs_reservation *r) {
  if(!s || !r || stream>=RS_STREAMS || !bytes || bytes>2U*RS_CHUNK_BYTES)return -1;
  uint32_t count;int status=admission(s,stream,bytes,&count);
  if(status){
    if(status==1){
      if(!s->failed_reservations){memcpy(s->first_failed_occupied,s->occupied,sizeof(s->occupied));s->first_failed_bytes=bytes;
        s->first_failed_used_units=s->used_units;s->first_failed_descriptors=s->count;}
      ++s->failed_reservations;
    }else ++s->invalid_transitions;
    return status;
  }
  uint64_t occupied[RS_BITMAP_WORDS];memcpy(occupied,s->occupied,sizeof(occupied));
  uint32_t remaining=bytes,used=0;
  for(uint32_t i=0;i<count;++i) {
    uint32_t index=(s->tail+i)%RS_DESCRIPTORS;
    uint32_t n=0,first=0;
    /* The read-only admission pass and mutation both run under the same short
     * caller lock. No large temporary descriptor array lives on sensor stacks. */
    (void)span(occupied,remaining,&first,&n);
    uint32_t units=(n+RS_UNIT_BYTES-1U)/RS_UNIT_BYTES;
    mark(occupied,first,units,1);used+=units;
    s->chunks[index]=(rs_chunk){(uint8_t)first,(uint8_t)units,RS_COPY,(uint8_t)stream,
                           n,s->next_sequence[stream]+i,s->next_offset[stream]+bytes-remaining,s->next_ticket+i};
    remaining-=n;
  }
  r->first=s->tail;r->count=count;r->ticket=s->next_ticket;s->next_ticket+=count;
  memcpy(s->occupied,occupied,sizeof(occupied));s->used_units+=used;s->count+=count;
  s->tail=(s->tail+count)%RS_DESCRIPTORS;
  s->next_offset[stream]+=bytes;s->next_sequence[stream]+=count;
  if(s->used_units>s->high_units)s->high_units=s->used_units;
  if(s->count>s->high_descriptors)s->high_descriptors=s->count;
  return 0;
}
int rs_publish(rs_stage *s,const rs_reservation *r) {
  if(!s || !r || !r->count || r->count>RS_MAX_ITEM_CHUNKS || r->first>=RS_DESCRIPTORS)return -1;
  for(uint32_t i=0;i<r->count;++i)
    if(s->chunks[(r->first+i)%RS_DESCRIPTORS].state!=RS_COPY || s->chunks[(r->first+i)%RS_DESCRIPTORS].ticket!=r->ticket+i){++s->invalid_transitions;return -1;}
  for(uint32_t i=0;i<r->count;++i)s->chunks[(r->first+i)%RS_DESCRIPTORS].state=RS_READY;
  return 0;
}
int rs_take(rs_stage *s,uint32_t *index) {
  if(!s || !index)return -1;
  if(!s->count || s->chunks[s->head].state!=RS_READY)return 1;
  *index=s->head;s->chunks[s->head].state=RS_ENCODING;return 0;
}
int rs_take_encode(rs_stage *s,uint32_t *index) {
  if(!s || !index)return -1;
  for(uint32_t i=0;i<s->count;++i) {
    uint32_t at=(s->head+i)%RS_DESCRIPTORS;
    if(s->chunks[at].state==RS_ENCODED)continue;
    if(s->chunks[at].state==RS_COPY)return 1;
    if(s->chunks[at].state!=RS_READY){++s->invalid_transitions;return -1;}
    *index=at;s->chunks[at].state=RS_ENCODING;return 0;
  }
  return 1;
}
int rs_pack(rs_stage *s,uint32_t index,uint64_t ticket,uint32_t bytes) {
  if(!s || index>=RS_DESCRIPTORS)return -1;
  rs_chunk *c=&s->chunks[index];
  if(c->state!=RS_ENCODING || c->ticket!=ticket || !bytes || bytes>c->units*RS_UNIT_BYTES) {
    ++s->invalid_transitions;return -1;
  }
  uint32_t units=(bytes+RS_UNIT_BYTES-1U)/RS_UNIT_BYTES;
  if(units<c->units) {
    mark(s->occupied,c->first_unit+units,c->units-units,0);
    s->used_units-=c->units-units;c->units=(uint8_t)units;
  }
  c->state=RS_ENCODED;return 0;
}
int rs_release(rs_stage *s,uint32_t index,uint64_t ticket) {
  if(!s || index>=RS_DESCRIPTORS)return -1;
  if(!s->count || index!=s->head || (s->chunks[index].state!=RS_ENCODING && s->chunks[index].state!=RS_ENCODED) || s->chunks[index].ticket!=ticket){++s->invalid_transitions;return -1;}
  rs_chunk *c=&s->chunks[index];mark(s->occupied,c->first_unit,c->units,0);
  s->used_units-=c->units;memset(c,0,sizeof(*c));
  s->head=(s->head+1U)%RS_DESCRIPTORS;--s->count;return 0;
}
uint8_t *rs_data(const rs_stage *s,uint8_t *arena,uint32_t index) {
  if(!s || !arena || index>=RS_DESCRIPTORS || s->chunks[index].state==RS_FREE)return 0;
  return arena+(uint32_t)s->chunks[index].first_unit*RS_UNIT_BYTES;
}
void rs_writer_init(rs_writer *w) { if(w)memset(w,0,sizeof(*w)); }
int rs_output_reserve(rs_writer *w,uint32_t *index,uint64_t *ticket) {
  if(!w || !index || !ticket || w->next_ticket==UINT64_MAX)return -1;
  if(w->count==2U)return 1;
  rs_output *o=&w->slots[w->tail];
  if(o->state!=RS_OUT_FREE){++w->invalid_transitions;return -1;}
  *index=w->tail;*ticket=w->next_ticket++;
  *o=(rs_output){*ticket,RS_OUT_FILL,0,0,0};
  w->tail^=1U;++w->count;if(w->count>w->high)w->high=w->count;return 0;
}
int rs_output_publish(rs_writer *w,uint32_t index,uint64_t ticket,uint32_t stream,
                      uint32_t physical_bytes,uint32_t native_bytes) {
  if(!w || index>=2U || stream>=RS_STREAMS || physical_bytes<29U ||
     physical_bytes>RS_CHUNK_BYTES+48U || native_bytes>RS_CHUNK_BYTES)return -1;
  rs_output *o=&w->slots[index];
  if(o->state!=RS_OUT_FILL || o->ticket!=ticket){++w->invalid_transitions;return -1;}
  o->stream=stream;o->physical_bytes=physical_bytes;o->native_bytes=native_bytes;
  o->state=RS_OUT_READY;return 0;
}
int rs_output_take(rs_writer *w,uint32_t *index,uint64_t *ticket) {
  if(!w || !index || !ticket)return -1;
  rs_output *o=&w->slots[w->head];if(!w->count || o->state!=RS_OUT_READY)return 1;
  *index=w->head;*ticket=o->ticket;o->state=RS_OUT_WRITE;return 0;
}
int rs_output_release(rs_writer *w,uint32_t index,uint64_t ticket) {
  if(!w || index>=2U)return -1;
  rs_output *o=&w->slots[index];
  if(!w->count || index!=w->head || o->ticket!=ticket || o->state!=RS_OUT_WRITE){++w->invalid_transitions;return -1;}
  memset(o,0,sizeof(*o));w->head^=1U;--w->count;return 0;
}
