#include "record_pipeline.h"
#include <string.h>

static int fail(rp_pipeline *p,uint32_t reason,uint32_t stream,int detail) {
  uint32_t key=p->hooks.lock(p->hooks.arg),first=!p->error;
  if(first){p->error=reason;p->failed_stream=stream;p->failure_detail=detail;}
  p->hooks.unlock(p->hooks.arg,key);
  if(first)p->hooks.failed(p->hooks.arg,reason,stream,detail);
  return -1;
}
static int overlap(const uint8_t *a,size_t an,const uint8_t *b,size_t bn) {
  uintptr_t x=(uintptr_t)a,y=(uintptr_t)b;
  if(x>UINTPTR_MAX-an || y>UINTPTR_MAX-bn)return 1;
  return x<y+bn && y<x+an;
}
int rp_init(rp_pipeline *p,const rp_hooks *h,uint8_t *arena,
            uint8_t *out0,uint8_t *out1,const rc_stream streams[RS_STREAMS]) {
  if(!p || !h || !arena || !out0 || !out1 || !streams || !h->lock || !h->unlock ||
     !h->hash || !h->write || !h->wake_worker || !h->wake_writer || !h->failed ||
     overlap(arena,RS_ARENA_BYTES,out0,RC_OUTPUT_BYTES) ||
     overlap(arena,RS_ARENA_BYTES,out1,RC_OUTPUT_BYTES) ||
     overlap(out0,RC_OUTPUT_BYTES,out1,RC_OUTPUT_BYTES))return -1;
  memset(p,0,sizeof(*p));p->hooks=*h;p->arena=arena;p->output[0]=out0;p->output[1]=out1;
  memcpy(p->streams,streams,sizeof(p->streams));rs_init(&p->stage);rs_writer_init(&p->writer);
  return 0;
}
int rp_enable_packing(rp_pipeline *p,uint8_t *scratch,uint32_t n) {
  if(!p || p->scratch || !scratch || n<RC_OUTPUT_BYTES || p->error ||
     p->stage.count || p->writer.count || p->collector_active ||
     overlap(scratch,n,p->arena,RS_ARENA_BYTES) ||
     overlap(scratch,n,p->output[0],RC_OUTPUT_BYTES) ||
     overlap(scratch,n,p->output[1],RC_OUTPUT_BYTES))return -1;
  p->scratch=scratch;return 0;
}
int rp_collect(rp_pipeline *p,uint32_t stream,const uint8_t *source,uint32_t n,
               int (*release)(void *),void *release_arg) {
  if(!p || !source || !release || stream>=RS_STREAMS || !n || n>2U*RS_CHUNK_BYTES)return -1;
  if(p->error)return -1;
  /* Reject an accidental acquisition/staging alias before copying. */
  if(overlap(source,n,p->arena,RS_ARENA_BYTES) ||
     overlap(source,n,p->output[0],RC_OUTPUT_BYTES) ||
     overlap(source,n,p->output[1],RC_OUTPUT_BYTES))return fail(p,RP_ADMISSION,stream,-1);
  rs_reservation reservation;
  uint32_t key=p->hooks.lock(p->hooks.arg);
  ++p->collector_active;
  int st=rs_reserve(&p->stage,stream,n,&reservation);
  p->hooks.unlock(p->hooks.arg,key);
  if(st){
    key=p->hooks.lock(p->hooks.arg);--p->collector_active;p->hooks.unlock(p->hooks.arg,key);
    return fail(p,RP_ADMISSION,stream,st);
  }
  uint32_t offset=0;
  for(uint32_t i=0;i<reservation.count;++i) {
    uint32_t index=(reservation.first+i)%RS_DESCRIPTORS;
    uint32_t bytes=p->stage.chunks[index].bytes;
    memcpy(rs_data(&p->stage,p->arena,index),source+offset,bytes);offset+=bytes;
  }
  /* No source reads after release. It may immediately recycle acquisition. */
  st=release(release_arg);
  if(st){
    key=p->hooks.lock(p->hooks.arg);--p->collector_active;p->hooks.unlock(p->hooks.arg,key);
    return fail(p,RP_ACQUISITION_RELEASE,stream,st);
  }
  key=p->hooks.lock(p->hooks.arg);
  p->copied[stream]+=n;st=rs_publish(&p->stage,&reservation);--p->collector_active;
  p->hooks.unlock(p->hooks.arg,key);
  if(st)return fail(p,RP_STAGE_PUBLISH,stream,st);
  st=p->hooks.wake_worker(p->hooks.arg);
  return st?fail(p,RP_WORKER_WAKE,stream,st):0;
}
static int packed_step(rp_pipeline *p) {
  uint32_t key=p->hooks.lock(p->hooks.arg),head=p->stage.head;
  int st=1;uint32_t out;uint64_t ticket;
  if(p->stage.count && p->stage.chunks[head].state==RS_ENCODED)
    st=rs_output_reserve(&p->writer,&out,&ticket);
  p->hooks.unlock(p->hooks.arg,key);
  if(st<0)return fail(p,RP_OUTPUT_RESERVE,RS_STREAMS,st);
  if(!st) {
    rs_chunk *c=&p->stage.chunks[head];rc_result *frame=&p->packed[head];
    uint32_t h=p->packed_header_bytes[head];
    if((h!=RC_HEADER_BYTES && h!=RC_BLOCK3_BYTES) || frame->physical_bytes<h ||
       frame->physical_bytes>RC_OUTPUT_BYTES)return fail(p,RP_FRAME,c->stream,-1);
    memcpy(p->output[out],p->packed_header[head],h);
    memcpy(p->output[out]+h,rs_data(&p->stage,p->arena,head),frame->physical_bytes-h);
    key=p->hooks.lock(p->hooks.arg);
    st=rs_output_publish(&p->writer,out,ticket,c->stream,frame->physical_bytes,frame->native_bytes);
    p->hooks.unlock(p->hooks.arg,key);
    if(st)return fail(p,RP_OUTPUT_PUBLISH,c->stream,st);
    st=p->hooks.wake_writer(p->hooks.arg);
    if(st)return fail(p,RP_WRITER_WAKE,c->stream,st);
    key=p->hooks.lock(p->hooks.arg);
    st=rs_release(&p->stage,head,c->ticket);
    if(!st){memset(frame,0,sizeof(*frame));p->packed_header_bytes[head]=0;}
    p->hooks.unlock(p->hooks.arg,key);
    return st?fail(p,RP_STAGE_RELEASE,RS_STREAMS,st):0;
  }
  key=p->hooks.lock(p->hooks.arg);
  st=rs_take_encode(&p->stage,&p->worker_index);
  if(!st)p->worker_held=1;
  uint32_t pressure=rs_level(&p->stage);
  p->hooks.unlock(p->hooks.arg,key);
  if(st==1)return 1;
  if(st)return fail(p,RP_STAGE_TAKE,RS_STREAMS,st);
  rs_chunk *c=&p->stage.chunks[p->worker_index];uint32_t sid=c->stream;
  uint8_t *source=rs_data(&p->stage,p->arena,p->worker_index);
  rc_stream *stream=&p->streams[sid];rc_result frame;
  st=rc_frame(stream,c,source,pressure,p->scratch,RC_OUTPUT_BYTES,&frame);
  if(st)return fail(p,RP_FRAME,sid,st);
  st=p->hooks.hash(p->hooks.arg,sid,source,c->bytes);
  if(st)return fail(p,RP_HASH,sid,st);
  st=rc_accept(stream,c);
  if(st)return fail(p,RP_SOURCE_ACCEPT,sid,st);
  uint32_t h=stream->version>=3U?RC_BLOCK3_BYTES:RC_HEADER_BYTES;
  uint32_t payload=frame.physical_bytes-h;
  if(!payload || payload>c->units*RS_UNIT_BYTES)return fail(p,RP_FRAME,sid,-1);
  /* Source hash/CRC complete before overwriting our own staging copy.
   * Headers remain metadata; RAW payload needs no extra arena unit. */
  memcpy(p->packed_header[p->worker_index],p->scratch,h);
  memcpy(source,p->scratch+h,payload);
  p->packed[p->worker_index]=frame;p->packed_header_bytes[p->worker_index]=h;
  key=p->hooks.lock(p->hooks.arg);
  uint32_t before=p->stage.used_units;
  st=rs_pack(&p->stage,p->worker_index,c->ticket,payload);
  if(!st) {
    p->hashed[sid]+=c->bytes;++p->fallback[frame.fallback];
    ++p->packed_blocks;p->packed_saved_units+=before-p->stage.used_units;p->worker_held=0;
  }
  p->hooks.unlock(p->hooks.arg,key);
  return st?fail(p,RP_STAGE_PUBLISH,sid,st):0;
}
int rp_worker_step(rp_pipeline *p) {
  if(!p || p->error)return -1;
  if(p->scratch)return packed_step(p);
  uint32_t key=p->hooks.lock(p->hooks.arg);
  int st=0;
  if(!p->worker_held) {
    st=rs_take(&p->stage,&p->worker_index);
    if(!st)p->worker_held=1;
  }
  p->hooks.unlock(p->hooks.arg,key);
  if(st==1)return 1;
  if(st)return fail(p,RP_STAGE_TAKE,RS_STREAMS,st);
  rs_chunk *chunk=&p->stage.chunks[p->worker_index];
  uint32_t index;uint64_t ticket;
  key=p->hooks.lock(p->hooks.arg);
  st=rs_output_reserve(&p->writer,&index,&ticket);
  uint32_t pressure=rs_level(&p->stage);
  p->hooks.unlock(p->hooks.arg,key);
  if(st==1)return 1; /* Only independent staging remains owned. */
  if(st)return fail(p,RP_OUTPUT_RESERVE,chunk->stream,st);
  uint8_t *source=rs_data(&p->stage,p->arena,p->worker_index);
  rc_result result;rc_stream *stream=&p->streams[chunk->stream];
  st=rc_frame(stream,chunk,source,pressure,p->output[index],RC_OUTPUT_BYTES,&result);
  if(st)return fail(p,RP_FRAME,chunk->stream,st);
  st=p->hooks.hash(p->hooks.arg,chunk->stream,source,chunk->bytes);
  if(st)return fail(p,RP_HASH,chunk->stream,st);
  st=rc_accept(stream,chunk);
  if(st)return fail(p,RP_SOURCE_ACCEPT,chunk->stream,st);
  key=p->hooks.lock(p->hooks.arg);
  p->hashed[chunk->stream]+=chunk->bytes;++p->fallback[result.fallback];
  st=rs_output_publish(&p->writer,index,ticket,chunk->stream,result.physical_bytes,result.native_bytes);
  p->hooks.unlock(p->hooks.arg,key);
  if(st)return fail(p,RP_OUTPUT_PUBLISH,chunk->stream,st);
  st=p->hooks.wake_writer(p->hooks.arg);
  if(st)return fail(p,RP_WRITER_WAKE,chunk->stream,st);
  key=p->hooks.lock(p->hooks.arg);
  st=rs_release(&p->stage,p->worker_index,chunk->ticket);
  if(!st)p->worker_held=0;
  p->hooks.unlock(p->hooks.arg,key);
  return st?fail(p,RP_STAGE_RELEASE,RS_STREAMS,st):0;
}
int rp_writer_step(rp_pipeline *p) {
  if(!p || p->error)return -1;
  uint32_t index;uint64_t ticket;
  uint32_t key=p->hooks.lock(p->hooks.arg);
  int st=rs_output_take(&p->writer,&index,&ticket);
  p->hooks.unlock(p->hooks.arg,key);
  if(st==1)return 1;
  if(st)return fail(p,RP_OUTPUT_TAKE,RS_STREAMS,st);
  const rs_output *out=&p->writer.slots[index];
  uint32_t stream=out->stream;
  st=p->hooks.write(p->hooks.arg,stream,p->output[index],out->physical_bytes);
  if(st)return fail(p,RP_SD_WRITE,stream,st); /* Retain WRITE and bytes. */
  key=p->hooks.lock(p->hooks.arg);
  p->persisted[stream]+=out->native_bytes;p->physical[stream]+=out->physical_bytes;
  st=rs_output_release(&p->writer,index,ticket);
  p->hooks.unlock(p->hooks.arg,key);
  if(st)return fail(p,RP_OUTPUT_RELEASE,stream,st);
  st=p->hooks.wake_worker(p->hooks.arg);
  return st?fail(p,RP_WORKER_WAKE,stream,st):0;
}
int rp_drained(rp_pipeline *p) {
  if(!p || p->error)return 0;
  uint32_t key=p->hooks.lock(p->hooks.arg);
  int drained=!p->collector_active && !p->stage.count && !p->writer.count && !p->worker_held;
  p->hooks.unlock(p->hooks.arg,key);return drained;
}
