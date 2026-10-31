/* Included by throughput_client.c: owns the same stopped-recorder socket and
 * packet pool. Source files are read-only; only COMMIT.BIN is written after a
 * verified COMMITTED receipt. This client never deletes acquisition files. */
#include "lossless_client.h"
#include "sd_diag.h"
#define HAL_HASH_MODULE_ENABLED
#include "stm32u5xx_hal_hash.h"
#include "../../../../../../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash.c"
#include "../../../../../../Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_hash_ex.c"

/* Diagnostic-only metadata fits in spare SRAM4. Source/DMA payload buffers
 * remain in the existing SysAlloc pool. NOLOAD objects are initialized below
 * or fully staged by the trusted host before use. */
#define LR_META __attribute__((section(".transfer_bss"),aligned(8)))
lr_stream_t lr_streams[LR_STREAMS] LR_META;
volatile lr_profile_t lr_profile LR_META;
volatile uint32_t lr_count, lr_raw_only;
volatile uint64_t lr_acquisition;
uint8_t lr_manifest_sha[32];
char lr_receipt_path[128];
typedef struct __attribute__((packed)) {
  uint32_t magic; uint16_t version, codec; uint64_t acquisition;
  uint32_t stream, index; uint64_t offset;
  uint32_t raw_length, encoded_length, crc, header_crc;
} lr_header_t;
_Static_assert(sizeof(lr_header_t)==48,"Lossless frame size");
typedef struct { lr_header_t header; uint8_t *data; } lr_slot_t;
static lr_slot_t lr_slots[2] LR_META;
static uint8_t *lr_raw, *lr_work;
static TX_THREAD lr_reader LR_META;
static TX_SEMAPHORE lr_empty[2] LR_META, lr_full[2] LR_META, lr_done LR_META;
static FX_MEDIA *lr_media;
static volatile uint32_t lr_abort, lr_producer_finished;
static uint64_t lr_offsets[LR_STREAMS] LR_META;
static uint32_t lr_completed_mask;
static NX_PACKET *lr_rx;
static ULONG lr_rx_offset;
static HASH_HandleTypeDef lr_hash LR_META;

static uint32_t lr_crc(const void *data, uint32_t n)
{ return transfer_bench_crc_update(UINT32_MAX,data,n)^UINT32_MAX; }
static UINT lr_read_exact(uint8_t *p, ULONG n)
{
  while(n) {
    if(!lr_rx) {
      UINT st=nx_tcp_socket_receive(&sock,&lr_rx,90000);if(st)return st;
      lr_rx_offset=0;
    }
    ULONG available=lr_rx->nx_packet_length-lr_rx_offset;
    if(!available) { nx_packet_release(lr_rx);lr_rx=NULL;continue; }
    ULONG k=n<available?n:available,got=0;
    UINT st=nx_packet_data_extract_offset(lr_rx,lr_rx_offset,p,k,&got);
    if(st || got!=k)return st?st:NX_INVALID_PACKET;
    p+=k;n-=k;lr_rx_offset+=k;
    if(lr_rx_offset==lr_rx->nx_packet_length) { nx_packet_release(lr_rx);lr_rx=NULL; }
  }
  return 0;
}
static UINT lr_hash_init(void)
{
  __HAL_RCC_HASH_CLK_ENABLE();
  memset(&lr_hash,0,sizeof(lr_hash));lr_hash.Init.DataType=HASH_DATATYPE_8B;
  return HAL_HASH_Init(&lr_hash)==HAL_OK?0:NX_NOT_SUCCESSFUL;
}
static UINT lr_hash_update(uint8_t *raw,uint32_t count,uint8_t *digest,uint32_t last)
{
  uint32_t t=transfer_cycles();
  HAL_StatusTypeDef st=last?HAL_HASHEx_SHA256_Accmlt_End(&lr_hash,raw,count,digest,1000):
                           HAL_HASHEx_SHA256_Accmlt(&lr_hash,raw,count);
  lr_profile.hash_cycles+=(uint32_t)(transfer_cycles()-t);
  return st==HAL_OK?0:NX_NOT_SUCCESSFUL;
}
static UINT lr_hash_selftest(void)
{
  static const uint8_t expected[32]={0xba,0x78,0x16,0xbf,0x8f,0x01,0xcf,0xea,0x41,0x41,0x40,0xde,0x5d,0xae,0x22,0x23,
    0xb0,0x03,0x61,0xa3,0x96,0x17,0x7a,0x9c,0xb4,0x10,0xff,0x61,0xf2,0x00,0x15,0xad};
  uint32_t input=0x00636261U;uint8_t digest[32];UINT st=lr_hash_init();
  if(!st)st=lr_hash_update((uint8_t *)&input,3,digest,1);
  if(!st && memcmp(expected,digest,32))st=NX_INVALID_PACKET;
  HAL_HASH_DeInit(&lr_hash);__HAL_RCC_HASH_CLK_DISABLE();return st;
}
static UINT lr_stat(FX_MEDIA *media)
{
  if(!lr_count || lr_count>LR_STREAMS)return NX_INVALID_PARAMETERS;
  for(uint32_t i=0;i<lr_count;i++) {
    if(!memchr(lr_streams[i].path,0,sizeof(lr_streams[i].path)))return NX_INVALID_PARAMETERS;
    FX_FILE source;UINT st=fx_file_open(media,&source,lr_streams[i].path,FX_OPEN_FOR_READ);
    if(st)return st;
    lr_streams[i].size=source.fx_file_current_file_size;st=fx_file_close(&source);
    if(st || !lr_streams[i].size)return st?st:FX_END_OF_FILE;
  }
  return 0;
}
static void lr_produce(ULONG unused)
{
  (void)unused;uint32_t slot_index=0;UINT st=0;
  for(uint32_t i=0;i<lr_count && !st && !lr_abort;i++) {
    if(lr_completed_mask&(1U<<i))continue;
    FX_FILE source;uint32_t opened=0;uint64_t pos=0;uint8_t digest[32];
    st=fx_file_open(lr_media,&source,lr_streams[i].path,FX_OPEN_FOR_READ);
    if(!st)opened=1;
    if(!st && source.fx_file_current_file_size!=lr_streams[i].size)st=FX_INVALID_OPTION;
    if(!st)st=lr_hash_init();
    while(!st && !lr_abort && pos<lr_streams[i].size) {
      ULONG n=lr_streams[i].size-pos>LR_BLOCK?LR_BLOCK:(ULONG)(lr_streams[i].size-pos),got=0;
      /* Prefix is re-read for source SHA, but never re-sent on resume. */
      uint32_t send=pos>=lr_offsets[i];
      if(send) {
        while(!lr_abort && (st=tx_semaphore_get(&lr_empty[slot_index],100))==TX_NO_INSTANCE) {}
        if(lr_abort || st)break;
      }
      uint32_t t=transfer_cycles();st=fx_file_read(&source,lr_raw,n,&got);
      lr_profile.read_cycles+=(uint32_t)(transfer_cycles()-t);lr_profile.read_bytes+=got;
      if(!st && got!=n)st=FX_END_OF_FILE;
      memset(lr_raw+n,0,4); /* HAL's last word may include up to three padding bytes. */
      if(!st)st=lr_hash_update(lr_raw,n,digest,pos+n==lr_streams[i].size);
      if(!send)lr_profile.prefix_bytes+=got;
      if(!st && send) {
        lr_slot_t *slot=&lr_slots[slot_index];size_t encoded=n;uint32_t codec=0;
        t=transfer_cycles();
        if(lr_raw_only || !lr_streams[i].schema.scalar_bytes)memcpy(slot->data,lr_raw,n);
        else st=(UINT)sl_encode(lr_raw,n,pos,&lr_streams[i].schema,lr_work,LR_BLOCK,
                             slot->data,LR_BLOCK+LR_BLOCK/256+1024,&encoded,&codec);
        lr_profile.codec_cycles+=(uint32_t)(transfer_cycles()-t);
        /* A resource/codec failure retains the raw bytes and sends RAW. */
        if(st) { memcpy(slot->data,lr_raw,n);encoded=n;codec=0;st=0; }
        slot->header=(lr_header_t){0x314e4843U,1,(uint16_t)codec,lr_acquisition,i,(uint32_t)(pos/LR_BLOCK),pos,n,(uint32_t)encoded,lr_crc(lr_raw,n),0};
        slot->header.header_crc=lr_crc(&slot->header,44);
        tx_semaphore_put(&lr_full[slot_index]);slot_index^=1U;
      }
      pos+=n;
    }
    if(opened) { UINT closed=fx_file_close(&source);if(!st)st=closed; }
    if(st || lr_abort)break;
    while(!lr_abort && (st=tx_semaphore_get(&lr_empty[slot_index],100))==TX_NO_INSTANCE) {}
    if(st || lr_abort)break;
    lr_slot_t *slot=&lr_slots[slot_index];memcpy(slot->data,digest,32);
    slot->header=(lr_header_t){0x314e4843U,1,65535,lr_acquisition,i,(uint32_t)(((uint64_t)lr_streams[i].size+LR_BLOCK-1U)/LR_BLOCK),lr_streams[i].size,0,32,0,0};
    slot->header.header_crc=lr_crc(&slot->header,44);
    tx_semaphore_put(&lr_full[slot_index]);slot_index^=1U;
  }
  HAL_HASH_DeInit(&lr_hash);__HAL_RCC_HASH_CLK_DISABLE();
  lr_profile.producer_status=st;lr_producer_finished=1;tx_semaphore_put(&lr_done);
}
static UINT lr_persist(FX_MEDIA *media,const uint8_t ack[64])
{
  if(!lr_receipt_path[0] || !memchr(lr_receipt_path,0,sizeof(lr_receipt_path)))return NX_INVALID_PARAMETERS;
  size_t length=strlen(lr_receipt_path);
  if(length<11 || strcmp(lr_receipt_path+length-11,"/COMMIT.BIN"))return NX_INVALID_PARAMETERS;
  for(uint32_t i=0;i<lr_count;i++) {
    if(strncmp(lr_receipt_path,lr_streams[i].path,length-10) || !strcmp(lr_receipt_path,lr_streams[i].path))return NX_INVALID_PARAMETERS;
  }
  UINT st=fx_file_create(media,lr_receipt_path);if(st!=FX_SUCCESS && st!=FX_ALREADY_CREATED)return st;
  FX_FILE receipt;st=fx_file_open(media,&receipt,lr_receipt_path,FX_OPEN_FOR_WRITE);if(st)return st;
  st=fx_file_seek(&receipt,0);
  if(!st)st=fx_file_write(&receipt,(void *)ack,64);
  if(!st)st=fx_file_truncate(&receipt,64);
  UINT close=fx_file_close(&receipt);if(!st)st=close;
  if(!st)st=fx_media_flush(media);
  if(st)return st;
  st=fx_file_open(media,&receipt,lr_receipt_path,FX_OPEN_FOR_READ);if(st)return st;
  uint8_t reread[64];ULONG got=0;st=fx_file_read(&receipt,reread,64,&got);
  if(!st && (got!=64 || memcmp(ack,reread,64)))st=FX_IO_ERROR;
  close=fx_file_close(&receipt);if(!st)st=close;return st;
}
static UINT lr_execute(NX_IP *ip,FX_MEDIA *media)
{
  if(app_model.log_controller_model.status || !lr_count || lr_count>LR_STREAMS || !lr_acquisition ||
     compression_input || lr_raw || lr_work || lr_raw_only>1 || (throughput_queue_depth<1 || throughput_queue_depth>20) || throughput_pool_bytes!=65536)return NX_INVALID_PARAMETERS;
  memset((void *)&lr_profile,0,sizeof(lr_profile));memset((void *)&throughput_profile,0,sizeof(throughput_profile));
  /* Each retry has a new TCP socket and durable server resume offsets. Clear
   * the failure latch only after every previous owned command was answered. */
  if(mipc_async_reset_idle())return NX_NOT_SUCCESSFUL;
  lr_profile.start_tick=tx_time_get();throughput_profile.clock_hz=SystemCoreClock;
  throughput_profile.pool_min=throughput_profile.min_peer_window=throughput_profile.min_congestion_window=UINT32_MAX;
  UINT st=0;uint32_t pool_created=0,socket_created=0,bound=0,sems=0,thread_created=0;
  uint8_t *pool_mem=NULL,*stack=NULL;lr_raw=lr_work=NULL;lr_slots[0].data=lr_slots[1].data=NULL;lr_rx=NULL;
  lr_raw=SysAlloc(LR_BLOCK+4);lr_work=SysAlloc(LR_BLOCK);
  for(uint32_t i=0;i<2;i++)lr_slots[i].data=SysAlloc(LR_BLOCK+LR_BLOCK/256+1024);
  pool_mem=SysAlloc(65536);stack=SysAlloc(8192);
  lr_profile.ram_bytes=(LR_BLOCK+4)+LR_BLOCK+2*(LR_BLOCK+LR_BLOCK/256+1024)+65536+8192;
  if(!lr_raw || !lr_work || !lr_slots[0].data || !lr_slots[1].data || !pool_mem || !stack){st=NX_NO_PACKET;goto done;}
  lr_profile.min_free_ram=sd_diag_pool?sd_diag_pool->tx_byte_pool_available:0;
  st=nx_packet_pool_create(&packets,"lossless transfer",1544,pool_mem,65536);if(st)goto done;
  pool_created=1;throughput_profile.pool_total=packets.nx_packet_pool_total;
  st=nx_tcp_socket_create(ip,&sock,"lossless TCP",NX_IP_NORMAL,NX_FRAGMENT_OKAY,NX_IP_TIME_TO_LIVE,32768,NULL,NULL);if(st)goto done;
  socket_created=1;st=nx_tcp_socket_transmit_configure(&sock,throughput_queue_depth,1000,10,0);if(st)goto done;
  st=nx_tcp_client_socket_bind(&sock,NX_ANY_PORT,10000);if(st)goto done;bound=1;
  st=nx_tcp_client_socket_connect(&sock,throughput_server_ip,throughput_server_port,30000);if(st)goto done;
  network_powersave_status=network_powersave<0?0:MX_WIFI_station_powersave(wifi_obj_get(),network_powersave);
  if(network_powersave_status){st=NX_NOT_SUCCESSFUL;goto done;}
  transfer_crc_mode=1;
  if(cycle30_manifest_len) {
    uint32_t prefix[2]={0x314e414dU,cycle30_manifest_len};
    st=send_block((uint8_t *)prefix,8);
    if(!st)st=send_block((uint8_t *)cycle30_manifest,cycle30_manifest_len);
    if(st)goto done;
  }
  uint8_t hello[48]={0};memcpy(hello,"LST1",4);memcpy(hello+4,(const void *)&lr_acquisition,8);memcpy(hello+12,lr_manifest_sha,32);
  uint32_t crc=lr_crc(hello,44);memcpy(hello+44,&crc,4);
  st=send_block(hello,48);if(st)goto done;
  uint8_t resume[24+8*LR_STREAMS+4];st=lr_read_exact(resume,24);if(st)goto done;
  uint64_t acquisition;uint32_t count,receipt;
  memcpy(&acquisition,resume+4,8);memcpy(&count,resume+12,4);memcpy(&lr_completed_mask,resume+16,4);memcpy(&receipt,resume+20,4);
  if(memcmp(resume,"RSM1",4) || acquisition!=lr_acquisition || count!=lr_count || (lr_completed_mask>>lr_count) || receipt>1){st=NX_INVALID_PACKET;goto done;}
  st=lr_read_exact(resume+24,count*8+4);if(st)goto done;
  memcpy(&crc,resume+24+count*8,4);if(crc!=lr_crc(resume,24+count*8)){st=NX_INVALID_PACKET;goto done;}
  memcpy(lr_offsets,resume+24,count*8);
  uint64_t total=0;for(uint32_t i=0;i<count;i++) {
    total+=lr_streams[i].size;
    if(lr_offsets[i]>lr_streams[i].size || (lr_offsets[i]!=lr_streams[i].size && lr_offsets[i]%LR_BLOCK) ||
       ((lr_completed_mask&(1U<<i)) && lr_offsets[i]!=lr_streams[i].size)){st=NX_INVALID_PACKET;goto done;}
    lr_profile.resume_bytes+=lr_offsets[i];if(lr_offsets[i])lr_profile.resumed_streams++;
  }
  lr_profile.completed_mask=lr_completed_mask;
  memset((void *)&network_probe,0,sizeof(network_probe));memset((void *)&throughput_spi_profile,0,sizeof(throughput_spi_profile));
  memset((void *)&transfer_wifi_profile,0,sizeof(transfer_wifi_profile));
  network_probe_socket=&sock;throughput_owner=tx_thread_identify();transfer_cpu_samples=transfer_cpu_thread_samples=0;
  transfer_cpu_active=1;probe_cs_active=probe_dma_active=probe_last_dma_valid=0;
  st=mipc_trace_begin();if(st)goto done;
  lr_profile.ram_bytes+=mipc_trace_bytes;
  if(sd_diag_pool && sd_diag_pool->tx_byte_pool_available<lr_profile.min_free_ram)
    lr_profile.min_free_ram=sd_diag_pool->tx_byte_pool_available;
  memset(rtt_pending,0,sizeof(rtt_pending));memset((void *)network_rtt_hist,0,sizeof(network_rtt_hist));
  network_rtt_samples=network_rtt_max_cycles=network_rtt_ambiguous=network_rtt_overflows=0;
  network_rtt_min_cycles=UINT32_MAX;network_rtt_cycles=0;
  throughput_profile.start_tick=lr_profile.start_tick;
  if(lr_completed_mask != (1U<<count)-1U) {
    memset(&lr_reader,0,sizeof(lr_reader));memset(lr_empty,0,sizeof(lr_empty));
    memset(lr_full,0,sizeof(lr_full));memset(&lr_done,0,sizeof(lr_done));
    for(uint32_t i=0;i<2;i++) {tx_semaphore_create(&lr_empty[i],"lossless empty",1);tx_semaphore_create(&lr_full[i],"lossless full",0);}
    tx_semaphore_create(&lr_done,"lossless done",0);sems=1;
    lr_abort=lr_producer_finished=0;lr_media=media;
    TX_THREAD *self=tx_thread_identify();
    st=tx_thread_create(&lr_reader,"lossless codec",lr_produce,0,stack,8192,self->tx_thread_priority+1,self->tx_thread_priority+1,TX_NO_TIME_SLICE,TX_DONT_START);
    if(st)goto done;
    thread_created=1;
    network_async_submitted=network_async_completed=network_async_errors=network_async_max_pending=0;network_async_wait_cycles=0;
    network_async_active=network_async_depth!=0;
    tx_thread_resume(&lr_reader);uint32_t index=0,ended=lr_completed_mask;
    while(ended != (1U<<count)-1U) {
      st=tx_semaphore_get(&lr_full[index],100);
      if(st==TX_NO_INSTANCE) {if(lr_producer_finished){st=lr_profile.producer_status?lr_profile.producer_status:NX_NOT_SUCCESSFUL;break;}continue;}
      if(st)break;
      lr_slot_t *slot=&lr_slots[index];
      st=send_block((const uint8_t *)&slot->header,48);if(!st)st=send_block(slot->data,slot->header.encoded_length);
      if(st)break;
      lr_profile.wire_bytes+=48+slot->header.encoded_length;
      if(slot->header.codec==65535)ended|=1U<<slot->header.stream;
      else {lr_profile.chunks++;lr_profile.raw_bytes+=slot->header.raw_length;lr_profile.encoded_bytes+=slot->header.encoded_length;if(!slot->header.codec)lr_profile.raw_fallbacks++;}
      tx_semaphore_put(&lr_empty[index]);index^=1U;
    }
    lr_profile.send_tick=tx_time_get();network_async_active=0;
    if(!st)st=mipc_async_drain(30000);
    if(st)goto done;
  }
  uint8_t ack[64];st=lr_read_exact(ack,64);if(st)goto done;
  uint32_t flags,nstreams;uint64_t ack_acquisition,ack_total;
  memcpy(&flags,ack+4,4);memcpy(&ack_acquisition,ack+8,8);memcpy(&ack_total,ack+16,8);memcpy(&nstreams,ack+56,4);memcpy(&crc,ack+60,4);
  if(memcmp(ack,"ACK1",4) || flags!=1 || ack_acquisition!=lr_acquisition || ack_total!=total ||
     memcmp(ack+24,lr_manifest_sha,32) || nstreams!=count || crc!=lr_crc(ack,60)){st=NX_INVALID_PACKET;goto done;}
  lr_profile.ack_tick=tx_time_get();lr_profile.committed=1;
  st=lr_persist(media,ack);if(st)goto done;
  lr_profile.persisted=1;lr_profile.persisted_tick=tx_time_get();
  st=send_block((const uint8_t *)"DCON",4);
done:
  mipc_trace_end();
  transfer_cpu_active=0;network_probe_socket=NULL;throughput_owner=NULL;
  throughput_profile.cpu_samples=transfer_cpu_samples;throughput_profile.cpu_thread_samples=transfer_cpu_thread_samples;
  network_async_active=0;lr_abort=1;
  if(thread_created) {
    for(uint32_t i=0;i<2;i++)tx_semaphore_put(&lr_empty[i]);
    UINT joined=tx_semaphore_get(&lr_done,30000);
    if(joined) {lr_profile.cleanup_status=joined;lr_profile.status=st?st:joined;return lr_profile.status;}
    tx_thread_terminate(&lr_reader);tx_thread_delete(&lr_reader);
  }
  if(sems) {for(uint32_t i=0;i<2;i++){tx_semaphore_delete(&lr_empty[i]);tx_semaphore_delete(&lr_full[i]);}tx_semaphore_delete(&lr_done);}
  if(lr_rx){nx_packet_release(lr_rx);lr_rx=NULL;}
  if(socket_created) {
    lr_profile.retransmits=sock.nx_tcp_socket_retransmit_packets;
    if(bound){nx_tcp_socket_disconnect(&sock,3000);nx_tcp_client_socket_unbind(&sock);}
    UINT deleted=nx_tcp_socket_delete(&sock);if(deleted)lr_profile.cleanup_status=deleted;
  }
  if(pool_created) {UINT deleted=nx_packet_pool_delete(&packets);if(deleted){lr_profile.cleanup_status=deleted;pool_mem=NULL;}}
  SysFree(lr_raw);SysFree(lr_work);for(uint32_t i=0;i<2;i++)SysFree(lr_slots[i].data);SysFree(pool_mem);SysFree(stack);
  lr_raw=lr_work=NULL;lr_slots[0].data=lr_slots[1].data=NULL;
  lr_profile.finish_tick=tx_time_get();lr_profile.status=st;return st;
}
