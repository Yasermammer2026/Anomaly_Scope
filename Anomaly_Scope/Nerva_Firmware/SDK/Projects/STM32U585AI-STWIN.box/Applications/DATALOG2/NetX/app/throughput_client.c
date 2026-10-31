/* Independent raw benchmarks: no hashing, protocol framing or database work
   in the measured payload loop. SD reads are always FX_OPEN_FOR_READ. */
#include "throughput_client.h"
#include "cycle30.h"
#include "services/sysmem.h"
#include "App_model.h"
#include "mx_wifi.h"
#include "mx_wifi_io.h"
#include "spi.h"
#include "core/mx_wifi_ipc.h"
#include "fx_utility.h"
#include <string.h>
#pragma GCC optimize ("O2")

volatile throughput_profile_t throughput_profile;
volatile throughput_spi_profile_t throughput_spi_profile;
volatile uint32_t throughput_mode, throughput_chunk = 4096, throughput_pool_bytes = 32768;
volatile uint32_t throughput_queue_depth = 20, throughput_pipeline, throughput_run_id;
volatile uint32_t throughput_server_ip, throughput_server_port = 19191;
volatile uint32_t throughput_spi_divider = 8;
volatile uint32_t network_receive_window = 8192;
volatile int32_t network_powersave = -1, network_powersave_status;
volatile uint32_t network_rtt_samples, network_rtt_min_cycles, network_rtt_max_cycles;
volatile uint32_t network_rtt_ambiguous, network_rtt_overflows, network_rtt_hist[256];
volatile uint64_t network_rtt_cycles;
typedef struct { uint32_t end, cycle, retransmits, active; } rtt_entry;
static rtt_entry rtt_pending[64] __attribute__((section(".transfer_bss"),aligned(8)));
void network_tcp_sent(NX_TCP_SOCKET *socket, uint32_t end)
{
  if (!transfer_cpu_active || socket != network_probe_socket) return;
  TX_INTERRUPT_SAVE_AREA
  TX_DISABLE
  uint32_t i;
  for (i=0; i<64; ++i) if (!rtt_pending[i].active) {
    rtt_pending[i].end=end; rtt_pending[i].cycle=transfer_cycles();
    rtt_pending[i].retransmits=socket->nx_tcp_socket_retransmit_packets; rtt_pending[i].active=1; break;
  }
  if (i==64) network_rtt_overflows++;
  TX_RESTORE
}
void network_tcp_acked(NX_TCP_SOCKET *socket, uint32_t ack)
{
  if (!transfer_cpu_active || socket != network_probe_socket) return;
  TX_INTERRUPT_SAVE_AREA
  TX_DISABLE
  uint32_t now=transfer_cycles();
  for (uint32_t i=0; i<64; ++i) if (rtt_pending[i].active && (int32_t)(ack-rtt_pending[i].end)>=0) {
    if (rtt_pending[i].retransmits == socket->nx_tcp_socket_retransmit_packets) {
      uint32_t elapsed=now-rtt_pending[i].cycle;
      network_rtt_samples++;network_rtt_cycles+=elapsed;
      if (elapsed<network_rtt_min_cycles) network_rtt_min_cycles=elapsed;
      if (elapsed>network_rtt_max_cycles) network_rtt_max_cycles=elapsed;
      uint32_t bin=elapsed/(SystemCoreClock/1000U);network_rtt_hist[bin>255?255:bin]++;
    } else network_rtt_ambiguous++;
    rtt_pending[i].active=0;
  }
  TX_RESTORE
}
volatile network_probe_t network_probe;
NX_TCP_SOCKET *network_probe_socket;
static uint32_t probe_cs_start, probe_cs_active, probe_dma_start, probe_dma_active;
static uint32_t probe_last_dma_end, probe_last_dma_valid;

void network_probe_cs(uint32_t asserted)
{
  if (!transfer_cpu_active) { probe_cs_active = 0; return; }
  uint32_t now = transfer_cycles();
  if (asserted) { probe_cs_start = now; probe_cs_active = 1; }
  else if (probe_cs_active) {
    uint32_t elapsed = now - probe_cs_start;
    network_probe.cs_calls++; network_probe.cs_cycles += elapsed;
    if (elapsed > network_probe.cs_max_cycles) network_probe.cs_max_cycles = elapsed;
    probe_cs_active = 0;
  }
}

void network_probe_dma_begin(uint32_t bytes)
{
  probe_dma_active = transfer_cpu_active;
  if (!probe_dma_active) return;
  uint32_t now = transfer_cycles();
  if (probe_last_dma_valid) {
    uint32_t gap = now-probe_last_dma_end;
    network_probe.gap_calls++; network_probe.gap_cycles += gap;
    if (gap > network_probe.gap_max_cycles) network_probe.gap_max_cycles = gap;
    uint32_t bin = gap/(SystemCoreClock/10000U);
    network_probe.gap_hist[bin > 255 ? 255 : bin]++;
  }
  probe_dma_start = now;
  if (bytes > network_probe.transaction_max_bytes) network_probe.transaction_max_bytes = bytes;
  network_probe.size_hist[bytes <= 8 ? 0 : bytes <= 128 ? 1 : bytes <= 512 ? 2 : bytes <= 1536 ? 3 : 4]++;
}

void network_probe_dma_complete(void)
{
  if (!probe_dma_active) return;
  uint32_t now = transfer_cycles();
  network_probe.dma_callback_cycles += (uint32_t)(now-probe_dma_start);
  network_probe.dma_completed++;
  probe_last_dma_end = now; probe_last_dma_valid = 1; probe_dma_active = 0;
}
volatile uint32_t throughput_sd_state[12];
volatile uint32_t fat_scan_status, fat_scan_total, fat_scan_free;
volatile uint64_t fat_scan_free_bytes;
volatile uint64_t throughput_bytes;
char throughput_path[128] = "/DL2_00020/ism330dhcx_gyro.dat";
char throughput_fw_version[24];
char throughput_directory_entries[4096];
TX_THREAD *throughput_owner;
extern AppModel_t app_model;
static NX_TCP_SOCKET sock;
static NX_PACKET_POOL packets;
static FX_FILE file;
static TX_THREAD reader;
static TX_SEMAPHORE empty[2], full[2], reader_done;
static volatile uint32_t abort_reader, slot_size[2], slot_status[2];
static uint8_t *slot[2];
static uint64_t work_bytes;
static uint32_t work_chunk;
#include "mipc_trace.inc"

/* Stopped-DAQ, RAM-only codec benchmark; no acquisition or SD writes. */
volatile uint64_t compression_phase_cycles[3];
static uint32_t compression_phase_start;
static void compression_mark(uint32_t stage)
{
  uint32_t now=transfer_cycles();
  if (stage) compression_phase_cycles[stage-1] += (uint32_t)(now-compression_phase_start);
  compression_phase_start=now;
}
#define SL_MARK(stage) compression_mark(stage)
#include "sensor_lossless.c"
#undef SL_MARK
#include "sensor_stream_codec.c"
#include "crc32_fast.h"
/* Stopped diagnostic work only; the recorder does not call this path. */
#define _NX_CRYPTO_INITIALIZE_
#include "nx_crypto_sha2.h"
#undef _NX_CRYPTO_INITIALIZE_
#include "../../../../../../Middlewares/ST/netxduo/crypto_libraries/src/nx_crypto_sha2.c"
volatile uint32_t compression_action, compression_size = 65536, compression_repeats = 50;
volatile uint32_t compression_algorithm, compression_order=1, compression_coding=1;
volatile uint32_t compression_output_limit=16384, compression_group=128;
volatile uint64_t compression_offset;
sl_schema compression_schema;
uint8_t *compression_input, *compression_work, *compression_output;
volatile uint32_t compression_status, compression_encoded, compression_codec;
volatile uint32_t compression_raw_crc, compression_encoded_crc, compression_cpu_samples;
volatile uint32_t compression_thread_samples, compression_ram_bytes, compression_clock_hz;
volatile uint64_t compression_cycles;
volatile uint32_t compression_extra_work,compression_crc_algorithm,compression_fast_raw_crc,compression_hash_algorithm;
volatile uint64_t compression_full_cycles,compression_hash_cycles,compression_crc_cycles,compression_copy_cycles;
uint8_t compression_source_sha256[32];
static uint32_t compression_capacity;
static UINT compression_hash_hardware(const uint8_t *raw,uint32_t size,uint8_t *digest);
static UINT compression_hash_context_test(void);
uint8_t compression_context_sha[12][32] __attribute__((section(".transfer_bss"),aligned(8)));
volatile uint32_t compression_context_bytes __attribute__((section(".transfer_bss")));
volatile uint32_t compression_context_status __attribute__((section(".transfer_bss")));

static __attribute__((noinline)) UINT compression_execute(FX_MEDIA *media)
{
  if (app_model.log_controller_model.status) return NX_INVALID_PARAMETERS;
  if (compression_action == 2) {
    SysFree(compression_input); SysFree(compression_work); SysFree(compression_output);
    compression_input = compression_work = compression_output = NULL;
    compression_ram_bytes = 0; return 0;
  }
  if (compression_action == 0) {
    if (compression_input || compression_work || compression_output ||
        compression_size < 1024 || compression_size > 262144 || compression_algorithm>1) return NX_INVALID_PARAMETERS;
    compression_capacity = compression_size + compression_size / 256 + 1024;
    compression_input = SysAlloc(compression_size);
    compression_work = compression_algorithm ? NULL : SysAlloc(compression_size);
    compression_output = SysAlloc(compression_capacity);
    if (!compression_input || (!compression_algorithm && !compression_work) || !compression_output) {
      SysFree(compression_input); SysFree(compression_work); SysFree(compression_output);
      compression_input = compression_work = compression_output = NULL;
      return compression_status = NX_NO_PACKET;
    }
    compression_ram_bytes = compression_size * (compression_algorithm ? 1U : 2U) + compression_capacity;
    return compression_status = 0;
  }
  if (compression_action == 4) {
    if(!compression_input || compression_size<24576U)return NX_INVALID_PARAMETERS;
    return compression_status=compression_hash_context_test();
  }
  if (compression_action == 3) {
    /* Fresh SD sample, read only, outside codec timing. No debugger function call. */
    if (!compression_input || !media || compression_offset>UINT32_MAX ||
        !memchr(throughput_path,0,sizeof(throughput_path)))return NX_INVALID_PARAMETERS;
    FX_FILE source;ULONG got=0;UINT st=fx_file_open(media,&source,throughput_path,FX_OPEN_FOR_READ);
    if(st)return st;
    st=fx_file_seek(&source,(ULONG)compression_offset);
    if(!st)st=fx_file_read(&source,compression_input,compression_size,&got);
    if(!st && got!=compression_size)st=FX_END_OF_FILE;
    UINT closed=fx_file_close(&source);
    compression_status=st?st:closed;return compression_status;
  }
  if (compression_action != 1 || !compression_input || !compression_output ||
      compression_algorithm>1 || (!compression_algorithm && !compression_work) ||
      !compression_repeats || compression_repeats > 1000)
    return NX_INVALID_PARAMETERS;
  size_t encoded = 0; uint32_t codec = 0;
  compression_cycles = 0; compression_clock_hz = SystemCoreClock;
  compression_full_cycles=compression_hash_cycles=compression_crc_cycles=compression_copy_cycles=0;
  memset((void *)compression_phase_cycles,0,sizeof(compression_phase_cycles));
  throughput_owner = tx_thread_identify();
  transfer_cpu_samples = transfer_cpu_thread_samples = 0;
  throughput_profile.cpu_self_samples = 0;
  transfer_cpu_active = 1;
  for (uint32_t i = 0; i < compression_repeats; ++i) {
    uint32_t full_start=transfer_cycles(),start=full_start;
    if(compression_extra_work) {
      memcpy(compression_output,compression_input,compression_size);
      compression_copy_cycles+=(uint32_t)(transfer_cycles()-start);
    }
    start=transfer_cycles();
    int st;
    if(compression_algorithm) {
      uint32_t limit=compression_output_limit<compression_capacity?compression_output_limit:compression_capacity;
      st=ssc_encode_grouped(compression_input,compression_size,compression_offset,&compression_schema,
                    compression_order,compression_coding,compression_group,compression_output,limit,&encoded);
      codec=6;
      if(st==1) {
        /* Benchmark keeps a full-size output allocation to dump RAW; a live
         * worker must instead use an independent staged input and writer buffer. */
        memcpy(compression_output,compression_input,compression_size);
        encoded=compression_size;codec=0;st=0;
      }
    } else st = sl_encode(compression_input, compression_size, compression_offset, &compression_schema,
                          compression_work, compression_size, compression_output, compression_capacity, &encoded, &codec);
    compression_cycles += (uint32_t)(transfer_cycles() - start);
    if(!st && compression_extra_work) {
      start=transfer_cycles();
      compression_raw_crc=(compression_crc_algorithm?stock_crc32_update(UINT32_MAX,compression_input,compression_size):transfer_bench_crc_update(UINT32_MAX,compression_input,compression_size))^UINT32_MAX;
      compression_fast_raw_crc=compression_raw_crc;
      compression_crc_cycles+=(uint32_t)(transfer_cycles()-start);
      NX_CRYPTO_SHA256 hash;
      start=transfer_cycles();
      if(compression_hash_algorithm) {
        st=(int)compression_hash_hardware(compression_input,compression_size,compression_source_sha256);
      } else {
        st=(int)_nx_crypto_sha256_initialize(&hash,NX_CRYPTO_HASH_SHA256);
        if(!st)st=(int)_nx_crypto_sha256_update(&hash,compression_input,compression_size);
        if(!st)st=(int)_nx_crypto_sha256_digest_calculate(&hash,compression_source_sha256,NX_CRYPTO_HASH_SHA256);
      }
      compression_hash_cycles+=(uint32_t)(transfer_cycles()-start);
    }
    compression_full_cycles+=(uint32_t)(transfer_cycles()-full_start);
    compression_status = (uint32_t)st;
    if (st) break;
  }
  transfer_cpu_active = 0; throughput_owner = NULL;
  compression_cpu_samples = transfer_cpu_samples; compression_thread_samples = transfer_cpu_thread_samples;
  compression_encoded = (uint32_t)encoded; compression_codec = codec;
  if (!compression_status) {
    compression_raw_crc = transfer_bench_crc_update(UINT32_MAX, compression_input, compression_size) ^ UINT32_MAX;
    compression_encoded_crc = transfer_bench_crc_update(UINT32_MAX, compression_output, encoded) ^ UINT32_MAX;
  }
  return compression_status;
}

static UINT read_block(uint8_t *p, ULONG n)
{
  ULONG got = 0;
  uint32_t t = transfer_cycles();
  UINT st = fx_file_read(&file, p, n, &got);
  uint32_t dt = transfer_cycles() - t;
  throughput_profile.read_cycles += dt;
  throughput_profile.read_calls++;
  if (dt > throughput_profile.read_max_cycles) throughput_profile.read_max_cycles = dt;
  uint32_t bin = dt / (SystemCoreClock / 10000U);
  if (bin > 127) bin = 127;
  throughput_profile.read_hist[bin]++;
  throughput_profile.read_bytes += got;
  return st ? st : (got == n ? FX_SUCCESS : FX_END_OF_FILE);
}

static UINT send_block(const uint8_t *p, ULONG n)
{
  while (n) {
    NX_PACKET *pkt;
    uint32_t t = transfer_cycles();
    UINT st = nx_packet_allocate(&packets, &pkt, NX_TCP_PACKET, 30000);
    uint32_t dt = transfer_cycles() - t;
    throughput_profile.alloc_cycles += dt;
    throughput_profile.alloc_calls++;
    if (dt > throughput_profile.alloc_max_cycles) throughput_profile.alloc_max_cycles = dt;
    if (st) { network_probe.alloc_failures++; return st; }
    ULONG k = pkt->nx_packet_data_end - pkt->nx_packet_prepend_ptr;
    if (k > sock.nx_tcp_socket_connect_mss) k = sock.nx_tcp_socket_connect_mss;
    if (k > n) k = n;
    if (!k) { nx_packet_release(pkt); return NX_INVALID_PACKET; }
    if (packets.nx_packet_pool_available < throughput_profile.pool_min)
      throughput_profile.pool_min = packets.nx_packet_pool_available;
    t = transfer_cycles();
    memcpy(pkt->nx_packet_prepend_ptr, p, k);
    pkt->nx_packet_append_ptr += k; pkt->nx_packet_length = k;
    throughput_profile.copy_cycles += (uint32_t)(transfer_cycles() - t);
    t = transfer_cycles();
    mipc_trace_event(MT_NETX_IN,0,0,k,0,0,0,0,pkt);
    st = nx_tcp_socket_send(&sock, pkt, 30000);
    mipc_trace_event(MT_NETX_OUT,0,0,k,0,(int32_t)st,0,0,pkt);
    dt = transfer_cycles() - t;
    throughput_profile.send_cycles += dt;
    throughput_profile.send_calls++;
    if (dt > throughput_profile.send_max_cycles) throughput_profile.send_max_cycles = dt;
    uint32_t w = sock.nx_tcp_socket_tx_window_advertised;
    if (w < throughput_profile.min_peer_window) throughput_profile.min_peer_window = w;
    if (w > throughput_profile.max_peer_window) throughput_profile.max_peer_window = w;
    w = sock.nx_tcp_socket_tx_window_congestion;
    if (w < throughput_profile.min_congestion_window) throughput_profile.min_congestion_window = w;
    if (w > throughput_profile.max_congestion_window) throughput_profile.max_congestion_window = w;
    if (sock.nx_tcp_socket_tx_outstanding_bytes > throughput_profile.max_outstanding)
      throughput_profile.max_outstanding = sock.nx_tcp_socket_tx_outstanding_bytes;
    if (st) { nx_packet_release(pkt); return st; }
    p += k; n -= k;
  }
  return NX_SUCCESS;
}

static void reader_entry(ULONG unused)
{
  (void)unused;
  uint64_t pos = 0;
  uint32_t i = 0;
  while (pos < work_bytes && !abort_reader) {
    if (tx_semaphore_get(&empty[i], 100) != TX_SUCCESS) continue;
    if (abort_reader) break;
    ULONG n = work_bytes - pos > work_chunk ? work_chunk : (ULONG)(work_bytes - pos);
    UINT st = read_block(slot[i], n);
    slot_size[i] = n; slot_status[i] = st;
    tx_semaphore_put(&full[i]);
    if (st) break;
    pos += n; i ^= 1U;
  }
  tx_semaphore_put(&reader_done);
}

/* Completed-acquisition codec/commit client; dormant during acquisition. */
#include "lossless_client.c"
#include "cycle30.inc"
/* Stopped single-owner experiment. The acquisition path never calls this.
 * Initialization and clock setup are included in the measured work. */
static UINT compression_hash_hardware(const uint8_t *raw,uint32_t size,uint8_t *digest)
{
  UINT st=lr_hash_init();
  if(!st && HAL_HASHEx_SHA256_Start(&lr_hash,raw,size,digest,1000)!=HAL_OK)
    st=NX_NOT_SUCCESSFUL;
  HAL_HASH_DeInit(&lr_hash);__HAL_RCC_HASH_CLK_DISABLE();
  return st;
}
#include "record_hash.c"
static UINT compression_hash_context_test(void)
{
  rh_context *contexts=SysAlloc(12U*sizeof(rh_context));
  if(!contexts)return NX_NO_PACKET;
  memset(contexts,0,12U*sizeof(rh_context));
  memset(compression_context_sha,0,sizeof(compression_context_sha));
  compression_context_bytes=12U*sizeof(rh_context);compression_context_status=0;
  rh_open();int st=0;
  for(uint32_t phase=0;phase<3U && !st;++phase) {
    for(uint32_t index=0;index<10U && !st;++index) {
      uint32_t i=phase==1U?9U-index:index;
      uint32_t offset=phase*8192U+i*4U;
      uint32_t n=phase==0U?1121U+i*4U:(phase==1U?1487U+i:2112U+i*3U);
      st=rh_update(&contexts[i],compression_input+offset,n,phase==2U,compression_context_sha[i]);
    }
  }
  /* Exact 4-byte accumulation followed by a zero-byte finish, and empty SHA. */
  if(!st)st=rh_update(&contexts[10],compression_input,compression_size,0,NULL);
  if(!st)st=rh_update(&contexts[11],NULL,0,1,compression_context_sha[11]);
  if(!st)st=rh_update(&contexts[10],NULL,0,1,compression_context_sha[10]);
  rh_shutdown();SysFree(contexts);compression_context_status=(uint32_t)st;
  return st?NX_NOT_SUCCESSFUL:0;
}
#include "native_tcp.c"

static __attribute__((noinline)) UINT throughput_execute_other(NX_IP *ip, FX_MEDIA *media)
{
  if (throughput_mode == 9) return native_execute();
  if (throughput_mode == 8 && !app_model.log_controller_model.status) return lr_hash_selftest();
  if (throughput_mode == 5) return lr_execute(ip,media);
  if (throughput_mode == 6 && !app_model.log_controller_model.status) return lr_stat(media);
  if (throughput_mode == 7 && !app_model.log_controller_model.status) {
    /* Read-only allocation recount. Do not repair FAT/FSInfo or alter FileX's
       advertised free count; report discrepancies for the preflight decision. */
    if (media->fx_media_id != FX_MEDIA_ID) return FX_MEDIA_NOT_OPEN;
    fat_scan_total=media->fx_media_total_clusters;fat_scan_free=0;fat_scan_free_bytes=0;
    fat_scan_status=tx_mutex_get(&media->fx_media_protect,TX_WAIT_FOREVER);
    if (fat_scan_status) return fat_scan_status;
    for (ULONG cluster=2;cluster<media->fx_media_total_clusters+2;cluster++) {
      ULONG value=0;fat_scan_status=_fx_utility_FAT_entry_read(media,cluster,&value);
      if (fat_scan_status) break;
      if (!value) fat_scan_free++;
    }
    tx_mutex_put(&media->fx_media_protect);
    fat_scan_free_bytes=(uint64_t)fat_scan_free*media->fx_media_sectors_per_cluster*media->fx_media_bytes_per_sector;
    return fat_scan_status;
  }
  if (throughput_mode == 3 && !app_model.log_controller_model.status) {
    /* Read-only directory diagnostic. Changes only the RAM default path. */
    char name[256]; uint32_t used = 0;
    memset(throughput_directory_entries, 0, sizeof(throughput_directory_entries));
    UINT found = fx_directory_default_set(media, throughput_path);
    if (!found) found = fx_directory_first_entry_find(media, name);
    while (!found) {
      uint32_t n = strlen(name);
      if (used + n + 2 >= sizeof(throughput_directory_entries)) break;
      memcpy(throughput_directory_entries + used, name, n); used += n;
      throughput_directory_entries[used++] = '\n';
      found = fx_directory_next_entry_find(media, name);
    }
    fx_directory_default_set(media, FX_NULL);
    throughput_profile.status = found;
    return found == FX_NO_MORE_ENTRIES ? NX_SUCCESS : found;
  }
  if (app_model.log_controller_model.status || throughput_mode > 2 || !throughput_bytes ||
      throughput_bytes > UINT32_MAX || throughput_chunk < 1024 || throughput_chunk > 65536 ||
      (throughput_chunk & (throughput_chunk - 1)) || throughput_pool_bytes < 12288 ||
      throughput_pool_bytes > 65536 || throughput_queue_depth < 1 || throughput_queue_depth > 40)
    return NX_INVALID_PARAMETERS;
  UINT st = NX_SUCCESS;
  uint32_t mode = throughput_mode, pipeline = mode == 2 && throughput_pipeline;
  uint32_t chunk = throughput_chunk, ram = mode == 1 ? 65536 : chunk * (pipeline ? 2 : 1);
  uint8_t *buf = SysAlloc(ram), *pool_mem = NULL, *reader_stack = NULL;
  uint32_t opened = 0, pool_created = 0, socket_created = 0, bound = 0, thread_created = 0;
  if (!buf) return NX_NO_PACKET;
  memset((void *)&throughput_profile, 0, sizeof(throughput_profile));
  throughput_profile.pool_min = throughput_profile.min_peer_window = throughput_profile.min_congestion_window = UINT32_MAX;
  throughput_profile.clock_hz = SystemCoreClock;
  if (mode && hspi1.Instance) {
    throughput_profile.spi_cfg1 = hspi1.Instance->CFG1;
    throughput_profile.spi_hz = HAL_RCC_GetSysClockFreq() >> (((hspi1.Instance->CFG1 >> 28) & 7U) + 1U);
  }
  throughput_profile.pipeline = pipeline;
  throughput_profile.ram_bytes = ram;
  if (mode == 1) {
    for (uint32_t i = 0; i < 65536; i++) buf[i] = (uint8_t)(i * 73U + (i >> 8) * 19U + 41U);
  } else {
    st = fx_file_open(media, &file, throughput_path, FX_OPEN_FOR_READ);
    if (st) goto done;
    opened = 1;
    if (file.fx_file_current_file_size < throughput_bytes) { st = FX_END_OF_FILE; goto done; }
  }
  if (mode) {
    pool_mem = SysAlloc(throughput_pool_bytes);
    if (!pool_mem) { st = NX_NO_PACKET; goto done; }
    throughput_profile.ram_bytes += throughput_pool_bytes;
    st = nx_packet_pool_create(&packets, "raw benchmark", 1544, pool_mem, throughput_pool_bytes);
    if (st) goto done;
    pool_created = 1; throughput_profile.pool_total = packets.nx_packet_pool_total;
    st = nx_tcp_socket_create(ip, &sock, "raw TCP", NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, network_receive_window, NULL, NULL);
    if (st) goto done;
    socket_created = 1;
    st = nx_tcp_socket_transmit_configure(&sock, throughput_queue_depth, 1000, 10, 0);
    if (st) goto done;
    throughput_profile.queue_depth = sock.nx_tcp_socket_transmit_queue_maximum;
    st = nx_tcp_client_socket_bind(&sock, NX_ANY_PORT, 10000);
    if (st) goto done;
    bound = 1;
    st = nx_tcp_client_socket_connect(&sock, throughput_server_ip, throughput_server_port, 30000);
    if (st) goto done;
    throughput_profile.mss = sock.nx_tcp_socket_connect_mss;
    memcpy(throughput_fw_version, wifi_obj_get()->SysInfo.FW_Rev, sizeof(throughput_fw_version));
  memset((void *)&network_probe, 0, sizeof(network_probe));
  if (mode) {
    wifi_get_linkinof_rparams_t link = {0};
    uint16_t length = sizeof(link);
    int32_t status = mipc_request(MIPC_API_WIFI_GET_LINKINFO_CMD, NULL, 0, (uint8_t *)&link, &length, MX_WIFI_CMD_TIMEOUT);
    network_probe.link_status = status ? status : link.status;
    if (!network_probe.link_status) {
      network_probe.association_rssi = link.info.rssi;
      network_probe.channel = link.info.channel;
      network_probe.connected = link.info.is_connected;
    }
    network_probe.socket_receive_window = sock.nx_tcp_socket_rx_window_default;
#ifdef NX_ENABLE_TCP_WINDOW_SCALING
    network_probe.window_scaling = 1;
    network_probe.send_scale = sock.nx_tcp_snd_win_scale_value;
    network_probe.receive_scale = sock.nx_tcp_rcv_win_scale_value;
#endif
  }
    network_powersave_status = network_powersave < 0 ? 0 : MX_WIFI_station_powersave(wifi_obj_get(), network_powersave);
    if (network_powersave_status) { st=NX_NOT_SUCCESSFUL;goto done; }
    uint32_t header[6] = {0x31505452U, throughput_run_id, mode, chunk, (uint32_t)throughput_bytes, 0}; /* RTP1 */
    st = send_block((uint8_t *)header, sizeof(header));
    if (st) goto done;
    /* Payload metrics exclude the tiny control header. */
    throughput_profile.alloc_calls = throughput_profile.send_calls = 0;
    throughput_profile.alloc_cycles = throughput_profile.send_cycles = throughput_profile.copy_cycles = 0;
    throughput_profile.alloc_max_cycles = throughput_profile.send_max_cycles = 0;
    memset((void *)&transfer_wifi_profile, 0, sizeof(transfer_wifi_profile));
  }
  if (pipeline) {
    reader_stack = SysAlloc(4096);
    if (!reader_stack) { st = NX_NO_PACKET; goto done; }
    throughput_profile.ram_bytes += 4096;
    for (uint32_t i = 0; i < 2; i++) {
      tx_semaphore_create(&empty[i], "empty raw buffer", 1);
      tx_semaphore_create(&full[i], "full raw buffer", 0);
      slot[i] = buf + i * chunk; slot_size[i] = slot_status[i] = 0;
    }
    tx_semaphore_create(&reader_done, "raw reader done", 0);
    abort_reader = 0; work_chunk = chunk; work_bytes = throughput_bytes;
    TX_THREAD *self = tx_thread_identify();
    st = tx_thread_create(&reader, "SD raw reader", reader_entry, 0, reader_stack, 4096,
                         self->tx_thread_priority + 1, self->tx_thread_priority + 1, TX_NO_TIME_SLICE, TX_DONT_START);
    if (st) goto pipe_cleanup;
    thread_created = 1;
  }
  probe_cs_active = probe_dma_active = probe_last_dma_valid = 0;
  memset(rtt_pending,0,sizeof(rtt_pending));memset((void *)network_rtt_hist,0,sizeof(network_rtt_hist));
  network_rtt_samples=network_rtt_max_cycles=network_rtt_ambiguous=network_rtt_overflows=0;
  network_rtt_min_cycles=UINT32_MAX;network_rtt_cycles=0;
  network_async_submitted = network_async_completed = network_async_errors = network_async_max_pending = 0;
  network_async_wait_cycles = 0;
  network_async_active = mode && network_async_depth;
  network_probe_socket = mode ? &sock : NULL;
  throughput_owner = tx_thread_identify();
  transfer_cpu_samples = transfer_cpu_thread_samples = 0;
  memset((void *)&throughput_spi_profile, 0, sizeof(throughput_spi_profile));
  transfer_cpu_active = 1;
  stock_rx_probe_begin();
  st = mipc_trace_begin();
  if (st) goto pipe_cleanup;
  throughput_profile.start_tick = tx_time_get();
  if (pipeline) tx_thread_resume(&reader);
  uint64_t pos = 0;
  uint32_t index = 0;
  while (pos < throughput_bytes) {
    ULONG n = throughput_bytes - pos > chunk ? chunk : (ULONG)(throughput_bytes - pos);
    uint8_t *data = mode == 1 ? buf + ((uint32_t)pos & 65535U) : buf;
    if (pipeline) {
      st = tx_semaphore_get(&full[index], 30000);
      if (st) break;
      st = slot_status[index]; data = slot[index]; n = slot_size[index];
    } else if (mode != 1) st = read_block(data, n);
    if (!st && mode) st = send_block(data, n);
    if (pipeline) { tx_semaphore_put(&empty[index]); index ^= 1U; }
    if (st) break;
    pos += n; throughput_profile.bytes = pos;
  }
  throughput_profile.send_end_tick = tx_time_get();
  network_async_active = 0;
  if (!st && mode) st = mipc_async_drain(30000);
  if (!st && mode) {
    /* RDON means counted by a fast RAM sink. It is NOT a durable receipt. */
    NX_PACKET *ack = NULL;
    st = nx_tcp_socket_receive(&sock, &ack, 30000);
    if (!st) {
      uint8_t response[4]; ULONG got = 0;
      st = nx_packet_data_extract_offset(ack, 0, response, 4, &got);
      if (!st && (got != 4 || memcmp(response, "RDON", 4))) st = NX_INVALID_PACKET;
      nx_packet_release(ack);
    }
  }
  throughput_profile.drain_tick = tx_time_get();
  mipc_trace_end();
  transfer_cpu_active = 0;
  throughput_owner = NULL; network_probe_socket = NULL;
  throughput_profile.cpu_samples = transfer_cpu_samples;
  throughput_profile.cpu_thread_samples = transfer_cpu_thread_samples;
pipe_cleanup:
  if (pipeline) {
    abort_reader = 1;
    for (uint32_t i = 0; i < 2; i++) tx_semaphore_put(&empty[i]);
    if (thread_created) {
      /* Never free buffers/file while a read is still owned by the producer. */
      UINT joined = tx_semaphore_get(&reader_done, 30000);
      if (joined) { throughput_profile.status = joined; return joined; } /* retain allocation until reset */
      tx_thread_terminate(&reader); tx_thread_delete(&reader);
    }
    for (uint32_t i = 0; i < 2; i++) { tx_semaphore_delete(&empty[i]); tx_semaphore_delete(&full[i]); }
    tx_semaphore_delete(&reader_done);
  }
  if (!st && mode == 0) {
    /* Independent second CRC pass, after ALL payload timers and CPU sampling. */
    st = fx_file_seek(&file, 0);
    uint32_t crc = UINT32_MAX, t = tx_time_get();
    uint64_t pos2 = 0;
    transfer_crc_mode = 1;
    while (!st && pos2 < throughput_bytes) {
      ULONG n = throughput_bytes - pos2 > chunk ? chunk : (ULONG)(throughput_bytes - pos2), got = 0;
      st = fx_file_read(&file, buf, n, &got);
      if (!st && got != n) st = FX_END_OF_FILE;
      if (!st) { crc = transfer_bench_crc_update(crc, buf, n); pos2 += n; }
    }
    throughput_profile.verify_tick = tx_time_get() - t;
    throughput_profile.crc32 = crc ^ UINT32_MAX;
    throughput_profile.verify_status = st;
  }
done:
  mipc_trace_end();
  transfer_cpu_active = 0; throughput_owner = NULL; network_async_active = 0;
  if (opened) fx_file_close(&file);
  if (socket_created) {
    throughput_profile.retransmits = sock.nx_tcp_socket_retransmit_packets;
    if (bound) { nx_tcp_socket_disconnect(&sock, 3000); nx_tcp_client_socket_unbind(&sock); }
    UINT c = nx_tcp_socket_delete(&sock);
    throughput_profile.cleanup_status = c;
  }
  if (pool_created) {
    throughput_profile.pool_empty = packets.nx_packet_pool_empty_requests;
    UINT c = nx_packet_pool_delete(&packets);
    if (c) { throughput_profile.cleanup_status = c; pool_mem = NULL; } /* preserve an owned pool on failure */
  }
  if (reader_stack) SysFree(reader_stack);
  if (pool_mem) SysFree(pool_mem);
  SysFree(buf);
  throughput_profile.finish_tick = tx_time_get();
  throughput_profile.status = st;
  return st;
}

/* Keep the stopped codec off the large network dispatcher stack frame. */
UINT throughput_execute(NX_IP *ip, FX_MEDIA *media)
{
  if(throughput_mode==4)return compression_execute(media);
  return throughput_execute_other(ip,media);
}
