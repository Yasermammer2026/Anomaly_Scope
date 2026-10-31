#ifndef TRANSFER_BENCH_H
#define TRANSFER_BENCH_H
#include <stdint.h>
#include "stm32u5xx.h"
#include "tx_api.h"
#include "fx_api.h"
#include "nx_api.h"

typedef struct {
  uint32_t clock_hz, start_tick, finish_tick;
  uint32_t read_calls, alloc_calls, send_calls, errors;
  uint32_t min_read, max_read, min_packets;
  uint64_t bytes, read_ticks, alloc_ticks, send_ticks;
  uint64_t read_cycles, alloc_cycles, send_cycles;
  uint32_t read_max_ticks, alloc_max_ticks, send_max_ticks;
  uint32_t read_error, send_error, retransmits, min_peer_window;
} transfer_ftp_profile_t;

typedef struct {
  uint32_t calls, errors, receive_pool_drops;
  uint64_t bytes, ticks, cycles;
  uint32_t max_ticks;
} transfer_wifi_profile_t;

extern volatile transfer_ftp_profile_t transfer_ftp_profile;
extern volatile transfer_wifi_profile_t transfer_wifi_profile;
extern volatile uint32_t transfer_debug_command, transfer_debug_completed;
extern volatile uint32_t transfer_debug_result, transfer_debug_wifi, transfer_debug_ftp;
extern volatile uint32_t transfer_ack_armed, transfer_ack_received;
extern uint8_t transfer_expected_ack[64];
typedef struct {
  uint32_t start_tick, finish_tick, payload_start_tick, payload_finish_tick, ack_tick;
  uint32_t connect_ticks, status, read_calls, alloc_calls, send_calls, max_send;
  uint32_t ram_bytes, pool_packets, min_packets, pool_empty, retransmits, crc32;
  uint32_t mqtt_pubacks, mqtt_puback_max_ticks, mqtt_chunks;
  uint32_t min_peer_window, max_peer_window, min_congestion_window, max_congestion_window, max_outstanding;
  uint64_t mqtt_puback_ticks;
  uint64_t bytes, read_ticks, alloc_ticks, send_ticks, crc_ticks;
} transfer_client_profile_t;
extern volatile transfer_client_profile_t transfer_client_profile;
extern volatile uint32_t transfer_protocol, transfer_source_mode, transfer_chunk_size;
extern volatile uint32_t transfer_mqtt_window;
extern volatile uint32_t transfer_client_pool_bytes, transfer_ftp_packet_payload, transfer_ftp_timeout_ticks;
extern volatile uint32_t transfer_ftp_pool_bytes;
UINT transfer_ftp_packets_allocate(TX_BYTE_POOL *pool, UCHAR **memory);
UINT transfer_ftp_packets_release(UCHAR *memory);
extern volatile uint32_t transfer_interrupt_after, transfer_interrupt_fired;
extern volatile uint32_t transfer_interrupt_tick, transfer_interrupt_status;
extern volatile uint32_t transfer_crc_mode;
void transfer_bench_maybe_interrupt(NX_IP *ip, uint64_t bytes);
extern volatile uint64_t transfer_resume_offset;
extern volatile int32_t transfer_scan_rssi, transfer_scan_matches, transfer_scan_channel, transfer_scan_status;
extern volatile uint32_t transfer_server_ip, transfer_server_port;
extern volatile uint64_t transfer_file_size;
extern char transfer_file_path[128];
extern volatile uint32_t transfer_cpu_active, transfer_cpu_samples, transfer_cpu_thread_samples;
extern volatile uint32_t transfer_record_start, transfer_record_stop, transfer_record_seconds;
UINT transfer_bench_execute(uint32_t command, NX_IP *ip, FX_MEDIA *media);
void transfer_bench_service(void);
void transfer_ftp_profile_reset(void);
int transfer_ftp_ack_check(FX_MEDIA *media);
static inline uint32_t transfer_cycles(void) { return DWT->CYCCNT; }
#endif
