#ifndef THROUGHPUT_CLIENT_H
#define THROUGHPUT_CLIENT_H
#include "transfer_bench.h"
/* Stopped recorder only. Read-only source; no on-card benchmark writes. */
typedef struct {
  uint32_t status, start_tick, send_end_tick, drain_tick, finish_tick;
  uint32_t clock_hz, spi_cfg1, spi_hz, mss, queue_depth;
  uint32_t ram_bytes, pool_total, pool_min, pool_empty, retransmits;
  uint32_t read_calls, send_calls, alloc_calls, read_max_cycles;
  uint32_t alloc_max_cycles, send_max_cycles, min_peer_window, max_peer_window;
  uint32_t min_congestion_window, max_congestion_window, max_outstanding;
  uint32_t cpu_samples, cpu_thread_samples, cpu_self_samples, cleanup_status;
  uint32_t verify_tick, crc32, verify_status, pipeline;
  uint64_t bytes, read_bytes, read_cycles, alloc_cycles, send_cycles, copy_cycles;
  uint32_t read_hist[128]; /* 100 us bins, last bin >=12.7 ms */
} throughput_profile_t;
typedef struct {
  uint32_t flow_calls, flow_errors, dma_calls, dma_errors;
  uint64_t flow_cycles, dma_cycles, wire_bytes;
} throughput_spi_profile_t;
extern volatile throughput_spi_profile_t throughput_spi_profile;
extern volatile throughput_profile_t throughput_profile;
extern volatile uint32_t throughput_mode, throughput_chunk, throughput_pool_bytes;
extern volatile uint32_t throughput_queue_depth, throughput_pipeline, throughput_run_id;
extern volatile uint32_t throughput_server_ip, throughput_server_port;
extern volatile uint64_t throughput_bytes;
extern char throughput_path[128];
extern TX_THREAD *throughput_owner;
extern char throughput_fw_version[24];
extern volatile uint32_t throughput_spi_divider;
extern volatile uint32_t throughput_sd_state[12];
/* Software call intervals, not electrical SCK measurements. */
typedef struct {
  uint32_t cs_calls, cs_max_cycles, dma_completed, dma_start_errors;
  uint32_t gap_calls, gap_max_cycles, transaction_max_bytes;
  uint32_t mipc_calls, mipc_errors, tcp_suspend_calls, tcp_window_waits, tcp_queue_waits;
  uint32_t alloc_failures, socket_receive_window, window_scaling, send_scale, receive_scale;
  int32_t association_rssi, link_status;
  uint32_t channel, connected;
  uint64_t cs_cycles, dma_callback_cycles, gap_cycles;
  uint64_t mipc_mutex_cycles, mipc_hci_cycles, mipc_response_cycles, tcp_suspend_cycles;
  uint32_t size_hist[5], gap_hist[256];
} network_probe_t;
extern volatile network_probe_t network_probe;
extern NX_TCP_SOCKET *network_probe_socket;
extern volatile uint32_t network_receive_window;
extern volatile uint32_t network_async_depth, network_async_active;
extern volatile uint32_t network_async_submitted, network_async_completed, network_async_errors, network_async_max_pending;
extern volatile uint64_t network_async_wait_cycles;
void network_tcp_sent(NX_TCP_SOCKET *socket, uint32_t end_sequence);
void network_tcp_acked(NX_TCP_SOCKET *socket, uint32_t ack_sequence);
void network_probe_cs(uint32_t asserted);
void network_probe_dma_begin(uint32_t bytes);
void network_probe_dma_complete(void);
/* Stopped-transfer diagnostic only. Allocation is bounded, and the first
 * explicit module -1 freezes the ring before any ownership is reclaimed. */
enum { MT_NETX_IN=1, MT_NETX_OUT, MT_SUBMIT, MT_HCI_DONE, MT_SPI_QUEUE,
       MT_SPI_SERVICE, MT_FLOW_IN, MT_FLOW_OUT, MT_DMA_IN, MT_DMA_OUT,
       MT_IRQ, MT_RX_ENQUEUE, MT_RESPONSE_IN, MT_RESPONSE_OUT, MT_COMPLETE,
       MT_UNMATCHED, MT_FREEZE, MT_CS_LOW, MT_CS_HIGH, MT_SPI_HEADER,
       MT_ALLOC_IN, MT_ALLOC_OUT, MT_WAIT_IN, MT_WAIT_OUT, MT_LINK_IN, MT_LINK_OUT,
       MT_RX_DROP, MT_RX_ALLOC_FAIL, MT_RX_OVERSIZE };
void mipc_trace_event(uint32_t event, uint32_t id, uint32_t api, uint32_t bytes,
                      uint32_t slot_depth, int32_t status, uint32_t arg0,
                      uint32_t arg1, const void *pointer);
void mipc_trace_submit(uint32_t id, uint32_t api, uint32_t bytes,
                       uint32_t slot_depth, const void *pointer);
void mipc_trace_response(uint32_t id, uint32_t api, uint32_t bytes,
                         uint32_t slot_depth, int32_t status, uint32_t valid,
                         const void *pointer);
UINT mipc_trace_begin(void);
void mipc_trace_end(void);
UINT mipc_trace_release(void);
extern volatile uint32_t mipc_trace_gpio_enable;
UINT throughput_execute(NX_IP *ip, FX_MEDIA *media);
uint32_t transfer_bench_crc_update(uint32_t crc, const uint8_t *data, uint32_t count);

void stock_rx_probe_begin(void);
extern volatile uint32_t stock_rx_min_free;
#endif
