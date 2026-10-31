#ifndef SD_DIAG_H
#define SD_DIAG_H
#include <stdint.h>
#include "filex_dctrl_class.h"

#define SD_DIAG_STREAMS 16U
#define SD_DIAG_EVENTS 1536U
#define SD_DIAG_HIST 282U
#define SD_DIAG_HISTORY 128U
#define SD_DIAG_UNKNOWN 0xffffffffU
enum sd_diag_reason {
  SD_FAIL_NONE, SD_FAIL_QUEUE_HIGH, SD_FAIL_BUFFER_EXHAUSTED,
  SD_FAIL_FILEX_WRITE, SD_FAIL_PARTIAL_OFFSET, SD_FAIL_FILEX_FLUSH,
  SD_FAIL_FILEX_CLOSE, SD_FAIL_DISKIO, SD_FAIL_HAL_START,
  SD_FAIL_HAL_CALLBACK, SD_FAIL_TIMEOUT, SD_FAIL_BUFFER_OWNERSHIP,
  SD_FAIL_STREAM_POST, SD_FAIL_QUEUE_SEND, SD_FAIL_UNKNOWN
};
enum sd_diag_event {
  SD_EV_START=1, SD_EV_PRODUCER, SD_EV_BUFFER_FILL, SD_EV_BUFFER_RELEASE,
  SD_EV_QUEUE_PUSH, SD_EV_QUEUE_POP, SD_EV_QUEUE_HIGH,
  SD_EV_WRITE_BEGIN, SD_EV_WRITE_END, SD_EV_FLUSH_BEGIN, SD_EV_FLUSH_END,
  SD_EV_CLOSE_BEGIN, SD_EV_CLOSE_END, SD_EV_DISK_BEGIN, SD_EV_DISK_END,
  SD_EV_DMA_BEGIN, SD_EV_DMA_END, SD_EV_HAL_ERROR, SD_EV_SEM_WAIT,
  SD_EV_SEM_END, SD_EV_CARD_WAIT, SD_EV_FIRST_FAILURE, SD_EV_SD_FAILED,
  SD_EV_STOP, SD_EV_RESOURCE, SD_EV_CARD_STATE, SD_EV_SELECT,
  SD_EV_FIFO_OVERFLOW, SD_EV_ACQ_DMA_OVERRUN, SD_EV_SENSOR_IO_ERROR,
  SD_EV_REGISTER_OVERRUN
};
typedef struct {
  uint32_t tick, cycle, event, arg0, arg1, arg2;
} sd_diag_event_t;
typedef struct {
  uint32_t count, min_us, max_us, errors, partials;
  uint64_t bytes, total_us;
  uint32_t over_ms[6], hist[SD_DIAG_HIST];
} sd_diag_latency_t;
typedef struct {
  uint32_t enabled, sensor_id, bytes_per_sample, item_size, items, min_free;
  uint32_t last_free, last_ready, max_burst_samples, first_tick, last_tick;
  uint64_t samples_delivered, payload_bytes, stream_input_bytes, filex_bytes;
  uint32_t producer_calls, buffer_exhaustions, post_errors, fifo_observed, fifo_overflows;
  uint32_t nominal_rate_millihz, samples_per_ts, fifo_max_level, fifo_last_level;
  uint32_t acquisition_queue_errors, sensor_io_errors, dma_callbacks, dma_last_seq;
  uint32_t dma_last_processed, dma_sequence_gaps, dma_stale_accesses, dma_fault_flags;
  uint64_t driver_samples_read, warmup_samples, dma_generated_samples;
  uint32_t dma_late_returns, dma_first_suspect_tick, dma_first_suspect_phase;
  uint32_t dma_warmup_late_callbacks;
  uint32_t register_status_reads, register_overruns, register_warmup_overruns, irq_callbacks;
  uint64_t irq_generated_samples;
} sd_diag_stream_t;
typedef struct {
  uint32_t valid, tick, cycle, elapsed_ticks, reason, stream, detail;
  uint32_t thread, queue_depth, queue_capacity, queue_high, free_items, ready_items;
  uint32_t op, op_start_tick, op_start_cycle, requested, actual_offset_delta;
  uint32_t filex_status, flush_status, close_status, disk_status, semaphore_status;
  uint32_t hal_state, hal_error, hal_context, sdmmc_sta, sdmmc_dctrl, sdmmc_dcount, idmactrl;
  uint32_t tx_ptr, rx_ptr, dma_tx_error, dma_rx_error, heap_available;
  uint64_t file_offset, file_size, sd_free_bytes;
  uint32_t last_write_us, max_write_us;
  uint32_t card_state, card_wait_active, card_wait_start_tick, card_wait_start_cycle;
  sd_diag_stream_t streams[SD_DIAG_STREAMS];
} sd_diag_failure_t;
typedef struct {
  uint32_t magic, version, active, frozen, halt_on_failure, start_tick, stop_tick;
  uint32_t clock_hz, trace_count, trace_next, trace_capacity, trace_alloc_failed;
  uint32_t queue_high, queue_pushes, queue_pops, queue_send_errors, buffer_errors;
  uint32_t trigger_count, trigger_reason, trigger_tick, current_thread, current_op;
  uint32_t op_tick, op_cycle, op_requested, op_stream, op_queue_before, op_free_before;
  uint64_t op_produced_before, produced_during_last_write;
  uint32_t last_write_us, last_queue_after, last_free_after;
  uint32_t filex_status, flush_status, close_status, disk_status, semaphore_status;
  uint32_t hal_starts, hal_completions, hal_errors, timeouts, card_wait_tick, card_wait_cycle;
  uint32_t card_state, card_wait_active, card_polls, card_busy_polls, card_wait_calls;
  uint32_t card_busy_waits, card_wait_last_us, card_wait_max_us;
  uint32_t urgency_enabled, urgency_selections, ready_messages, blocks_completed, empty_ready_messages;
  uint32_t chunk_bytes, tail_fixed, chunk_calls, post_failure_write_us, post_failure_write_status;
  uint32_t adf_error_flags, mdf_error_flags, adf_dma_error_flags, mdf_dma_error_flags;
  uint32_t split_ring;
  uint32_t trace_cost_cycles, trace_cost_max_cycles, min_heap_available;
  uint32_t cpu_samples, idle_samples, writer_samples, cpu_thread[32];
  uint32_t writer_ready_max_ticks, writer_ready_since, writer_wait_kind;
  uint32_t history_next, history_count;
  struct { uint32_t tick, stream, bytes, us, queue_before, queue_after, free_before, free_after;
           uint64_t produced_during; } history[SD_DIAG_HISTORY];
  sd_diag_latency_t write, flush, close;
  sd_diag_stream_t streams[SD_DIAG_STREAMS];
  sd_diag_failure_t first;
  sd_diag_event_t *trace;
} sd_diag_state_t;
extern volatile sd_diag_state_t sd_diag;
extern TX_BYTE_POOL *sd_diag_pool;
void sd_diag_begin(filex_dctrl_class_t *obj);
void sd_diag_end(void);
void sd_diag_event(uint32_t event, uint32_t a, uint32_t b, uint32_t c);
void sd_diag_fail(uint32_t reason, uint32_t stream, uint32_t detail);
void sd_diag_trigger(void);
void sd_diag_failure_marker(void);
void sd_diag_finished_marker(void);
void sd_diag_producer(uint32_t stream, uint32_t sensor, uint32_t samples, uint32_t bps);
void sd_diag_buffers(uint32_t stream, uint32_t event);
void sd_diag_queue(uint32_t event, TX_QUEUE *queue, uint32_t message);
UINT sd_diag_queue_send(TX_QUEUE *queue,VOID *message,ULONG wait);
uint16_t sd_diag_release(CircularBuffer *buffer,CBItem *item);
UINT sd_diag_file_write(FX_FILE *file, VOID *buffer, ULONG count);
UINT sd_diag_media_flush(FX_MEDIA *media);
UINT sd_diag_file_close(FX_FILE *file);
VOID sd_diag_disk_driver(FX_MEDIA *media);
UINT sd_diag_semaphore_get(TX_SEMAPHORE *sem, ULONG wait, uint32_t site);
void sd_diag_hal_begin(uint32_t sector, uint32_t blocks, uint32_t write);
void sd_diag_hal_end(uint32_t status);
void sd_diag_hal_callback(uint32_t error);
void sd_diag_tick(void);
void sd_diag_card_begin(void);
void sd_diag_card_poll(uint32_t state);
void sd_diag_card_end(void);
uint32_t sd_diag_choose(uint32_t message);
void sd_diag_ready_result(uint32_t consumed);
void sd_diag_config(uint32_t stream,uint32_t sensor,uint32_t bps,uint32_t spts,uint32_t rate_millihz);
void sd_diag_sensor_fifo(uint32_t sensor,uint32_t level,uint32_t overflow,uint32_t io_status);
void sd_diag_sensor_read(uint32_t sensor,uint32_t samples,uint32_t warmup);
uint16_t sd_diag_sensor_dma(uint32_t sensor,uint32_t samples);
void sd_diag_sensor_dma_check(uint32_t sensor,uint16_t sequence,uint32_t consumed);
void sd_diag_sensor_queue_error(uint32_t sensor);
void sd_diag_sensor_io(uint32_t sensor,int32_t status);
void sd_diag_sensor_register(uint32_t sensor,uint32_t overrun,int32_t status,uint32_t warmup);
void sd_diag_sensor_irq(uint32_t sensor,uint32_t generated_samples);
void sd_diag_ready_message(void);
UINT sd_diag_chunk_ready(uint32_t message);
UINT sd_diag_drain(uint32_t stream);
uint32_t sd_diag_item_size(uint32_t stream,uint32_t original);
#endif
