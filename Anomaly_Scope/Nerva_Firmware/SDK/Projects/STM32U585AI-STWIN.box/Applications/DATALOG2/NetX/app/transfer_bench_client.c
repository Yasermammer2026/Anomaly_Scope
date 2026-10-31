/* Completed-file experiments only. Native recorder/FileX/RTOS sources stay unchanged. */
#include "transfer_bench.h"
#include "throughput_client.h"
#include "nx_api.h"
#include "services/sysmem.h"
#include "DatalogAppTask.h"
#include "App_model.h"
#include "mx_wifi.h"
#include "mx_wifi_io.h"
#include <string.h>
#include <stdio.h>

#pragma GCC optimize ("O1")

volatile transfer_ftp_profile_t transfer_ftp_profile;
volatile transfer_wifi_profile_t transfer_wifi_profile;
volatile uint32_t transfer_debug_command, transfer_debug_completed;
volatile uint32_t transfer_debug_result, transfer_debug_wifi, transfer_debug_ftp;
volatile uint32_t transfer_ack_armed, transfer_ack_received;
uint8_t transfer_expected_ack[64];
volatile transfer_client_profile_t transfer_client_profile;
volatile uint32_t transfer_protocol, transfer_source_mode, transfer_chunk_size = 4096;
volatile uint32_t transfer_mqtt_window = 1;
volatile uint32_t transfer_client_pool_bytes = 12288;
volatile uint32_t transfer_ftp_packet_payload = 512;
volatile uint32_t transfer_ftp_timeout_ticks = TX_TIMER_TICKS_PER_SECOND;
volatile uint32_t transfer_ftp_pool_bytes = 12288;
volatile uint32_t transfer_interrupt_after, transfer_interrupt_fired;
volatile uint32_t transfer_interrupt_tick, transfer_interrupt_status;
volatile uint32_t transfer_crc_mode;
volatile uint32_t transfer_cleanup_profile[6];
volatile uint64_t transfer_resume_offset;
volatile int32_t transfer_scan_rssi, transfer_scan_matches, transfer_scan_channel, transfer_scan_status;
extern char wifi_ssid[];
volatile uint32_t transfer_server_ip, transfer_server_port = 19090;
volatile uint64_t transfer_file_size;
char transfer_file_path[128] = "/BENCH/T010M.BIN";
volatile uint32_t transfer_cpu_active, transfer_cpu_samples, transfer_cpu_thread_samples;
volatile uint32_t transfer_record_start, transfer_record_stop, transfer_record_seconds = 1800;
static uint32_t record_requested, record_deadline;
static NX_TCP_SOCKET client_socket;
static NX_PACKET_POOL client_pool;
static FX_FILE source_file;
static NX_PACKET *rx_packet;
static ULONG rx_offset;
static char mqtt_root[32], mqtt_ack_topic[40];
static uint16_t mqtt_ack_id;
static uint32_t ftp_packets_from_sys;

UINT transfer_ftp_packets_allocate(TX_BYTE_POOL *pool, UCHAR **memory)
{
  if (transfer_ftp_pool_bytes < 12288 || transfer_ftp_pool_bytes > 65536 ||
      (transfer_ftp_packet_payload != 512 && transfer_ftp_packet_payload != 1544)) return TX_SIZE_ERROR;
  ftp_packets_from_sys = transfer_ftp_pool_bytes > 12288;
  if (!ftp_packets_from_sys) return tx_byte_allocate(pool,(VOID **)memory,transfer_ftp_pool_bytes,TX_NO_WAIT);
  *memory = SysAlloc(transfer_ftp_pool_bytes);
  return *memory ? TX_SUCCESS : TX_NO_MEMORY;
}

UINT transfer_ftp_packets_release(UCHAR *memory)
{
  if (!ftp_packets_from_sys) return tx_byte_release(memory);
  SysFree(memory);
  return TX_SUCCESS;
}

extern TX_THREAD *_tx_thread_current_ptr;
extern AppModel_t app_model;
void __real__tx_timer_interrupt(void);
void __wrap__tx_timer_interrupt(void)
{
  /* 1 kHz thread-occupancy sampling. IRQ utilization is not included. */
  if (transfer_cpu_active) {
    transfer_cpu_samples++;
    if (_tx_thread_current_ptr != NULL) transfer_cpu_thread_samples++;
    if (throughput_owner && _tx_thread_current_ptr == throughput_owner) throughput_profile.cpu_self_samples++;
  }
  __real__tx_timer_interrupt();
}

void transfer_ftp_profile_reset(void)
{
  memset((void *)&transfer_ftp_profile, 0, sizeof(transfer_ftp_profile));
  memset((void *)&transfer_wifi_profile, 0, sizeof(transfer_wifi_profile));
  transfer_ftp_profile.clock_hz = SystemCoreClock;
  transfer_ftp_profile.start_tick = tx_time_get();
  transfer_ftp_profile.min_read = UINT32_MAX;
  transfer_ftp_profile.min_packets = UINT32_MAX;
  transfer_ftp_profile.min_peer_window = UINT32_MAX;
  transfer_interrupt_fired = transfer_interrupt_tick = transfer_interrupt_status = 0;
  transfer_cpu_samples = transfer_cpu_thread_samples = 0;
  memset((void *)transfer_cleanup_profile,0,sizeof(transfer_cleanup_profile));
  transfer_cpu_active = 1;
}

int transfer_ftp_ack_check(FX_MEDIA *media)
{
  FX_FILE file;
  uint8_t received[65];
  ULONG count = 0;
  if (!transfer_ack_armed) return 0;
  UINT status = fx_file_open(media, &file, "/BENCH/ACK.BIN", FX_OPEN_FOR_READ);
  if (status != FX_SUCCESS) return 0;
  status = fx_file_read(&file, received, sizeof(received), &count);
  fx_file_close(&file);
  if (status != FX_SUCCESS || count != 64 || memcmp(received, transfer_expected_ack, 64)) return 0;
  transfer_ack_received = 1;
  transfer_ack_armed = 0;
  return 1;
}

static const uint32_t crc_table[256] = {
  0x00000000U, 0x77073096U, 0xEE0E612CU, 0x990951BAU, 0x076DC419U, 0x706AF48FU, 0xE963A535U, 0x9E6495A3U,
  0x0EDB8832U, 0x79DCB8A4U, 0xE0D5E91EU, 0x97D2D988U, 0x09B64C2BU, 0x7EB17CBDU, 0xE7B82D07U, 0x90BF1D91U,
  0x1DB71064U, 0x6AB020F2U, 0xF3B97148U, 0x84BE41DEU, 0x1ADAD47DU, 0x6DDDE4EBU, 0xF4D4B551U, 0x83D385C7U,
  0x136C9856U, 0x646BA8C0U, 0xFD62F97AU, 0x8A65C9ECU, 0x14015C4FU, 0x63066CD9U, 0xFA0F3D63U, 0x8D080DF5U,
  0x3B6E20C8U, 0x4C69105EU, 0xD56041E4U, 0xA2677172U, 0x3C03E4D1U, 0x4B04D447U, 0xD20D85FDU, 0xA50AB56BU,
  0x35B5A8FAU, 0x42B2986CU, 0xDBBBC9D6U, 0xACBCF940U, 0x32D86CE3U, 0x45DF5C75U, 0xDCD60DCFU, 0xABD13D59U,
  0x26D930ACU, 0x51DE003AU, 0xC8D75180U, 0xBFD06116U, 0x21B4F4B5U, 0x56B3C423U, 0xCFBA9599U, 0xB8BDA50FU,
  0x2802B89EU, 0x5F058808U, 0xC60CD9B2U, 0xB10BE924U, 0x2F6F7C87U, 0x58684C11U, 0xC1611DABU, 0xB6662D3DU,
  0x76DC4190U, 0x01DB7106U, 0x98D220BCU, 0xEFD5102AU, 0x71B18589U, 0x06B6B51FU, 0x9FBFE4A5U, 0xE8B8D433U,
  0x7807C9A2U, 0x0F00F934U, 0x9609A88EU, 0xE10E9818U, 0x7F6A0DBBU, 0x086D3D2DU, 0x91646C97U, 0xE6635C01U,
  0x6B6B51F4U, 0x1C6C6162U, 0x856530D8U, 0xF262004EU, 0x6C0695EDU, 0x1B01A57BU, 0x8208F4C1U, 0xF50FC457U,
  0x65B0D9C6U, 0x12B7E950U, 0x8BBEB8EAU, 0xFCB9887CU, 0x62DD1DDFU, 0x15DA2D49U, 0x8CD37CF3U, 0xFBD44C65U,
  0x4DB26158U, 0x3AB551CEU, 0xA3BC0074U, 0xD4BB30E2U, 0x4ADFA541U, 0x3DD895D7U, 0xA4D1C46DU, 0xD3D6F4FBU,
  0x4369E96AU, 0x346ED9FCU, 0xAD678846U, 0xDA60B8D0U, 0x44042D73U, 0x33031DE5U, 0xAA0A4C5FU, 0xDD0D7CC9U,
  0x5005713CU, 0x270241AAU, 0xBE0B1010U, 0xC90C2086U, 0x5768B525U, 0x206F85B3U, 0xB966D409U, 0xCE61E49FU,
  0x5EDEF90EU, 0x29D9C998U, 0xB0D09822U, 0xC7D7A8B4U, 0x59B33D17U, 0x2EB40D81U, 0xB7BD5C3BU, 0xC0BA6CADU,
  0xEDB88320U, 0x9ABFB3B6U, 0x03B6E20CU, 0x74B1D29AU, 0xEAD54739U, 0x9DD277AFU, 0x04DB2615U, 0x73DC1683U,
  0xE3630B12U, 0x94643B84U, 0x0D6D6A3EU, 0x7A6A5AA8U, 0xE40ECF0BU, 0x9309FF9DU, 0x0A00AE27U, 0x7D079EB1U,
  0xF00F9344U, 0x8708A3D2U, 0x1E01F268U, 0x6906C2FEU, 0xF762575DU, 0x806567CBU, 0x196C3671U, 0x6E6B06E7U,
  0xFED41B76U, 0x89D32BE0U, 0x10DA7A5AU, 0x67DD4ACCU, 0xF9B9DF6FU, 0x8EBEEFF9U, 0x17B7BE43U, 0x60B08ED5U,
  0xD6D6A3E8U, 0xA1D1937EU, 0x38D8C2C4U, 0x4FDFF252U, 0xD1BB67F1U, 0xA6BC5767U, 0x3FB506DDU, 0x48B2364BU,
  0xD80D2BDAU, 0xAF0A1B4CU, 0x36034AF6U, 0x41047A60U, 0xDF60EFC3U, 0xA867DF55U, 0x316E8EEFU, 0x4669BE79U,
  0xCB61B38CU, 0xBC66831AU, 0x256FD2A0U, 0x5268E236U, 0xCC0C7795U, 0xBB0B4703U, 0x220216B9U, 0x5505262FU,
  0xC5BA3BBEU, 0xB2BD0B28U, 0x2BB45A92U, 0x5CB36A04U, 0xC2D7FFA7U, 0xB5D0CF31U, 0x2CD99E8BU, 0x5BDEAE1DU,
  0x9B64C2B0U, 0xEC63F226U, 0x756AA39CU, 0x026D930AU, 0x9C0906A9U, 0xEB0E363FU, 0x72076785U, 0x05005713U,
  0x95BF4A82U, 0xE2B87A14U, 0x7BB12BAEU, 0x0CB61B38U, 0x92D28E9BU, 0xE5D5BE0DU, 0x7CDCEFB7U, 0x0BDBDF21U,
  0x86D3D2D4U, 0xF1D4E242U, 0x68DDB3F8U, 0x1FDA836EU, 0x81BE16CDU, 0xF6B9265BU, 0x6FB077E1U, 0x18B74777U,
  0x88085AE6U, 0xFF0F6A70U, 0x66063BCAU, 0x11010B5CU, 0x8F659EFFU, 0xF862AE69U, 0x616BFFD3U, 0x166CCF45U,
  0xA00AE278U, 0xD70DD2EEU, 0x4E048354U, 0x3903B3C2U, 0xA7672661U, 0xD06016F7U, 0x4969474DU, 0x3E6E77DBU,
  0xAED16A4AU, 0xD9D65ADCU, 0x40DF0B66U, 0x37D83BF0U, 0xA9BCAE53U, 0xDEBB9EC5U, 0x47B2CF7FU, 0x30B5FFE9U,
  0xBDBDF21CU, 0xCABAC28AU, 0x53B39330U, 0x24B4A3A6U, 0xBAD03605U, 0xCDD70693U, 0x54DE5729U, 0x23D967BFU,
  0xB3667A2EU, 0xC4614AB8U, 0x5D681B02U, 0x2A6F2B94U, 0xB40BBE37U, 0xC30C8EA1U, 0x5A05DF1BU, 0x2D02EF8DU,
};

static uint32_t crc_update(uint32_t crc, const uint8_t *data, uint32_t count)
{
  if (transfer_crc_mode == 1) {
    while (count--) crc = (crc >> 8) ^ crc_table[(crc ^ *data++) & 255U];
    return crc;
  }
  while (count--) {
    crc ^= *data++;
    for (uint32_t bit = 0; bit < 8; bit++) crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
  }
  return crc;
}

uint32_t transfer_bench_crc_update(uint32_t crc, const uint8_t *data, uint32_t count)
{ return crc_update(crc, data, count); }

void transfer_bench_service(void)
{
  if (!record_requested) return;
  uint32_t now = tx_time_get();
  if (!transfer_record_start && app_model.log_controller_model.status) {
    transfer_record_start = now;
    record_deadline = now + transfer_record_seconds * TX_TIMER_TICKS_PER_SECOND;
  }
  if (transfer_record_start && !app_model.log_controller_model.status) {
    /* Observe an early native stop; never turn the planned timer into a
       misleading claim that sensor recording lasted the whole interval. */
    transfer_record_stop = now;
    record_requested = 0;
    return;
  }
  if (transfer_record_start && (int32_t)(now - record_deadline) >= 0) {
    if (app_model.log_controller_model.status) DatalogAppTask_msg(DT_USER_BUTTON);
    transfer_record_stop = now;
    record_requested = 0;
  }
}

void transfer_bench_maybe_interrupt(NX_IP *ip, uint64_t bytes)
{
  if (transfer_interrupt_after && !transfer_interrupt_fired && bytes >= transfer_interrupt_after) {
    ULONG driver_status = 0;
    transfer_interrupt_tick = tx_time_get();
    transfer_interrupt_fired = 1;
    UINT status = nx_ip_driver_direct_command(ip, NX_LINK_DISABLE, &driver_status);
    transfer_interrupt_status = status ? status : (uint32_t)driver_status;
  }
}

static UINT send_bytes(const void *data, ULONG length)
{
  const uint8_t *position = data;
  while (length) {
    transfer_bench_maybe_interrupt(client_socket.nx_tcp_socket_ip_ptr, transfer_client_profile.bytes);
    NX_PACKET *packet;
    uint32_t start = tx_time_get();
    UINT status = nx_packet_allocate(&client_pool, &packet, NX_TCP_PACKET, TX_TIMER_TICKS_PER_SECOND * 30);
    transfer_client_profile.alloc_ticks += tx_time_get() - start;
    transfer_client_profile.alloc_calls++;
    if (status) return status;
    ULONG count = packet->nx_packet_data_end - packet->nx_packet_prepend_ptr;
    /* mss is an optional user override (zero by default). connect_mss is
       the negotiated effective bound used by the native FTP implementation. */
    if (count > client_socket.nx_tcp_socket_connect_mss) count = client_socket.nx_tcp_socket_connect_mss;
    if (!count) { nx_packet_release(packet); return NX_INVALID_PACKET; }
    if (count > length) count = length;
    if (client_pool.nx_packet_pool_available < transfer_client_profile.min_packets)
      transfer_client_profile.min_packets = client_pool.nx_packet_pool_available;
    /* A single packet, bounded by MSS: the WiFi driver rejects packet chains. */
    memcpy(packet->nx_packet_prepend_ptr, position, count);
    packet->nx_packet_append_ptr += count;
    packet->nx_packet_length = count;
    start = tx_time_get();
    status = nx_tcp_socket_send(&client_socket, packet, TX_TIMER_TICKS_PER_SECOND * 30);
    transfer_client_profile.send_ticks += tx_time_get() - start;
    transfer_client_profile.send_calls++;
    uint32_t peer = client_socket.nx_tcp_socket_tx_window_advertised;
    uint32_t congestion = client_socket.nx_tcp_socket_tx_window_congestion;
    if (peer < transfer_client_profile.min_peer_window) transfer_client_profile.min_peer_window = peer;
    if (peer > transfer_client_profile.max_peer_window) transfer_client_profile.max_peer_window = peer;
    if (congestion < transfer_client_profile.min_congestion_window) transfer_client_profile.min_congestion_window = congestion;
    if (congestion > transfer_client_profile.max_congestion_window) transfer_client_profile.max_congestion_window = congestion;
    if (client_socket.nx_tcp_socket_tx_outstanding_bytes > transfer_client_profile.max_outstanding)
      transfer_client_profile.max_outstanding = client_socket.nx_tcp_socket_tx_outstanding_bytes;
    if (count > transfer_client_profile.max_send) transfer_client_profile.max_send = count;
    if (status) { nx_packet_release(packet); return status; }
    position += count;
    length -= count;
  }
  return NX_SUCCESS;
}

static UINT read_bytes(uint8_t *data, ULONG count)
{
  while (count) {
    if (!rx_packet) {
      UINT status = nx_tcp_socket_receive(&client_socket, &rx_packet, TX_TIMER_TICKS_PER_SECOND * 120);
      if (status) return status;
      rx_offset = 0;
    }
    ULONG available = rx_packet->nx_packet_length - rx_offset;
    if (available > count) available = count;
    ULONG copied = 0;
    UINT status = nx_packet_data_extract_offset(rx_packet, rx_offset, data, available, &copied);
    if (status || copied != available) return NX_INVALID_PACKET;
    rx_offset += copied; data += copied; count -= copied;
    if (rx_offset == rx_packet->nx_packet_length) { nx_packet_release(rx_packet); rx_packet = NULL; }
  }
  return NX_SUCCESS;
}

static void pattern_fill(uint8_t *buffer, uint32_t count, uint64_t offset)
{
  for (uint32_t i = 0; i < count; i++) {
    uint32_t j = (uint32_t)(offset + i) & 65535U;
    buffer[i] = (uint8_t)(j * 73U + (j >> 8) * 19U + 41U);
  }
}

static ULONG mqtt_length(uint8_t *output, ULONG value)
{
  ULONG count = 0;
  do {
    uint8_t byte = value % 128U; value /= 128U;
    if (value) byte |= 128U;
    output[count++] = byte;
  } while (value);
  return count;
}

static UINT mqtt_control(uint8_t type, const uint8_t *data, ULONG count)
{
  uint8_t header[5]; header[0] = type;
  ULONG length = mqtt_length(header + 1, count);
  UINT status = send_bytes(header, length + 1);
  return status ? status : send_bytes(data, count);
}

static UINT mqtt_publish_small(const char *suffix, const void *data, ULONG count, uint16_t id)
{
  uint8_t packet[192];
  char topic[48];
  int topic_length = snprintf(topic, sizeof(topic), "%s/%s", mqtt_root, suffix);
  ULONG remaining = 2U + topic_length + (id ? 2U : 0U) + count;
  packet[0] = id ? 0x32 : 0x30;
  ULONG offset = 1 + mqtt_length(packet + 1, remaining);
  if (offset + remaining > sizeof(packet)) return NX_INVALID_PARAMETERS;
  packet[offset++] = (uint8_t)(topic_length >> 8); packet[offset++] = (uint8_t)topic_length;
  memcpy(packet + offset, topic, topic_length); offset += topic_length;
  if (id) { packet[offset++] = (uint8_t)(id >> 8); packet[offset++] = (uint8_t)id; }
  memcpy(packet + offset, data, count);
  return send_bytes(packet, offset + count);
}

static UINT mqtt_event(uint8_t *type, uint16_t *id)
{
  uint8_t control, byte, body[256];
  UINT status = read_bytes(&control, 1);
  if (status) return status;
  ULONG remaining = 0, multiplier = 1;
  for (uint32_t i = 0; i < 4; i++) {
    status = read_bytes(&byte, 1); if (status) return status;
    remaining += (byte & 127U) * multiplier;
    if (!(byte & 128U)) break;
    if (i == 3) return NX_INVALID_PACKET;
    multiplier *= 128U;
  }
  if (remaining > sizeof(body)) return NX_INVALID_PACKET;
  status = read_bytes(body, remaining); if (status) return status;
  *type = control >> 4; *id = 0;
  if (*type == 2) return remaining == 2 && body[1] == 0 ? NX_SUCCESS : NX_NOT_SUCCESSFUL;
  if (*type == 4 || *type == 9) {
    if (remaining < 2) return NX_INVALID_PACKET;
    *id = (body[0] << 8) | body[1];
    if (*type == 9 && (remaining != 3 || body[2] > 1)) return NX_NOT_SUCCESSFUL;
  } else if (*type == 3) {
    if (remaining < 2) return NX_INVALID_PACKET;
    ULONG topic_length = ((ULONG)body[0] << 8) | body[1];
    ULONG offset = 2 + topic_length;
    if (offset > remaining || topic_length != strlen(mqtt_ack_topic) || memcmp(body + 2, mqtt_ack_topic, topic_length)) return NX_INVALID_PACKET;
    if ((control & 6U) == 2U) {
      if (offset + 2 > remaining) return NX_INVALID_PACKET;
      *id = ((uint16_t)body[offset] << 8) | body[offset + 1]; offset += 2;
    } else if (control & 6U) return NX_INVALID_PACKET;
    if (remaining - offset != 64 || !transfer_ack_armed || memcmp(body + offset, transfer_expected_ack, 64)) return NX_NOT_SUCCESSFUL;
    transfer_ack_received = 1; transfer_ack_armed = 0;
    transfer_client_profile.ack_tick = tx_time_get();
    mqtt_ack_id = *id;
    if (*id) {
      uint8_t puback[2] = { (uint8_t)(*id >> 8), (uint8_t)*id };
      status = mqtt_control(0x40, puback, 2);
    }
  } else return NX_INVALID_PACKET;
  return status;
}

static UINT mqtt_wait(uint8_t wanted, uint16_t packet_id)
{
  for (;;) {
    uint8_t type; uint16_t id;
    UINT status = mqtt_event(&type, &id); if (status) return status;
    if (type == wanted && (!packet_id || id == packet_id)) return NX_SUCCESS;
    if (type == 3 && wanted == 4) continue; /* application ACK can precede finish PUBACK */
    return NX_INVALID_PACKET;
  }
}

static UINT mqtt_begin(const uint8_t header[64])
{
  uint64_t run; memcpy(&run, transfer_expected_ack + 4, 8);
  snprintf(mqtt_root, sizeof(mqtt_root), "bench/%016llx", (unsigned long long)run);
  snprintf(mqtt_ack_topic, sizeof(mqtt_ack_topic), "%s/ack", mqtt_root);
  char identifier[24]; snprintf(identifier, sizeof(identifier), "dev-%016llx", (unsigned long long)run);
  uint8_t connect[64] = {0,4,'M','Q','T','T',4,2,0,60};
  ULONG length = strlen(identifier);
  connect[10] = 0; connect[11] = length; memcpy(connect + 12, identifier, length);
  UINT status = mqtt_control(0x10, connect, 12 + length);
  if (status) return status;
  status = mqtt_wait(2,0); if (status) return status;
  uint8_t subscribe[64] = {0,1};
  length = strlen(mqtt_ack_topic);
  subscribe[2] = 0; subscribe[3] = length;
  memcpy(subscribe + 4, mqtt_ack_topic, length); subscribe[length + 4] = 1;
  status = mqtt_control(0x82, subscribe, length + 5); if (status) return status;
  status = mqtt_wait(9,1); if (status) return status;
  status = mqtt_publish_small("start", header, 64, 2); if (status) return status;
  return mqtt_wait(4,2);
}

static UINT mqtt_stream(FX_MEDIA *media, uint8_t *allocation, uint32_t chunk)
{
  uint8_t *buffer = allocation + 192;
  UINT status = NX_SUCCESS;
  uint64_t remaining = transfer_file_size, index = 0;
  uint16_t ids[4]; uint32_t sent_ticks[4], pending = 0;
  uint32_t window = transfer_mqtt_window;
  if (window < 1 || window > 4) return NX_INVALID_PARAMETERS;
  if (transfer_source_mode == 0) {
    status = fx_file_open(media, &source_file, transfer_file_path, FX_OPEN_FOR_READ);
    if (status) return status;
    if (source_file.fx_file_current_file_size != transfer_file_size) { fx_file_close(&source_file); return FX_INVALID_OPTION; }
  }
  while (remaining || pending) {
    while (remaining && (!pending || pending < window)) {
      ULONG count = remaining > chunk ? chunk : (ULONG)remaining;
      uint64_t offset = transfer_file_size - remaining;
      if (transfer_source_mode == 1) pattern_fill(buffer, count, offset);
      else {
        ULONG actual; uint32_t start = tx_time_get();
        status = fx_file_read(&source_file, buffer, count, &actual);
        transfer_client_profile.read_ticks += tx_time_get() - start;
        transfer_client_profile.read_calls++;
        if (!status && actual != count) status = FX_IO_ERROR;
      }
      if (status) goto end;
      uint8_t metadata[44]; memcpy(metadata,"MCH1",4);
      memcpy(metadata+4,transfer_expected_ack+4,16);
      memcpy(metadata+20,&index,8); memcpy(metadata+28,&offset,8);
      memcpy(metadata+36,&count,4);
      uint32_t start = tx_time_get();
      uint32_t crc = crc_update(UINT32_MAX,buffer,count)^UINT32_MAX;
      transfer_client_profile.crc_ticks += tx_time_get() - start;
      memcpy(metadata+40,&crc,4);
      char topic[40]; int topic_length=snprintf(topic,sizeof(topic),"%s/data",mqtt_root);
      uint16_t packet_id = transfer_protocol == 4 ? (uint16_t)(3U + index % 65530U) : 0;
      ULONG body_length = 2U + topic_length + (packet_id ? 2U : 0U) + 44U + count;
      uint8_t prefix[128]; prefix[0]=packet_id ? 0x32 : 0x30;
      ULONG head_length=1+mqtt_length(prefix+1,body_length);
      prefix[head_length++]=(uint8_t)(topic_length>>8);prefix[head_length++]=(uint8_t)topic_length;
      memcpy(prefix+head_length,topic,topic_length);head_length+=topic_length;
      if(packet_id) {prefix[head_length++]=(uint8_t)(packet_id>>8);prefix[head_length++]=(uint8_t)packet_id;}
      memcpy(prefix+head_length,metadata,44);head_length+=44;
      memcpy(buffer-head_length,prefix,head_length);
      status=send_bytes(buffer-head_length,head_length+count);
      if(status) goto end;
      if(packet_id) {ids[pending]=packet_id;sent_ticks[pending]=tx_time_get();pending++;}
      remaining-=count;index++;
      transfer_client_profile.bytes+=count;transfer_client_profile.mqtt_chunks++;
    }
    if(pending) {
      uint8_t type;uint16_t id;
      status=mqtt_event(&type,&id);if(status)goto end;
      if(type!=4){status=NX_INVALID_PACKET;goto end;}
      uint32_t found=0;while(found<pending&&ids[found]!=id)found++;
      if(found==pending){status=NX_INVALID_PACKET;goto end;}
      uint32_t latency=tx_time_get()-sent_ticks[found];
      transfer_client_profile.mqtt_pubacks++;transfer_client_profile.mqtt_puback_ticks+=latency;
      if(latency>transfer_client_profile.mqtt_puback_max_ticks)transfer_client_profile.mqtt_puback_max_ticks=latency;
      for(uint32_t i=found+1;i<pending;i++){ids[i-1]=ids[i];sent_ticks[i-1]=sent_ticks[i];}
      pending--;
    }
  }
end:
  if(transfer_source_mode==0)fx_file_close(&source_file);
  return status;
}

static UINT file_work(FX_MEDIA *media, uint8_t *buffer, uint32_t chunk)
{
  UINT status;
  uint64_t remaining = transfer_file_size;
  uint32_t crc = UINT32_MAX;
  if (transfer_source_mode == 2) {
    /* Create only private synthetic benchmark files; never overwrite data. */
    if (strncmp(transfer_file_path, "/BENCH/T", 8) || strstr(transfer_file_path, "..")) return FX_INVALID_NAME;
    status = fx_file_create(media, transfer_file_path);
    if (status != FX_SUCCESS) return status;
    status = fx_file_open(media, &source_file, transfer_file_path, FX_OPEN_FOR_WRITE);
  } else status = fx_file_open(media, &source_file, transfer_file_path, FX_OPEN_FOR_READ);
  if (status) return status;
  if (transfer_source_mode != 2 && source_file.fx_file_current_file_size != transfer_file_size) {
    fx_file_close(&source_file); return FX_INVALID_OPTION;
  }
  while (remaining) {
    ULONG count = remaining > chunk ? chunk : (ULONG)remaining;
    if (transfer_source_mode == 2) {
      pattern_fill(buffer, count, transfer_file_size - remaining);
      status = fx_file_write(&source_file, buffer, count);
    } else {
      ULONG actual;
      uint32_t start = tx_time_get();
      status = fx_file_read(&source_file, buffer, count, &actual);
      transfer_client_profile.read_ticks += tx_time_get() - start;
      transfer_client_profile.read_calls++;
      if (!status && actual != count) status = FX_IO_ERROR;
    }
    if (status) break;
    /* CRC measured separately, outside SD-read timing. */
    uint32_t start = tx_time_get();
    crc = crc_update(crc, buffer, count);
    transfer_client_profile.crc_ticks += tx_time_get() - start;
    transfer_client_profile.bytes += count;
    remaining -= count;
  }
  fx_file_close(&source_file);
  if (!status && transfer_source_mode == 2) status = fx_media_flush(media);
  transfer_client_profile.crc32 = crc ^ UINT32_MAX;
  return status;
}

static UINT stream_work(FX_MEDIA *media, uint8_t *buffer, uint32_t chunk)
{
  UINT status = NX_SUCCESS;
  uint64_t remaining = transfer_file_size - transfer_resume_offset;
  if (transfer_source_mode == 0) {
    status = fx_file_open(media, &source_file, transfer_file_path, FX_OPEN_FOR_READ);
    if (status) return status;
    if (source_file.fx_file_current_file_size != transfer_file_size) { fx_file_close(&source_file); return FX_INVALID_OPTION; }
    if (transfer_resume_offset) {
      status = fx_file_seek(&source_file, (ULONG)transfer_resume_offset);
      if (status) { fx_file_close(&source_file); return status; }
    }
  }
  while (remaining) {
    ULONG count = remaining > chunk ? chunk : (ULONG)remaining;
    if (transfer_source_mode == 1) pattern_fill(buffer, count, transfer_file_size - remaining);
    else {
      ULONG actual;
      uint32_t start = tx_time_get();
      status = fx_file_read(&source_file, buffer, count, &actual);
      transfer_client_profile.read_ticks += tx_time_get() - start;
      transfer_client_profile.read_calls++;
      if (!status && actual != count) status = FX_IO_ERROR;
    }
    if (status) break;
    status = send_bytes(buffer, count);
    if (status) break;
    transfer_client_profile.bytes += count;
    remaining -= count;
  }
  if (transfer_source_mode == 0) fx_file_close(&source_file);
  return status;
}

UINT transfer_bench_execute(uint32_t command, NX_IP *ip, FX_MEDIA *media)
{
  if (command == 11) {
    if (app_model.log_controller_model.status || record_requested) return NX_NOT_SUCCESSFUL;
    transfer_record_start = transfer_record_stop = 0;
    record_requested = 1;
    return DatalogAppTask_msg(DT_USER_BUTTON);
  }
  if (command == 12) {
    record_requested = 0;
    if (app_model.log_controller_model.status) return DatalogAppTask_msg(DT_USER_BUTTON);
    return NX_SUCCESS;
  }
  if (app_model.log_controller_model.status) return NX_NOT_SUCCESSFUL;
  if (command == 13) {
    /* Scan proxy for the configured SSID; not an association RSSI getter. */
    mwifi_ap_info_t aps[10];
    transfer_scan_matches = 0;
    transfer_scan_rssi = -128;
    transfer_scan_status = MX_WIFI_Scan(wifi_obj_get(), MC_SCAN_ACTIVE, wifi_ssid, strlen(wifi_ssid));
    if (transfer_scan_status) return (UINT)transfer_scan_status;
    int32_t count = MX_WIFI_Get_scan_result(wifi_obj_get(),(uint8_t *)aps,10);
    if (count < 0) return NX_NOT_SUCCESSFUL;
    for (int32_t i=0;i<count;i++) {
      if (!strncmp(aps[i].ssid,wifi_ssid,sizeof(aps[i].ssid))) {
        transfer_scan_matches++;
        if (aps[i].rssi > transfer_scan_rssi) {
          transfer_scan_rssi = aps[i].rssi;
          transfer_scan_channel = aps[i].channel;
        }
      }
    }
    return NX_SUCCESS;
  }
  if (transfer_protocol > 5) return NX_INVALID_PARAMETERS;
  uint32_t chunk = transfer_chunk_size;
  if (chunk < 4096 || chunk > 65536 || (chunk & (chunk - 1U))) return NX_INVALID_PARAMETERS;
  uint32_t mqtt = transfer_protocol == 3 || transfer_protocol == 4;
  uint32_t allocation_size = chunk + (mqtt ? 192U : 0U);
  uint8_t *buffer = SysAlloc(allocation_size);
  if (!buffer) return NX_NO_PACKET;
  memset((void *)&transfer_client_profile, 0, sizeof(transfer_client_profile));
  memset((void *)&transfer_wifi_profile, 0, sizeof(transfer_wifi_profile));
  transfer_client_profile.min_packets = UINT32_MAX;
  transfer_client_profile.min_peer_window = UINT32_MAX;
  transfer_client_profile.min_congestion_window = UINT32_MAX;
  transfer_client_profile.ram_bytes = allocation_size;
  transfer_client_profile.start_tick = tx_time_get();
  transfer_cpu_samples = transfer_cpu_thread_samples = 0;
  memset((void *)transfer_cleanup_profile,0,sizeof(transfer_cleanup_profile));
  transfer_cpu_active = 1;
  transfer_interrupt_fired = transfer_interrupt_tick = transfer_interrupt_status = 0;
  transfer_resume_offset = 0;
  UINT status;
  if (!transfer_protocol) { status = file_work(media, buffer, chunk); goto finished; }
  uint32_t pool_bytes = transfer_client_pool_bytes;
  if (pool_bytes < 12288 || pool_bytes > 65536) { status = NX_INVALID_PARAMETERS; goto finished; }
  uint8_t *pool_memory = SysAlloc(pool_bytes);
  if (!pool_memory) { status = NX_NO_PACKET; goto finished; }
  transfer_client_profile.ram_bytes += pool_bytes;
  status = nx_packet_pool_create(&client_pool, "bench packet pool", 1544, pool_memory, pool_bytes);
  if (status) goto release_pool;
  transfer_client_profile.pool_packets = client_pool.nx_packet_pool_total;
  uint32_t created = 0, bound = 0;
  status = nx_tcp_socket_create(ip, &client_socket, "bench socket", NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 8192, NULL, NULL);
  if (status) goto delete_pool;
  created = 1;
  status = nx_tcp_client_socket_bind(&client_socket, NX_ANY_PORT, TX_TIMER_TICKS_PER_SECOND * 10);
  if (status) goto socket_cleanup;
  bound = 1;
  status = nx_tcp_client_socket_connect(&client_socket, transfer_server_ip, transfer_server_port, TX_TIMER_TICKS_PER_SECOND * 30);
  if (status) goto socket_cleanup;
  rx_packet = NULL;
  uint8_t header[64];
  memcpy(header, transfer_expected_ack, 64);
  memcpy(header, transfer_protocol == 5 ? "BTR1" : "BTF1", 4);
  uint32_t crc = crc_update(UINT32_MAX, header, 60) ^ UINT32_MAX;
  memcpy(header + 60, &crc, 4);
  if (transfer_protocol == 2) {
    char http[256];
    int length = snprintf(http, sizeof(http), "POST /bench HTTP/1.1\r\nHost: receiver\r\nContent-Length: %llu\r\nConnection: close\r\nContent-Type: application/octet-stream\r\n\r\n", (unsigned long long)transfer_file_size + 64ULL);
    if (length <= 0 || length >= (int)sizeof(http)) { status = NX_INVALID_PARAMETERS; goto socket_cleanup; }
    status = send_bytes(http, length);
    if (status) goto socket_cleanup;
  }
  status = mqtt ? mqtt_begin(header) : send_bytes(header, 64);
  if (status) goto socket_cleanup;
  if (transfer_protocol == 5) {
    uint8_t checkpoint[16];
    status = read_bytes(checkpoint, sizeof(checkpoint));
    if (status) goto socket_cleanup;
    uint32_t check; memcpy(&check, checkpoint + 12, 4);
    if (memcmp(checkpoint,"BRS1",4) || (crc_update(UINT32_MAX,checkpoint,12)^UINT32_MAX) != check) { status = NX_INVALID_PACKET; goto socket_cleanup; }
    memcpy((void *)&transfer_resume_offset,checkpoint+4,8);
    if (transfer_resume_offset > transfer_file_size || transfer_resume_offset > UINT32_MAX) { status=NX_INVALID_PARAMETERS;goto socket_cleanup; }
    transfer_client_profile.bytes = transfer_resume_offset;
  }
  transfer_client_profile.connect_ticks = tx_time_get() - transfer_client_profile.start_tick;
  transfer_client_profile.payload_start_tick = tx_time_get();
  status = mqtt ? mqtt_stream(media, buffer, chunk) : stream_work(media, buffer, chunk);
  transfer_client_profile.payload_finish_tick = tx_time_get();
  if (status) goto socket_cleanup;
  if (mqtt) {
    status = mqtt_publish_small("finish", "MEND", 4, 65534);
    if (status) goto socket_cleanup;
    status = mqtt_wait(4,65534);
    if (status) goto socket_cleanup;
    if (!transfer_ack_received) { status = mqtt_wait(3,0); if (status) goto socket_cleanup; }
    status = mqtt_publish_small("done", "BDON", 4, 65535);
    if (!status) status = mqtt_wait(4,65535);
    goto socket_cleanup;
  }
  if (transfer_protocol == 2) {
    /* Parse a bounded HTTP response header; require an actual 200 response. */
    char response[256]; uint32_t count = 0;
    do {
      if (count == sizeof(response) - 1) { status = NX_INVALID_PARAMETERS; goto socket_cleanup; }
      status = read_bytes((uint8_t *)&response[count++], 1);
      if (status) goto socket_cleanup;
    } while (count < 4 || memcmp(response + count - 4, "\r\n\r\n", 4));
    response[count] = 0;
    if (strncmp(response, "HTTP/1.1 200 ", 13)) { status = NX_NOT_SUCCESSFUL; goto socket_cleanup; }
  }
  uint8_t acknowledgement[64];
  status = read_bytes(acknowledgement, 64);
  if (status) goto socket_cleanup;
  if (!transfer_ack_armed || memcmp(acknowledgement, transfer_expected_ack, 64)) { status = NX_NOT_SUCCESSFUL; goto socket_cleanup; }
  transfer_ack_received = 1;
  transfer_ack_armed = 0;
  transfer_client_profile.ack_tick = tx_time_get();
  /* Echo only after accepting the durable receipt: receiver's timer ends here. */
  status = send_bytes("BDON", 4);
socket_cleanup:
  transfer_client_profile.retransmits = client_socket.nx_tcp_socket_retransmit_packets;
  transfer_client_profile.pool_empty = client_pool.nx_packet_pool_empty_requests;
  if (rx_packet) { nx_packet_release(rx_packet); rx_packet = NULL; }
  if (created) transfer_cleanup_profile[0]=nx_tcp_socket_disconnect(&client_socket, TX_TIMER_TICKS_PER_SECOND * 3);
  if (bound) transfer_cleanup_profile[1]=nx_tcp_client_socket_unbind(&client_socket);
  if (created) transfer_cleanup_profile[2]=nx_tcp_socket_delete(&client_socket);
delete_pool:
  transfer_cleanup_profile[4]=client_pool.nx_packet_pool_available;
  transfer_cleanup_profile[5]=client_pool.nx_packet_pool_total;
  transfer_cleanup_profile[3]=nx_packet_pool_delete(&client_pool);
release_pool:
  SysFree(pool_memory);
finished:
  transfer_cpu_active = 0;
  transfer_client_profile.finish_tick = tx_time_get();
  transfer_client_profile.status = status;
  SysFree(buffer);
  return status;
}
