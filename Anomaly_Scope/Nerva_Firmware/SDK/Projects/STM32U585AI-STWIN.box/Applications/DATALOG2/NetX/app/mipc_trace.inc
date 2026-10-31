/* No printf, no file access and no dynamic allocation in the hot path.
 * Included after the benchmark's socket/pool declarations. IRQ-masked writes
 * make the record publication atomic across SPI ISR and ThreadX contexts. */
#define MT_RING 512U
#define MT_BINS 2048U
#define MT_METRICS 11U /* nested software intervals; never sum as exclusive duty */
typedef struct {
  uint32_t cycles, tick, ordinal, id, event_api, bytes, slot_depth, gpio;
  uint32_t spi, tx_queue, pool_free, status, arg0, arg1, pointer, thread;
} mt_record;
typedef struct { uint32_t id, cycles, api, bytes; } mt_pending;
typedef struct {
  uint32_t magic, version, size, active, frozen, freeze_reason, freeze_id;
  uint32_t events, overwritten, submissions, responses, malformed;
  uint32_t explicit_minus1, unmatched, collisions, lost_tracking;
  uint32_t clock_hz, bin_us, metrics_count[MT_METRICS];
  uint32_t metrics_max[MT_METRICS];
  uint64_t metrics_sum[MT_METRICS];
  uint32_t histogram[MT_METRICS][MT_BINS];
  mt_pending pending[64];
  mt_record ring[MT_RING];
} mt_block;
mt_block *mipc_trace;
volatile uint32_t mipc_trace_bytes;
volatile uint32_t mipc_trace_gpio_enable;
static uint32_t mt_flow_start, mt_dma_end, mt_dma_valid, mt_irq_start, mt_irq_pending;
static uint32_t mt_netx_start, mt_alloc_start, mt_hci_start, mt_response_start;
static uint32_t mt_wait_start, mt_link_start, mt_dma_start;

static void mt_hist(uint32_t metric,uint32_t elapsed)
{
  uint32_t unit=SystemCoreClock/100000U; /* exact 10 us bins at current clock */
  uint32_t bin=unit ? elapsed/unit : MT_BINS-1;
  if (bin>=MT_BINS) bin=MT_BINS-1;
  mipc_trace->histogram[metric][bin]++;
  mipc_trace->metrics_count[metric]++;
  mipc_trace->metrics_sum[metric]+=elapsed;
  if (elapsed>mipc_trace->metrics_max[metric]) mipc_trace->metrics_max[metric]=elapsed;
}

void mipc_trace_event(uint32_t event,uint32_t id,uint32_t api,uint32_t bytes,
                      uint32_t slot_depth,int32_t status,uint32_t arg0,
                      uint32_t arg1,const void *pointer)
{
  if (!mipc_trace || !mipc_trace->active || mipc_trace->frozen) return;
  uint32_t primask=__get_PRIMASK();__disable_irq();
  if (!mipc_trace->active || mipc_trace->frozen) { __set_PRIMASK(primask);return; }
  uint32_t now=transfer_cycles();
  if (event==MT_NETX_IN) mt_netx_start=now;
  if (event==MT_NETX_OUT) mt_hist(4,now-mt_netx_start);
  if (event==MT_ALLOC_IN) mt_alloc_start=now;
  if (event==MT_ALLOC_OUT) mt_hist(5,now-mt_alloc_start);
  if (event==MT_SUBMIT) mt_hci_start=now;
  if (event==MT_HCI_DONE) mt_hist(6,now-mt_hci_start);
  if (event==MT_RESPONSE_IN) mt_response_start=now;
  if (event==MT_RESPONSE_OUT) mt_hist(7,now-mt_response_start);
  if (event==MT_WAIT_IN) mt_wait_start=now;
  if (event==MT_WAIT_OUT) mt_hist(8,now-mt_wait_start);
  if (event==MT_LINK_IN) mt_link_start=now;
  if (event==MT_LINK_OUT) mt_hist(9,now-mt_link_start);
  if (event==MT_DMA_IN) mt_dma_start=now;
  if (event==MT_DMA_OUT) mt_hist(10,now-mt_dma_start);
  if (event==MT_UNMATCHED) mipc_trace->unmatched++;
  if (event==MT_FLOW_IN) mt_flow_start=now;
  if (event==MT_FLOW_OUT) mt_hist(1,(uint32_t)(now-mt_flow_start));
  if (event==MT_DMA_IN && mt_dma_valid) mt_hist(2,(uint32_t)(now-mt_dma_end));
  if (event==MT_DMA_OUT) { mt_dma_end=now;mt_dma_valid=1; }
  if (event==MT_IRQ && arg0==1) { if (!mt_irq_pending) mt_irq_start=now;mt_irq_pending=1; }
  if (event==MT_SPI_SERVICE && mt_irq_pending) { mt_hist(3,(uint32_t)(now-mt_irq_start));mt_irq_pending=0; }
  uint32_t ordinal=mipc_trace->events++;
  mt_record *r=&mipc_trace->ring[ordinal%MT_RING];
  TX_THREAD *thread=tx_thread_identify();
  r->cycles=now;r->tick=HAL_GetTick();r->ordinal=ordinal;r->id=id;
  r->event_api=(event&0xffffU)|((api&0xffffU)<<16);r->bytes=bytes;r->slot_depth=slot_depth;
  r->gpio=((GPIOG->IDR>>15)&1U)|(((GPIOE->IDR>>7)&1U)<<1)|(((GPIOH->ODR>>7)&1U)<<2)|(__get_IPSR()<<16);
  r->spi=hspi1.Instance ? hspi1.Instance->SR : 0;
  r->tx_queue=network_probe_socket ? network_probe_socket->nx_tcp_socket_transmit_sent_count : UINT32_MAX;
  r->pool_free=network_probe_socket ? packets.nx_packet_pool_available : UINT32_MAX;
  r->status=(uint32_t)status;r->arg0=arg0;r->arg1=arg1;r->pointer=(uint32_t)pointer;
  r->thread=thread ? thread->tx_thread_priority : UINT32_MAX;
  if (ordinal>=MT_RING) mipc_trace->overwritten++;
  if (event==MT_RX_DROP || event==MT_RX_ALLOC_FAIL || event==MT_RX_OVERSIZE) {
    /* Publish FIRST RX failure before the original driver release/drop. */
    mipc_trace->freeze_reason=100U+event;mipc_trace->freeze_id=id;
    mipc_trace->frozen=1;
  }
  __DMB();__set_PRIMASK(primask);
}

void mipc_trace_submit(uint32_t id,uint32_t api,uint32_t bytes,
                       uint32_t slot_depth,const void *pointer)
{
  if (!mipc_trace || !mipc_trace->active || mipc_trace->frozen) return;
  uint32_t primask=__get_PRIMASK();__disable_irq();
  mt_pending *p=&mipc_trace->pending[id%64];
  if (p->id) mipc_trace->collisions++;
  p->id=id;p->cycles=transfer_cycles();p->api=api;p->bytes=bytes;mipc_trace->submissions++;
  __set_PRIMASK(primask);
  if (mipc_trace_gpio_enable) GPIOF->BSRR=GPIO2_EX_Pin;
  mipc_trace_event(MT_SUBMIT,id,api,bytes,slot_depth,0,0,0,pointer);
}

void mipc_trace_response(uint32_t id,uint32_t api,uint32_t bytes,
                         uint32_t slot_depth,int32_t status,uint32_t valid,
                         const void *pointer)
{
  if (!mipc_trace || !mipc_trace->active || mipc_trace->frozen) return;
  if (mipc_trace_gpio_enable) GPIOF->BSRR=GPIO3_EX_Pin;
  uint32_t primask=__get_PRIMASK();__disable_irq();
  mt_pending *p=&mipc_trace->pending[id%64];
  uint32_t expected_api=p->id==id ? p->api : UINT32_MAX;
  if (p->id==id) { mt_hist(0,(uint32_t)(transfer_cycles()-p->cycles));p->id=0; }
  else mipc_trace->lost_tracking++;
  mipc_trace->responses++;
  if (!valid) mipc_trace->malformed++;
  if (valid && status==-1) mipc_trace->explicit_minus1++;
  __set_PRIMASK(primask);
  mipc_trace_event(MT_RESPONSE_OUT,id,api,bytes,slot_depth,status,valid,expected_api,pointer);
  if ((valid && status==-1) || !valid) {
    mipc_trace_event(MT_FREEZE,id,api,bytes,slot_depth,status,valid,expected_api,pointer);
    primask=__get_PRIMASK();__disable_irq();
    mipc_trace->freeze_id=id;mipc_trace->freeze_reason=valid ? 1U : 2U;
    mipc_trace->frozen=1;__DMB();__set_PRIMASK(primask);
  }
  if (mipc_trace_gpio_enable) GPIOF->BSRR=(uint32_t)GPIO3_EX_Pin<<16;
}

UINT mipc_trace_begin(void)
{
  if (!mipc_trace) mipc_trace=(mt_block *)SysAlloc(sizeof(mt_block));
  if (!mipc_trace) return TX_NO_MEMORY;
  memset(mipc_trace,0,sizeof(*mipc_trace));
  mipc_trace->magic=0x3152544dU;mipc_trace->version=2;mipc_trace->size=sizeof(*mipc_trace);
  mipc_trace->clock_hz=SystemCoreClock;mipc_trace->bin_us=10;mipc_trace_bytes=sizeof(*mipc_trace);
  mt_flow_start=mt_dma_end=mt_dma_valid=mt_irq_start=mt_irq_pending=0;
  if (mipc_trace_gpio_enable) {
    GPIO_InitTypeDef pin={0};__HAL_RCC_GPIOF_CLK_ENABLE();
    GPIOF->BSRR=((uint32_t)(GPIO2_EX_Pin|GPIO3_EX_Pin))<<16;
    pin.Pin=GPIO2_EX_Pin|GPIO3_EX_Pin;pin.Mode=GPIO_MODE_OUTPUT_PP;
    pin.Pull=GPIO_NOPULL;pin.Speed=GPIO_SPEED_FREQ_VERY_HIGH;HAL_GPIO_Init(GPIOF,&pin);
  }
  __DMB();mipc_trace->active=1;return TX_SUCCESS;
}
void mipc_trace_end(void)
{
  if (mipc_trace) { mipc_trace->active=0;__DMB(); }
}
UINT mipc_trace_release(void)
{
  if (mipc_trace && mipc_trace->active) return TX_NOT_DONE;
  if (mipc_trace) { SysFree(mipc_trace);mipc_trace=NULL;mipc_trace_bytes=0; }
  if (mipc_trace_gpio_enable) {
    GPIO_InitTypeDef pin={0};pin.Pin=GPIO2_EX_Pin|GPIO3_EX_Pin;
    pin.Mode=GPIO_MODE_ANALOG;pin.Pull=GPIO_NOPULL;HAL_GPIO_Init(GPIOF,&pin);
  }
  return TX_SUCCESS;
}
