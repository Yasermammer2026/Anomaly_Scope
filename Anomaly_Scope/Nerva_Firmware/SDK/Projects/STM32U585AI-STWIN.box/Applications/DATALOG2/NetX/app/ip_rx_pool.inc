/* Stopped-transfer experiment. Default 10 packets uses the original reserved
 * NetX allocator. Larger RX pools get independent SysAlloc storage while Wi-Fi
 * is connected and release it only through the original post-IP-delete paths.
 * Never resize a live pool or borrow a sensor/data buffer. */
volatile uint32_t stock_rx_packets=10,stock_rx_pool_bytes;
static UCHAR *stock_rx_owned_memory;
static uint32_t stock_rx_owned_heap;

static UINT stock_ip_packets_allocate(TX_BYTE_POOL *pool,UCHAR **memory)
{
  uint32_t count=stock_rx_packets;
  if(stock_rx_owned_memory || (count!=10 && count!=16 && count!=32 && count!=64))
    return TX_POOL_ERROR;
  stock_rx_pool_bytes=(PAYLOAD_SIZE+sizeof(NX_PACKET))*count;
  UINT status;
  if(count==10) {
    status=tx_byte_allocate(pool,(VOID **)memory,stock_rx_pool_bytes,TX_NO_WAIT);
    stock_rx_owned_heap=0;
  } else {
    *memory=SysAlloc(stock_rx_pool_bytes);
    status=*memory?TX_SUCCESS:TX_NO_MEMORY;stock_rx_owned_heap=1;
  }
  if(status==TX_SUCCESS)stock_rx_owned_memory=*memory;
  return status;
}

static UINT stock_ip_packets_release(UCHAR *memory)
{
  if(!memory || memory!=stock_rx_owned_memory)return TX_PTR_ERROR;
  /* Both public delete operations clear their IDs on success. Preserve all
   * backing memory if the IP/pool is still live; reset is safer than reuse. */
  if(ip_instance.nx_ip_id || ip_pool.nx_packet_pool_id)return TX_NOT_DONE;
  UINT status=TX_SUCCESS;
  if(stock_rx_owned_heap)SysFree(memory);else status=tx_byte_release(memory);
  if(status==TX_SUCCESS){stock_rx_owned_memory=NULL;stock_rx_owned_heap=0;}
  return status;
}
