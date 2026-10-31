/**
  ******************************************************************************
  * @file    mx_wifi_ipc.c
  * @author  MCD Application Team
  * @brief   Host driver IPC protocol of MXCHIP Wi-Fi component.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "mx_wifi.h"
#include "throughput_client.h"
#include "mx_wifi_ipc.h"
#include "mx_wifi_hci.h"
#include "io_pattern/mx_wifi_io.h"


#ifdef MX_WIFI_IPC_DEBUG
#define DEBUG_LOG(...)       (void)printf(__VA_ARGS__) /*;*/
#else
#define DEBUG_LOG(...)
#endif /* MX_WIFI_IPC_DEBUG */

#define DEBUG_ERROR(...)     (void)printf(__VA_ARGS__) /*;*/


/**
  * @brief IPC API event handlers
  */
typedef void (*event_callback_t)(mx_buf_t *mx_buff);

typedef struct _event_item_s
{
  uint16_t api_id;
  event_callback_t callback;
} event_item_t;


/**
  * @brief IPC API request list
  */
typedef struct _mipc_req_s
{
  uint32_t req_id;
  SEM_DECLARE(resp_flag);
  uint16_t *rbuffer_size; /* in/out */
  uint8_t *rbuffer;
} mipc_req_t;

#define MIPC_REQ_ID_RESET_VAL  ((uint32_t)(0xFFFFFFFF))


static mipc_req_t PendingRequest;

/* Bounded experimental bypass pipeline. Each command owns its payload until
 * the matching module response, so returning to NetX cannot release DMA data.
 * Disabled by default. Control commands always drain it before using the
 * original single PendingRequest. Late errors fail the complete benchmark. */
#define ASYNC_SLOTS 4U
typedef struct {
  uint8_t command[MIPC_PKT_MAX_SIZE];
  volatile uint32_t id, state; /* 0 free, 1 submitted, 2 response received */
  volatile int32_t status;
} async_slot_t;
static async_slot_t AsyncSlots[ASYNC_SLOTS];
static SEM_DECLARE(AsyncResponseSem);
volatile uint32_t network_async_depth, network_async_submitted, network_async_completed;
volatile uint32_t network_async_active;
volatile uint32_t network_async_errors, network_async_max_pending;
volatile uint64_t network_async_wait_cycles;
static uint32_t AsyncFailed;
static uint32_t trace_slot_depth(uint32_t slot)
{
  uint32_t outstanding=0,held=0;
  for (uint32_t i=0;i<ASYNC_SLOTS;++i) {
    outstanding += AsyncSlots[i].state==1;held += AsyncSlots[i].state!=0;
  }
  return (slot&255U)|((slot<ASYNC_SLOTS ? AsyncSlots[slot].state : 1U)<<8)
         |(outstanding<<16)|(held<<24);
}
volatile int32_t network_async_last_status __attribute__((section(".transfer_bss")));
volatile uint32_t network_async_last_id __attribute__((section(".transfer_bss")));
volatile uint32_t network_async_late_errors __attribute__((section(".transfer_bss")));
volatile uint32_t network_async_timeout_errors __attribute__((section(".transfer_bss")));
volatile uint32_t network_async_hci_errors __attribute__((section(".transfer_bss")));

static uint32_t async_reap(void)
{
  uint32_t busy = 0;
  for (uint32_t i=0; i<ASYNC_SLOTS; ++i) {
    if (AsyncSlots[i].state == 2) {
      mipc_trace_event(MT_COMPLETE,AsyncSlots[i].id,MIPC_API_WIFI_BYPASS_OUT_CMD,0,
                       trace_slot_depth(i),AsyncSlots[i].status,0,0,AsyncSlots[i].command);
      network_async_completed++;
      if (AsyncSlots[i].status != MIPC_CODE_SUCCESS) {
        network_async_errors++; network_async_late_errors++;
        network_async_last_status=AsyncSlots[i].status; network_async_last_id=AsyncSlots[i].id;
        AsyncFailed=1;
      }
      AsyncSlots[i].state=0;
    }
    if (AsyncSlots[i].state) busy++;
  }
  if (busy > network_async_max_pending) network_async_max_pending=busy;
  return busy;
}

/* Caller holds the command mutex; RX never takes it. */
static int32_t async_wait_below(uint32_t limit, uint32_t timeout)
{
  uint32_t start = tx_time_get(), cycles = transfer_cycles();
  mipc_trace_event(MT_WAIT_IN,0,MIPC_API_WIFI_BYPASS_OUT_CMD,0,trace_slot_depth(0),0,limit,timeout,NULL);
  while (async_reap() >= limit && !AsyncFailed) {
    uint32_t elapsed = tx_time_get() - start;
    if (elapsed >= timeout || SEM_WAIT(AsyncResponseSem, timeout-elapsed, NULL) != SEM_OK) {
      network_async_errors++; network_async_timeout_errors++; AsyncFailed=1; break;
    }
  }
  network_async_wait_cycles += (uint32_t)(transfer_cycles()-cycles);
  mipc_trace_event(MT_WAIT_OUT,0,MIPC_API_WIFI_BYPASS_OUT_CMD,0,trace_slot_depth(0),
                   AsyncFailed?MIPC_CODE_ERROR:0,limit,timeout,NULL);
  return AsyncFailed ? MIPC_CODE_ERROR : MIPC_CODE_SUCCESS;
}

int32_t mipc_async_drain(uint32_t timeout)
{
  LOCK(wifi_obj_get()->lockcmd);
  int32_t result=async_wait_below(1,timeout);
  UNLOCK(wifi_obj_get()->lockcmd);
  return result;
}

int32_t mipc_async_reset_idle(void)
{
  LOCK(wifi_obj_get()->lockcmd);
  /* A matching response is the only evidence that its owned buffer is free.
   * A timed-out command remains busy and requires driver reinitialization. */
  uint32_t busy=async_reap();
  int32_t result=MIPC_CODE_ERROR;
  if (!network_async_active && !busy) {
    AsyncFailed=0;
    network_async_last_status=0; network_async_last_id=0;
    network_async_late_errors=network_async_timeout_errors=network_async_hci_errors=0;
    result=MIPC_CODE_SUCCESS;
  }
  UNLOCK(wifi_obj_get()->lockcmd);
  return result;
}

static uint8_t *byte_pointer_add_signed_offset(uint8_t *BytePointer, int32_t Offset);
static uint32_t get_new_req_id(void);
static uint32_t mpic_get_req_id(const uint8_t Buffer[]);
static uint16_t mpic_get_api_id(const uint8_t Buffer[]);
static void mipc_event(mx_buf_t *netbuf);


static uint8_t *byte_pointer_add_signed_offset(uint8_t *BytePointer, int32_t Offset)
{
  return BytePointer + Offset;
}


/* unique sequence number */
static uint32_t get_new_req_id(void)
{
  static uint32_t id = 1;
  return id++;
}


static uint32_t mpic_get_req_id(const uint8_t Buffer[])
{
  return *((const uint32_t *) &Buffer[MIPC_PKT_REQ_ID_OFFSET]);
}


static uint16_t mpic_get_api_id(const uint8_t Buffer[])
{
  return *((const uint16_t *) &Buffer[MIPC_PKT_API_ID_OFFSET]);
}


static void mipc_event(mx_buf_t *netbuf)
{
  static const event_item_t event_table[] =
  {
    /* System */
    {MIPC_API_SYS_REBOOT_EVENT,         mapi_reboot_event_callback},
    {MIPC_API_SYS_FOTA_STATUS_EVENT,    mapi_fota_status_event_callback},

    /* WiFi */
    {MIPC_API_WIFI_STATUS_EVENT,        mapi_wifi_status_event_callback},
    {MIPC_API_WIFI_BYPASS_INPUT_EVENT,  mapi_wifi_netlink_input_callback}
  };

  if (NULL != netbuf)
  {
    uint8_t *const buffer_in = MX_NET_BUFFER_PAYLOAD(netbuf);
    const uint32_t buffer_in_size = MX_NET_BUFFER_GET_PAYLOAD_SIZE(netbuf);

    if ((NULL != buffer_in) && (buffer_in_size >= MIPC_PKT_MIN_SIZE))
    {
      const uint32_t req_id = mpic_get_req_id(buffer_in);
      const uint16_t api_id = mpic_get_api_id(buffer_in);
      mipc_trace_event(MT_RESPONSE_IN,req_id,api_id,buffer_in_size,0,0,0,0,buffer_in);

      DEBUG_LOG("%-15s(): req_id: 0x%08" PRIx32 ", api_id: 0x%04" PRIx32 "\n",
                __FUNCTION__, req_id, (uint32_t)api_id);

      if ((0 == (api_id & MIPC_API_EVENT_BASE)) && (MIPC_REQ_ID_NONE != req_id))
      {
        for (uint32_t i=0; i<ASYNC_SLOTS; ++i) {
          if (AsyncSlots[i].state == 1 && AsyncSlots[i].id == req_id) {
            int32_t status=MIPC_CODE_ERROR;
            uint32_t valid=(api_id == MIPC_API_WIFI_BYPASS_OUT_CMD && buffer_in_size == MIPC_PKT_MIN_SIZE+sizeof(status));
            if (valid)
              memcpy(&status, buffer_in+MIPC_PKT_PARAMS_OFFSET, sizeof(status));
            mipc_trace_response(req_id,api_id,buffer_in_size,trace_slot_depth(i),status,valid,buffer_in);
            AsyncSlots[i].status=status;
            AsyncSlots[i].state=2;
            SEM_SIGNAL(AsyncResponseSem);
            mx_wifi_hci_free(netbuf);
            return;
          }
        }
        /* The command response must match pending req id. */
        if (PendingRequest.req_id == req_id)
        {
          int32_t trace_status=0;
          uint32_t trace_valid=buffer_in_size>=MIPC_PKT_MIN_SIZE;
          if (api_id==MIPC_API_WIFI_BYPASS_OUT_CMD) {
            trace_valid=buffer_in_size==MIPC_PKT_MIN_SIZE+sizeof(trace_status);
            if (trace_valid) memcpy(&trace_status,buffer_in+MIPC_PKT_PARAMS_OFFSET,sizeof(trace_status));
          }
          mipc_trace_response(req_id,api_id,buffer_in_size,255U|(1U<<8)|(1U<<16),trace_status,trace_valid,buffer_in);
          /* return params */
          if ((PendingRequest.rbuffer_size != NULL) && (*PendingRequest.rbuffer_size > 0) &&
              (NULL != PendingRequest.rbuffer))
          {
            *(PendingRequest.rbuffer_size) = *PendingRequest.rbuffer_size < (buffer_in_size - MIPC_PKT_MIN_SIZE) ? \
                                             *PendingRequest.rbuffer_size : (uint16_t)(buffer_in_size - MIPC_PKT_MIN_SIZE);
            (void)memcpy(PendingRequest.rbuffer, byte_pointer_add_signed_offset(buffer_in, MIPC_PKT_PARAMS_OFFSET),
                         *PendingRequest.rbuffer_size);
          }
          /* printf("Signal for %d\n",pending_request.req_id); */
          PendingRequest.req_id = MIPC_REQ_ID_RESET_VAL;
          if (SEM_OK != SEM_SIGNAL(PendingRequest.resp_flag))
          {
            DEBUG_ERROR("Failed to signal command response\n");
            MX_ASSERT(false);
          }
          MX_STAT(cmd_get_answer);
        }
        else
        {
          mipc_trace_event(MT_UNMATCHED,req_id,api_id,buffer_in_size,0,0,PendingRequest.req_id,0,buffer_in);
          DEBUG_LOG("response req_id: 0x%08"PRIx32" not match pending req_id: 0x%08" PRIx32 "!\n",
                    req_id, PendingRequest.req_id);
        }
        mx_wifi_hci_free(netbuf);
      }
      else /* event callback */
      {
        const uint32_t event_table_count = sizeof(event_table) / sizeof(event_table[0]);
        uint32_t i;

        for (i = 0; i < event_table_count; i++)
        {
          if (event_table[i].api_id == api_id)
          {
            const event_callback_t callback = event_table[i].callback;
            if (NULL != callback)
            {
              /* DEBUG_LOG("callback with %p\n", buffer_in); */
              callback(netbuf);
              break;
            }
          }
        }
        if (i == event_table_count)
        {
          DEBUG_ERROR("Unknown event: 0x%04" PRIx32 "!\n", (uint32_t)api_id);
          mx_wifi_hci_free(netbuf);
        }
      }
    }
    else
    {
      DEBUG_LOG("Unknown buffer content\n");
      mx_wifi_hci_free(netbuf);
    }
  }
}


/*******************************************************************************
  * IPC API implementations for mx_wifi over HCI
  ******************************************************************************/

int32_t mipc_init(mipc_send_func_t ipc_send)
{
  int32_t ret;

  PendingRequest.req_id = MIPC_REQ_ID_RESET_VAL;
  SEM_INIT(PendingRequest.resp_flag, 1);
  memset(AsyncSlots,0,sizeof(AsyncSlots)); AsyncFailed=0;
  network_async_last_status=0; network_async_last_id=0;
  network_async_late_errors=network_async_timeout_errors=network_async_hci_errors=0;
  SEM_INIT(AsyncResponseSem,ASYNC_SLOTS);

  ret = mx_wifi_hci_init(ipc_send);

  return ret;
}


int32_t mipc_deinit(void)
{
  int32_t ret;

  SEM_DEINIT(PendingRequest.resp_flag);
  SEM_DEINIT(AsyncResponseSem);

  ret = mx_wifi_hci_deinit();

  return ret;
}


int32_t mipc_request(uint16_t api_id,
                     uint8_t *cparams, uint16_t cparams_size,
                     uint8_t *rbuffer, uint16_t *rbuffer_size,
                     uint32_t timeout_ms)
{
  int32_t ret = MIPC_CODE_ERROR;
  uint8_t *cbuf;
  bool copy_buffer = true;
  uint32_t prof = transfer_cpu_active && api_id == MIPC_API_WIFI_BYPASS_OUT_CMD;
  uint32_t tick = transfer_cycles();

  LOCK(wifi_obj_get()->lockcmd);
  if (prof) { network_probe.mipc_calls++; network_probe.mipc_mutex_cycles += (uint32_t)(transfer_cycles()-tick); }

  if (api_id == MIPC_API_WIFI_BYPASS_OUT_CMD && network_async_depth && network_async_active) {
    uint32_t depth=network_async_depth > ASYNC_SLOTS ? ASYNC_SLOTS : network_async_depth;
    if (cparams_size > MX_WIFI_IPC_PAYLOAD_SIZE || !cparams || !rbuffer || !rbuffer_size ||
        *rbuffer_size < sizeof(int32_t) || async_wait_below(depth,timeout_ms)) {
      UNLOCK(wifi_obj_get()->lockcmd); return MIPC_CODE_ERROR;
    }
    mipc_trace_event(MT_ALLOC_IN,0,api_id,cparams_size,trace_slot_depth(0),0,1,0,NULL);
    async_slot_t *slot=NULL;
    for (uint32_t i=0; i<ASYNC_SLOTS; ++i) if (!AsyncSlots[i].state) { slot=&AsyncSlots[i]; break; }
    if (!slot) { UNLOCK(wifi_obj_get()->lockcmd); return MIPC_CODE_ERROR; }
    uint32_t req_id=get_new_req_id();
    memcpy(slot->command+MIPC_PKT_REQ_ID_OFFSET,&req_id,sizeof(req_id));
    memcpy(slot->command+MIPC_PKT_API_ID_OFFSET,&api_id,sizeof(api_id));
    memcpy(slot->command+MIPC_PKT_PARAMS_OFFSET,cparams,cparams_size);
    slot->id=req_id; slot->status=MIPC_CODE_ERROR; slot->state=1;
    mipc_trace_event(MT_ALLOC_OUT,req_id,api_id,cparams_size,
      trace_slot_depth((uint32_t)(slot-AsyncSlots)),0,1,0,slot->command);
    mipc_trace_submit(req_id,api_id,cparams_size,trace_slot_depth((uint32_t)(slot-AsyncSlots)),slot->command);
    uint32_t pending=0;
    for (uint32_t i=0; i<ASYNC_SLOTS; ++i) pending += AsyncSlots[i].state != 0;
    if (pending > network_async_max_pending) network_async_max_pending=pending;
    tick=transfer_cycles();
    ret=mx_wifi_hci_send(slot->command,MIPC_HEADER_SIZE+cparams_size);
    mipc_trace_event(MT_HCI_DONE,req_id,api_id,cparams_size,trace_slot_depth((uint32_t)(slot-AsyncSlots)),ret,0,0,slot->command);
    if (mipc_trace_gpio_enable) GPIOF->BSRR=(uint32_t)GPIO2_EX_Pin<<16;
    if (prof) network_probe.mipc_hci_cycles += (uint32_t)(transfer_cycles()-tick);
    if (ret) { network_async_errors++; network_async_hci_errors++; AsyncFailed=1; } /* retain owned buffer until reset */
    else { network_async_submitted++; int32_t accepted=MIPC_CODE_SUCCESS;
      memcpy(rbuffer,&accepted,sizeof(accepted)); *rbuffer_size=sizeof(accepted); }
    async_reap();
    UNLOCK(wifi_obj_get()->lockcmd);
    return ret;
  }
  if (async_wait_below(1,timeout_ms)) {
    UNLOCK(wifi_obj_get()->lockcmd); return MIPC_CODE_ERROR;
  }

  /* DEBUG_LOG("\n%s()>  %" PRIu32 "\n", __FUNCTION__, (uint32_t)cparams_size); */

  if (cparams_size <= MX_WIFI_IPC_PAYLOAD_SIZE)
  {
    /* Create the command data. */
    const uint16_t cbuf_size = MIPC_PKT_REQ_ID_SIZE + MIPC_PKT_API_ID_SIZE + cparams_size;
    mipc_trace_event(MT_ALLOC_IN,0,api_id,cparams_size,255U,0,0,0,NULL);

#if MX_WIFI_TX_BUFFER_NO_COPY
    if (api_id == MIPC_API_WIFI_BYPASS_OUT_CMD)
    {
      cbuf = byte_pointer_add_signed_offset(cparams, - (MIPC_PKT_REQ_ID_SIZE + MIPC_PKT_API_ID_SIZE));
      copy_buffer = false;
    }
    else
#endif /* MX_WIFI_TX_BUFFER_NO_COPY */
    {
      DEBUG_LOG("\n%-15s(): Allocate %" PRIu32 " bytes\n", __FUNCTION__, (uint32_t)cbuf_size);

      cbuf = (uint8_t *)MX_WIFI_MALLOC(cbuf_size);

      MX_STAT(alloc);
    }

    if (NULL != cbuf)
    {
      /* Get an unique identifier. */
      const uint32_t req_id = get_new_req_id();

      /* Copy the protocol parameter to the head part of the buffer. */
      (void)memcpy(byte_pointer_add_signed_offset(cbuf, MIPC_PKT_REQ_ID_OFFSET), &req_id, sizeof(req_id));
      (void)memcpy(byte_pointer_add_signed_offset(cbuf, MIPC_PKT_API_ID_OFFSET), &api_id, sizeof(api_id));

      if ((true == copy_buffer) && (cparams_size > 0))
      {
        (void)memcpy(byte_pointer_add_signed_offset(cbuf, MIPC_PKT_PARAMS_OFFSET), cparams, cparams_size);
      }

      mipc_trace_event(MT_ALLOC_OUT,req_id,api_id,cparams_size,255U,0,copy_buffer,0,cbuf);
      /* A single pending request due to LOCK usage on command. */
      if (PendingRequest.req_id != MIPC_REQ_ID_RESET_VAL)
      {
        DEBUG_LOG("Error req_id must be 0xffffffff here %" PRIu32 "\n", PendingRequest.req_id);
        MX_ASSERT(false);
      }

      PendingRequest.req_id = req_id;
      PendingRequest.rbuffer = rbuffer;
      PendingRequest.rbuffer_size = rbuffer_size;
      mipc_trace_submit(req_id,api_id,cparams_size,255U|(1U<<8)|(1U<<16),cbuf);

      /* static int iter=0;                       */
      /* printf("%d push %d\n",iter++,cbuf_size); */

      /* Send the command. */
      DEBUG_LOG("%-15s(): req_id: 0x%08" PRIx32 " : %" PRIu32 "\n", __FUNCTION__, req_id, (uint32_t)cbuf_size);

      tick = transfer_cycles();
      ret = mx_wifi_hci_send(cbuf, cbuf_size);
      mipc_trace_event(MT_HCI_DONE,req_id,api_id,cparams_size,255U|(1U<<8)|(1U<<16),ret,0,0,cbuf);
      if (mipc_trace_gpio_enable) GPIOF->BSRR=(uint32_t)GPIO2_EX_Pin<<16;
      if (prof) network_probe.mipc_hci_cycles += (uint32_t)(transfer_cycles()-tick);
      tick = transfer_cycles();
      if (ret == 0)
      {
        /* Wait for the command answer. */
        if (SEM_WAIT(PendingRequest.resp_flag, timeout_ms, mipc_poll) != SEM_OK)
        {
          DEBUG_ERROR("Error: command 0x%04" PRIx32 " timeout(%" PRIu32 " ms) waiting answer %" PRIu32 "\n",
                      (uint32_t)api_id, timeout_ms, PendingRequest.req_id);
          PendingRequest.req_id = MIPC_REQ_ID_RESET_VAL;
          ret = MIPC_CODE_ERROR;
        }
      }
      else
      {
        DEBUG_ERROR("Failed to send command to HCI\n");
        MX_ASSERT(false);
      }

      if (prof) { network_probe.mipc_response_cycles += (uint32_t)(transfer_cycles()-tick); if (ret) network_probe.mipc_errors++; }
      mipc_trace_event(MT_COMPLETE,req_id,api_id,cparams_size,255U,ret,0,0,cbuf);

      DEBUG_LOG("%-15s()< req_id: 0x%08" PRIx32 " done (%" PRId32 ")\n\n", __FUNCTION__, req_id, ret);

      if (true == copy_buffer)
      {
        MX_WIFI_FREE(cbuf);

        MX_STAT(free);
      }
    }
  }

  UNLOCK(wifi_obj_get()->lockcmd);

  return ret;
}


void mipc_poll(uint32_t timeout)
{
  mx_buf_t *nbuf;

  /* Process the received data inside the RX buffer. */
  nbuf = mx_wifi_hci_recv(timeout);

  if (NULL != nbuf)
  {
    const uint32_t len = MX_NET_BUFFER_GET_PAYLOAD_SIZE(nbuf);

    DEBUG_LOG("%-15s(): %p HCI recv len %" PRIu32 "\n", __FUNCTION__, nbuf, len);

    if (len > 0U)
    {
      mipc_event(nbuf);
    }
    else
    {
      MX_NET_BUFFER_FREE(nbuf);

      MX_STAT(free);
    }
  }
}


int32_t mipc_echo(uint8_t *in, uint16_t in_len, uint8_t *out, uint16_t *out_len,
                  uint32_t timeout)
{
  int32_t ret = MIPC_CODE_ERROR;

  if ((NULL != in) && (NULL != out) && (NULL != out_len))
  {
    ret = mipc_request(MIPC_API_SYS_ECHO_CMD, in, in_len, out, out_len, timeout);
    if (MIPC_CODE_SUCCESS != ret)
    {
      *out_len = 0;
    }
  }
  return ret;
}


/*******************************************************************************
  * IPC API event callbacks
  ******************************************************************************/

/* System */

void mapi_reboot_event_callback(mx_buf_t *mxbuff)
{
  if (mxbuff != NULL)
  {
    mx_wifi_hci_free(mxbuff);
  }
  DEBUG_LOG("\nEVENT: reboot done.\n");
}


void mapi_fota_status_event_callback(mx_buf_t *nbuf)
{
  if (NULL != nbuf)
  {
    uint8_t *payload = MX_NET_BUFFER_PAYLOAD(nbuf);
    if (NULL != payload)
    {
      mx_wifi_fota_status_e status = *((mx_wifi_fota_status_e *)(byte_pointer_add_signed_offset(payload, MIPC_PKT_PARAMS_OFFSET)));

      DEBUG_LOG("\nEVENT: FOTA status: %02x\n", status);

      mx_wifi_hci_free(nbuf);

      {
        mx_wifi_fota_status_cb_t const status_cb = wifi_obj_get()->Runtime.fota_status_cb;
        const uint32_t cb_args = wifi_obj_get()->Runtime.fota_user_args;
        if (NULL != status_cb)
        {
          status_cb(status, cb_args);
        }
      }
    }
  }
}


/* WiFi */

void mapi_wifi_status_event_callback(mx_buf_t *netbuf)
{
  uint8_t cate;
  mx_wifi_status_callback_t status_cb = NULL;
  void *cb_args = NULL;

  if (NULL != netbuf)
  {
    uint8_t *payload = MX_NET_BUFFER_PAYLOAD(netbuf);
    mwifi_status_t status = *((mwifi_status_t *)(byte_pointer_add_signed_offset(payload, MIPC_PKT_PARAMS_OFFSET)));

    DEBUG_LOG("\nEVENT: wifi status: %02x\n", status);

    mx_wifi_hci_free(netbuf);

    switch (status)
    {
      case MWIFI_EVENT_STA_UP:
      case MWIFI_EVENT_STA_DOWN:
      case MWIFI_EVENT_STA_GOT_IP:
        cate = MC_STATION;
        status_cb = wifi_obj_get()->Runtime.status_cb[0];
        cb_args = wifi_obj_get()->Runtime.callback_arg[0];
        break;

      case MWIFI_EVENT_AP_UP:
      case MWIFI_EVENT_AP_DOWN:
        cate = MC_SOFTAP;
        status_cb = wifi_obj_get()->Runtime.status_cb[1];
        cb_args = wifi_obj_get()->Runtime.callback_arg[1];
        break;

      default:
        cate = MC_SOFTAP;
        MX_ASSERT(false);
        /* break; */
    }

    if (NULL != status_cb)
    {
      status_cb(cate, status, cb_args);
    }
  }
}


void mapi_wifi_netlink_input_callback(mx_buf_t *netbuf)
{
  if (NULL != netbuf)
  {
    uint8_t *const buffer_in = MX_NET_BUFFER_PAYLOAD(netbuf);
    wifi_bypass_in_rparams_t *const in_rprarams = (wifi_bypass_in_rparams_t *)(byte_pointer_add_signed_offset(buffer_in, MIPC_PKT_PARAMS_OFFSET));

    MX_STAT(callback);

    if ((NULL != wifi_obj_get()->Runtime.netlink_input_cb) && \
        (in_rprarams->tot_len > 0))
    {
      uint32_t low_level_netif_idx = (uint32_t)in_rprarams->idx;

      MX_NET_BUFFER_HIDE_HEADER(netbuf, MIPC_PKT_PARAMS_OFFSET + sizeof(wifi_bypass_in_rparams_t));
      wifi_obj_get()->Runtime.netlink_input_cb(netbuf, (void *)&low_level_netif_idx);
    }
    else
    {
      MX_NET_BUFFER_FREE(netbuf);

      MX_STAT(free);
    }
  }
}
