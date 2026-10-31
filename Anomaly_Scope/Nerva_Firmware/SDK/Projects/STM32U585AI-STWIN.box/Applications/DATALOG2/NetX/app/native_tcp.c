/* Controlled architecture experiment: module TCP instead of STM32 NetX TCP.
 * Uses the exact socket MIPC ABI in this bundled MX_WIFI 2.3.4 source. No SDK or
 * module firmware upgrade; no SD access. Bypass mode/callback are restored. */
#include "core/mx_address.h"
extern char wifi_ssid[],wifi_password[];
volatile uint32_t native_stage,native_status,native_restore_status,native_ip;
static uint8_t *native_command;
static int32_t native_request(uint16_t api,const void *cp,uint16_t length)
{
  int32_t value=-1;uint16_t size=4;
  if(mipc_request(api,(uint8_t *)cp,length,(uint8_t *)&value,&size,30000) || size!=4)return -1;
  return value;
}
static UINT native_send(int32_t fd,const uint8_t *p,uint32_t n)
{
  while(n) {
    uint32_t k=n>1524?1524:n;int32_t flags=0;
    memcpy(native_command,&fd,4);memcpy(native_command+4,&k,4);memcpy(native_command+8,&flags,4);memcpy(native_command+12,p,k);
    uint32_t t=transfer_cycles();int32_t sent=native_request(MIPC_API_SOCKET_SEND_CMD,native_command,(uint16_t)(k+12));
    throughput_profile.send_cycles+=(uint32_t)(transfer_cycles()-t);throughput_profile.send_calls++;
    if(sent<=0 || (uint32_t)sent>k)return NX_NOT_SUCCESSFUL;
    p+=sent;n-=(uint32_t)sent;
  }
  return 0;
}
static UINT native_execute(void)
{
  if(app_model.log_controller_model.status || !transfer_debug_wifi || transfer_debug_ftp ||
     !throughput_bytes || throughput_bytes>UINT32_MAX || network_async_active)return NX_INVALID_PARAMETERS;
  MX_WIFIObject_t *obj=wifi_obj_get();mx_wifi_netlink_input_cb_t callback=obj->Runtime.netlink_input_cb;
  void *argument=obj->Runtime.netlink_user_args;MX_WIFI_Network_t settings=obj->NetSettings;
  uint8_t *raw=SysAlloc(65536);native_command=SysAlloc(1536);int32_t fd=-1;UINT st=0;uint32_t switched=0;
  if(!raw || !native_command){SysFree(raw);SysFree(native_command);native_command=NULL;return NX_NO_PACKET;}
  memset((void *)&throughput_profile,0,sizeof(throughput_profile));
  native_stage=native_status=native_restore_status=native_ip=0;
  for(uint32_t i=0;i<65536;i++)raw[i]=(uint8_t)(i*73U+(i>>8)*19U+41U);
  native_stage=1;st=MX_WIFI_Network_bypass_mode_set(obj,0,NULL,NULL);if(st)goto done;switched=1;
  native_stage=2;st=MX_WIFI_Disconnect(obj);if(st)goto done;
  obj->NetSettings.DHCP_IsEnabled=1;
  native_stage=3;st=MX_WIFI_Connect(obj,wifi_ssid,wifi_password,MX_WIFI_SEC_AUTO);if(st)goto done;
  native_stage=4;
  for(uint32_t attempt=0;attempt<30;attempt++) {
    uint8_t ip[4]={0};
    if(MX_WIFI_GetIPAddress(obj,ip,MC_STATION)==MX_WIFI_STATUS_OK && (ip[0]||ip[1]||ip[2]||ip[3])) {
      native_ip=((uint32_t)ip[0]<<24)|((uint32_t)ip[1]<<16)|((uint32_t)ip[2]<<8)|ip[3];break;
    }
    tx_thread_sleep(1000);
  }
  if(!native_ip){st=NX_NOT_SUCCESSFUL;goto done;}
  native_stage=5;int32_t create[3]={MX_AF_INET,MX_SOCK_STREAM,MX_IPPROTO_TCP};
  fd=native_request(MIPC_API_SOCKET_CREATE_CMD,create,12);if(fd<0){st=NX_NOT_SUCCESSFUL;goto done;}
  struct __attribute__((packed)){int32_t fd;struct mx_sockaddr_storage address;int32_t length;} connect;
  memset(&connect,0,sizeof(connect));connect.fd=fd;connect.length=16;connect.address.s2_len=16;connect.address.ss_family=MX_AF_INET;
  connect.address.s2_data1[0]=(uint8_t)(throughput_server_port>>8);connect.address.s2_data1[1]=(uint8_t)throughput_server_port;
  connect.address.s2_data2[0]=__REV(throughput_server_ip);
  native_stage=6;if(native_request(MIPC_API_SOCKET_CONNECT_CMD,&connect,sizeof(connect))){st=NX_NOT_SUCCESSFUL;goto done;}
  uint32_t header[6]={0x31505452U,throughput_run_id,1,65536,(uint32_t)throughput_bytes,0};
  st=native_send(fd,(uint8_t *)header,24);if(st)goto done;
  memset((void *)&network_probe,0,sizeof(network_probe));memset((void *)&throughput_spi_profile,0,sizeof(throughput_spi_profile));
  probe_cs_active=probe_dma_active=probe_last_dma_valid=0;
  transfer_cpu_samples=transfer_cpu_thread_samples=0;throughput_owner=tx_thread_identify();transfer_cpu_active=1;
  throughput_profile.send_calls=0;throughput_profile.send_cycles=0;throughput_profile.clock_hz=SystemCoreClock;throughput_profile.ram_bytes=65536+1536;
  throughput_profile.spi_cfg1=hspi1.Instance->CFG1;
  throughput_profile.spi_hz=HAL_RCC_GetSysClockFreq()>>(((hspi1.Instance->CFG1>>28)&7U)+1U);
  throughput_profile.start_tick=tx_time_get();native_stage=7;uint64_t pos=0;
  while(pos<throughput_bytes) {
    uint32_t offset=(uint32_t)pos&65535U;
    uint32_t n=throughput_bytes-pos>65536-offset?65536-offset:(uint32_t)(throughput_bytes-pos);
    st=native_send(fd,raw+offset,n);if(st)break;pos+=n;throughput_profile.bytes=pos;
  }
  throughput_profile.send_end_tick=tx_time_get();native_stage=8;
  if(!st) {
    int32_t request[3]={fd,4,0};uint8_t response[8]={0};uint16_t size=8;
    int32_t status=mipc_request(MIPC_API_SOCKET_RECV_CMD,(uint8_t *)request,12,response,&size,30000);int32_t count;
    memcpy(&count,response,4);if(status || size!=8 || count!=4 || memcmp(response+4,"RDON",4))st=NX_INVALID_PACKET;
  }
  throughput_profile.drain_tick=tx_time_get();transfer_cpu_active=0;throughput_owner=NULL;
  throughput_profile.cpu_samples=transfer_cpu_samples;throughput_profile.cpu_thread_samples=transfer_cpu_thread_samples;
done:
  transfer_cpu_active=0;throughput_owner=NULL;native_status=st;
  if(fd>=0 && native_request(MIPC_API_SOCKET_CLOSE_CMD,&fd,4))native_restore_status=NX_NOT_SUCCESSFUL;
  if(switched && MX_WIFI_Network_bypass_mode_set(obj,1,callback,argument))native_restore_status=NX_NOT_SUCCESSFUL;
  obj->NetSettings=settings;SysFree(raw);SysFree(native_command);native_command=NULL;
  throughput_profile.status=st;throughput_profile.cleanup_status=native_restore_status;throughput_profile.finish_tick=tx_time_get();
  return st?st:native_restore_status;
}
