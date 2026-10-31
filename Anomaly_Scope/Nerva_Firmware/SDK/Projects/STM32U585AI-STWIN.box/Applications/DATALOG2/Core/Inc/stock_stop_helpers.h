#ifndef STOCK_STOP_HELPERS_H
#define STOCK_STOP_HELPERS_H
/* Only invoked from fenced stop hooks after all OnEnterPowerMode hooks ran.
 * Existing native event formatting/copying is reused; no source is retained. */
static inline int stock_stop_emit(IEventSrc *source,EMData_t *data,void *buffer,
                                 uint16_t type,uint32_t samples,uint16_t axes,uint32_t id) {
  if(!samples)return 0;
  if(axes==1U)EMD_1dInit(data,buffer,type,samples);
  else EMD_Init(data,buffer,type,E_EM_MODE_INTERLEAVED,2,samples,axes);
  DataEvent_t event;
  DataEventInit((IEvent *)&event,source,data,SysTsGetTimestampF(SysGetTimestampSrv()),id);
  uint32_t previous=stock_native_stop_collecting;stock_native_stop_collecting=1;
  int status=IEventSrcSendEvent(source,(IEvent *)&event,NULL);
  stock_native_stop_collecting=previous;
  if(status)sd_diag_sensor_io(id,status);
  else {sd_diag_sensor_read(id,samples,0);sd_diag_terminal_result(id,samples,0,0,0);}
  return status;
}
static inline int stock_stop_error(uint32_t id,int status) {
  sd_diag_sensor_stop_error(id,status);
  return status?status:-1;
}
#endif
