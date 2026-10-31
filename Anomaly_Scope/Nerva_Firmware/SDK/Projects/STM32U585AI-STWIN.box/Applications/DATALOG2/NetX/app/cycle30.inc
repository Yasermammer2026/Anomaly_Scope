/* Autonomous stopped-recording transfer. Included after lossless_client.c.
 * Original recordings are never deleted. Only a verified durable ACK advances
 * the cycle. An early acquisition/storage failure latches ERROR and retains SD.
 * Network retries resume the same immutable files and acquisition identity.
 */
#include "cycle30.h"
#include "cycle30_network_private.h"
#include "DatalogAppTask.h"
#include "App_model_Log_Controller.h"
#include "rng.h"
#include "parson.h"
#include <stdio.h>

volatile uint32_t cycle30_enabled = 1, cycle30_state, cycle30_error;
volatile uint32_t cycle30_period_s = 1800, cycle30_completed, cycle30_max_cycles;
volatile uint32_t cycle30_config_request;
volatile uint32_t cycle30_record_start, cycle30_record_stop, cycle30_upload_start;
volatile uint32_t cycle30_last_status, cycle30_attempts;
volatile uint64_t cycle30_free_bytes, cycle30_native_bytes;
char cycle30_folder[64];
char *cycle30_manifest;
uint32_t cycle30_manifest_len;
static uint32_t cycle30_deadline, cycle30_initialized;
static uint32_t cycle30_pending_checked;
static uint32_t cycle30_config_loaded,cycle30_config_generation,cycle30_error_deadline;
extern volatile uint32_t stock_rx_packets;
static uint32_t cycle30_fail(uint32_t error);

/* Alternating CRC-protected records preserve test/production configuration
   across the clean restart. A torn update retains the previous valid copy. */
static UINT cycle30_config_save(FX_MEDIA *media)
{
  if(!cycle30_period_s || cycle30_period_s>3600)return FX_INVALID_OPTION;
  uint32_t record[8]={0x31435943U,1,cycle30_config_generation+1,
    cycle30_period_s,cycle30_max_cycles,cycle30_completed,0,0};
  record[7]=lr_crc(record,28);
  const char *path=(record[2]&1U)?"/CYCLEA.CFG":"/CYCLEB.CFG";
  UINT st=fx_file_create(media,(CHAR *)path);
  if(st && st!=FX_ALREADY_CREATED)return st;
  FX_FILE file;st=fx_file_open(media,&file,(CHAR *)path,FX_OPEN_FOR_WRITE);if(st)return st;
  st=fx_file_seek(&file,0);
  if(!st)st=fx_file_write(&file,record,sizeof(record));
  if(!st)st=fx_file_truncate(&file,sizeof(record));
  UINT close=fx_file_close(&file);if(!st)st=close;
  if(!st)st=fx_media_flush(media);
  if(!st)cycle30_config_generation=record[2];
  return st;
}

static UINT cycle30_config_load(FX_MEDIA *media)
{
  uint32_t best[8]={0},found=0,exists=0;
  for(uint32_t copy=0;copy<2;copy++) {
    FX_FILE file;uint32_t record[8];ULONG got=0;
    CHAR *path=copy?"/CYCLEB.CFG":"/CYCLEA.CFG";
    UINT st=fx_file_open(media,&file,path,FX_OPEN_FOR_READ);
    if(st==FX_NOT_FOUND)continue;
    if(st)return st;
    exists=1;st=fx_file_read(&file,record,sizeof(record),&got);
    UINT close=fx_file_close(&file);
    if(st || close || got!=sizeof(record) || record[0]!=0x31435943U ||
       record[1]!=1 || !record[3] || record[3]>3600 || record[7]!=lr_crc(record,28))continue;
    if(!found || (int32_t)(record[2]-best[2])>0){memcpy(best,record,sizeof(best));found=1;}
  }
  if(!found) {
    if(exists)return FX_IO_ERROR;
    cycle30_config_loaded=1;return cycle30_config_save(media);
  }
  cycle30_config_generation=best[2];cycle30_period_s=best[3];
  cycle30_max_cycles=best[4];cycle30_completed=best[5];
  cycle30_config_loaded=1;return FX_SUCCESS;
}

static uint32_t cycle30_restart(void)
{
  if(app_model.log_controller_model.status || cycle30_sd_pending())return cycle30_fail(120);
  if(DatalogAppTask_sd_stream_disable())return cycle30_fail(121);
  __DSB();HAL_NVIC_SystemReset();return 0;
}

static UINT cycle30_saved_schemas(FX_MEDIA *media)
{
  FX_FILE source;ULONG got=0;
  UINT st=fx_file_open(media,&source,lr_streams[10].path,FX_OPEN_FOR_READ);
  if(st)return st;
  ULONG size=source.fx_file_current_file_size;
  char *text=(size && size<=65536)?SysAlloc(size+1):NULL;
  if(!text){fx_file_close(&source);return FX_NOT_ENOUGH_MEMORY;}
  st=fx_file_read(&source,text,size,&got);
  UINT close=fx_file_close(&source);
  if(st || close || got!=size){SysFree(text);return FX_IO_ERROR;}
  text[size]=0;JSON_Value *root=json_parse_string(text);SysFree(text);
  JSON_Array *devices=json_object_get_array(json_value_get_object(root),"devices");
  JSON_Object *device=json_array_get_object(devices,0);
  JSON_Array *components=json_object_get_array(device,"components");
  for(uint32_t i=0;i<10;i++) {
    char name[64];const char *base=strrchr(lr_streams[i].path,'/')+1;
    snprintf(name,sizeof(name),"%s",base);char *ext=strrchr(name,'.');if(ext)*ext=0;
    JSON_Object *sensor=NULL;
    for(size_t j=0;j<json_array_get_count(components);j++) {
      JSON_Object *candidate=json_object_get_object(json_array_get_object(components,j),name);
      if(candidate){sensor=candidate;break;}
    }
    uint32_t item=(uint32_t)json_object_get_number(sensor,"sd_dps");
    uint32_t spts=(uint32_t)json_object_get_number(sensor,"samples_per_ts");
    if(!sensor || item<8 || item>131072 || !spts){st=FX_INVALID_OPTION;break;}
    lr_streams[i].schema.item_size=item;lr_streams[i].schema.samples_per_ts=spts;
  }
  json_value_free(root);return st;
}

static UINT cycle30_find_pending(FX_MEDIA *media)
{
  char (*folders)[32]=SysAlloc(8192);if(!folders)return FX_NOT_ENOUGH_MEMORY;
  UINT st=fx_directory_default_set(media,FX_NULL);
  char name[128];uint32_t count=0;
  if(!st)st=fx_directory_first_entry_find(media,name);
  while(!st) {
    size_t length=strlen(name);
    if((length==9 && !strncmp(name,"DL2_",4)) ||
       (length==17 && name[8]=='_' && name[11]=='_' && name[14]=='_')) {
      if(count==256){SysFree(folders);return FX_INVALID_OPTION;}
      memcpy(folders[count],name,length+1);count++;
    }
    st=fx_directory_next_entry_find(media,name);
  }
  if(st!=FX_NO_MORE_ENTRIES){SysFree(folders);return st;}
  UINT result=FX_NOT_FOUND;
  for(uint32_t i=0;i<count;i++) {
    char path[128];FX_FILE source;ULONG got=0;uint64_t id=0;
    snprintf(path,sizeof(path),"/%s/CYCLE.ID",folders[i]);
    st=fx_file_open(media,&source,path,FX_OPEN_FOR_READ);
    if(st==FX_NOT_FOUND)continue;
    if(st){result=st;break;}
    st=fx_file_read(&source,&id,8,&got);UINT close=fx_file_close(&source);
    if(st || close || got!=8 || !id){result=FX_IO_ERROR;break;}
    snprintf(path,sizeof(path),"/%s/COMMIT.BIN",folders[i]);
    uint8_t ack[64];uint64_t ack_id=0;uint32_t crc=0;
    st=fx_file_open(media,&source,path,FX_OPEN_FOR_READ);
    if(!st) {
      st=fx_file_read(&source,ack,64,&got);close=fx_file_close(&source);
      memcpy(&ack_id,ack+8,8);memcpy(&crc,ack+60,4);
      if(!st && !close && got==64 && !memcmp(ack,"ACK1",4) &&
         ack_id==id && crc==lr_crc(ack,60))continue;
    } else if(st!=FX_NOT_FOUND){result=st;break;}
    snprintf(cycle30_folder,sizeof(cycle30_folder),"/%s",folders[i]);result=FX_SUCCESS;break;
  }
  SysFree(folders);return result;
}

void cycle30_folder_started(const char *folder)
{
  if(cycle30_enabled && cycle30_state == 2)
    snprintf(cycle30_folder,sizeof(cycle30_folder),"/%s",folder);
}

static uint32_t cycle30_fail(uint32_t error)
{
  cycle30_error=error;cycle30_state=99;
  return 0;
}

static UINT cycle30_prepare(FX_MEDIA *media)
{
  static const struct {const char *name;uint32_t dim,width;} sensors[10]={
    {"imp23absu_mic",1,2},{"iis3dwb_acc",3,2},{"imp34dt05_mic",1,2},
    {"ism330dhcx_gyro",3,2},{"ism330dhcx_acc",3,2},{"iis2dlpc_acc",3,2},
    {"iis2iclx_acc",2,2},{"ilps22qs_press",1,4},{"stts22h_temp",1,4},
    {"iis2mdc_mag",3,2}};
  if(!cycle30_folder[0])return FX_INVALID_NAME;
  memset(lr_streams,0,sizeof(lr_streams));lr_count=12;
  for(uint32_t i=0;i<10;i++) {
    SensorModel_t *model=NULL;
    for(uint32_t j=0;j<SENSOR_NUMBER;j++)
      if(app_model.s_models[j] && !strcmp(app_model.s_models[j]->comp_name,sensors[i].name))
        model=app_model.s_models[j];
    if(!model)return FX_NOT_FOUND;
    snprintf(lr_streams[i].path,sizeof(lr_streams[i].path),"%s/%s.dat",cycle30_folder,sensors[i].name);
    lr_streams[i].schema=(sl_schema){model->stream_params.sd_dps,sensors[i].dim,
                                   model->stream_params.spts,sensors[i].width};
  }
  snprintf(lr_streams[10].path,sizeof(lr_streams[10].path),"%s/device_config.json",cycle30_folder);
  snprintf(lr_streams[11].path,sizeof(lr_streams[11].path),"%s/acquisition_info.json",cycle30_folder);
  UINT st=lr_stat(media);if(st)return st;
  st=cycle30_saved_schemas(media);if(st)return st;
  cycle30_native_bytes=0;
  for(uint32_t i=0;i<lr_count;i++)cycle30_native_bytes+=lr_streams[i].size;
  /* Persist identity before network activity so retries keep the same receipt. */
  char id_path[128];snprintf(id_path,sizeof(id_path),"%s/CYCLE.ID",cycle30_folder);
  FX_FILE id_file;ULONG got=0;
  st=fx_file_open(media,&id_file,id_path,FX_OPEN_FOR_READ);
  if(st==FX_SUCCESS) {
    st=fx_file_read(&id_file,(void *)&lr_acquisition,8,&got);
    UINT close=fx_file_close(&id_file);if(st || close || got!=8 || !lr_acquisition)return FX_IO_ERROR;
  } else if(st==FX_NOT_FOUND) {
    uint32_t hi,lo;
    MX_RNG_Init();
    if(HAL_RNG_GenerateRandomNumber(&hrng,&hi)!=HAL_OK ||
       HAL_RNG_GenerateRandomNumber(&hrng,&lo)!=HAL_OK)return FX_IO_ERROR;
    lr_acquisition=((uint64_t)(hi&0x7fffffffU)<<32)|lo;
    if(!lr_acquisition)return FX_IO_ERROR;
    st=fx_file_create(media,id_path);if(st)return st;
    st=fx_file_open(media,&id_file,id_path,FX_OPEN_FOR_WRITE);if(st)return st;
    st=fx_file_write(&id_file,(void *)&lr_acquisition,8);
    UINT close=fx_file_close(&id_file);if(!st)st=close;
    if(!st)st=fx_media_flush(media);
    if(st)return st;
  } else return st;
  if(!cycle30_manifest)cycle30_manifest=SysAlloc(8196);
  if(!cycle30_manifest)return FX_NOT_ENOUGH_MEMORY;
  uint32_t n=(uint32_t)snprintf(cycle30_manifest,8192,
    "{\"acquisition_id\":%llu,\"block_bytes\":65536,\"files\":[",(unsigned long long)lr_acquisition);
  for(uint32_t i=0;i<lr_count;i++) {
    if(n>=7500U)return FX_INVALID_OPTION;
    const char *name=strrchr(lr_streams[i].path,'/')+1;
    n+=(uint32_t)snprintf(cycle30_manifest+n,8192-n,"%s{\"name\":\"%s\",\"schema\":",i?",":"",name);
    sl_schema *s=&lr_streams[i].schema;
    if(s->scalar_bytes)
      n+=(uint32_t)snprintf(cycle30_manifest+n,8192-n,
        "{\"dim\":%lu,\"item_size\":%lu,\"samples_per_ts\":%lu,\"scalar_bytes\":%lu}",
        (unsigned long)s->dim,(unsigned long)s->item_size,
        (unsigned long)s->samples_per_ts,(unsigned long)s->scalar_bytes);
    else n+=(uint32_t)snprintf(cycle30_manifest+n,8192-n,"null");
    n+=(uint32_t)snprintf(cycle30_manifest+n,8192-n,",\"size\":%lu}",(unsigned long)lr_streams[i].size);
  }
  n+=(uint32_t)snprintf(cycle30_manifest+n,8192-n,"]}");
  if(n>8192)return FX_INVALID_OPTION;
  cycle30_manifest_len=n;memset(cycle30_manifest+n,0,4);
  st=lr_hash_init();
  if(!st)st=lr_hash_update((uint8_t *)cycle30_manifest,n,lr_manifest_sha,1);
  HAL_HASH_DeInit(&lr_hash);__HAL_RCC_HASH_CLK_DISABLE();
  snprintf(lr_receipt_path,sizeof(lr_receipt_path),"%s/COMMIT.BIN",cycle30_folder);
  lr_raw_only=0;throughput_pool_bytes=65536;throughput_queue_depth=8;
  network_async_depth=0;network_powersave=0;
  throughput_server_ip=IP_ADDRESS(192,168,8,4);throughput_server_port=19195;
  return st;
}

uint32_t cycle30_service(NX_IP *ip,FX_MEDIA *media,uint32_t wifi,uint32_t ftp)
{
  uint32_t now=tx_time_get();
  if(!cycle30_initialized) {
    cycle30_initialized=1;cycle30_deadline=now+15U*TX_TIMER_TICKS_PER_SECOND;
    memcpy(wifi_ssid,cycle30_ssid,sizeof(cycle30_ssid));
    memcpy(wifi_password,cycle30_password,sizeof(cycle30_password));
    app_model.automode_model.enabled=0;
    memset((void *)&lr_profile,0,sizeof(lr_profile));
    stock_rx_packets=32;
    sd_diag.tail_fixed=1;sd_diag.split_ring=1;sd_diag.urgency_enabled=1;
    cycle30_state=1;
  }
  if(!cycle30_enabled || cycle30_state==20)return 0;
  if(cycle30_state==99) {
    if(cycle30_error>=200U) {
      if(!cycle30_error_deadline)cycle30_error_deadline=now+60U*TX_TIMER_TICKS_PER_SECOND;
      if((int32_t)(now-cycle30_error_deadline)>=0)return cycle30_restart();
    }
    return 0;
  }
  if(ftp)return cycle30_fail(101);
  if(cycle30_state==1) {
    if((int32_t)(now-cycle30_deadline)<0)return 0;
    if(wifi)return 2;
    if(app_model.log_controller_model.status)return cycle30_fail(102);
    UINT st=DatalogAppTask_sd_stream_enable();if(st)return cycle30_fail(103);
    ULONG64 free_bytes=0;st=fx_media_extended_space_available(media,&free_bytes);
    cycle30_free_bytes=free_bytes;
    if(st)return cycle30_fail(104);
    if(cycle30_sd_pending())return 0;
    if(!cycle30_config_loaded) {
      st=cycle30_config_load(media);if(st)return cycle30_fail(122);
    }
    if(cycle30_config_request) {
      st=cycle30_config_save(media);if(st)return cycle30_fail(123);
      cycle30_config_request=0;
    }
    if(!cycle30_pending_checked) {
      cycle30_pending_checked=1;
      st=cycle30_find_pending(media);
      if(!st) {
        cycle30_release_sd_trace();
        st=cycle30_prepare(media);cycle30_last_status=st;
        if(st)return cycle30_fail(115);
        cycle30_attempts=0;cycle30_state=4;return 1;
      }
      if(st!=FX_NOT_FOUND)return cycle30_fail(116);
    }
    if(cycle30_max_cycles && cycle30_completed>=cycle30_max_cycles) {
      if(DatalogAppTask_sd_stream_disable())return cycle30_fail(124);
      cycle30_state=20;return 0;
    }
    if(free_bytes<(uint64_t)cycle30_period_s*800000U+67108864U)return cycle30_fail(104);
    cycle30_folder[0]=0;cycle30_error=0;cycle30_state=2;
    if(log_controller_start_log(LOG_CTRL_MODE_SD))return cycle30_fail(105);
    cycle30_record_start=tx_time_get();cycle30_record_stop=0;
    cycle30_deadline=cycle30_record_start+cycle30_period_s*TX_TIMER_TICKS_PER_SECOND;
    return 0;
  }
  if(cycle30_state==2) {
    if(!app_model.log_controller_model.status || app_model.log_controller_model.sd_failed || sd_diag.first.valid)
      return cycle30_fail(106);
    if((int32_t)(now-cycle30_deadline)<0)return 0;
    cycle30_record_stop=now;cycle30_state=3;
    if(log_controller_stop_log())return cycle30_fail(107);
    cycle30_deadline=now+120U*TX_TIMER_TICKS_PER_SECOND;return 0;
  }
  if(cycle30_state==3) {
    if(app_model.log_controller_model.status || !cycle30_sd_closed()) {
      if((int32_t)(now-cycle30_deadline)>=0)return cycle30_fail(108);
      return 0;
    }
    if(app_model.log_controller_model.sd_failed || sd_diag.first.valid)return cycle30_fail(109);
    cycle30_release_sd_trace();
    UINT st=DatalogAppTask_sd_stream_enable();if(st)return cycle30_fail(110);
    st=cycle30_prepare(media);cycle30_last_status=st;if(st)return cycle30_fail(111);
    cycle30_attempts=0;cycle30_state=4;
  }
  if(cycle30_state==4) {
    if(!wifi)return 1;
    cycle30_state=5;
  }
  if(cycle30_state==5) {
    cycle30_attempts++;cycle30_upload_start=tx_time_get();
    UINT st=lr_execute(ip,media);cycle30_last_status=st;
    if(lr_profile.cleanup_status)return cycle30_fail(112);
    if(st || !lr_profile.committed || !lr_profile.persisted) {
      cycle30_state=6;cycle30_deadline=tx_time_get()+30U*TX_TIMER_TICKS_PER_SECOND;
      return 0;
    }
    if(mipc_trace_release())return cycle30_fail(117);
    cycle30_completed++;
    if(cycle30_config_save(media))return cycle30_fail(125);
    SysFree(cycle30_manifest);cycle30_manifest=NULL;cycle30_manifest_len=0;
    cycle30_state=7;cycle30_deadline=tx_time_get()+2U*TX_TIMER_TICKS_PER_SECOND;
    return 0;
  }
  if(cycle30_state==6) {
    if((int32_t)(now-cycle30_deadline)>=0)return cycle30_restart();
    return 0;
  }
  if(cycle30_state==7) {
    if((int32_t)(now-cycle30_deadline)>=0)return cycle30_restart();
  }
  return 0;
}
