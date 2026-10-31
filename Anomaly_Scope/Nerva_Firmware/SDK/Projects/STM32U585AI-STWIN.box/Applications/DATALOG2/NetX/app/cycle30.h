#ifndef CYCLE30_H
#define CYCLE30_H
#include "nx_api.h"
#include "fx_api.h"
#include <stdint.h>
extern volatile uint32_t cycle30_enabled, cycle30_state, cycle30_error;
extern volatile uint32_t cycle30_period_s, cycle30_completed, cycle30_max_cycles;
extern volatile uint32_t cycle30_config_request;
extern volatile uint32_t cycle30_record_start, cycle30_record_stop, cycle30_upload_start;
extern volatile uint32_t cycle30_last_status, cycle30_attempts;
extern volatile uint64_t cycle30_free_bytes, cycle30_native_bytes;
extern char cycle30_folder[64];
extern char *cycle30_manifest;
extern uint32_t cycle30_manifest_len;
uint32_t cycle30_service(NX_IP *ip, FX_MEDIA *media, uint32_t wifi, uint32_t ftp);
void cycle30_folder_started(const char *folder);
uint32_t cycle30_sd_closed(void);
uint32_t cycle30_sd_pending(void);
void cycle30_release_sd_trace(void);
#endif
