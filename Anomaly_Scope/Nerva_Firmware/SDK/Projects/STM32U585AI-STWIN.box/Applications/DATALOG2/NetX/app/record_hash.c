#include "record_hash.h"
#include <string.h>

_Static_assert(sizeof(HASH->CSR)/sizeof(uint32_t)==54U,"U585 HASH context register ABI");
void rh_open(void) {
  __HAL_RCC_HASH_CLK_ENABLE();
  __HAL_RCC_HASH_FORCE_RESET();__HAL_RCC_HASH_RELEASE_RESET();
}
void rh_shutdown(void) {
  __HAL_RCC_HASH_FORCE_RESET();__HAL_RCC_HASH_RELEASE_RESET();
  __HAL_RCC_HASH_CLK_DISABLE();
}
static int rh_push(rh_context *c,const uint8_t *data,uint32_t n) {
  if(!n)return 0;
  if((uintptr_t)data&3U) {
    uint32_t aligned[16];
    while(n) {
      uint32_t part=n>sizeof(aligned)?sizeof(aligned):n;
      memcpy(aligned,data,part);
      if(HAL_HASHEx_SHA256_Accmlt(&c->handle,(uint8_t *)aligned,part)!=HAL_OK)return -2;
      data+=part;n-=part;
    }
    return 0;
  }
  return HAL_HASHEx_SHA256_Accmlt(&c->handle,data,n)==HAL_OK?0:-2;
}
int rh_update(rh_context *c,const uint8_t *data,uint32_t n,uint32_t last,uint8_t digest[32]) {
  if(!c || c->complete || (n && !data) || last>1U || (last && !digest) || c->pending_bytes>67U)return -1;
  uint32_t input_bytes=n;
  if(c->saved) {
    HAL_HASH_ContextRestoring(&c->handle,(uint8_t *)c->registers);
  }
  uint8_t *tail=(uint8_t *)c->pending_words;
  /* Context registers are readable only at DINIS, after 17 initial words
   * and subsequent 16-word blocks. BUSY==0 alone is insufficient. Retain
   * the unprocessed suffix in owned storage; never retain a source pointer. */
  uint32_t block=c->initialized?64U:68U;
  uint32_t part=block-c->pending_bytes;if(part>n)part=n;
  if(part)memcpy(tail+c->pending_bytes,data,part);
  c->pending_bytes+=part;
  if(part)data+=part;
  n-=part;
  if(c->pending_bytes==block) {
    if(!c->initialized) {
      memset(&c->handle,0,sizeof(c->handle));c->handle.Init.DataType=HASH_DATATYPE_8B;
      if(HAL_HASH_Init(&c->handle)!=HAL_OK)return -2;
      c->initialized=1;
    }
    if(rh_push(c,tail,block))return -2;
    c->fed_bytes+=block;c->pending_bytes=0;
    uint32_t full=n&~63U;
    if(rh_push(c,data,full))return -2;
    c->fed_bytes+=full;
    if(full)data+=full;
    n-=full;
    if(n){memcpy(tail,data,n);c->pending_bytes=n;}
  }
  if(last) {
    if(!c->initialized) {
      memset(&c->handle,0,sizeof(c->handle));c->handle.Init.DataType=HASH_DATATYPE_8B;
      if(HAL_HASH_Init(&c->handle)!=HAL_OK)return -2;
      c->initialized=1;
    }
    if(HAL_HASHEx_SHA256_Accmlt_End(&c->handle,tail,c->pending_bytes,digest,1000)!=HAL_OK)return -2;
    c->complete=1;c->saved=0;
  } else if(c->initialized) {
    uint32_t start=HAL_GetTick();
    while(__HAL_HASH_GET_FLAG(HASH_FLAG_BUSY)) {
      if((uint32_t)(HAL_GetTick()-start)>1000U)return -3;
    }
    if(!__HAL_HASH_GET_FLAG(HASH_FLAG_DINIS) || (c->fed_bytes&63U)!=4U)return -4;
    HAL_HASH_ContextSaving(&c->handle,(uint8_t *)c->registers);c->saved=1;
  }
  /* HAL normal polling completion no longer needs a source/output pointer. */
  c->handle.pHashInBuffPtr=NULL;c->handle.pHashOutBuffPtr=NULL;
  c->bytes+=input_bytes;
  return 0;
}
