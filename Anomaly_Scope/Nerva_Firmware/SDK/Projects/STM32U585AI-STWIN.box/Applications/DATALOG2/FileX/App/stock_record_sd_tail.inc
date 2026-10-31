#include "record_sd_tail.h"
#include <string.h>
#include <stddef.h>

int rsd_append(rsd_tails *t,uint32_t sid,const uint8_t *p,uint32_t n,rsd_write write,void *arg) {
  if(!t || sid>=RSD_STREAMS || !p || !n || n>32816U || (n&3U) ||
     ((uintptr_t)p&3U) || !write || t->used[sid]>=RSD_SECTOR || (t->used[sid]&3U))return -1;
  if(t->used[sid]) {
    uint32_t take=RSD_SECTOR-t->used[sid];if(take>n)take=n;
    memcpy(t->data[sid]+t->used[sid],p,take);t->used[sid]+=take;p+=take;n-=take;
    if(t->used[sid]==RSD_SECTOR) {
      int st=write(arg,sid,t->data[sid],RSD_SECTOR);if(st)return st;
      t->used[sid]=0;
    }
  }
  uint32_t bulk=n&~(RSD_SECTOR-1U);
  if(bulk) {
    int st=write(arg,sid,p,bulk);if(st)return st;
    p+=bulk;n-=bulk;
  }
  if(n){memcpy(t->data[sid],p,n);t->used[sid]=n;}
  return 0;
}
int rsd_flush(rsd_tails *t,uint32_t sid,rsd_write write,void *arg) {
  if(!t || sid>=RSD_STREAMS || !write || t->used[sid]>RSD_SECTOR || (t->used[sid]&3U))return -1;
  if(t->used[sid]) {
    int st=write(arg,sid,t->data[sid],t->used[sid]);if(st)return st;
    t->used[sid]=0;
  }
  return 0;
}
