#ifndef STOCK_STOP_MIC_H
#define STOCK_STOP_MIC_H
/* Opt-in STOP only, before the vendor's destructive HAL_DMA_Abort. */
static inline int stock_stop_pause_mic(MDFDriver_t *driver,TX_QUEUE *queue,uint32_t id) {
  MX_MDFParams_t *cfg=driver->mx_handle.p_mx_mdf_cfg;
  MDF_HandleTypeDef *filter=cfg->p_mdf;DMA_HandleTypeDef *dma=filter->hdma;
  sd_diag_terminal_t *t=sd_diag_get_terminal(id);
  uint32_t bytes=driver->buffer_size*2U,start=HAL_GetTick();
  if(!t || !bytes || (bytes&3U) || dma->State!=HAL_DMA_STATE_BUSY)
    return stock_stop_error(id,-10);
  HAL_NVIC_DisableIRQ(cfg->irq_n);
  filter->Instance->DFLTCR &= ~MDF_DFLTCR_DFLTEN;
  /* Let the enabled DMA empty the NOT_EMPTY filter FIFO after generation ends. */
  while((filter->Instance->DFLTCR&MDF_DFLTCR_DFLTACTIVE) ||
        (filter->Instance->DFLTISR&MDF_DFLTISR_RXNEF)) {
    if((uint32_t)(HAL_GetTick()-start)>3U)return stock_stop_error(id,-11);
  }
  __HAL_DMA_DISABLE_IT(dma,DMA_IT_SUSP);
  dma->Instance->CCR |= DMA_CCR_SUSP;
  while(!(dma->Instance->CSR&DMA_CSR_SUSPF)) {
    if((uint32_t)(HAL_GetTick()-start)>3U)return stock_stop_error(id,-12);
  }
  __DSB();
  if((dma->Instance->CSR&(DMA_FLAG_HT|DMA_FLAG_TC))==(DMA_FLAG_HT|DMA_FLAG_TC))
    return stock_stop_error(id,-13);
  /* Service a final pending full/half callback exactly once with DMA IRQ masked. */
  HAL_DMA_IRQHandler(dma);
  uint32_t remaining=dma->Instance->CBR1&DMA_CBR1_BNDT;
  if(remaining>bytes || (remaining&1U))return stock_stop_error(id,-14);
  uint32_t pos=(bytes-remaining)%bytes;
  uint32_t destination=dma->Instance->CDAR;
  uint32_t base=(uint32_t)driver->p_buffer;
  if(destination<base || destination>base+bytes || (destination-base)%bytes!=pos)
    return stock_stop_error(id,-15);
  uint32_t begin=(sd_diag_get_dma_sequence(id)&1U)?bytes/2U:0U;
  if(pos<begin || pos-begin>bytes/2U)return stock_stop_error(id,-16);
  if(filter->Instance->DFLTISR&(MDF_DFLTISR_DOVRF|MDF_DFLTISR_RFOVRF))
    return stock_stop_error(id,-17);
  sd_diag_terminal(id,queue,3U,dma->Instance->CSR,0,0,remaining,filter->Instance->DFLTCR);
  t->mic_tail_offset_bytes=begin;t->mic_tail_samples=(pos-begin)/2U;t->mic_pause_complete=1;
  sd_diag_mic_tail_generated(id,t->mic_tail_samples);
  sd_diag_quiesce(id,0);
  /* The existing IDrvStop now aborts/resets this already suspended channel. */
  return 0;
}
#endif
