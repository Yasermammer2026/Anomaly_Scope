/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32U585xI Device from STM32U5 series
**                      2048Kbytes FLASH
**                      784Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** Copyright (c) 2025 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM) - 4;	/* end of "RAM" Ram type memory */

_Min_Heap_Size = 0x1000;	/* required amount of heap  */
_Min_Stack_Size = 0x800 ;	/* required amount of stack */

/* Memories definition */
MEMORY
{
  RAM	(xrw)	: ORIGIN = 0x20000000,	LENGTH = 768K
  SRAM4	(xrw)	: ORIGIN = 0x28000000,	LENGTH = 16K
  FLASH	(rx)	: ORIGIN = 0x08000000,	LENGTH = 1016K
  BENCH_FLASH (rx) : ORIGIN = 0x08100000, LENGTH = 1016K
}

/* Sections */
SECTIONS
{
  /* The startup code into "FLASH" Rom type memory */
  /* New stopped-transfer metadata only; payload/DMA buffers stay in RAM. */
  .transfer_bss (NOLOAD) : { *(.transfer_bss*) } >SRAM4

  .isr_vector :
  {
    KEEP(*(.isr_vector)) /* Startup code */
  } >FLASH

  /* Bench-only code uses bank2. Both OTA-ID pages are reserved.
     OTA update and bank switching are unavailable on this benchmark image. */
  .bench_text : {
    *ILPS22QSTask.o(.text.sd_diag_ilps_sources)
    *STTS22HTask.o(.text.STTS22HTaskSensorInitTaskParams)
    *IIS2MDCTask.o(.text.IIS2MDCTaskSensorInitTaskParams)
    *ILPS22QSTask.o(.text.ILPS22QSTaskSensorInit)
    *DatalogAppTask.o(.text.DatalogAppTask_UpdateStreamingStatus)
    *DatalogAppTask.o(.text.DatalogAppTask_handle_sd_failure .text.DatalogAppTask_filex_queue_full_cb)
    *filex_dctrl_class.o(.text.filex_check_root_folder)
    *sd_diag.o(.text* .rodata*)
    *sd_ready_policy.o(.text* .rodata*)
    *transfer_bench_client.o(.text* .rodata*)
    *throughput_client.o(.text* .rodata*)
    *fx_stm32_sd_driver_glue.o(.text.fx_stm32_sd_read_blocks .text.HAL_SD_ErrorCallback)
    *nx_ip_driver_direct_command.o(.text*)
    *nxe_ip_driver_direct_command.o(.text*)
    *mx_wifi.o(.text.MX_WIFI_Scan .text.MX_WIFI_Get_scan_result)
    *app_netxduo.o(.text.NetX_Thread_Entry .text.WIFI_Connect .text.WIFI_Disconnect .text.FTP_Srv_Init .text.FTP_Srv_DeInit .text.FTP_Srv_Start .text.FTP_Srv_Stop .text.ip_address_change_notify_callback)
  } >BENCH_FLASH
  /* The program code and other data into "FLASH" Rom type memory */
  .text :
  {
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data into "FLASH" Rom type memory */
  .rodata :
  {
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
  } >FLASH

  .ARM.extab (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH
  .ARM (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* Used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM" Ram type memory */
  .data :
  {
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH

  /* Uninitialized data section into "RAM" Ram type memory */
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* User_heap_stack section, used to check that there is enough "RAM" Ram type memory left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
    /* start of unused RAM space */
    __RAM_segment_used_end__ = .; 
  } >RAM

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

/* ThreadX creates its 2000-byte init stack after this symbol. Keep it below
   the unchanged 2KiB MSP reservation; fail the build before runtime overlap. */
ASSERT(__RAM_segment_used_end__ + 2004 <= _estack - _Min_Stack_Size, "Insufficient ThreadX init-stack headroom")
