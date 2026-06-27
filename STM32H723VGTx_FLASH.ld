/*
******************************************************************************
**

**  File        : LinkerScript.ld
**
**  Author		: STM32CubeMX
**
**  Abstract    : Linker script for STM32H723VGTx series
**                1024Kbytes FLASH and 368Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
**
** Redistribution and use in source and binary forms, with or without modification,
** are permitted provided that the following conditions are met:
**   1. Redistributions of source code must retain the above copyright notice,
**      this list of conditions and the following disclaimer.
**   2. Redistributions in binary form must reproduce the above copyright notice,
**      this list of conditions and the following disclaimer in the documentation
**      and/or other materials provided with the distribution.
**   3. Neither the name of STMicroelectronics nor the names of its contributors
**      may be used to endorse or promote products derived from this software
**      without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);    /* end of DTCM RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x5000;      /* required amount of heap  */
_Min_Stack_Size = 0x5000; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
DTCMRAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K
RAM (xrw)      : ORIGIN = 0x24000000, LENGTH = 128K
RAM_D2 (xrw)      : ORIGIN = 0x30000000, LENGTH = 32K
RAM_D3 (xrw)      : ORIGIN = 0x38000000, LENGTH = 16K
ITCMRAM (xrw)      : ORIGIN = 0x00000000, LENGTH = 64K
FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 1024K
}

PHDRS
{
  flash_text PT_LOAD FLAGS(5);   /* R + X */
  flash_rodata PT_LOAD FLAGS(4); /* R */
  ram_data PT_LOAD FLAGS(6);     /* R + W */
  ram_bss PT_LOAD FLAGS(6);      /* R + W */
  dtcm_data PT_LOAD FLAGS(6);    /* R + W */
  dtcm_bss PT_LOAD FLAGS(6);     /* R + W */
  ram_d2_data PT_LOAD FLAGS(6);  /* R + W */
  ram_d3_data PT_LOAD FLAGS(6);  /* R + W */
  itcm_text PT_LOAD FLAGS(5);    /* R + X */
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH :flash_text

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH :flash_text

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH :flash_rodata

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH :flash_rodata
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH :flash_rodata

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH :flash_rodata
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH :flash_rodata
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH :flash_rodata

  _siitcm_text = LOADADDR(.itcm_text);

  /* Time-critical code copied from FLASH to ITCM at startup */
  .itcm_text :
  {
    . = ALIGN(4);
    _sitcm_text = .;
    *(.itcm_text)
    *(.itcm_text*)
    . = ALIGN(4);
    _eitcm_text = .;
  } >ITCMRAM AT> FLASH :itcm_text

  _sidtcm_data = LOADADDR(.dtcm_data);

  /* Initialized CPU-only data copied from FLASH to DTCM at startup */
  .dtcm_data :
  {
    . = ALIGN(4);
    _sdtcm_data = .;
    *(.dtcm_data)
    *(.dtcm_data*)
    . = ALIGN(4);
    _edtcm_data = .;
  } >DTCMRAM AT> FLASH :dtcm_data

  /* Zero-initialized CPU-only data in DTCM */
  .dtcm_bss (NOLOAD) :
  {
    . = ALIGN(8);
    _sdtcm_bss = .;
    *(.dtcm_bss)
    *(.dtcm_bss*)
    . = ALIGN(8);
    _edtcm_bss = .;
  } >DTCMRAM :NONE

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH :ram_data

  
  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM :NONE

  /* DMA-safe non-cacheable buffers in D2 SRAM */
  .dma_buffer (NOLOAD) :
  {
    . = ALIGN(32);
    _sdma_buffer = .;
    *(.dma_buffer)
    *(.dma_buffer*)
    . = ALIGN(32);
    _edma_buffer = .;
  } >RAM_D2 :NONE

  .ram_d2 (NOLOAD) :
  {
    . = ALIGN(4);
    _sram_d2 = .;
    *(.ram_d2)
    *(.ram_d2*)
    . = ALIGN(4);
    _eram_d2 = .;
  } >RAM_D2 :NONE

  .ram_d3 (NOLOAD) :
  {
    . = ALIGN(4);
    _sram_d3 = .;
    *(.ram_d3)
    *(.ram_d3*)
    . = ALIGN(4);
    _eram_d3 = .;
  } >RAM_D3 :NONE

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    PROVIDE ( _heap_start = . );
    . = . + _Min_Heap_Size;
    . = ALIGN(8);
    PROVIDE ( _heap_min_end = . );
  } >RAM :NONE

  PROVIDE ( _heap_end = ORIGIN(RAM) + LENGTH(RAM) );

  ASSERT(_edtcm_bss + _Min_Stack_Size <= ORIGIN(DTCMRAM) + LENGTH(DTCMRAM), "DTCMRAM overflowed by stack/sections")

  

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
