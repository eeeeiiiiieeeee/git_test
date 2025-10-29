#ifndef __SPI_DMA_FLASH_H__
#define __SPI_DMA_FLASH_H__ 

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "gd32e50x.h"
#include "systick.h"
#include "gd32e503v_eval.h"

#define WRITE_ENABLE        0x06    /* 写使能 */
#define WRITE_DISABLE       0x04    /* 写禁止 */

#define READ_SR1            0x05    /* 读状态寄存器 */
#define READ_SR2            0x35
#define READ_SR3            0x15

#define WRITE_SR1           0x01    /* 写状态寄存器 */
#define WRITE_SR2           0x31
#define WRITE_SR3           0x11

#define READ_DATA           0x03    /* 普通读 */
#define FAST_READ           0x0B    /* 快速读 */
#define PAGE_PROGRAM        0x02    /* 页编程 */
#define SECTOR_ERASE_4K     0x20    /* 4K 扇区擦除 */
#define BLOCK_ERASE_32K     0x52    /* 32K 块擦除 */
#define BLOCK_ERASE_64K     0xD8    /* 64K 块擦除 */
#define CHIP_ERASE          0xC7    /* 整片擦除 */
#define READ_JEDEC_ID       0x9F    /* 读 JEDEC ID */
#define READ_UNIQUE_ID      0x4B    /* 读 Unique ID */
#define POWER_DOWN          0xB9    /* 进入掉电模式 */
#define RELEASE_POWER_DOWN  0xAB    /* 释放掉电模式 */
#define ENABLE_RESET        0x66    /* 使能复位 */
#define RESET_DEVICE        0x99    /* 复位芯片 */


#define NOR_FLASH_PAGE_SIZE      256U
#define NOR_FLASH_PAGE_MASK      (NOR_FLASH_PAGE_SIZE - 1U)   /* 0xFF */

/* 当前地址所在页号（从 0 开始） */
#define FLASH_PAGE_OF(addr)      ((addr) / NOR_FLASH_PAGE_SIZE)

/* 当前地址所在页的页头（起始）地址 */
#define FLASH_PAGE_HEAD(addr)    ((addr) & ~NOR_FLASH_PAGE_MASK)

/* ----------------  常量定义  ---------------- */
#define CMD_JEDEC_ID    0x9F
#define CMD_SFDP        0x5A
#define SFDP_SIGNATURE  0x50444653   /* "SFDP" */

/* SFDP 基本参数表头偏移 */
#define SFDP_PARAM_HEADER_OFFSET  8
#define SFDP_PARAM_LEN            8     /* 每个 header 8 B */

/* JEDEC Basic Flash Parameter Table 偏移（位于 Parameter Header 中） */
#define BFPT_DWORD1_CAPACITY      1     /* 第 1 个 DWORD，容量字段 */
#define BFPT_DWORD1_VOLTAGE       7     /* 第 7 个 DWORD，供电电压 */



/* ----------------  结构体  ---------------- */
typedef struct {
    uint8_t  manu;      /* Manufacturer ID */
    uint8_t  mem_type;  /* Memory Type */
    uint8_t  capacity;  /* Device ID 一字节容量代号 */
    uint32_t density;   /* 真实容量，单位 bit */
    uint8_t  voltage;   /* 0=3V, 1=1.8V, 2=2.5V … */
    char     name[16];  /* 最终型号字符串 */
} flash_info_t;



void SPI_X_DMA_X_Init(void);
void NOR_Write_Test(void);
int flash_9F_decode(flash_info_t *f);

#endif// __SPI_DMA_FLASH_H__