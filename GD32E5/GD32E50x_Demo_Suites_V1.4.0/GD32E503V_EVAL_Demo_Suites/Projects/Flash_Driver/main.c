/*!
    \file    main.c
    \brief   SPI Flash demo

    \version 2024-01-09, V1.4.0, demo for GD32E50x
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "gd32e50x.h"
#include "systick.h"
#include "gd32e503v_eval.h"
#include "spi_dma_flash.h"


#define UART_TX_BUF_SIZE   2048          // 必须是2的幂
#define UART_RX_BUF_SIZE   256

#if (UART_TX_BUF_SIZE & (UART_TX_BUF_SIZE - 1))
#error "UART_TX_BUF_SIZE must be power of two"
#endif
#define UART_TX_MASK       (UART_TX_BUF_SIZE - 1)

/******************* 参数与内部状态 **************************/
static uint8_t  s_tx_buf[UART_TX_BUF_SIZE];
static uint8_t  s_rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t s_tx_head = 0;       /* 写入位置（生产者，单调加） */
static volatile uint16_t s_tx_tail = 0;       /* 读取位置（消费者，单调加） */
static volatile uint8_t  s_dma_busy = 0;      /* 0=空闲，1=DMA 正在搬运 */
static volatile uint16_t s_dma_chunk_len = 0; /* 本次 DMA 发送长度 */

static volatile uint16_t s_rx_tail = 0;



/* 简易关中断保护 */
static inline uint32_t irq_lock(void){ uint32_t primask = __get_PRIMASK(); __disable_irq(); return primask; }
static inline void irq_unlock(uint32_t key){ if(!key) __enable_irq(); }



/* 当前已用字节数（环形缓冲区中待发送的数据量） */
static inline uint16_t ring_used(void){
    return (uint16_t)(s_tx_head - s_tx_tail);
}

/* 从 tail 到缓冲区末尾的连续空间 */
static inline uint16_t cont_to_end_from_tail(void){
    return (uint16_t)(UART_TX_BUF_SIZE - (s_tx_tail & UART_TX_MASK));
}

/* 写入环形缓冲区：不做阻塞，尽量写入；返回实际写入字节数 */
static inline uint16_t ring_buf_write(const uint8_t *data, uint16_t len){
    uint32_t key = irq_lock();
    uint16_t head = s_tx_head;
    uint16_t used = (uint16_t)(head - s_tx_tail);
    uint16_t free_space = (uint16_t)(UART_TX_BUF_SIZE - used);
    uint16_t to_write = (len <= free_space) ? len : free_space; // 防止覆盖未发数据

    uint16_t idx = head & UART_TX_MASK;
    uint16_t first = UART_TX_BUF_SIZE - idx;            // 末尾还能写多少
    if(first > to_write) first = to_write;
    memcpy(&s_tx_buf[idx], data, first);
    if(to_write > first){
        memcpy(&s_tx_buf[0], data + first, to_write - first);
    }
    s_tx_head = head + to_write;
    irq_unlock(key);
    return to_write;
}

/* 计算本次 DMA 可发送长度（严格不超过已用字节数，且不跨越缓冲区末尾） */
static inline uint16_t This_Time_TX_len(void){
    uint16_t used = ring_used();
    if(used == 0) return 0;
    uint16_t cont = cont_to_end_from_tail();
    return (used < cont) ? used : cont;
}

/* 启动一次 DMA 发送 */
static inline uint16_t DMA_TX_start(void){
    if(s_dma_busy) return 0;  // 避免重复启动

    uint16_t len = This_Time_TX_len();
    if(len == 0) return 0;

    dma_channel_disable(DMA0, DMA_CH3);

    s_dma_chunk_len = len;
    dma_memory_address_config(DMA0, DMA_CH3,
        (uint32_t)&s_tx_buf[s_tx_tail & UART_TX_MASK]);
    dma_transfer_number_config(DMA0, DMA_CH3, len);

    dma_channel_enable(DMA0, DMA_CH3);
    s_dma_busy = 1;
    return len;
}

void strip_crlf(char *str) {
    char *p = str;
    while (*p) {
        if (*p == '\r' || *p == '\n') {
            *p = '\0';
            break;
        }
        p++;
    }
}

/* 类似 printf 的接口，非阻塞，立即返回（尽量写入） */
void uprintf(const char *fmt, ...)
{
    static uint8_t tmp[128];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf((char *)tmp, sizeof(tmp), fmt, ap);
    va_end(ap);

    if(len > 0){
        (void)ring_buf_write(tmp, (uint16_t)len);  // 可以根据返回值判断是否丢字节
        DMA_TX_start();
    }
}

/* UART + DMA 初始化 */
void uart_dma_init(void)
{
    /* 时钟 */
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_DMA0);

    /* GPIO */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);        // TX
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10); // RX

    /* USART 参数 */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

    /* ---------------- TX DMA ---------------- */
    dma_parameter_struct dma_tx;
    dma_deinit(DMA0, DMA_CH3);
    dma_tx.periph_addr  = (uint32_t)&USART_DATA(USART0);
    dma_tx.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_tx.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_tx.memory_addr  = (uint32_t)&s_tx_buf[0];
    dma_tx.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_tx.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_tx.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_tx.number       = 1;  // 初值无所谓，真正发送前会重配
    dma_tx.priority     = DMA_PRIORITY_MEDIUM;
    dma_init(DMA0, DMA_CH3, &dma_tx);

    dma_interrupt_enable(DMA0, DMA_CH3, DMA_INT_FTF);
    nvic_irq_enable(DMA0_Channel3_IRQn, 2, 0);

    /* 允许 USART0 触发 DMA 发送 */
    usart_dma_transmit_config(USART0, USART_TRANSMIT_DMA_ENABLE);

    dma_parameter_struct dma_rx;
    dma_deinit(DMA0, DMA_CH4);
    dma_rx.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_rx.memory_addr  = (uint32_t)&s_rx_buf[0];
    dma_rx.periph_addr  = (uint32_t)&USART_DATA(USART0);
    dma_rx.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_rx.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_rx.periph_inc  = DMA_PERIPH_INCREASE_DISABLE;
    dma_rx.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_rx.number = UART_RX_BUF_SIZE;
    dma_rx.priority = DMA_PRIORITY_MEDIUM;
    dma_init(DMA0, DMA_CH4, &dma_rx);

    //dma_circulation_enable(DMA0, DMA_CH4);
    dma_channel_enable(DMA0, DMA_CH4);
    usart_dma_receive_config(USART0, USART_RECEIVE_DMA_ENABLE);

    /*启用 USART IDLE 中断*/
    usart_interrupt_enable(USART0, USART_INT_IDLE);
    nvic_irq_enable(USART0_IRQn, 1, 0);
}

flash_info_t flash_info;

int main(void)
{
    systick_config();
    uart_dma_init();

    uprintf("System start!\r\n");
	//unsigned char test[256];
    SPI_X_DMA_X_Init();

    while(1){

        
        //NOR_Write_Test();
        //flash_9F_decode(&flash_info);
        //uprintf("%s\n",flash_info.name);
        NOR_Write_Test();
        delay_1ms(100000);
    
    // 填充数组，存储0到255的十六进制值
    // for (int i = 0; i < 256; i++) {
	// 				test[i] = i; // 整数在内存中本质上是二进制，这里直接赋值，打印时用十六进制格式即可
	// 				uprintf("0x%02X ", test[i]);
	// 		}
			
	// 			uprintf("\n");
    //     delay_1ms(1000);
    }
}

/* DMA 发送完成中断 */
void DMA0_Channel3_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_FTF))
    {
        dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_FTF);

        /* 本次 DMA 完成，更新 tail */
        s_tx_tail = (uint16_t)(s_tx_tail + s_dma_chunk_len);

        s_dma_busy = 0;

        /* 如果还有数据，继续启动 */
        (void)DMA_TX_start();
    }
}

void USART0_IRQHandler(void){

    if(usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE)){

        usart_interrupt_flag_clear(USART0, USART_INT_FLAG_IDLE);

        uint16_t len = (uint16_t)(UART_RX_BUF_SIZE - dma_transfer_number_get(DMA0, DMA_CH4));

        s_rx_buf[len]='\0';
         // 去掉结尾的 \r\n
        strip_crlf((char*)s_rx_buf);

        if (strcmp((char*)s_rx_buf, "write") == 0) {
            uprintf("write\n");
        } else if (strcmp((char*)s_rx_buf, "read") == 0) {
            uprintf("read\n");
        }

        // 重新启动 DMA 接收
        dma_channel_disable(DMA0, DMA_CH4);
        dma_transfer_number_config(DMA0, DMA_CH4, UART_RX_BUF_SIZE);
        dma_channel_enable(DMA0, DMA_CH4);

        // if (cur != s_rx_tail) {
        //     if (cur > s_rx_tail) {
        //         /* 未跨界：直接处理 [s_rx_tail, cur) */
        //         uint16_t chunk = (uint16_t)(cur - s_rx_tail);
        //         ring_buf_write(&s_rx_buf[s_rx_tail], chunk); // 回显到 TX
        //     } else {
        //         /* 跨界：先处理尾部 [s_rx_tail, end)，再处理头部 [0, cur) */
        //         uint16_t tail_chunk = (uint16_t)(UART_RX_BUF_SIZE - s_rx_tail);
        //         ring_buf_write(&s_rx_buf[s_rx_tail], tail_chunk);
        //         if (cur > 0) {
        //             ring_buf_write(&s_rx_buf[0], cur);
        //         }
        //     }
        //     s_rx_tail = cur;   /* 更新“已处理位置” */
        //     DMA_TX_start();    /* 触发发送（若空闲） */
        // }
    
    }
}
