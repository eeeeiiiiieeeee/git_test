#ifndef __SPI_DMA_FLASH_C__
#define __SPI_DMA_FLASH_C__ 

#include "spi_dma_flash.h"

static uint8_t flash_TX_buf[256];
static uint8_t flash_RX_buf[256];


#define BOARD_USE_SPI2

#if defined(BOARD_USE_SPI0)

    #define SPI_X     SPI0

    #define RCU_GPIO_X RCU_GPIOA
    #define RCU_SPI_X RCU_SPI0

    #define SPI_GPIO GPIOA
    #define CS_PIN GPIO_PIN_4
    #define SCK_PIN GPIO_PIN_5
    #define MISO_PIN GPIO_PIN_6
    #define MOSI_PIN GPIO_PIN_7

    #define RCU_DMA_X RCU_DMA0
    #define DMA_X DMA0
    #define DMA_CH_RX DMA_CH1
    #define DMA_CH_TX DMA_CH2


#elif defined(BOARD_USE_SPI1)

#elif defined(BOARD_USE_SPI2)

    #define SPI_X     SPI2

    #define RCU_GPIO_X RCU_GPIOB
    #define RCU_SPI_X RCU_SPI2

    #define SPI_GPIO GPIOB
    #define CS_PIN GPIO_PIN_6
    #define SCK_PIN GPIO_PIN_3
    #define MISO_PIN GPIO_PIN_4
    #define MOSI_PIN GPIO_PIN_5

    #define RCU_DMA_X RCU_DMA1
    #define DMA_X DMA1
    #define DMA_CH_RX DMA_CH0
    #define DMA_CH_TX DMA_CH1

#else
    #error "Please define one of BOARD_USE_SPI0/1/2 in spi_dma_flash.h"

#endif


#define NOR_CS_LOW()    gpio_bit_reset(SPI_GPIO, CS_PIN)
#define NOR_CS_HIGH()   gpio_bit_set(SPI_GPIO, CS_PIN)

void SPI_X_DMA_X_Init(void){

    spi_parameter_struct spi_init_struct;

    rcu_periph_clock_enable(RCU_GPIO_X);
    rcu_periph_clock_enable(RCU_SPI_X);

    gpio_init(SPI_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, SCK_PIN | MOSI_PIN);
    gpio_init(SPI_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, MISO_PIN);
    gpio_init(SPI_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, CS_PIN);

    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_16;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI_X, &spi_init_struct);

    /* enable SPI0 */
    spi_enable(SPI_X);
    /* set crc polynomial */
    //spi_crc_polynomial_set(SPI_X,7);

    
        /* 共同配置 */

    rcu_periph_clock_enable(RCU_DMA_X);
    dma_parameter_struct dma_init_struct;
    dma_deinit(DMA_X,DMA_CH_RX);
    dma_deinit(DMA_X,DMA_CH_TX);

    dma_init_struct.periph_addr  = (uint32_t)&SPI_DATA(SPI_X);
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.priority     = DMA_PRIORITY_MEDIUM;
    dma_init_struct.number       = 256;          // 默认长度，运行时可改
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    
    /* TX channel */
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr  = (uint32_t)&flash_TX_buf[0];
    dma_init(DMA_X,DMA_CH_TX, &dma_init_struct);
    dma_circulation_disable(DMA_X, DMA_CH_TX); 
    dma_memory_to_memory_disable(DMA_X, DMA_CH_TX);
    //dma_circulation_enable(DMA_X, DMA_CH_RX); 

    //spi_dma_enable(SPI_X, SPI_DMA_TRANSMIT);
    //dma_interrupt_enable(DMA_X,DMA_CH_RX, DMA_INT_FTF);   // 传输完成中断

    /* RX channel */
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr  = (uint32_t)&flash_RX_buf[0];
    dma_init(DMA_X,DMA_CH_RX, &dma_init_struct);
    dma_circulation_disable(DMA_X, DMA_CH_RX); 
    dma_memory_to_memory_disable(DMA_X, DMA_CH_RX);
	NOR_CS_HIGH();

}

/* 阻塞式 1 字节收发 */
static uint8_t spi2rw(uint8_t tx)
{

    while (spi_i2s_flag_get(SPI_X, SPI_FLAG_TBE) == RESET);
    spi_i2s_data_transmit(SPI_X, tx);
    while (spi_i2s_flag_get(SPI_X, SPI_FLAG_TRANS)==SET);
    
    while (spi_i2s_flag_get(SPI_X, SPI_FLAG_RBNE) == RESET);
    return spi_i2s_data_receive(SPI_X);
}


/* 写使能 */
static void nor_write_enable(void)
{
    NOR_CS_LOW();
    spi2rw(0x06);
    //delay_1ms(1);

    NOR_CS_HIGH();
}

/* 阻塞式读状态寄存器 */
static uint8_t nor_read_sr(const uint8_t cmd)
{
    uint8_t sr;
    NOR_CS_LOW();
    spi2rw(cmd);
    sr = spi2rw(0xFF);
    NOR_CS_HIGH();
    return sr;
}


/* 等待空闲 */
static void nor_wait_busy(void)
{
    while (nor_read_sr(0x05) & 0x01);
}

void nor_write_status(const uint8_t data,const uint8_t cmd)
{
    nor_write_enable();          // 06h
    //delay_1ms(2);
    NOR_CS_LOW();
    spi2rw(cmd);                // WRSR
    spi2rw(data);
    NOR_CS_HIGH();
    //delay_1ms(2);
    //nor_wait_busy();             // 必须等待
}


void DMA_Write_Test(void){

    uint8_t i;
    uint32_t addr = 0x000000;
    for(i=0;i<128;i++){
        flash_TX_buf[i] = i;
    }
    nor_write_enable();          // 06h
    NOR_CS_LOW();
    spi2rw(0x02);
    spi2rw((addr>>16)&0xFF);
    spi2rw((addr>>8)&0xFF);
    spi2rw(addr&0xFF);
    // NOR_CS_LOW();
    dma_channel_disable(DMA_X,DMA_CH_TX);
    spi_dma_enable(SPI_X, SPI_DMA_TRANSMIT);
    
    dma_transfer_number_config(DMA_X, DMA_CH_TX, 2);
    dma_channel_enable(DMA_X,DMA_CH_TX);

    while(!dma_flag_get(DMA_X,DMA_CH_TX,DMA_FLAG_FTF));
    
    dma_flag_clear(DMA_X,DMA_CH_TX, DMA_FLAG_FTF);
	dma_channel_disable(DMA_X,DMA_CH_TX);

    while (spi_i2s_flag_get(SPI_X, SPI_FLAG_TRANS) == SET); // 确保发送缓冲区为空

    NOR_CS_HIGH();

    nor_wait_busy();             // 必须等待


}





void NOR_Write_Test(void){

    DMA_Write_Test();
    
}






#endif// __SPI_DMA_FLASH_C__