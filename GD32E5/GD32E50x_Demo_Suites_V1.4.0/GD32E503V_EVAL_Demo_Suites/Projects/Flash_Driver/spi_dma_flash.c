#ifndef __SPI_DMA_FLASH_C__
#define __SPI_DMA_FLASH_C__ 

#include "spi_dma_flash.h"

static uint8_t flash_TX_buf[256];
static uint8_t flash_RX_buf[256];


#define BOARD_USE_SPI0

#if defined(BOARD_USE_SPI0)

    #define SPI_X     SPI0

    #define RCU_GPIO_X RCU_GPIOA
    #define RCU_GPIO_CS RCU_GPIOE
    #define RCU_SPI_X RCU_SPI0

    #define SPI_GPIO GPIOA
    #define SPI_CS_GPIO GPIOE
    #define CS_PIN GPIO_PIN_3
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


#define NOR_CS_LOW()    gpio_bit_reset(SPI_CS_GPIO, CS_PIN)
#define NOR_CS_HIGH()   gpio_bit_set(SPI_CS_GPIO, CS_PIN)

void SPI_X_DMA_X_Init(void){

    spi_parameter_struct spi_init_struct;

    rcu_periph_clock_enable(RCU_GPIO_X);
    rcu_periph_clock_enable(RCU_GPIO_CS);
    rcu_periph_clock_enable(RCU_SPI_X);

    gpio_init(SPI_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, SCK_PIN | MOSI_PIN);
    gpio_init(SPI_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, MISO_PIN);
    gpio_init(SPI_CS_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, CS_PIN);

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

static uint8_t nor_write_sr(const uint8_t data,const uint8_t cmd)
{
    if (cmd!=0x01 || cmd!=0x11 || cmd!=0x31) return 0;
    
    nor_wait_busy();
    nor_write_enable();          // 06h
    NOR_CS_LOW();
    spi2rw(cmd);
    spi2rw(data);
    NOR_CS_HIGH();
    nor_wait_busy();
    return 1;
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

    dma_transfer_number_config(DMA_X, DMA_CH_TX, 128);
    dma_channel_enable(DMA_X,DMA_CH_TX);

    while(!dma_flag_get(DMA_X,DMA_CH_TX,DMA_FLAG_FTF));
    while (spi_i2s_flag_get(SPI_X, SPI_FLAG_TRANS) == SET); // 确保发送缓冲区为空

    dma_flag_clear(DMA_X,DMA_CH_TX, DMA_FLAG_FTF);
	dma_channel_disable(DMA_X,DMA_CH_TX);

    NOR_CS_HIGH();

    nor_wait_busy();

}

void DMA_Read_Test(void){ 
    uint8_t i;
    uint32_t addr = 0x000000;
    NOR_CS_LOW();
    spi2rw(0x03);
    spi2rw((addr>>16)&0xFF);
    spi2rw((addr>>8)&0xFF);
    spi2rw(addr&0xFF);
    dma_channel_disable(DMA_X,DMA_CH_RX);
    spi_dma_enable(SPI_X, SPI_DMA_RECEIVE);
    dma_transfer_number_config(DMA_X, DMA_CH_RX, 128);
    dma_channel_enable(DMA_X,DMA_CH_RX);
    while(!dma_flag_get(DMA_X,DMA_CH_RX,DMA_FLAG_FTF));
    dma_flag_clear(DMA_X,DMA_CH_RX, DMA_FLAG_FTF);
    dma_channel_disable(DMA_X,DMA_CH_RX);
    NOR_CS_HIGH();
    
}

uint32_t NOR_read_ID(void){ 
    uint32_t id;
    NOR_CS_LOW();
    spi2rw(0x9F);
    spi2rw(0x00);
    spi2rw(0x00);
    spi2rw(0x00);
    id=(spi2rw(0xFF)<<16)|(spi2rw(0xFF)<<8)|spi2rw(0xFF);
    spi2rw(0xFF);
    spi2rw(0xFF);
    NOR_CS_HIGH();
	return id;
}

void SFDP_Read(void){ 

    uint32_t addr;

    NOR_CS_LOW();
    
    spi2rw(0x5A);
    spi2rw(0x00);
    spi2rw(0x00);
    spi2rw(0x00);
    spi2rw(0x00);
    spi2rw(0xFF);
    spi2rw(0xFF);
    spi2rw(0xFF);
    spi2rw(0xFF);

    NOR_CS_HIGH();

}



static void flash_read(uint8_t cmd, uint32_t addr, uint8_t *buf, uint32_t len){
    NOR_CS_LOW();
    spi2rw(cmd);
    if (cmd == CMD_SFDP)
    {
        /* code */
        spi2rw((addr>>16)&0xFF);
        spi2rw((addr>>8)&0xFF);
        spi2rw(addr&0xFF);
        spi2rw(0x00);//dummy
    }
    while (len--)
    {
        *buf++ = spi2rw(0xFF);
    }
    NOR_CS_HIGH();

}

/* ----------------  型号反查表  ---------------- */
static const struct {
    uint8_t  manu;
    uint8_t  capacity;
    uint32_t density;
    uint8_t  voltage;
    const char *name;
} dev_table[] = {
    {0xEF, 0x14, 0x01000000, 0, "W25Q80" },
    {0xEF, 0x15, 0x02000000, 0, "W25Q16" },
    {0xEF, 0x16, 0x04000000, 0, "W25Q32" },
    {0xEF, 0x17, 0x08000000, 0, "W25Q64" },
    {0xEF, 0x18, 0x10000000, 0, "W25Q128"},
    {0xEF, 0x19, 0x20000000, 0, "W25Q256"},
    /* 1.8 V 系列 */
    {0xEF, 0x17, 0x08000000, 1, "W25Q64DW"},
    {0xEF, 0x18, 0x10000000, 1, "W25Q128JW"},
};

int flash_9F_decode(flash_info_t *f)
{
    uint8_t hdr[16];
    uint8_t param[64];
    uint8_t jedec[3];
    flash_read(CMD_JEDEC_ID, 0, jedec, sizeof(jedec));

    for (uint32_t i = 0; i < sizeof(dev_table)/sizeof(dev_table[0]); i++){
        if (dev_table[i].manu == jedec[0] && dev_table[i].capacity == jedec[1] && dev_table[i].density == jedec[2]){
            strcpy(f->name, dev_table[i].name);
            f->density = dev_table[i].density;
            return 0;
        }
    }

     return -1;
    // f->manu = jedec[0];
    // f->mem_type = jedec[1];
    // f->capacity = jedec[2];
    // flash_read(CMD_SFDP, 0, hdr, sizeof(hdr));
    // uint32_t sig=hdr[0] | (hdr[1] << 8) | (hdr[2] << 16) | (hdr[3] << 24);
    // if (sig != SFDP_SIGNATURE) return -1;
    // uint32_t param_addr = hdr[4] | (hdr[5] << 8) | (hdr[6] << 16) ;
    // flash_read(CMD_SFDP, 0x80, param, sizeof(param));
    // f->density = param[BFPT_DWORD1_CAPACITY*4] | (param[BFPT_DWORD1_CAPACITY*4 + 1] << 8) | (param[BFPT_DWORD1_CAPACITY*4 + 2] << 16) | (param[BFPT_DWORD1_CAPACITY*4 + 3] << 24);

    // for (uint32_t i = 0; i < sizeof(dev_table)/sizeof(dev_table[0]); i++){
    //     if (dev_table[i].manu == f->manu && dev_table[i].capacity == f->capacity ){
    //         strcpy(f->name, dev_table[i].name);

    //         return 0;
    //     }
    // }
	// 	return -1;
}

#define WPS(a) a&0x04
#define CMP(a) a&0x40
void NOR_Erase(uint8_t cmd, uint32_t addr){ 

    /* 1. 参数校验 */
    if (cmd != 0x20 && cmd != 0x52 && cmd != 0xD8 &&
        cmd != 0x60 && cmd != 0xC7) return;

    if ((cmd != 0x60 && cmd != 0xC7) && (addr >= 0x1000000)) return;

    uint8_t NOR_SR1 = nor_read_sr(0x05);
    uint8_t NOR_SR2 = nor_read_sr(0x35);
    uint8_t NOR_SR3 = nor_read_sr(0x15);

    /* printf SR1 SR2 SR3 */

    /*reset SR1 SR2 SR3 */
//    nor_write_sr(0x00,0x01);
//    nor_write_sr(0x00,0x31);
//    nor_write_sr(0x60,0x11);


//    // if (WPS(NOR_SR3) != 0 && CMP(NOR_SR2) != 0)
//    // {
//    //     /* code */
//    // }
//    

//    nor_write_enable();
//    NOR_CS_LOW();
//    spi2rw(cmd);
//    if (cmd!=0x60 && cmd!=0xC7)
//    {
//        spi2rw((addr>>16)&0xFF);
//        spi2rw((addr>>8)&0xFF);
//        spi2rw(addr&0xFF);
//    }
//    NOR_CS_HIGH();
//    nor_wait_busy();
}




void NOR_Write_Test(void){

//    DMA_Write_Test();
//    DMA_Read_Test();
//			NOR_read_ID();
//        SFDP_Read();
    NOR_Erase(0x60, 0x00000000);

}


#ifndef __NOR_STATE_MACHINE__
#define __NOR_STATE_MACHINE__

typedef enum {
    NOR_STATE_IDLE=0,
    NOR_STATE_READ,
    NOR_STATE_WRITE,
    NOR_STATE_ERASE,
    NOR_STATE_READ_ID,
    NOR_STATE_SFDP,
    NOR_STATE_READ_STATUS,
    NOR_STATE_WRITE_STATUS,
    NOR_STATE_WRITE_ENABLE,

}nor_state_t;



#endif







#endif// __SPI_DMA_FLASH_C__