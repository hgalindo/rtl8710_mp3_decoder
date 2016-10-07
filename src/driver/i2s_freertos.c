/******************************************************************************
 * Copyright 2013-2015 Espressif Systems
 *
 * FileName: i2s_freertos.c
 *
 * Description: I2S output routines for a FreeRTOS system. Uses DMA and a queue
 * to abstract away the nitty-gritty details.
 *
 * Modification history:
 *     2015/06/01, v1.0 File created.
*******************************************************************************/

/*
How does this work? Basically, to get sound, you need to:
- Connect an I2S codec to the I2S pins on the RTL.
- Start up a thread that's going to do the sound output
- Call I2sInit()
- Call I2sSetRate() with the sample rate you want.
- Generate sound and call i2sPushSample() with 32-bit samples.
The 32bit samples basically are 2 16-bit signed values (the analog values for
the left and right channel) concatenated as (Rout<<16)+Lout

I2sPushSample will block when you're sending data too quickly, so you can just
generate and push data as fast as you can and I2sPushSample will regulate the
speed.
*/


#include "rtl_common.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

//RTL's
#include "i2s_api.h"

//ESP's
#include "i2s_reg.h"
#include "slc_register.h"
#include "sdio_slv.h"
#include "i2s_freertos.h"

//We need some defines that aren't in some RTOS SDK versions. Define them here if we can't find them.
#ifndef i2c_bbpll
#define i2c_bbpll                                 0x67
#define i2c_bbpll_en_audio_clock_out            4
#define i2c_bbpll_en_audio_clock_out_msb        7
#define i2c_bbpll_en_audio_clock_out_lsb        7
#define i2c_bbpll_hostid                           4

#define i2c_writeReg_Mask(block, host_id, reg_add, Msb, Lsb, indata)  //sk//rom_i2c_writeReg_Mask(block, host_id, reg_add, Msb, Lsb, indata)
#define i2c_readReg_Mask(block, host_id, reg_add, Msb, Lsb)  rom_i2c_readReg_Mask(block, host_id, reg_add, Msb, Lsb)
#define i2c_writeReg_Mask_def(block, reg_add, indata) \
      i2c_writeReg_Mask(block, block##_hostid,  reg_add,  reg_add##_msb,  reg_add##_lsb,  indata)
#define i2c_readReg_Mask_def(block, reg_add) \
      i2c_readReg_Mask(block, block##_hostid,  reg_add,  reg_add##_msb,  reg_add##_lsb)
#endif
#ifndef ETS_SLC_INUM
#define ETS_SLC_INUM       1
#endif

//RTL's
i2s_t i2s_obj;
u8 i2s_tx_buf[I2S_DMA_PAGE_SIZE*I2S_DMA_PAGE_NUM];
u8 i2s_rx_buf[I2S_DMA_PAGE_SIZE*I2S_DMA_PAGE_NUM];

#include <section_config.h>
int sample_size=139916;

SECTION(".sdram.data")
short sample[]={-1,0};

int curr_cnt=0;

//ESP's
//Pointer to the I2S DMA buffer data
static unsigned int *i2sBuf[I2S_DMA_PAGE_NUM];
//I2S DMA buffer descriptors
static struct sdio_queue i2sBufDesc[I2S_DMA_PAGE_NUM];
//Queue which contains empty DMA buffers
static xQueueHandle dmaQueue;
//DMA underrun counter
static long underrunCnt;

//This routine is called as soon as the DMA routine has something to tell us. All we
//handle here is the RX_EOF_INT status, which indicate the DMA has sent a buffer whose
//descriptor has the 'EOF' field set to 1.
LOCAL void slc_isr(void) {
	portBASE_TYPE HPTaskAwoken=0;
	struct sdio_queue *finishedDesc;
	uint32 slc_intr_status;
	int dummy;

	//Grab int status
	slc_intr_status = READ_PERI_REG(SLC_INT_STATUS);
	//clear all intr flags
	WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);//slc_intr_status);
	if (slc_intr_status & SLC_RX_EOF_INT_ST) {
		//The DMA subsystem is done with this block: Push it on the queue so it can be re-used.
		finishedDesc=(struct sdio_queue*)READ_PERI_REG(SLC_RX_EOF_DES_ADDR);
		if (xQueueIsQueueFullFromISR(dmaQueue)) {
			//All buffers are empty. This means we have an underflow on our hands.
			underrunCnt++;
			//Pop the top off the queue; it's invalid now anyway.
			xQueueReceiveFromISR(dmaQueue, &dummy, &HPTaskAwoken);
		}
		//Dump the buffer on the queue so the rest of the software can fill it.
		xQueueSendFromISR(dmaQueue, (void*)(&finishedDesc->buf_ptr), &HPTaskAwoken);
	}
	//We're done.
	portEND_SWITCHING_ISR(HPTaskAwoken);
}

//RTL's interrupt callback
void test_tx_complete(void *data, char *pbuf)
{
    int *ptx_buf;
    
    i2s_t *obj = (i2s_t *)data;
    static u32 count=0;
    //DBG_8195A_I2S_LVL(VERI_I2S_LVL, "I2S%d %s\n",pI2SDemoHnd->DevNum,__func__);
    count++;
    if ((count&1023) == 1023)
    {
         DBG_8195A_I2S_LVL(VERI_I2S_LVL, ",\n");
    }

    ptx_buf = i2s_get_tx_page(obj);
    //ptx_buf = (int*)pbuf;
    _memcpy((void*)ptx_buf, (void*)&sample[curr_cnt], I2S_DMA_PAGE_SIZE);
	curr_cnt+=(I2S_DMA_PAGE_SIZE/sizeof(short));
	if(curr_cnt >= sample_size*(obj->channel_num==CH_MONO?1:2)) {
		curr_cnt = 0;
    }
    i2s_send_page(obj, (uint32_t*)ptx_buf);
}

void test_rx_complete(void *data, char* pbuf)
{
    i2s_t *obj = (i2s_t *)data;
    int *ptx_buf;

    static u32 count=0;
    count++;
    if ((count&1023) == 1023)
    {
         DBG_8195A_I2S_LVL(VERI_I2S_LVL, ".\n");
    }

    ptx_buf = i2s_get_tx_page(obj);
    _memcpy((void*)ptx_buf, (void*)pbuf, I2S_DMA_PAGE_SIZE);
    i2s_recv_page(obj);    // submit a new page for receive
    i2s_send_page(obj, (uint32_t*)ptx_buf);    // loopback
}

//Initialize I2S subsystem for DMA circular buffer use
void ICACHE_FLASH_ATTR i2sInit() {
	//RTL's I2S init
    int *ptx_buf;
    int i,j;

	i2s_obj.channel_num = CH_STEREO;
	i2s_obj.sampling_rate = SR_44p1KHZ;
	i2s_obj.word_length = WL_16b;
	i2s_obj.direction = I2S_DIR_TXRX; //consider switching to TX only  
	i2s_init(&i2s_obj, I2S_SCLK_PIN, I2S_WS_PIN, I2S_SD_PIN);
    i2s_set_dma_buffer(&i2s_obj, (char*)i2s_tx_buf, (char*)i2s_rx_buf, \
        I2S_DMA_PAGE_NUM, I2S_DMA_PAGE_SIZE);
    i2s_tx_irq_handler(&i2s_obj, (i2s_irq_handler)test_tx_complete, (uint32_t)&i2s_obj);
    i2s_rx_irq_handler(&i2s_obj, (i2s_irq_handler)test_rx_complete, (uint32_t)&i2s_obj);

	//We use a queue to keep track of the DMA buffers that are empty. The ISR will push buffers to the back of the queue,
	//the mp3 decode will pull them from the front and fill them. For ease, the queue will contain *pointers* to the DMA
	//buffers, not the data itself. The queue depth is one smaller than the amount of buffers we have, because there's
	//always a buffer that is being used by the DMA subsystem *right now* and we don't want to be able to write to that
	//simultaneously.
	dmaQueue=xQueueCreate(I2S_DMA_PAGE_NUM-1, sizeof(int*));
	
	i2s_set_param(&i2s_obj, i2s_obj.channel_num, i2s_obj.sampling_rate, WL_16b);
	DBG_8195A("I2S Init\n");
    for (i=0;i<I2S_DMA_PAGE_NUM;i++) {
        ptx_buf = i2s_get_tx_page(&i2s_obj);
        if (ptx_buf) {
            _memcpy((void*)ptx_buf, (void*)&sample[curr_cnt], I2S_DMA_PAGE_SIZE);
            i2s_send_page(&i2s_obj, (uint32_t*)ptx_buf);
            curr_cnt+=(I2S_DMA_PAGE_SIZE/sizeof(short));
            if(curr_cnt >= sample_size*(i2s_obj.channel_num==CH_MONO?1:2)) {
                curr_cnt = 0;
            }
        }
    }

	//ESP's
#if 0
	int x, y;
	
	underrunCnt=0;
	
	//First, take care of the DMA buffers.
	for (y=0; y<I2SDMABUFCNT; y++) {
		//Allocate memory for this DMA sample buffer.
		i2sBuf[y]=malloc(I2SDMABUFLEN*4); //4 bytes = 32*2*4
		//Clear sample buffer. We don't want noise.
		for (x=0; x<I2SDMABUFLEN; x++) {
			i2sBuf[y][x]=0;
		}
	}

	//Initialize DMA buffer descriptors in such a way that they will form a circular
	//buffer.
	for (x=0; x<I2SDMABUFCNT; x++) {
		i2sBufDesc[x].owner=1;
		i2sBufDesc[x].eof=1;
		i2sBufDesc[x].sub_sof=0;
		i2sBufDesc[x].datalen=I2SDMABUFLEN*4;
		i2sBufDesc[x].blocksize=I2SDMABUFLEN*4;
		i2sBufDesc[x].buf_ptr=(uint32_t)&i2sBuf[x][0];
		i2sBufDesc[x].unused=0;
		i2sBufDesc[x].next_link_ptr=(int)((x<(I2SDMABUFCNT-1))?(&i2sBufDesc[x+1]):(&i2sBufDesc[0]));
	}
	
	//Attach the DMA interrupt
	//sk//_xt_isr_attach(ETS_SLC_INUM, slc_isr);
	//enable DMA intr in cpu
	//sk//_xt_isr_unmask(1<<ETS_SLC_INUM);

	//We use a queue to keep track of the DMA buffers that are empty. The ISR will push buffers to the back of the queue,
	//the mp3 decode will pull them from the front and fill them. For ease, the queue will contain *pointers* to the DMA
	//buffers, not the data itself. The queue depth is one smaller than the amount of buffers we have, because there's
	//always a buffer that is being used by the DMA subsystem *right now* and we don't want to be able to write to that
	//simultaneously.
	dmaQueue=xQueueCreate(I2SDMABUFCNT-1, sizeof(int*));

#endif
}


//#define BASEFREQ (160000000L)
#define ABS(x) (((x)>0)?(x):(-(x)))

//Set the I2S sample rate, in HZ
void ICACHE_FLASH_ATTR i2sSetRate(int rate, int lockBitcount) {		
	//(lockBitcount?17:20) - 16+1 or 19+1 bits
	int sample_rate = SR_96KHZ;
	if (rate<=96000 || ABS(rate-96000)<ABS(rate-88200)) sample_rate = SR_96KHZ;
    else if (rate<=88200 || ABS(rate-88200)<ABS(rate-48000)) sample_rate = SR_88p2KHZ;
	else if (rate<=48000 || ABS(rate-48000)<ABS(rate-44100)) sample_rate = SR_48KHZ;
    else if (rate<=44100 || ABS(rate-44100)<ABS(rate-32000)) sample_rate = SR_44p1KHZ;
	else if (rate<=32000 || ABS(rate-32000)<ABS(rate-24000)) sample_rate = SR_32KHZ;
	else if (rate<=24000 || ABS(rate-24000)<ABS(rate-22050)) sample_rate = SR_24KHZ;
	else if (rate<=22050 || ABS(rate-22050)<ABS(rate-16000)) sample_rate = SR_22p05KHZ;
	else if (rate<=16000 || ABS(rate-16000)<ABS(rate-11020)) sample_rate = SR_16KHZ;
	else if (rate<=11020 || ABS(rate-11020)<ABS(rate- 8000)) sample_rate = SR_11p02KHZ;
	else if (rate<= 8000 || ABS(rate- 8000)<ABS(rate- 7350)) sample_rate = SR_8KHZ;
	else sample_rate = SR_7p35KHZ;
	
	i2s_obj.sampling_rate = sample_rate;
    	
	i2s_set_param(&i2s_obj, i2s_obj.channel_num, i2s_obj.sampling_rate, WL_16b);

	DBG_8195A("ReqRate %d Sample Rate %d\n", rate, sample_rate);
	
	// ESP's
	//Find closest divider 

}

//Current DMA buffer we're writing to
static unsigned int *currDMABuff=NULL;
//Current position in that DMA buffer
static int currDMABuffPos=0;


//This routine pushes a single, 32-bit sample to the I2S buffers. Call this at (on average) 
//at least the current sample rate. You can also call it quicker: it will suspend the calling
//thread if the buffer is full and resume when there's room again.
void i2sPushSample(unsigned int sample) {
	//Check if current DMA buffer is full.
	if (currDMABuffPos==I2S_DMA_PAGE_SIZE || currDMABuff==NULL) {
		//We need a new buffer. Pop one from the queue.
		xQueueReceive(dmaQueue, &currDMABuff, portMAX_DELAY);
		currDMABuffPos=0;
	}
	currDMABuff[currDMABuffPos++]=sample;
}


long ICACHE_FLASH_ATTR i2sGetUnderrunCnt() {
	return underrunCnt;
}
