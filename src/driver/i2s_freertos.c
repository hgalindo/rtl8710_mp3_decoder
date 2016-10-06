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

//ESP's
//Pointer to the I2S DMA buffer data
static unsigned int *i2sBuf[I2SDMABUFCNT];
//I2S DMA buffer descriptors
static struct sdio_queue i2sBufDesc[I2SDMABUFCNT];
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
#if defined(SAMPLE_FILE)	
    _memcpy((void*)ptx_buf, (void*)&sample[curr_cnt], I2S_DMA_PAGE_SIZE);
	curr_cnt+=(I2S_DMA_PAGE_SIZE/sizeof(short));
	if(curr_cnt >= sample_size*(obj->channel_num==CH_MONO?1:2)) {
		curr_cnt = 0;
    }
#else
	if(obj->word_length == WL_16b){
		gen_sound_sample16((short*)ptx_buf, I2S_DMA_PAGE_SIZE/sizeof(short), obj->channel_num==CH_MONO?1:2);
	}else{
		gen_sound_sample24((int*)ptx_buf, I2S_DMA_PAGE_SIZE/sizeof(int), obj->channel_num==CH_MONO?1:2);
	}
#endif
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

	i2s_set_param(&i2s_obj,SAMPLE_FILE_CHNUM,SAMPLE_FILE_RATE,WL_16b);
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
	int x, y;
	
	underrunCnt=0;
	
	//First, take care of the DMA buffers.
	for (y=0; y<I2SDMABUFCNT; y++) {
		//Allocate memory for this DMA sample buffer.
		i2sBuf[y]=malloc(I2SDMABUFLEN*4);
		//Clear sample buffer. We don't want noise.
		for (x=0; x<I2SDMABUFLEN; x++) {
			i2sBuf[y][x]=0;
		}
	}

	//Reset DMA
	SET_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST|SLC_TXLINK_RST);
	CLEAR_PERI_REG_MASK(SLC_CONF0, SLC_RXLINK_RST|SLC_TXLINK_RST);

	//Clear DMA int flags
	SET_PERI_REG_MASK(SLC_INT_CLR,  0xffffffff);
	CLEAR_PERI_REG_MASK(SLC_INT_CLR,  0xffffffff);

	//Enable and configure DMA
	CLEAR_PERI_REG_MASK(SLC_CONF0, (SLC_MODE<<SLC_MODE_S));
	SET_PERI_REG_MASK(SLC_CONF0,(1<<SLC_MODE_S));
	SET_PERI_REG_MASK(SLC_RX_DSCR_CONF,SLC_INFOR_NO_REPLACE|SLC_TOKEN_NO_REPLACE);
	CLEAR_PERI_REG_MASK(SLC_RX_DSCR_CONF, SLC_RX_FILL_EN|SLC_RX_EOF_MODE | SLC_RX_FILL_MODE);


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
	
	//Feed dma the 1st buffer desc addr
	//To send data to the I2S subsystem, counter-intuitively we use the RXLINK part, not the TXLINK as you might
	//expect. The TXLINK part still needs a valid DMA descriptor, even if it's unused: the DMA engine will throw
	//an error at us otherwise. Just feed it any random descriptor.
	CLEAR_PERI_REG_MASK(SLC_TX_LINK,SLC_TXLINK_DESCADDR_MASK);
	SET_PERI_REG_MASK(SLC_TX_LINK, ((uint32)&i2sBufDesc[1]) & SLC_TXLINK_DESCADDR_MASK); //any random desc is OK, we don't use TX but it needs something valid
	CLEAR_PERI_REG_MASK(SLC_RX_LINK,SLC_RXLINK_DESCADDR_MASK);
	SET_PERI_REG_MASK(SLC_RX_LINK, ((uint32)&i2sBufDesc[0]) & SLC_RXLINK_DESCADDR_MASK);

	//Attach the DMA interrupt
	//sk//_xt_isr_attach(ETS_SLC_INUM, slc_isr);
	//Enable DMA operation intr
	WRITE_PERI_REG(SLC_INT_ENA,  SLC_RX_EOF_INT_ENA);
	//clear any interrupt flags that are set
	WRITE_PERI_REG(SLC_INT_CLR, 0xffffffff);
	///enable DMA intr in cpu
	///sk//_xt_isr_unmask(1<<ETS_SLC_INUM);

	//We use a queue to keep track of the DMA buffers that are empty. The ISR will push buffers to the back of the queue,
	//the mp3 decode will pull them from the front and fill them. For ease, the queue will contain *pointers* to the DMA
	//buffers, not the data itself. The queue depth is one smaller than the amount of buffers we have, because there's
	//always a buffer that is being used by the DMA subsystem *right now* and we don't want to be able to write to that
	//simultaneously.
	dmaQueue=xQueueCreate(I2SDMABUFCNT-1, sizeof(int*));

	//Start transmission
	SET_PERI_REG_MASK(SLC_TX_LINK, SLC_TXLINK_START);
	SET_PERI_REG_MASK(SLC_RX_LINK, SLC_RXLINK_START);

//----

	//Init pins to i2s functions
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_I2SO_DATA);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_I2SO_WS);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_I2SO_BCK);

	//Enable clock to i2s subsystem
	i2c_writeReg_Mask_def(i2c_bbpll, i2c_bbpll_en_audio_clock_out, 1);

	//Reset I2S subsystem
	CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);
	SET_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);
	CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);

	//Select 16bits per channel (FIFO_MOD=0), no DMA access (FIFO only)
	CLEAR_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN|(I2S_I2S_RX_FIFO_MOD<<I2S_I2S_RX_FIFO_MOD_S)|(I2S_I2S_TX_FIFO_MOD<<I2S_I2S_TX_FIFO_MOD_S));
	//Enable DMA in i2s subsystem
	SET_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN);

	//tx/rx binaureal
	CLEAR_PERI_REG_MASK(I2SCONF_CHAN, (I2S_TX_CHAN_MOD<<I2S_TX_CHAN_MOD_S)|(I2S_RX_CHAN_MOD<<I2S_RX_CHAN_MOD_S));

	//Clear int
	SET_PERI_REG_MASK(I2SINT_CLR,   I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
			I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);
	CLEAR_PERI_REG_MASK(I2SINT_CLR, I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
			I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);

	//trans master&rece slave,MSB shift,right_first,msb right
	CLEAR_PERI_REG_MASK(I2SCONF, I2S_TRANS_SLAVE_MOD|
						(I2S_BITS_MOD<<I2S_BITS_MOD_S)|
						(I2S_BCK_DIV_NUM <<I2S_BCK_DIV_NUM_S)|
						(I2S_CLKM_DIV_NUM<<I2S_CLKM_DIV_NUM_S));
	SET_PERI_REG_MASK(I2SCONF, I2S_RIGHT_FIRST|I2S_MSB_RIGHT|I2S_RECE_SLAVE_MOD|
						I2S_RECE_MSB_SHIFT|I2S_TRANS_MSB_SHIFT|
						((16&I2S_BCK_DIV_NUM )<<I2S_BCK_DIV_NUM_S)|
						((7&I2S_CLKM_DIV_NUM)<<I2S_CLKM_DIV_NUM_S));


	//No idea if ints are needed...
	//clear int
	SET_PERI_REG_MASK(I2SINT_CLR,   I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
			I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);
	CLEAR_PERI_REG_MASK(I2SINT_CLR,   I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
			I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);
	//enable int
	SET_PERI_REG_MASK(I2SINT_ENA,   I2S_I2S_TX_REMPTY_INT_ENA|I2S_I2S_TX_WFULL_INT_ENA|
	I2S_I2S_RX_REMPTY_INT_ENA|I2S_I2S_TX_PUT_DATA_INT_ENA|I2S_I2S_RX_TAKE_DATA_INT_ENA);

	//Start transmission
	SET_PERI_REG_MASK(I2SCONF,I2S_I2S_TX_START);
}


#define BASEFREQ (160000000L)
#define ABS(x) (((x)>0)?(x):(-(x)))

//Set the I2S sample rate, in HZ
void ICACHE_FLASH_ATTR i2sSetRate(int rate, int lockBitcount) {
	//Find closest divider 
	int bestclkmdiv, bestbckdiv, bestbits, bestfreq=0;
	int tstfreq;
	int bckdiv, clkmdiv, bits;
	/*
		CLK_I2S = 160MHz / I2S_CLKM_DIV_NUM
		BCLK = CLK_I2S / I2S_BCK_DIV_NUM
		WS = BCLK/ 2 / (16 + I2S_BITS_MOD)
		Note that I2S_CLKM_DIV_NUM must be >5 for I2S data
		I2S_CLKM_DIV_NUM - 5-63
		I2S_BCK_DIV_NUM - 2-63
		
		We also have the option to send out more than 2x16 bit per sample. Most I2S codecs will
		ignore the extra bits and in the case of the 'fake' PWM/delta-sigma outputs, they will just lower the output
		voltage a bit, so we add them when it makes sense. Some of them, however, won't accept it, that's
		why we have the option not to do this.
	*/
	for (bckdiv=2; bckdiv<64; bckdiv++) {
		for (clkmdiv=5; clkmdiv<64; clkmdiv++) {
			for (bits=16; bits<(lockBitcount?17:20); bits++) {
				tstfreq=BASEFREQ/(bckdiv*clkmdiv*bits*2);
				if (ABS(rate-tstfreq)<ABS(rate-bestfreq)) {
					bestfreq=tstfreq;
					bestclkmdiv=clkmdiv;
					bestbckdiv=bckdiv;
					bestbits=bits;
				}
			}
		}
	}

	printf("ReqRate %d MDiv %d BckDiv %d Bits %d  Frq %d\n", 
		rate, bestclkmdiv, bestbckdiv, bestbits, (int)(BASEFREQ/(bckdiv*clkmdiv*bits*2)));

	CLEAR_PERI_REG_MASK(I2SCONF, I2S_TRANS_SLAVE_MOD|
						(I2S_BITS_MOD<<I2S_BITS_MOD_S)|
						(I2S_BCK_DIV_NUM <<I2S_BCK_DIV_NUM_S)|
						(I2S_CLKM_DIV_NUM<<I2S_CLKM_DIV_NUM_S));
	SET_PERI_REG_MASK(I2SCONF, I2S_RIGHT_FIRST|I2S_MSB_RIGHT|I2S_RECE_SLAVE_MOD|
						I2S_RECE_MSB_SHIFT|I2S_TRANS_MSB_SHIFT|
						((bestbits-16)<<I2S_BITS_MOD_S)|
						(((bestbckdiv)&I2S_BCK_DIV_NUM )<<I2S_BCK_DIV_NUM_S)|
						(((bestclkmdiv)&I2S_CLKM_DIV_NUM)<<I2S_CLKM_DIV_NUM_S));
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
	if (currDMABuffPos==I2SDMABUFLEN || currDMABuff==NULL) {
		//We need a new buffer. Pop one from the queue.
		xQueueReceive(dmaQueue, &currDMABuff, portMAX_DELAY);
		currDMABuffPos=0;
	}
	currDMABuff[currDMABuffPos++]=sample;
}


long ICACHE_FLASH_ATTR i2sGetUnderrunCnt() {
	return underrunCnt;
}
