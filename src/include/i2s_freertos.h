#ifndef _I2S_FREERTOS_H_
#define _I2S_FREERTOS_H_

//RTL's
#define I2S_DMA_PAGE_SIZE	768   // 2 ~ 4096
#define I2S_DMA_PAGE_NUM    4   // Vaild number is 2~4

#define SAMPLE_FILE
#define SAMPLE_FILE_RATE 44100
#define SAMPLE_FILE_CHNUM 2

#define I2S_SCLK_PIN            PC_1
#define I2S_WS_PIN              PC_0
#define I2S_SD_PIN              PC_2


//ESP's Parameters for the I2S DMA behaviour
#define I2SDMABUFCNT (14)			//Number of buffers in the I2S circular buffer
#define I2SDMABUFLEN (32*2)		//Length of one buffer, in 32-bit words.


void ICACHE_FLASH_ATTR i2sInit();
void i2sSetRate(int rate, int lockBitcount);
void i2sPushSample(unsigned int sample);
long ICACHE_FLASH_ATTR i2sGetUnderrunCnt();


#endif
