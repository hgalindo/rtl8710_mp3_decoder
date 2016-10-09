#ifndef _I2S_FREERTOS_H_
#define _I2S_FREERTOS_H_

//RTL's
#define I2S_DMA_PAGE_SIZE	256   // 2 ~ 4096, MP3 demo was 768
#define I2S_DMA_PAGE_NUM    4   // Valid number is 2~4

#define I2S_SCLK_PIN            PC_1
#define I2S_WS_PIN              PC_0
#define I2S_SD_PIN              PC_2

void ICACHE_FLASH_ATTR i2sInit();
void i2sSetRate(int rate, int lockBitcount);
void i2sPushSample(unsigned int sample);
long ICACHE_FLASH_ATTR i2sGetUnderrunCnt();


#endif
