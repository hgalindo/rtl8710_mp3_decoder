1/ This is a port of ESP8266's MP3 decoder (webradio) to RTL8710, RTL8711 and RTL8195 based modules such as RTL-00 and RTL-01.

2/ This is WORK IN PROGESS

Current status:
- It compiles with RTL8711 SDK 3.5 GCC 1.0.0
- Hooks to RTL's wifi library - completed but not tested
- Hooks to RTL's TCP/IP stack, UART, GPIO and I2S have not been updated

3/ How to compile and flash your RTL based module

3/1/ Install SDK RTL8711 version 3.5a GCC 1.0.0
3/2/ git clone this repository to /project folder (where the ameba example is)
3/3/ cd GCC_RELEASE; make; make flash

4/ Hardware

4/1/ (optional) I2S DAC such as TDA1387

```
RTL pin   - I2S signal
----------------------
GPIO_C0   - LRCK
GPIO_C2   - DATA
GPIO_C1   - BCLK
```

4/2/ (optional) SPI RAM such as Microchip 23LC1024 

```
RTL pin   - 23LC1024 pin
------------------------
GPIO0     - /CS (1)
SD_D0     - SO/SI1 (2)
SD_D3     - SIO2 (3) *
gnd       - gnd (4)
SD_D1     - SI/SIO0 (5)
SD_CLK    - SCK (6)
SD_D2     - /HOLD/SIO3 (7) *
3.3V      - VCC (8)

*=optional, may also be connected to Vcc on 23LC1024 side.
```

5/ and, everything else is described on the original ESP8266's mp3 decoder 
espressif/ESP8266_MP3_DECODER

