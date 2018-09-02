/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "ds18b20.h"

#define DS_PIN 26

// Commands
#define STARTCONVO      0x44
#define READSCRATCH     0xBE
#define WRITESCRATCH    0x4E

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1

// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

typedef uint8_t ScratchPad[9];

/**@brief Function for sending one bit to bus.
 */
void ds18b20_send(char bit)
{
    nrf_gpio_cfg_output(DS_PIN);
    nrf_gpio_pin_clear(DS_PIN);

    nrf_delay_us(5);

    if(bit==1)
    {
        nrf_gpio_pin_set(DS_PIN);
    }
    nrf_delay_us(80);
    nrf_gpio_pin_set(DS_PIN);
}


/**@brief Function for reading one bit from bus.
 */
unsigned char ds18b20_read(void)
{
    unsigned char presence=0;
    nrf_gpio_cfg_output(DS_PIN);
    nrf_gpio_pin_clear(DS_PIN);

    nrf_delay_us(2);

    nrf_gpio_pin_set(DS_PIN);;
    nrf_delay_us(15);

    nrf_gpio_cfg_input(DS_PIN,NRF_GPIO_PIN_NOPULL);

    if(nrf_gpio_pin_read(DS_PIN))
    {
        presence = 1;
    }
    else
    {
        presence = 0;
    }
    
    return presence;
}


/**@brief Function for sending one byte to bus.
 */
void ds18b20_send_byte(char data)
{
    unsigned char i;
    unsigned char x;
    for(i=0;i<8;i++)
    {
      x = data>>i;
      x &= 0x01;
      ds18b20_send(x);
    }
    nrf_delay_us(100);
}


/**@brief Function for reading one byte from bus.
 */
unsigned char ds18b20_read_byte(void)
{
    unsigned char i;
    unsigned char data = 0;
    for (i=0;i<8;i++)
    {
        if(ds18b20_read()) data|=0x01<<i;
        nrf_delay_us(15);
    }
    return(data);
}


/**@brief Function for sending reset pulse.
 */
unsigned char ds18b20_reset(void)
{
  unsigned char presence;

  nrf_gpio_cfg_output(DS_PIN);
  nrf_gpio_pin_clear(DS_PIN);

  nrf_delay_us(500);
  nrf_gpio_pin_set(DS_PIN);

  nrf_gpio_cfg_input(DS_PIN,NRF_GPIO_PIN_NOPULL); // usikkert p� pull her. m� sjekkes
  nrf_delay_us(30);


    if(nrf_gpio_pin_read(DS_PIN) == 0)
    {
        presence = 1;
    }
    else
    {
        presence = 0;
    }

    nrf_delay_us(470);

    if(nrf_gpio_pin_read(DS_PIN) == 1)
    {
        presence = 1;
    }
    else
    {
        presence = 0;
    }

  return presence;
}


/**@brief Function for reading temperature.
 */
float ds18b20_get_temp(void)
{
    unsigned char check;
    char temp1=0, temp2=0;

    check=ds18b20_reset();
    if(check)
    {
        ds18b20_send_byte(0xCC);
        ds18b20_send_byte(0x44);
        nrf_delay_ms(600UL);
        check=ds18b20_reset();
        ds18b20_send_byte(0xCC);
        ds18b20_send_byte(0xBE);
        temp1=ds18b20_read_byte();
        temp2=ds18b20_read_byte();
        check=ds18b20_reset();
        float temp=0;
        temp=(float)(temp1+(temp2*256))/16;
        return temp;
    }
      return 0;
}


/**@brief Function for reading bit.
 */
uint8_t OneWire_read_bit(void)
{
    uint8_t r;
    nrf_gpio_cfg_output(DS_PIN);
    nrf_gpio_pin_clear(DS_PIN);
    nrf_delay_us(3);
    nrf_gpio_cfg_input(DS_PIN,NRF_GPIO_PIN_NOPULL);
    nrf_delay_us(10);
    r =nrf_gpio_pin_read(DS_PIN);
    nrf_delay_us(53);
    return r;
}


/**@brief Function for reading.
 */
uint8_t OneWire_read()
{
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	if ( OneWire_read_bit()) r |= bitMask;
    }
    return r;
}


/**@brief Function for reading scratchpad value
 */
void ds18b20_readScratchPad(uint8_t *scratchPad, uint8_t fields)
{
    ds18b20_reset();
    ds18b20_send_byte(0xCC);
    ds18b20_send_byte(READSCRATCH);

    for(uint8_t i=0; i < fields; i++)
    {
        scratchPad[i] = OneWire_read();
    }
    ds18b20_reset();
}


/**@brief Function for request temperature reading
 */
void DS18B20_requestTemperatures(void)
{
    ds18b20_reset();
    ds18b20_send_byte(0xCC);
    ds18b20_send_byte(STARTCONVO);
}


/**@brief Function for reading temperature method 2
 */
float ds18b20_get_temp_method_2(void)
{
    DS18B20_requestTemperatures();
    unsigned char check;

    ScratchPad scratchPad;
    ds18b20_readScratchPad(scratchPad, 2);
    int16_t rawTemperature = (((int16_t)scratchPad[TEMP_MSB]) << 8) | scratchPad[TEMP_LSB];
    float temp = 0.0625 * rawTemperature;

    return temp;
}


/**@brief Function for setting temperature resolution
 */
void ds18b20_setResolution(uint8_t resolution)
{
    ds18b20_reset();
    ds18b20_send_byte(0xCC);
    ds18b20_send_byte(WRITESCRATCH);
    // two dummy values for LOW & HIGH ALARM
    ds18b20_send_byte(0);
    ds18b20_send_byte(100);
    switch (resolution)
    {
        case 12:
            ds18b20_send_byte(TEMP_12_BIT);
            break;

        case 11:
            ds18b20_send_byte(TEMP_11_BIT);
            break;

        case 10:
            ds18b20_send_byte(TEMP_10_BIT);
            break;

        case 9:
        default:
            ds18b20_send_byte(TEMP_9_BIT);
            break;
    }
    ds18b20_reset();
}

