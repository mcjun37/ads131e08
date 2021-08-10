/*
 * Copyright 2021 Minchul Jun (mcjun37@naver.com).  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY MINCHUL JUN ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL MINCHUL JUN OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of Minchul Jun.
 *
 */

#include "ADS131E08.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f7xx_it.h"

_adcConfE* adcConfE0 = NULL;
_adcConfE* adcConfE1 = NULL;

void ADS131E08_control_start_signal(_signalState onOff);
void ADS131E08_control_reset_signal(_signalState onOff);
void ADS131E08_control_cs_signal(_adcType adcType, _signalState onOff);
GPIO_PinState ADS131E08_read_cs_signal(_adcType adcType);
HAL_StatusTypeDef ADS131E08_startup(_adcType adcType);
HAL_StatusTypeDef ADS131E08_send_command(_adcConfE* adcConfE, uint16_t cmd, uint16_t addr, uint8_t regs, uint16_t regNo);
void ADS131E08_set_default_register(_adcConfE* adcConfE);

void ADS131E08_init(_adcType adcType, SPI_HandleTypeDef* hspi)
{

    if(adcType == ADS131E08_ADC20)
    {
        adcConfE0                  = (_adcConfE*)malloc(sizeof(_adcConfE));
        memset(adcConfE0, 0, sizeof(_adcConfE));
        adcConfE0->type            = adcType;
        adcConfE0->stat            = ADS131E08_INIT;
        adcConfE0->hspi            = hspi;
        adcConfE0->nReset.port     = SPI1_NRESET_GPIO_Port;
        adcConfE0->nReset.pin      = SPI1_NRESET_Pin;
        adcConfE0->cs.port         = SPI1_CS0_GPIO_Port;
        adcConfE0->cs.pin          = SPI1_CS0_Pin;
        adcConfE0->nDrdy.port      = SPI1_INT_GPIO_Port;
        adcConfE0->nDrdy.pin       = SPI1_INT_Pin;
        adcConfE0->start.port      = SPI1_START_GPIO_Port;
        adcConfE0->start.pin       = SPI1_START_Pin;
        adcConfE0->bufLen          = E08_WORDS_IN_FRAME * E08_WORD_LENGTH + 1;      // 9 Words x 24 Bits = 216 Bits (27 Bytes) + 1 (command byte)
        adcConfE0->rxBuf           = (uint8_t*)malloc(adcConfE0->bufLen);
        memset(adcConfE0->rxBuf, 0, adcConfE0->bufLen);
        adcConfE0->txBuf           = (uint8_t*)malloc(adcConfE0->bufLen);

        ADS131E08_set_default_register(adcConfE0);
    }
    else if(adcType == ADS131E08_ADC21)
    {
        adcConfE1                  = (_adcConfE*)malloc(sizeof(_adcConfE));
        memset(adcConfE1, 0, sizeof(_adcConfE));
        adcConfE1->type            = adcType;
        adcConfE1->stat            = ADS131E08_INIT;
        adcConfE1->hspi            = hspi;
        adcConfE1->nReset.port     = SPI1_NRESET_GPIO_Port;
        adcConfE1->nReset.pin      = SPI1_NRESET_Pin;
        adcConfE1->cs.port         = SPI1_CS1_GPIO_Port;
        adcConfE1->cs.pin          = SPI1_CS1_Pin;
        adcConfE1->nDrdy.port      = SPI1_INT_GPIO_Port;
        adcConfE1->nDrdy.pin       = SPI1_INT_Pin;
        adcConfE1->start.port      = SPI1_START_GPIO_Port;
        adcConfE1->start.pin       = SPI1_START_Pin;
        adcConfE1->bufLen          = E08_WORDS_IN_FRAME * E08_WORD_LENGTH;      // 9 Words x 24 Bits
        adcConfE1->rxBuf           = (uint8_t*)malloc(adcConfE1->bufLen);
        memset(adcConfE1->rxBuf, 0, adcConfE1->bufLen);
        adcConfE1->txBuf           = (uint8_t*)malloc(adcConfE1->bufLen);

        ADS131E08_set_default_register(adcConfE1);
    }
    else 
    {
        // DO NOTHING
    }

    return;
}

SPI_HandleTypeDef* ADS131E08_is_spi_type(_adcType adcType)
{
    if(adcType == ADS131E08_ADC20)
    {
        return adcConfE0->hspi;
    }
    else if(adcType == ADS131E08_ADC21)
    {
        return adcConfE1->hspi;
    }      
    else
    {
        return NULL;
    }
}

uint8_t* ADS131E08_is_rxbuf(_adcType adcType)
{
    if(adcType == ADS131E08_ADC20)
    {
        return adcConfE0->rxBuf;
    }
    else if(adcType == ADS131E08_ADC21)
    {
        return adcConfE1->rxBuf;
    }     
    else
    {
        return NULL;
    }
}

_ADS131E08_stat ADS131E08_is_adc_status(_adcType adcType)
{
    if(adcType == ADS131E08_ADC20)
    {
        return adcConfE0->stat;
    }
    else if(adcType == ADS131E08_ADC21)
    {
        return adcConfE1->stat;
    }    
    else
    {
        return 0;
    }
}

void ADS131E08_set_adc_status(_adcType adcType, _ADS131E08_stat stat)
{
    if(adcType == ADS131E08_ADC20)
    {
        adcConfE0->stat  = stat;
    }
    else if(adcType == ADS131E08_ADC21)
    {
        adcConfE1->stat = stat;
    }
    else
    {
        // DO NOTHING
    }
    return;
}

uint8_t ADS131E08_is_buf_len(_adcType adcType)
{
    if(adcType == ADS131E08_ADC20)
    {
        return adcConfE1->bufLen;
    }
    else if(adcType == ADS131E08_ADC21)
    {
        return adcConfE1->bufLen;
    }        
    else
    {
        return 0;
    }
}

HAL_StatusTypeDef ADS131E08_send_command(_adcConfE* adcConfE, uint16_t cmd, uint16_t addr, uint8_t regs, uint16_t regNo)
{
    const uint16_t cmdLen = 2;        // Command byte length (2 Bytes)
    HAL_StatusTypeDef result = HAL_OK;

    memset(adcConfE->txBuf, 0, adcConfE->bufLen);
    switch(cmd)
    {
        case ADS131E08_CMD_WAKEUP:
        case ADS131E08_CMD_STANDBY:
        case ADS131E08_CMD_RESET:
        case ADS131E08_CMD_RDATAC:
        case ADS131E08_CMD_SDATAC:
        case ADS131E08_CMD_RDATA:
            adcConfE->txBuf[0] = ADS131E08_UPPER(cmd);
            adcConfE->txBuf[1] = ADS131E08_LOWER(cmd);
            adcConfE->txBuf[2] = 0;
            adcConfE->txBuf[3] = 0;
            adcConfE->command = cmd;
            adcConfE->regAddr = 0;
            break;
        case ADS131E08_CMD_RREG:    // ADS131E08_CMD_RREGS
            adcConfE->txBuf[0] = ADS131E08_UPPER(ADS131E08_REG_COMMAND(cmd,addr,regNo));
            adcConfE->txBuf[1] = ADS131E08_LOWER(ADS131E08_REG_COMMAND(cmd,addr,regNo));
            adcConfE->txBuf[2] = 0;
            adcConfE->txBuf[3] = 0;
            adcConfE->command = cmd;
            adcConfE->regAddr = addr;
            adcConfE->regs = regs;
            break;
        case ADS131E08_CMD_WREG:
            adcConfE->txBuf[0] = ADS131E08_UPPER(ADS131E08_REG_COMMAND(cmd,addr,regNo));
            adcConfE->txBuf[1] = ADS131E08_LOWER(ADS131E08_REG_COMMAND(cmd,addr,regNo));
            adcConfE->txBuf[2] = ADS131E08_UPPER(ADS131E08_REG_REGDATA(regs));
            adcConfE->txBuf[3] = ADS131E08_LOWER(ADS131E08_REG_REGDATA(regs));
            adcConfE->command = cmd;
            adcConfE->regAddr = addr;
            adcConfE->regs = regs;
            break;
        default:
            break;
    }

    // Wait until cs pin is reset to send data
    while(GPIO_PIN_RESET == ADS131E08_read_cs_signal(adcConfE->type))
    {}

    ADS131E08_control_cs_signal(adcConfE->type, RESET_SIGNAL);
    if(cmd == ADS131E08_CMD_RREG)        
    {
        result = HAL_SPI_TransmitReceive(adcConfE->hspi, adcConfE->txBuf, adcConfE->rxBuf, cmdLen, 10);
        memset(adcConfE->rxBuf, 0, adcConfE->bufLen);
        result = HAL_SPI_Receive(adcConfE->hspi, adcConfE->rxBuf, regNo, 500);        

        // Update Register Status
        adcConfE->sr.mp[adcConfE->regAddr] = adcConfE->rxBuf[0];
    }
    else if(cmd == ADS131E08_CMD_WREG)
    {
        result = HAL_SPI_TransmitReceive(adcConfE->hspi, adcConfE->txBuf, adcConfE->rxBuf, regNo + cmdLen, 10);
    }
    else if(cmd == ADS131E08_CMD_RDATA)
    {
        memset(adcConfE->rxBuf, 0, adcConfE->bufLen);
        result = HAL_SPI_TransmitReceive(adcConfE->hspi, adcConfE->txBuf, adcConfE->rxBuf, 28, 10);
    }
    else
    {
        result = HAL_SPI_TransmitReceive(adcConfE->hspi, adcConfE->txBuf, adcConfE->rxBuf, cmdLen, 10);
    }
    ADS131E08_control_cs_signal(adcConfE->type, SET_SIGNAL);

    return result;
}

uint32_t ADS131E08_convert_adc_data(const uint8_t* dataBuf)
{
    uint32_t upperByte = 0;
    uint32_t middleByte = 0;
    uint32_t lowerByte = 0;

    // The output data extends to 32 bits with eight zeroes(0b00000000, 1Byte) added to the least significant bits when using the 32-bit device word length setting, datasheet 38p
    upperByte    = ((uint32_t)(dataBuf[0] << 16) & 0x00FF0000);
    middleByte   = ((uint32_t)(dataBuf[1] << 8) & 0x0000FF00);
    lowerByte    = ((uint32_t)(dataBuf[2] << 0) & 0x000000FF);

    return (upperByte | middleByte | lowerByte);
}

float ADS131E08_convert_to_mVolt(uint32_t reg)
{
    const float unitFS = 5000.0f / 8388607.0f; // unit: mV (if unit is V, calculated value is out of 'float' range)
	const uint32_t boundaryValue = 0x7FFFFF; // threshold of positive value 
	int signFlg = 0;		// +: 1, -: -1

    // negative value
    if (reg > boundaryValue)
    {
        reg = (0xFFFFFF - reg) + 1; // if value is 0xFFFFFF, register is -FS/2^23. so plus 1 
        signFlg = -1;
    }
    // positive value
    else
    {
        signFlg = 1;
    }

    // convert register to mVolt
    return (unitFS * (float)reg * (float)signFlg);
}

void ADS131E08_parse_adc_data(_adcType adcType)
{
    _ADS131E08_ch ch;
    uint8_t index;
    uint32_t temp;
    uint32_t sTemp;
    const uint8_t preBitCode = 0b1100;
    uint8_t recvPreBitCode;
    _adcConfE* adcConfE;
    uint8_t* buf;

    if(adcType == ADS131E08_ADC20)
    {
        adcConfE = adcConfE0;
    }
    else if(adcType == ADS131E08_ADC21)
    {
        adcConfE = adcConfE1;
    }
    else
    {
        // DO NOTHING
        return;
    }

    // Reading Back Packet
    // 24 Status Bits + 24 Bits per channel x 8 channels = 216 bits

    /* Status Word Content (24 bits)
    * ---------------------------------------------------------------------------------------------------------------------
    * | Bit 23 | Bit 22 | Bit 21 | Bit 20 | Bit 19 |  ....  | Bit 12 | Bit 11 |  ....  | Bit 04 | Bit 03 |  .... | Bit 00 |
    * ---------------------------------------------------------------------------------------------------------------------
    * |    1   |    1   |    0   |    0   |     FAULT_STATP[7:0]     |     FAULT_STATN[7:0]     |        GPIO[7:4]        |
    * ---------------------------------------------------------------------------------------------------------------------
    */

    // rxBuf[0] is response for command(RDATA), so status buf start at rxBuf[1]
    temp = ADS131E08_MERGE_BYTE(adcConfE->rxBuf[1], adcConfE->rxBuf[2], adcConfE->rxBuf[3]);

    recvPreBitCode = (uint8_t)((temp & 0x00F00000) >> 20);

    if(recvPreBitCode != preBitCode)
    {
        // DO NOTHING
    }

    // Shift 4 bits to remove pre bit code(0b1100)
    sTemp = temp << 4;
    // Update Status Register
    adcConfE->sr.mp[E08_FAULT_STATP_ADDRESS]    = ADS131E08_FETCH_UPPER(sTemp);
    adcConfE->sr.mp[E08_FAULT_STATN_ADDRESS]    = ADS131E08_FETCH_MIDDLE(sTemp);
    adcConfE->sr.mp[E08_GPIO_ADDRESS]           = ADS131E08_FETCH_LOWER(sTemp) & 0xF0;

    // data buf start at rxBuf[4]
    buf = &adcConfE->rxBuf[4];

    for(ch = ADC_CH1, index = 0; ch < NUMB_ADC_CH; ch++, index++)
    {
        adcConfE->chData[ch].r = ADS131E08_convert_adc_data(&buf[index * E08_WORD_LENGTH]);        
        adcConfE->chData[ch].v = ADS131E08_convert_to_mVolt(adcConfE->chData[ch].r);
    }
}
// ADC1 and ADC2 share a start signal.
// Start signal is related to generate /DRDY signal
void ADS131E08_control_start_signal(_signalState onOff)
{
    if(onOff == SET_SIGNAL)
    {
       HAL_GPIO_WritePin(adcConfE0->start.port, adcConfE0->start.pin, GPIO_PIN_SET);
    }
    else if(onOff == RESET_SIGNAL)
    {
        HAL_GPIO_WritePin(adcConfE0->start.port, adcConfE0->start.pin, GPIO_PIN_RESET);
    }
    else
    {
        // Do Nothing
    }
}

void ADS131E08_receive_data(_adcType adcType)
{
    _adcConfE* adcConfE = NULL;
    if(adcType == ADS131E08_ADC20)
    {
        adcConfE = adcConfE0;
    }
    else if(adcType == ADS131E08_ADC21)
    {
        adcConfE = adcConfE1;
    }
    else
    {
        // DO NOTHING
        return;
    }

    // Wait until cs pin is reset to send data
    while(GPIO_PIN_RESET == ADS131E08_read_cs_signal(adcConfE->type))
    {}
    ADS131E08_control_cs_signal(adcConfE->type, RESET_SIGNAL);
    HAL_SPI_Receive(adcConfE->hspi, adcConfE->rxBuf, adcConfE->bufLen, 50);
    ADS131E08_control_cs_signal(adcConfE->type, SET_SIGNAL);
}

// RDATA command is related to conversion data
void ADS131E08_send_rdatac_cmd()
{
    ADS131E08_send_command(adcConfE0, ADS131E08_CMD_RDATAC, 0, 0, 1);
    ADS131E08_send_command(adcConfE1, ADS131E08_CMD_RDATAC, 0, 0, 1);
}

// RDATA command is related to conversion data
void ADS131E08_send_rdata_cmd(_adcType adcType)
{
    if(adcType == ADS131E08_ADC20)
    {
        ADS131E08_send_command(adcConfE0, ADS131E08_CMD_RDATA, 0, 0, 1);
    }
    else if(adcType == ADS131E08_ADC21)
    {
        ADS131E08_send_command(adcConfE1, ADS131E08_CMD_RDATA, 0, 0, 1);
    }
}


void ADS131E08_control_reset_signal(_signalState onOff)
{
    if(onOff == SET_SIGNAL)
    {
       HAL_GPIO_WritePin(adcConfE0->nReset.port, adcConfE0->nReset.pin, GPIO_PIN_SET);
    }
    else if(onOff == RESET_SIGNAL)
    {
        HAL_GPIO_WritePin(adcConfE0->nReset.port, adcConfE0->nReset.pin, GPIO_PIN_RESET);
    }
    else
    {
        // Do Nothing
    }
}

void ADS131E08_control_cs_signal(_adcType adcType, _signalState onOff)
{
    if(onOff == SET_SIGNAL)
    {
        if(adcType == ADS131E08_ADC20)
        {
            HAL_GPIO_WritePin(adcConfE0->cs.port, adcConfE0->cs.pin, GPIO_PIN_SET);
        }
        else if(adcType == ADS131E08_ADC21)
        {
            HAL_GPIO_WritePin(adcConfE1->cs.port, adcConfE1->cs.pin, GPIO_PIN_SET);
        }
        else
        {
            // DO NOTHING
        }

    }
    else if(onOff == RESET_SIGNAL)
    {
        if(adcType == ADS131E08_ADC20)
        {
            HAL_GPIO_WritePin(adcConfE0->cs.port, adcConfE0->cs.pin, GPIO_PIN_RESET);
        }
        else if(adcType == ADS131E08_ADC21)
        {
            HAL_GPIO_WritePin(adcConfE1->cs.port, adcConfE1->cs.pin, GPIO_PIN_RESET);
        }
        else
        {
            // DO NOTHING
        }
    }
    else
    {
        // Do Nothing
    }
}

GPIO_PinState ADS131E08_read_cs_signal(_adcType adcType)
{
    if(adcType == ADS131E08_ADC20)
    {
        return HAL_GPIO_ReadPin(adcConfE0->cs.port, adcConfE0->cs.pin);
    }
    else if(adcType == ADS131E08_ADC21)
    {
        return HAL_GPIO_ReadPin(adcConfE1->cs.port, adcConfE1->cs.pin);
    }

    return 0;
}

void ADS131E08_set_default_register(_adcConfE* adcConfE)
{
    adcConfE->sr.mp[E08_ID_ADDRESS]              =   0x00;                                  /* NOTE: This a read-only register */
    adcConfE->sr.mp[E08_CONFIG1_ADDRESS]         =   E08_CONFIG1_DEFAULT;                   /* NOTE: REV_ID value is unknown until read */
    adcConfE->sr.mp[E08_CONFIG2_ADDRESS]         =   E08_CONFIG2_DEFAULT;
    adcConfE->sr.mp[E08_CONFIG3_ADDRESS]         =   E08_CONFIG3_DEFAULT;
    adcConfE->sr.mp[E08_FAULT_ADDRESS]           =   E08_FAULT_DEFAULT;
    adcConfE->sr.mp[E08_CH1SET_ADDRESS]          =   E08_CH1SET_DEFAULT;
    adcConfE->sr.mp[E08_CH2SET_ADDRESS]          =   E08_CH2SET_DEFAULT;
    adcConfE->sr.mp[E08_CH3SET_ADDRESS]          =   E08_CH3SET_DEFAULT;
    adcConfE->sr.mp[E08_CH4SET_ADDRESS]          =   E08_CH4SET_DEFAULT;
    adcConfE->sr.mp[E08_CH5SET_ADDRESS]          =   E08_CH5SET_DEFAULT;
    adcConfE->sr.mp[E08_CH6SET_ADDRESS]          =   E08_CH6SET_DEFAULT;
    adcConfE->sr.mp[E08_CH7SET_ADDRESS]          =   E08_CH7SET_DEFAULT;
    adcConfE->sr.mp[E08_CH8SET_ADDRESS]          =   E08_CH8SET_DEFAULT;
    adcConfE->sr.mp[E08_FAULT_STATP_ADDRESS]     =   E08_FAULT_STATP_DEFAULT;
    adcConfE->sr.mp[E08_FAULT_STATN_ADDRESS]     =   E08_FAULT_STATN_DEFAULT;
    adcConfE->sr.mp[E08_GPIO_ADDRESS]            =   E08_GPIO_DEFAULT;
}

void ADS131E08_initial_flow_at_power_up(_adcConfE* adcConfE)
{
    int i;
    volatile uint8_t regs, addr;

    // Always send a SDATAC command first to stop continuous conversion and write register.
    ADS131E08_send_command(adcConfE, ADS131E08_CMD_SDATAC, 0, 0, 1);

    ///////////////////////////////////////
    // Read ID Register
    ///////////////////////////////////////
    ADS131E08_send_command(adcConfE, ADS131E08_CMD_RREG, E08_ID_ADDRESS, 0, 1);

    ///////////////////////////////////////
    // Write Configuration 1 Register
    ///////////////////////////////////////
    // Set register with default
    regs = E08_CONFIG1_DEFAULT;

    ADS131E08_send_command(adcConfE, ADS131E08_CMD_WREG, E08_CONFIG1_ADDRESS, regs, 2);
    // Confirm whether written or not.
    ADS131E08_send_command(adcConfE, ADS131E08_CMD_RREG, E08_CONFIG1_ADDRESS, 0, 1);


    ///////////////////////////////////////
    // Write Configuration 2 Register
    ///////////////////////////////////////
    // Set register with default
    regs = E08_CONFIG2_DEFAULT;

    ADS131E08_send_command(adcConfE, ADS131E08_CMD_WREG, E08_CONFIG2_ADDRESS, regs, 2);
    // Confirm whether written or not.
    ADS131E08_send_command(adcConfE, ADS131E08_CMD_RREG, E08_CONFIG2_ADDRESS, 0, 1);

    ///////////////////////////////////////
    // Write CHnSET Register
    ///////////////////////////////////////
    // Set input short to middle voltage (Input shorted to (AVDD + AVSS) / 2 (for offset or noisemeasurements))
    regs = E08_CH1SET_MUX_MASK & E08_CH1SET_MUX_INPUT_SHORT_TO_MID;
    addr = E08_CH1SET_ADDRESS;
    for(i = 0; i < NUMB_ADC_CH; i++, addr++)
    {
        ADS131E08_send_command(adcConfE, ADS131E08_CMD_WREG, addr, regs, 2);
        // Confirm whether written or not.
        ADS131E08_send_command(adcConfE, ADS131E08_CMD_RREG, addr, 0, 1);
    }
    // Activate Conversion
    ADS131E08_control_start_signal(SET_SIGNAL);

    // Send RDATAC command
    ADS131E08_send_command(adcConfE, ADS131E08_CMD_RDATAC, 0, 0, 1);

    // Capture Data and Check Noise
    // Look for /DRDY and Issue 24 + n x 24 SCLKs
    for(i = 0; i < 1; i++)
    {
        ADS131E08_wait_drdy_int(100);
        ADS131E08_control_cs_signal(adcConfE->type, RESET_SIGNAL);
        HAL_SPI_Receive(adcConfE->hspi, adcConfE->rxBuf, adcConfE->bufLen, 500);
        ADS131E08_control_cs_signal(adcConfE->type, SET_SIGNAL);
    }

    // Always send a SDATAC command first to stop continuous conversion and write register.
    ADS131E08_send_command(adcConfE, ADS131E08_CMD_SDATAC, 0, 0, 1);

    ///////////////////////////////////////
    // Write Configuration 2 Register
    ///////////////////////////////////////
    // Set that test signal are generated internally (Activate a (1mV / 2.4V) Square-Wave Test Signal)
    regs = E08_CONFIG2_INT_TEST_MASK & E08_CONFIG2_INT_TEST_ENABLE;

    ADS131E08_send_command(adcConfE, ADS131E08_CMD_WREG, E08_CONFIG2_ADDRESS, regs, 2);
    // Confirm whether written or not.
    ADS131E08_send_command(adcConfE, ADS131E08_CMD_RREG, E08_CONFIG2_ADDRESS, 0, 1);

    ///////////////////////////////////////
    // Write CHnSET Register
    ///////////////////////////////////////
    // Set test signal enable on All Channels
    regs = E08_CH1SET_MUX_MASK & E08_CH1SET_MUX_TEST_SIGNAL;
    addr = E08_CH1SET_ADDRESS;
    for(i = 0; i < NUMB_ADC_CH; i++, addr++)
    {
        ADS131E08_send_command(adcConfE, ADS131E08_CMD_WREG, addr, regs, 2);
        // Confirm whether written or not.
        ADS131E08_send_command(adcConfE, ADS131E08_CMD_RREG, addr, 0, 1);
    }

    // Send RDATAC command
    ADS131E08_send_command(adcConfE, ADS131E08_CMD_RDATAC, 0, 0, 1);

    // Capture Data and Test Signal
    // Look for /DRDY and Issue 24 + n x 24 SCLKs
    for(i = 0; i < 1; i++)
    {
        ADS131E08_wait_drdy_int(100);
        ADS131E08_control_cs_signal(adcConfE->type, RESET_SIGNAL);
        HAL_SPI_Receive(adcConfE->hspi, adcConfE->rxBuf, adcConfE->bufLen, 500);
        ADS131E08_control_cs_signal(adcConfE->type, SET_SIGNAL);
    }

    // Always send a SDATAC command first to stop continuous conversion and write register.
    ADS131E08_send_command(adcConfE, ADS131E08_CMD_SDATAC, 0, 0, 1);

    // Stop Conversion
    ADS131E08_control_start_signal(RESET_SIGNAL);    
}

HAL_StatusTypeDef ADS131E08_startup(_adcType adcType)
{
    int i;
    volatile uint8_t addr;
    volatile uint8_t regs;
    _adcConfE* adcConfE;

    if(adcType == ADS131E08_ADC20)
    {
        adcConfE = adcConfE0;
    }
    else if(adcType == ADS131E08_ADC21)
    {
        adcConfE = adcConfE1;
    }
    else
    {
        // DO NOTHING
        return HAL_ERROR;
    }

    HAL_Delay(5);
    
    ADS131E08_initial_flow_at_power_up(adcConfE);

    // Always send a SDATAC command first to stop continuous conversion and write register.
    ADS131E08_send_command(adcConfE, ADS131E08_CMD_SDATAC, 0, 0, 1);

    ///////////////////////////////////////
    // Write Configuration 1 Register
    ///////////////////////////////////////
    // Remove register value related with data rate
    regs = E08_CONFIG1_DEFAULT & (~E08_CONFIG1_DR_MASK);
    // Set Data rate to 1 kSPS
    regs |= E08_CONFIG1_DR_MASK & E08_DR_1KSPS;

    ADS131E08_send_command(adcConfE, ADS131E08_CMD_WREG, E08_CONFIG1_ADDRESS, regs, 2);
    // Confirm whether written or not.
    ADS131E08_send_command(adcConfE, ADS131E08_CMD_RREG, E08_CONFIG1_ADDRESS, 0, 1);

    if(adcConfE->sr.mp[E08_CONFIG1_ADDRESS] != regs)
    {
        return HAL_ERROR;
    }
    else
    {
        // DO NOTHING, Keep going
    }

    ///////////////////////////////////////
    // Write Configuration 2 Register
    ///////////////////////////////////////
    // Set register with default
    regs = E08_CONFIG2_DEFAULT;

    ADS131E08_send_command(adcConfE, ADS131E08_CMD_WREG, E08_CONFIG2_ADDRESS, regs, 2);
    // Confirm whether written or not.
    ADS131E08_send_command(adcConfE, ADS131E08_CMD_RREG, E08_CONFIG2_ADDRESS, 0, 1);

    if(adcConfE->sr.mp[E08_CONFIG2_ADDRESS] != regs)
    {
        return HAL_ERROR;
    }
    else
    {
        // DO NOTHING, Keep going
    }

    ///////////////////////////////////////
    // Write CHnSET Register
    ///////////////////////////////////////
    // Set register with default
    regs = E08_CH1SET_DEFAULT;
    addr = E08_CH1SET_ADDRESS;
    for(i = 0; i < NUMB_ADC_CH; i++, addr++)
    {
        ADS131E08_send_command(adcConfE, ADS131E08_CMD_WREG, addr, regs, 2);
        // Confirm whether written or not.
        ADS131E08_send_command(adcConfE, ADS131E08_CMD_RREG, addr, 0, 1);

        if(adcConfE->sr.mp[addr] != regs)
        {
            return HAL_ERROR;
        }
        else
        {
            // DO NOTHING, Keep going
        }        
    }

    ADS131E08_set_adc_status(adcConfE->type, ADS131E08_NORMAL);

    return HAL_OK;
}

void ADS131E08_send_rdatac_command()
{
    // Send RDATAC command to ADC20, ADC21 simultaneously.
    ADS131E08_send_command(adcConfE0, ADS131E08_CMD_RDATAC, 0, 0, 1);
    ADS131E08_send_command(adcConfE1, ADS131E08_CMD_RDATAC, 0, 0, 1);
}

void ADS131E08_initialize_signal()
{
    // Generate Reset Pulse
    ADS131E08_control_reset_signal(SET_SIGNAL);
    ADS131E08_control_start_signal(RESET_SIGNAL);
    ADS131E08_control_reset_signal(RESET_SIGNAL);
    HAL_Delay(5);
    ADS131E08_control_reset_signal(SET_SIGNAL);
    // Initialize Chip Select signal
    ADS131E08_control_cs_signal(ADS131E08_ADC20, SET_SIGNAL);
    ADS131E08_control_cs_signal(ADS131E08_ADC21, SET_SIGNAL);
}