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

#ifndef __ADS131E08_HEADER
#define __ADS131E08_HEADER
#include "main.h"

typedef enum
{
  RESET_SIGNAL,
  SET_SIGNAL
}_signalState;

typedef enum
{
    ADS131E08_ADC20,
    ADS131E08_ADC21
    /* you can add more below */
}_adcType;


// System Commands
#define ADS131E08_CMD_WAKEUP        0x0200
#define ADS131E08_CMD_STANDBY       0x0400
#define ADS131E08_CMD_RESET         0x0600
#define ADS131E08_CMD_START         0x0800
#define ADS131E08_CMD_STOP          0x0A00
#define ADS131E08_CMD_OFFSETCAL     0x1A00

// Data Read Commands
#define ADS131E08_CMD_RDATAC        0x1000  // Enable read data continously mode
#define ADS131E08_CMD_SDATAC        0x1100  // Stop read data continous mode
#define ADS131E08_CMD_RDATA         0x1200  // Read data by command

// Register Read Commands
#define ADS131E08_CMD_RREG          0b001      //(001r rrrr 000n nnnn)b
#define ADS131E08_CMD_WREG          0b010      //(011r rrrr 000n nnnn)b

#define ADS131E08_CMD_ADDR_MASK     0x1F
#define ADS131E08_CMD_NUM_MASK      0x1F
#define ADS131E08_REGDATA_MASK      0xFF

#define ADS131E08_DATA_WORD_MASK    0xFFFFFF00  // (1111 1111 1111 1111 1111 1111 0000 0000)b, The output data are extended to 32 bits with EIGHT ZEROES(00000000) added to the lease significant bits when using the 32-bit device word length setting.

#define ADS131E08_UPPER(x)      ((uint8_t) ((x & 0xFF00) >> 8))
#define ADS131E08_LOWER(x)      ((uint8_t) (x & 0x00FF))

#define ADS131E08_FETCH_UPPER(x)    ((uint8_t) ((x & 0xFF0000) >> 16))
#define ADS131E08_FETCH_MIDDLE(x)   ((uint8_t) ((x & 0x00FF00) >> 8))
#define ADS131E08_FETCH_LOWER(x)    ((uint8_t) (x & 0x0000FF))

#define ADS131E08_MERGE_BYTE(b1, b2, b3)      ((uint32_t)((b1 << 16) & 0x00FF0000) | (uint32_t)((b2 << 8) & 0x0000FF00) | (uint32_t)(b3 & 0x000000FF))

#define ADS131E08_REG_COMMAND(cmd,addr,regno)     ((uint16_t)((cmd << 13) | ((ADS131E08_CMD_ADDR_MASK & addr) << 8 ) | (ADS131E08_CMD_NUM_MASK & (regno - 1))))
#define ADS131E08_REG_REGDATA(regs)               ((uint16_t)(((ADS131E08_REGDATA_MASK & regs) << 8 ) & 0xFF00))

typedef enum
{
    ADC_CH1,
    ADC_CH2,
    ADC_CH3,
    ADC_CH4,
    ADC_CH5,
    ADC_CH6,
    ADC_CH7,
    ADC_CH8,                
    NUMB_ADC_CH
}_ADS131E08_ch;

// 24 bits per channel
#define E08_WORD_DATA_LENGTH_BITS    ((uint8_t) 24)

// Determine number of bytes per command based on WORD_LENGTH
#define E08_WORD_LENGTH             (E08_WORD_DATA_LENGTH_BITS >> 3)

#define E08_WORDS_IN_FRAME          ((uint8_t)NUMB_ADC_CH + 1)           // 9 Words (in case of disable CRC, Status(1) + CH1 ADC DATA(2) + CH2 ADC DATA(3) + CH3 ADC DATA(4) + CH4 ADC DATA(5) + CH5 ADC DATA(6) + CH6 ADC DATA(7) + CH7 ADC DATA(8) + CH8 ADC DATA(9)

//**********************************************************************************
//
// Register definitions
//
//**********************************************************************************

#define ADS131E08_NUM_REGISTERS           ((uint8_t) 21)


/* Register 0x00 (ID) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |             REV_ID[2:0]           |      1    |      0    |     0     |      NU_CH[1:0]       |
 * -------------------------------------------------------------------------------------------------
 */

    /* ID register address */
    #define E08_ID_ADDRESS										    			((uint8_t) 0x00)

    /* ID register field masks */
    #define E08_ID_REV_ID_MASK											    	((uint8_t) 0xE0)

    /* NU_CH field values */
    #define E08_ID_REV_ID_ADS131E0X 									    	((uint8_t) 0xC0)

    /* ID register field masks */
    #define E08_ID_NU_CH_MASK											    	((uint8_t) 0x03)

    /* NU_CH field values */
    #define E08_ID_NU_CH_4												    	((uint8_t) 0x00)
    #define E08_ID_NU_CH_6												    	((uint8_t) 0x01)
    #define E08_ID_NU_CH_8												    	((uint8_t) 0x02)



/* Register 0x01 (CONFIG1) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     1     |  DAISY_IN |   CLK_EN  |      1    |      0    |               DR[2:0]             |
 * -------------------------------------------------------------------------------------------------
 */

    /* CONFIG1 register address */
    #define E08_CONFIG1_ADDRESS													((uint8_t) 0x01)

    /* CONFIG1 default (reset) value */
    #define E08_CONFIG1_DEFAULT													((uint8_t) 0x91)

    /* CONFIG1 register field masks */
    #define E08_CONFIG1_DAISY_IN_MASK											((uint8_t) 0x40)
    #define E08_CONFIG1_CLK_EN_MASK											    ((uint8_t) 0x20)
    #define E08_CONFIG1_DR_MASK											        ((uint8_t) 0x07)

    /* DR field values */
    #define E08_DR_64KSPS   													((uint8_t) 0x00)
    #define E08_DR_32KSPS   													((uint8_t) 0x01)    // Default Value
    #define E08_DR_16KSPS   													((uint8_t) 0x02)
    #define E08_DR_8KSPS   													    ((uint8_t) 0x03)
    #define E08_DR_4KSPS   													    ((uint8_t) 0x04)
    #define E08_DR_2KSPS   													    ((uint8_t) 0x05)
    #define E08_DR_1KSPS   													    ((uint8_t) 0x06)
    #define E08_DR_NOT_USE   													((uint8_t) 0x07)


/* Register 0x02 (CONFIG2) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     1     |     1     |     1     |  INT_TEST |     0     |  TEST_AMP |     TEST_FREQ[1:0]    |
 * -------------------------------------------------------------------------------------------------
 */

    /* CONFIG2 register address */
    #define E08_CONFIG2_ADDRESS													((uint8_t) 0x02)

    /* CONFIG2 default (reset) value */
    #define E08_CONFIG2_DEFAULT													((uint8_t) 0xE0)

    /* CONFIG2 register field masks */
    #define E08_CONFIG2_INT_TEST_MASK											((uint8_t) 0x10)
    #define E08_CONFIG2_TEST_AMP_MASK											((uint8_t) 0x04)
    #define E08_CONFIG2_TEST_FREQ_MASK											((uint8_t) 0x03)

    /* INT_TEST field values */
    #define E08_CONFIG2_INT_TEST_ENABLE                                         ((uint8_t) 0x10)    // Test Signals are generated internally
    #define E08_CONFIG2_INT_TEST_DISABLE                                        ((uint8_t) 0x00)    // Test Signals are driven externally

    /* TEST_FREQ field values */
    #define E08_CONFIG2_TEST_FREQ_21                                            ((uint8_t) 0x00)    // 2^21
    #define E08_CONFIG2_TEST_FREQ_20                                            ((uint8_t) 0x01)    // 2^20
    #define E08_CONFIG2_TEST_FREQ_NOT_USE                                       ((uint8_t) 0x02)
    #define E08_CONFIG2_TEST_FREQ_AT_DC                                         ((uint8_t) 0x03)



/* Register 0x03 (CONFIG3) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * | PDB_REFBUF|     1     |  VREF_4V  |     0     | OPAMP_REF | PDB_OPAMP |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */

    /* CONFIG3 register address */
    #define E08_CONFIG3_ADDRESS													((uint8_t) 0x03)

    /* CONFIG3 default (reset) value */
    #define E08_CONFIG3_DEFAULT													((uint8_t) 0x40)

     /* CONFIG3 register field masks */
    #define E08_CONFIG3_PDB_REFBUF_MASK											((uint8_t) 0x80)
    #define E08_CONFIG3_VREF_4V_MASK											((uint8_t) 0x20)
    #define E08_CONFIG3_OPAMP_REF_MASK											((uint8_t) 0x08)
    #define E08_CONFIG3_PDB_OPAMP_MASK											((uint8_t) 0x04)


/* Register 0x04 (FAULT) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |            COMP_TH[2:0]           |     0     |     0     |     0     |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */

    /* FAULT register address */
    #define E08_FAULT_ADDRESS													((uint8_t) 0x04)

    /* FAULT default (reset) value */
    #define E08_FAULT_DEFAULT													((uint8_t) 0x00)

    /* FAULT register field masks */
    #define E08_FAULT_MASK												        ((uint8_t) 0xE0)

    /* COMP_TH field values */
    #define E08_FAULT_COMP_TH_HS_95P                                            ((uint8_t) 0x00)    // HS 95P: High side 95%
    #define E08_FAULT_COMP_TH_HS_92_5P                                          ((uint8_t) 0x01)
    #define E08_FAULT_COMP_TH_HS_90P                                            ((uint8_t) 0x02)
    #define E08_FAULT_COMP_TH_HS_87_5P                                          ((uint8_t) 0x03)
    #define E08_FAULT_COMP_TH_HS_85P                                            ((uint8_t) 0x04)
    #define E08_FAULT_COMP_TH_HS_80P                                            ((uint8_t) 0x05)
    #define E08_FAULT_COMP_TH_HS_75P                                            ((uint8_t) 0x06)
    #define E08_FAULT_COMP_TH_HS_70P                                            ((uint8_t) 0x07)


/* Register 0x05 (CH1SET) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |    PD    |             GAIN[2:0]             |     0     |              MUX[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* CH1SET register address */
    #define E08_CH1SET_ADDRESS													((uint8_t) 0x05)

    /* CH1SET default (reset) value */
    #define E08_CH1SET_DEFAULT													((uint8_t) 0x10)

    /* CH1SET register field masks */
    #define E08_CH1SET_PD_MASK											        ((uint8_t) 0x80)
    #define E08_CH1SET_GAIN_MASK											    ((uint8_t) 0x70)
    #define E08_CH1SET_MUX_MASK											        ((uint8_t) 0x07)

    /* PGA Gain field values */
    #define E08_CH1SET_PGA_GAIN_NOTUSE                                          ((uint8_t) 0x00)
    #define E08_CH1SET_PGA_GAIN_1                                               ((uint8_t) 0x10)
    #define E08_CH1SET_PGA_GAIN_2                                               ((uint8_t) 0x20)
    #define E08_CH1SET_PGA_GAIN_4                                               ((uint8_t) 0x40)
    #define E08_CH1SET_PGA_GAIN_8                                               ((uint8_t) 0x50)
    #define E08_CH1SET_PGA_GAIN_12                                              ((uint8_t) 0x60)

    /* MUX field values */
    #define E08_CH1SET_MUX_NORMAL_INPUT                                         ((uint8_t) 0x00)
    #define E08_CH1SET_MUX_INPUT_SHORT_TO_MID                                   ((uint8_t) 0x01) // Input shorted to (AVDD + AVSS) / 2 (for offset or noisemeasurements)
    #define E08_CH1SET_MUX_MVDD_FOR_SUPPLY                                      ((uint8_t) 0x03)
    #define E08_CH1SET_MUX_TEMP_SENSOR                                          ((uint8_t) 0x04)
    #define E08_CH1SET_MUX_TEST_SIGNAL                                          ((uint8_t) 0x05)


/* Register 0x06 (CH2SET) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |    PD    |             GAIN[2:0]             |     0     |              MUX[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* CH2SET register address */
    #define E08_CH2SET_ADDRESS													((uint8_t) 0x06)

    /* CH2SET default (reset) value */
    #define E08_CH2SET_DEFAULT													((uint8_t) 0x10)

    /* CH2SET register field masks */
    #define E08_CH2SET_PD_MASK											        ((uint8_t) 0x80)
    #define E08_CH2SET_GAIN_MASK											    ((uint8_t) 0x70)
    #define E08_CH2SET_MUX_MASK											        ((uint8_t) 0x07)

    /* PGA Gain field values */
    #define E08_CH2SET_PGA_GAIN_NOTUSE                                          ((uint8_t) 0x00)
    #define E08_CH2SET_PGA_GAIN_1                                               ((uint8_t) 0x10)
    #define E08_CH2SET_PGA_GAIN_2                                               ((uint8_t) 0x20)
    #define E08_CH2SET_PGA_GAIN_4                                               ((uint8_t) 0x40)
    #define E08_CH2SET_PGA_GAIN_8                                               ((uint8_t) 0x50)
    #define E08_CH2SET_PGA_GAIN_12                                              ((uint8_t) 0x60)

    /* MUX field values */
    #define E08_CH2SET_MUX_NORMAL_INPUT                                         ((uint8_t) 0x00)
    #define E08_CH2SET_MUX_INPUT_SHORT_TO_MID                                   ((uint8_t) 0x01) // Input shorted to (AVDD + AVSS) / 2 (for offset or noisemeasurements)
    #define E08_CH2SET_MUX_MVDD_FOR_SUPPLY                                      ((uint8_t) 0x03)
    #define E08_CH2SET_MUX_TEMP_SENSOR                                          ((uint8_t) 0x04)
    #define E08_CH2SET_MUX_TEST_SIGNAL                                          ((uint8_t) 0x05)


/* Register 0x07 (CH3SET) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |    PD    |             GAIN[2:0]             |     0     |              MUX[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* CH3SET register address */
    #define E08_CH3SET_ADDRESS													((uint8_t) 0x07)

    /* CH3SET default (reset) value */
    #define E08_CH3SET_DEFAULT													((uint8_t) 0x10)

    /* CH3SET register field masks */
    #define E08_CH3SET_PD_MASK											        ((uint8_t) 0x80)
    #define E08_CH3SET_GAIN_MASK											    ((uint8_t) 0x70)
    #define E08_CH3SET_MUX_MASK											        ((uint8_t) 0x07)

    /* PGA Gain field values */
    #define E08_CH3SET_PGA_GAIN_NOTUSE                                          ((uint8_t) 0x00)
    #define E08_CH3SET_PGA_GAIN_1                                               ((uint8_t) 0x10)
    #define E08_CH3SET_PGA_GAIN_2                                               ((uint8_t) 0x20)
    #define E08_CH3SET_PGA_GAIN_4                                               ((uint8_t) 0x40)
    #define E08_CH3SET_PGA_GAIN_8                                               ((uint8_t) 0x50)
    #define E08_CH3SET_PGA_GAIN_12                                              ((uint8_t) 0x60)

    /* MUX field values */
    #define E08_CH3SET_MUX_NORMAL_INPUT                                         ((uint8_t) 0x00)
    #define E08_CH3SET_MUX_INPUT_SHORT_TO_MID                                   ((uint8_t) 0x01) // Input shorted to (AVDD + AVSS) / 2 (for offset or noisemeasurements)
    #define E08_CH3SET_MUX_MVDD_FOR_SUPPLY                                      ((uint8_t) 0x03)
    #define E08_CH3SET_MUX_TEMP_SENSOR                                          ((uint8_t) 0x04)
    #define E08_CH3SET_MUX_TEST_SIGNAL                                          ((uint8_t) 0x05)


/* Register 0x08 (CH4SET) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |    PD    |             GAIN[2:0]             |     0     |              MUX[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* CH4SET register address */
    #define E08_CH4SET_ADDRESS													((uint8_t) 0x08)

    /* CH4SET default (reset) value */
    #define E08_CH4SET_DEFAULT													((uint8_t) 0x10)

    /* CH4SET register field masks */
    #define E08_CH4SET_PD_MASK											        ((uint8_t) 0x80)
    #define E08_CH4SET_GAIN_MASK											    ((uint8_t) 0x70)
    #define E08_CH4SET_MUX_MASK											        ((uint8_t) 0x07)

    /* PGA Gain field values */
    #define E08_CH4SET_PGA_GAIN_NOTUSE                                          ((uint8_t) 0x00)
    #define E08_CH4SET_PGA_GAIN_1                                               ((uint8_t) 0x10)
    #define E08_CH4SET_PGA_GAIN_2                                               ((uint8_t) 0x20)
    #define E08_CH4SET_PGA_GAIN_4                                               ((uint8_t) 0x40)
    #define E08_CH4SET_PGA_GAIN_8                                               ((uint8_t) 0x50)
    #define E08_CH4SET_PGA_GAIN_12                                              ((uint8_t) 0x60)

    /* MUX field values */
    #define E08_CH4SET_MUX_NORMAL_INPUT                                         ((uint8_t) 0x00)
    #define E08_CH4SET_MUX_INPUT_SHORT_TO_MID                                   ((uint8_t) 0x01) // Input shorted to (AVDD + AVSS) / 2 (for offset or noisemeasurements)
    #define E08_CH4SET_MUX_MVDD_FOR_SUPPLY                                      ((uint8_t) 0x03)
    #define E08_CH4SET_MUX_TEMP_SENSOR                                          ((uint8_t) 0x04)
    #define E08_CH4SET_MUX_TEST_SIGNAL                                          ((uint8_t) 0x05)


/* Register 0x09 (CH5SET) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |    PD    |             GAIN[2:0]             |     0     |              MUX[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* CH5SET register address */
    #define E08_CH5SET_ADDRESS													((uint8_t) 0x09)

    /* CH5SET default (reset) value */
    #define E08_CH5SET_DEFAULT													((uint8_t) 0x10)

    /* CH5SET register field masks */
    #define E08_CH5SET_PD_MASK											        ((uint8_t) 0x80)
    #define E08_CH5SET_GAIN_MASK											    ((uint8_t) 0x70)
    #define E08_CH5SET_MUX_MASK											        ((uint8_t) 0x07)

    /* PGA Gain field values */
    #define E08_CH5SET_PGA_GAIN_NOTUSE                                          ((uint8_t) 0x00)
    #define E08_CH5SET_PGA_GAIN_1                                               ((uint8_t) 0x10)
    #define E08_CH5SET_PGA_GAIN_2                                               ((uint8_t) 0x20)
    #define E08_CH5SET_PGA_GAIN_4                                               ((uint8_t) 0x40)
    #define E08_CH5SET_PGA_GAIN_8                                               ((uint8_t) 0x50)
    #define E08_CH5SET_PGA_GAIN_12                                              ((uint8_t) 0x60)

    /* MUX field values */
    #define E08_CH5SET_MUX_NORMAL_INPUT                                         ((uint8_t) 0x00)
    #define E08_CH5SET_MUX_INPUT_SHORT_TO_MID                                   ((uint8_t) 0x01) // Input shorted to (AVDD + AVSS) / 2 (for offset or noisemeasurements)
    #define E08_CH5SET_MUX_MVDD_FOR_SUPPLY                                      ((uint8_t) 0x03)
    #define E08_CH5SET_MUX_TEMP_SENSOR                                          ((uint8_t) 0x04)
    #define E08_CH5SET_MUX_TEST_SIGNAL                                          ((uint8_t) 0x05)


/* Register 0x0A (CH6SET) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |    PD    |             GAIN[2:0]             |     0     |              MUX[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* CH6SET register address */
    #define E08_CH6SET_ADDRESS													((uint8_t) 0x0A)

    /* CH6SET default (reset) value */
    #define E08_CH6SET_DEFAULT													((uint8_t) 0x10)

    /* CH6SET register field masks */
    #define E08_CH6SET_PD_MASK											        ((uint8_t) 0x80)
    #define E08_CH6SET_GAIN_MASK											    ((uint8_t) 0x70)
    #define E08_CH6SET_MUX_MASK											        ((uint8_t) 0x07)

    /* PGA Gain field values */
    #define E08_CH6SET_PGA_GAIN_NOTUSE                                          ((uint8_t) 0x00)
    #define E08_CH6SET_PGA_GAIN_1                                               ((uint8_t) 0x10)
    #define E08_CH6SET_PGA_GAIN_2                                               ((uint8_t) 0x20)
    #define E08_CH6SET_PGA_GAIN_4                                               ((uint8_t) 0x40)
    #define E08_CH6SET_PGA_GAIN_8                                               ((uint8_t) 0x50)
    #define E08_CH6SET_PGA_GAIN_12                                              ((uint8_t) 0x60)

    /* MUX field values */
    #define E08_CH6SET_MUX_NORMAL_INPUT                                         ((uint8_t) 0x00)
    #define E08_CH6SET_MUX_INPUT_SHORT_TO_MID                                   ((uint8_t) 0x01) // Input shorted to (AVDD + AVSS) / 2 (for offset or noisemeasurements)
    #define E08_CH6SET_MUX_MVDD_FOR_SUPPLY                                      ((uint8_t) 0x03)
    #define E08_CH6SET_MUX_TEMP_SENSOR                                          ((uint8_t) 0x04)
    #define E08_CH6SET_MUX_TEST_SIGNAL                                          ((uint8_t) 0x05)


/* Register 0x0B (CH7SET) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |    PD    |             GAIN[2:0]             |     0     |              MUX[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* CH7SET register address */
    #define E08_CH7SET_ADDRESS													((uint8_t) 0x0B)

    /* CH7SET default (reset) value */
    #define E08_CH7SET_DEFAULT													((uint8_t) 0x10)

    /* CH7SET register field masks */
    #define E08_CH7SET_PD_MASK											        ((uint8_t) 0x80)
    #define E08_CH7SET_GAIN_MASK											    ((uint8_t) 0x70)
    #define E08_CH7SET_MUX_MASK											        ((uint8_t) 0x07)

    /* PGA Gain field values */
    #define E08_CH7SET_PGA_GAIN_NOTUSE                                          ((uint8_t) 0x00)
    #define E08_CH7SET_PGA_GAIN_1                                               ((uint8_t) 0x10)
    #define E08_CH7SET_PGA_GAIN_2                                               ((uint8_t) 0x20)
    #define E08_CH7SET_PGA_GAIN_4                                               ((uint8_t) 0x40)
    #define E08_CH7SET_PGA_GAIN_8                                               ((uint8_t) 0x50)
    #define E08_CH7SET_PGA_GAIN_12                                              ((uint8_t) 0x60)

    /* MUX field values */
    #define E08_CH7SET_MUX_NORMAL_INPUT                                         ((uint8_t) 0x00)
    #define E08_CH7SET_MUX_INPUT_SHORT_TO_MID                                   ((uint8_t) 0x01) // Input shorted to (AVDD + AVSS) / 2 (for offset or noisemeasurements)
    #define E08_CH7SET_MUX_MVDD_FOR_SUPPLY                                      ((uint8_t) 0x03)
    #define E08_CH7SET_MUX_TEMP_SENSOR                                          ((uint8_t) 0x04)
    #define E08_CH7SET_MUX_TEST_SIGNAL                                          ((uint8_t) 0x05)


/* Register 0x0C (CH8SET) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |    PD    |             GAIN[2:0]             |     0     |              MUX[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* CH8SET register address */
    #define E08_CH8SET_ADDRESS													((uint8_t) 0x0C)

    /* CH8SET default (reset) value */
    #define E08_CH8SET_DEFAULT													((uint8_t) 0x10)

    /* CH8SET register field masks */
    #define E08_CH8SET_PD_MASK											        ((uint8_t) 0x80)
    #define E08_CH8SET_GAIN_MASK											    ((uint8_t) 0x70)
    #define E08_CH8SET_MUX_MASK											        ((uint8_t) 0x07)

    /* PGA Gain field values */
    #define E08_CH8SET_PGA_GAIN_NOTUSE                                          ((uint8_t) 0x00)
    #define E08_CH8SET_PGA_GAIN_1                                               ((uint8_t) 0x10)
    #define E08_CH8SET_PGA_GAIN_2                                               ((uint8_t) 0x20)
    #define E08_CH8SET_PGA_GAIN_4                                               ((uint8_t) 0x40)
    #define E08_CH8SET_PGA_GAIN_8                                               ((uint8_t) 0x50)
    #define E08_CH8SET_PGA_GAIN_12                                              ((uint8_t) 0x60)

    /* MUX field values */
    #define E08_CH8SET_MUX_NORMAL_INPUT                                         ((uint8_t) 0x00)
    #define E08_CH8SET_MUX_INPUT_SHORT_TO_MID                                   ((uint8_t) 0x01) // Input shorted to (AVDD + AVSS) / 2 (for offset or noisemeasurements)
    #define E08_CH8SET_MUX_MVDD_FOR_SUPPLY                                      ((uint8_t) 0x03)
    #define E08_CH8SET_MUX_TEMP_SENSOR                                          ((uint8_t) 0x04)
    #define E08_CH8SET_MUX_TEST_SIGNAL                                          ((uint8_t) 0x05)


/* Register 0x0D (RESERVED0) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |     0     |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */



/* Register 0x0E (RESERVED1) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |     0     |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */



/* Register 0x0F (RESERVED2) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |     0     |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */



/* Register 0x10 (RESERVED3) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |     0     |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */



/* Register 0x11 (RESERVED4) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |     0     |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */



/* Register 0x12 (FAULT_STATP) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * | IN8P_FAULT| IN7P_FAULT| IN6P_FAULT| IN5P_FAULT| IN4P_FAULT| IN3P_FAULT| IN2P_FAULT| IN1P_FAULT|
 * -------------------------------------------------------------------------------------------------
 */

    /* FAULT_STATP register address */
    #define E08_FAULT_STATP_ADDRESS												((uint8_t) 0x12)

    /* FAULT_STATP default (reset) value */
    #define E08_FAULT_STATP_DEFAULT												((uint8_t) 0x00)

    /* FAULT_STATP register field masks */
    #define E08_FAULT_STATP_IN8P_FAULT_MASK										((uint8_t) 0x80)
    #define E08_FAULT_STATP_IN7P_FAULT_MASK										((uint8_t) 0x40)
    #define E08_FAULT_STATP_IN6P_FAULT_MASK										((uint8_t) 0x20)
    #define E08_FAULT_STATP_IN5P_FAULT_MASK										((uint8_t) 0x10)
    #define E08_FAULT_STATP_IN4P_FAULT_MASK										((uint8_t) 0x08)
    #define E08_FAULT_STATP_IN3P_FAULT_MASK										((uint8_t) 0x04)
    #define E08_FAULT_STATP_IN2P_FAULT_MASK										((uint8_t) 0x02)
    #define E08_FAULT_STATP_IN1P_FAULT_MASK										((uint8_t) 0x01)        



/* Register 0x13 (FAULT_STATN) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * | IN8_NFAULT| IN7N_FAULT| IN6N_FAULT| IN5N_FAULT| IN4N_FAULT| IN3N_FAULT| IN2N_FAULT| IN1N_FAULT|
 * -------------------------------------------------------------------------------------------------
 */

    /* FAULT_STATN register address */
    #define E08_FAULT_STATN_ADDRESS												((uint8_t) 0x13)

    /* FAULT_STATN default (reset) value */
    #define E08_FAULT_STATN_DEFAULT												((uint8_t) 0x00)

    /* FAULT_STATN register field masks */
    #define E08_FAULT_STATN_IN8N_FAULT_MASK										((uint8_t) 0x80)
    #define E08_FAULT_STATN_IN7N_FAULT_MASK										((uint8_t) 0x40)
    #define E08_FAULT_STATN_IN6N_FAULT_MASK										((uint8_t) 0x20)
    #define E08_FAULT_STATN_IN5N_FAULT_MASK										((uint8_t) 0x10)
    #define E08_FAULT_STATN_IN4N_FAULT_MASK										((uint8_t) 0x08)
    #define E08_FAULT_STATN_IN3N_FAULT_MASK										((uint8_t) 0x04)
    #define E08_FAULT_STATN_IN2N_FAULT_MASK										((uint8_t) 0x02)
    #define E08_FAULT_STATN_IN1N_FAULT_MASK										((uint8_t) 0x01)        



/* Register 0x14 (GPIO) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |                   GPIOD[7:4]                  |                   GPIOC[4:1]                  |
 * -------------------------------------------------------------------------------------------------
 */

    /* GPIO register address */
    #define E08_GPIO_ADDRESS												    ((uint8_t) 0x14)

    /* GPIO default (reset) value */
    #define E08_GPIO_DEFAULT												    ((uint8_t) 0x0F)

    /* GPIO register field masks */
    #define E08_GPIO_GPIOD_MASK										            ((uint8_t) 0xF0)
    #define E08_GPIO_GPIOC_MASK										            ((uint8_t) 0x0F)


#pragma pack(push, 1)

typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
}_ADS131E08_io;

typedef enum
{
    ADS131E08_INIT,
    ADS131E08_INIT_COMPLT,      // Initialize Complete
    ADS131E08_READY,            // Locked Status
    ADS131E08_STANDBY,          // Stand by mode, Low current consumption NOT convert ADC data
    ADS131E08_NORMAL,           // Normal mode, start to convert ADC data Continuously
    ADS131E08_NORMAL_CHIPSEL,   // Normal mode, Enable Chip Select Status
    ADS131E08_INIT_FAIL        
}_ADS131E08_stat;

typedef struct
{
    uint8_t ID;             // address = 0x00, reset = 0xXX, ID Control Register
    uint8_t CONFIG1;        // address = 0x01, reset = 0x91, Configuration Register 1
    uint8_t CONFIG2;        // address = 0x01, reset = 0xE0, Configuration Register 2
    uint8_t CONFIG3;        // address = 0x01, reset = 0x40, Configuration Register 3
    uint8_t FAULT;          // address = 0x04, reset = 0x00, Fault Detect Control Register
    uint8_t CH1SET;         // address = 0x05, reset = 0x10, Individual Channel 1 Settings
    uint8_t CH2SET;         // address = 0x06, reset = 0x10, Individual Channel 2 Settings
    uint8_t CH3SET;         // address = 0x07, reset = 0x10, Individual Channel 3 Settings
    uint8_t CH4SET;         // address = 0x08, reset = 0x10, Individual Channel 4 Settings
    uint8_t CH5SET;         // address = 0x09, reset = 0x10, Individual Channel 5 Settings
    uint8_t CH6SET;         // address = 0x0A, reset = 0x10, Individual Channel 6 Settings
    uint8_t CH7SET;         // address = 0x0B, reset = 0x10, Individual Channel 7 Settings
    uint8_t CH8SET;         // address = 0x0C, reset = 0x10, Individual Channel 8 Settings
    uint8_t RESV_REGS[5];   // address = 0x0D, 0x0E, 0x0F, 0x10, 0x11, reset = 0x00, Reserved Register
    uint8_t FAULT_STATP;    // address = 0x12, reset = 0x00, Fault Detect Positive Input Status
    uint8_t FAULT_STATN;    // address = 0x13, reset = 0x00, Fault Detect Negative Input Status
    uint8_t GPIO;           // address = 0x14, reset = 0x0F, General-Purpose IO Register
}_ADS131E08_status_reg;

typedef union
{
    _ADS131E08_status_reg nm;               // list by name
    uint8_t mp[ADS131E08_NUM_REGISTERS];    // list by map
}_ADS131E08_regs;

typedef struct 
{
    uint32_t r;
    float   v;
}_e08ChData;

typedef struct
{
    _adcType type;
    _ADS131E08_stat stat;
    _ADS131E08_regs sr;
    uint16_t command;
    uint16_t regAddr;
    uint8_t regs;
    uint16_t response;
    SPI_HandleTypeDef *hspi;
    _ADS131E08_io nReset;
    _ADS131E08_io cs;
    _ADS131E08_io nDrdy;
    _ADS131E08_io start;
    uint8_t* rxBuf;
    uint8_t* txBuf;
    uint8_t bufLen;
    _e08ChData chData[8];
}_adcConfE;         // Type E: ADS131E08

typedef enum
{
    CMD_WITH_RESPONSE,        // Transfer SPI packet direct (control CS signal in send function)
    CMD_WITHOUT_RESPONSE      // Transfer SPI packet by DMA (control CS signal in DMA callback function)
}_e08SendFlg;

#pragma pack(pop)


// Remove below 2line after test
extern _adcConfE* adcConfE0;
extern _adcConfE* adcConfE1;

extern void ADS131E08_init(_adcType adcType, SPI_HandleTypeDef* hspi);
// extern HAL_StatusTypeDef ADS131E08_startup();
// extern SPI_HandleTypeDef* is_spi_type(_adcType adcType);
// extern uint8_t* is_rxbuf(_adcType adcType);
// extern uint8_t is_buf_len(_adcType adcType);
extern _ADS131E08_stat ADS131E08_is_adc_status(_adcType adcType);
extern void ADS131E08_set_adc_status(_adcType adcType, _ADS131E08_stat stat);
extern HAL_StatusTypeDef ADS131E08_send_command(_adcConfE* adcConfE, uint16_t cmd, uint16_t addr, uint8_t regs, uint16_t regNo);
// extern HAL_StatusTypeDef ADS131E08_parse_packet();
// extern uint8_t ADS131E08_confirm_ready_word();
extern void ADS131E08_control_cs_signal(_adcType adcType, _signalState onOff);
extern void ADS131E08_control_start_signal(_signalState onOff);
extern void ADS131E08_receive_data(_adcType adcType);
// extern HAL_StatusTypeDef ADS131E08_recv_response(uint16_t cmd, uint16_t addr, uint8_t regs, _sendFlg sendFlg);
extern void ADS131E08_parse_adc_data(_adcType adcType);
extern void ADS131E08_set_adc_status(_adcType adcType, _ADS131E08_stat stat);
extern void ADS131E08_send_rdatac_cmd();
extern HAL_StatusTypeDef ADS131E08_startup(_adcType adcType);
extern void ADS131E08_control_reset_signal(_signalState onOff);
extern void ADS131E08_send_rdatac_command();
extern void ADS131E08_initialize_signal();
extern void ADS131E08_send_rdata_cmd(_adcType adcType);
#endif  // __ADS131E08_HEADER