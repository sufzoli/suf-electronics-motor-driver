/**************************************************************************//**
 * @file     SPI.h
 * @version  V2.1
 * $Revision: 8 $
 * $Date: 9/03/13 1:46p $
 * @brief    M051 Series SPI Driver Header File
 *
 * @note
 * Copyright (C) 2011 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __SPI_H__
#define __SPI_H__

/*---------------------------------------------------------------------------------------------------------*/
/* Include related headers                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#include "M051Series.h"


/** @addtogroup M051_FUNC M051 Function Interface
  * @{
  */

/** @addtogroup SPI_FUNC SPI Device Function Interface
  * @{
  */



/*---------------------------------------------------------------------------------------------------------*/
/* SPI_CNTRL constant definitions                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define SPI_CNTRL_FIFO_MODE_EN           (1UL << 21) /*!< SPI_CNTRL setting for enable FIFO mode. */
#define SPI_CNTRL_REORDER_ON             (1UL << 19) /*!< SPI_CNTRL setting for enable both byte reorder and byte suspend. */
#define SPI_CNTRL_MASTER_MODE            (0UL << 18) /*!< SPI_CNTRL setting for SPI master mode */
#define SPI_CNTRL_SLAVE_MODE             (1UL << 18) /*!< SPI_CNTRL setting for SPI slave mode */
#define SPI_CNTRL_IE_EN                  (1UL << 17) /*!< SPI_CNTRL setting for eanble interrupt */
#define SPI_CNTRL_IF                     (1UL << 16) /*!< SPI_CNTRL interrupt flag */
#define SPI_CNTRL_SP_CYCLE(x)            ((x) << 12) /*!< SPI_CNTRL setting for suspend interval. It could be 0~15. */
#define SPI_CNTRL_CLK_IDLE_HIGH          (1UL << 11) /*!< SPI_CNTRL setting for clock idle high */
#define SPI_CNTRL_CLK_IDLE_LOW           (0)         /*!< SPI_CNTRL setting for clock idle low */
#define SPI_CNTRL_LSB_FIRST              (1UL << 10) /*!< SPI_CNTRL setting for data format as LSB first */
#define SPI_CNTRL_MSB_FIRST              (0)         /*!< SPI_CNTRL setting for data format as MSB first */
#define SPI_CNTRL_TX_BIT_LEN(x)          ((((x)!=32)?(x):0) << 3) /*!< SPI_CNTRL setting for bit length of a transaction. It could be 8~32. */
#define SPI_CNTRL_TX_RISING              (0)         /*!< SPI_CNTRL setting for output data at SPICLK rising edge */
#define SPI_CNTRL_TX_FALLING             (4)         /*!< SPI_CNTRL setting for TX output data at SPICLK falling edge */
#define SPI_CNTRL_RX_RISING              (0)         /*!< SPI_CNTRL setting for RX latch data at SPICLK rising edge */
#define SPI_CNTRL_RX_FALLING             (2)         /*!< SPI_CNTRL setting for RX latch data at SPICLK falling edge */
#define SPI_CNTRL_GO_BUSY                (1)         /*!< SPI_CNTRL setting for trigger SPI */


/*---------------------------------------------------------------------------------------------------------*/
/* SPI_DIVIDER constant definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define SPI_DIVIDER_DIV(x)  (x)         /*!< SPI_DIVIDER setting for SPI master clock divider. It could be 0~255. */


/*---------------------------------------------------------------------------------------------------------*/
/* SPI_SSR constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define SPI_SSR_SLAVE_FALLING_EDGE_TRIGGER (0x0)  /*!< SPI_SSR setting for falling edge trigger in SPI slave mode */
#define SPI_SSR_SLAVE_RISING_EDGE_TRIGGER  (0x4)  /*!< SPI_SSR setting for rising edge trigger in SPI slave mode */
#define SPI_SSR_SLAVE_LOW_LEVEL_ACTIVE     (0x10) /*!< SPI_SSR setting for low level active in SPI slave mode */
#define SPI_SSR_SLAVE_HIGH_LEVEL_ACTIVE    (0x14) /*!< SPI_SSR setting for high level active in SPI slave mode */
#define SPI_SSR_HW_AUTO_ACTIVE_LOW         (0x9)  /*!< SPI_SSR setting for SPI master SS signal is controled by hardware automatically and active low */
#define SPI_SSR_HW_AUTO_ACTIVE_HIGH        (0xD)  /*!< SPI_SSR setting for SPI master SS signal is controled by hardware automatically active high */
#define SPI_SSR_SW_SS_PIN_LOW              (0x1)  /*!< SPI_SSR setting to force SPI master SS signal to low by software */
#define SPI_SSR_SW_SS_PIN_HIGH             (0x0)  /*!< SPI_SSR setting to force SPI master SS signal to high by software */

/*---------------------------------------------------------------------------------------------------------*/
/* SPI_CNTRL2 constant definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define SPI_CNTRL2_3WIRE_START_INTSTS                    (1UL << 11) /*!< SPI_CNTRL2 3-wire mode start interrupt flag */
#define SPI_CNTRL2_3WIRE_START_INT_EN                    (1UL << 10) /*!< SPI_CNTRL2 setting for 3-wire mode start interrupt enable */
#define SPI_CNTRL2_3WIRE_ABORT                           (1UL << 9)  /*!< SPI_CNTRL2 setting for 3-wire mode transfer abort */
#define SPI_CNTRL2_3WIRE_MODE_EN                         (1UL << 8)  /*!< SPI_CNTRL2 setting for 3-wire mode enable */

/*---------------------------------------------------------------------------------------------------------*/
/* SPI_FIFO_CTL constant definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define SPI_FIFO_CTL_TX_THRESHOLD(x)   ((x) << SPI_FIFO_CTL_TX_THRESHOLD_Pos) /*!< SPI_FIFO_CTL setting for Tx FIFO threshold         */
#define SPI_FIFO_CTL_RX_THRESHOLD(x)   ((x) << SPI_FIFO_CTL_RX_THRESHOLD_Pos) /*!< SPI_FIFO_CTL setting for Rx FIFO threshold         */
#define SPI_FIFO_CTL_RX_TIMEOUT_INT_EN (1UL << 21) /*!< SPI_FIFO_CTL setting for RX FIFO time-out interrupt enable */
#define SPI_FIFO_CTL_RX_OVERRUN_INT_EN (1UL << 6)  /*!< SPI_FIFO_CTL setting for Rx FIFO overrun interrupt enable */
#define SPI_FIFO_CTL_TX_INT_EN         (1UL << 3)  /*!< SPI_FIFO_CTL setting for Tx FIFO threshold interrupt enable */
#define SPI_FIFO_CTL_RX_INT_EN         (1UL << 2)  /*!< SPI_FIFO_CTL setting for Rx FIFO threshold interrupt enable */
#define SPI_FIFO_CTL_TX_CLEAR          (1UL << 1)  /*!< SPI_FIFO_CTL setting for clearing Tx FIFO */
#define SPI_FIFO_CTL_RX_CLEAR          (1)         /*!< SPI_FIFO_CTL setting for clearing Rx FIFO */

/*---------------------------------------------------------------------------------------------------------*/
/* SPI_STATUS constant definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define SPI_STATUS_TIMEOUT_FLAG        (1UL << 20) /*!< SPI_STATUS time-out flag */
#define SPI_STATUS_IF                  (1UL << 16) /*!< SPI_STATUS interrupt flag */
#define SPI_STATUS_3WIRE_START_INTSTS  (1UL << 11) /*!< SPI_STATUS 3-wire mode start interrupt flag */
#define SPI_STATUS_RX_OVERRUN          (1UL << 2)  /*!< SPI_STATUS Rx FIFO overrun flag */


/*---------------------------------------------------------------------------------------------------------*/
/*  Define Macros and functions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/**
 * @details    Configure the user-specified SPI port as a master
 */
#define _SPI_SET_MASTER_MODE(port) ((port)->CNTRL &= (~SPI_CNTRL_SLAVE_MODE))

/**
 * @details    Configure the user-specified SPI port as a slave
 */
#define _SPI_SET_SLAVE_MODE(port) ((port)->CNTRL |= SPI_CNTRL_SLAVE_MODE)

/**
 * @details    Configure the transfer bit length of the user-specified SPI port
 */
#define _SPI_SET_TRANSFER_BIT_LENGTH(port, x) ((port)->CNTRL = ((port)->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk)) | SPI_CNTRL_TX_BIT_LEN(x))

/**
 * @details    Enable the automatic slave select function of the user-specified SPI port
 */
#define _SPI_ENABLE_HW_AUTO_SLAVE_SELECT(port) ((port)->SSR |= SPI_SSR_AUTOSS_Msk)

/**
 * @details    Disable the automatic slave select function of the user-specified SPI port
 */
#define _SPI_DISABLE_HW_AUTO_SLAVE_SELECT(port) ((port)->SSR &= (~SPI_SSR_AUTOSS_Msk))

/**
 * @details    Configure the timing type of the user-specified SPI port
 */
#define _SPI_SET_TIMING_TYPE1(port) ((port)->CNTRL = ((port)->CNTRL & (~(SPI_CNTRL_CLKP_Msk|SPI_CNTRL_TX_NEG_Msk|SPI_CNTRL_RX_NEG_Msk)) | SPI_CNTRL_TX_NEG_Msk))

/**
 * @details    Configure the timing type of the user-specified SPI port
 */
#define _SPI_SET_TIMING_TYPE2(port) ((port)->CNTRL = ((port)->CNTRL & (~(SPI_CNTRL_CLKP_Msk|SPI_CNTRL_TX_NEG_Msk|SPI_CNTRL_RX_NEG_Msk)) | SPI_CNTRL_RX_NEG_Msk))

/**
 * @details    Configure the timing type of the user-specified SPI port
 */
#define _SPI_SET_TIMING_TYPE5(port) ((port)->CNTRL = ((port)->CNTRL & (~(SPI_CNTRL_CLKP_Msk|SPI_CNTRL_TX_NEG_Msk|SPI_CNTRL_RX_NEG_Msk)) | SPI_CNTRL_CLKP_Msk | SPI_CNTRL_TX_NEG_Msk))

/**
 * @details    Configure the timing type of the user-specified SPI port
 */
#define _SPI_SET_TIMING_TYPE6(port) ((port)->CNTRL = ((port)->CNTRL & (~(SPI_CNTRL_CLKP_Msk|SPI_CNTRL_TX_NEG_Msk|SPI_CNTRL_RX_NEG_Msk)) | SPI_CNTRL_CLKP_Msk | SPI_CNTRL_RX_NEG_Msk))

/**
 * @details    Configure the user-specified SPI port as LSB-first transfer type
 */
#define _SPI_SET_TRANSFER_LSB_FIRST(port) ((port)->CNTRL |= SPI_CNTRL_LSB_FIRST)

/**
 * @details    Configure the user-specified SPI port as MSB-first transfer type
 */
#define _SPI_SET_TRANSFER_MSB_FIRST(port) ((port)->CNTRL &= (~SPI_CNTRL_LSB_FIRST))

/**
 * @details    Enable byte reorder function.
 */
#define _SPI_ENABLE_REORDER_FUNCTION(port) ((port)->CNTRL |= SPI_CNTRL_REORDER_ON)

/**
 * @details    Disable byte reorder function.
 */
#define _SPI_DISABLE_REORDER_FUNCTION(port) ((port)->CNTRL &= (~SPI_CNTRL_REORDER_ON))

/**
 * @details    Configure the suspend interval of the user-specified SPI port. The x setting can be 0-0xF.
 */
#define _SPI_SET_SUSPEND_CYCLE(port, x) ((port)->CNTRL = ((port)->CNTRL & (~SPI_CNTRL_SP_CYCLE_Msk)) | SPI_CNTRL_SP_CYCLE(x))

/**
 * @details    Configure the user-specified SPI port as a low-level-active device when it is in slave mode
 */
#define _SPI_SET_SLAVE_SELECT_LOW_LEVEL_ACTIVE(port) ((port)->SSR = (port)->SSR & (~(SPI_SSR_SS_LTRIG_Msk|SPI_SSR_SS_LVL_Msk)) | SPI_SSR_SS_LTRIG_Msk)

/**
 * @details    Configure the user-specified SPI port as a high-level-active device when it is in slave mode
 */
#define _SPI_SET_SLAVE_SELECT_HIGH_LEVEL_ACTIVE(port) ((port)->SSR |= (SPI_SSR_SS_LTRIG_Msk|SPI_SSR_SS_LVL_Msk))

/**
 * @details    Configure the user-specified SPI port as a falling-edge-active device when it is in slave mode
 */
#define _SPI_SET_SLAVE_SELECT_FALLING_EDGE_ACTIVE(port) ((port)->SSR &= (~(SPI_SSR_SS_LTRIG_Msk|SPI_SSR_SS_LVL_Msk)))

/**
 * @details    Configure the user-specified SPI port as a rising-edge-active device when it is in slave mode
 */
#define _SPI_SET_SLAVE_SELECT_RISING_EDGE_ACTIVE(port) ((port)->SSR = (port)->SSR & (~(SPI_SSR_SS_LTRIG_Msk|SPI_SSR_SS_LVL_Msk)) | SPI_SSR_SS_LVL_Msk)

/**
 * @details    Get the level trigger Accomplish flag of the user-specified SPI port
 */
#define _SPI_GET_SLAVE_LEVEL_TRIG_SUCCESS_FLAG(port) (((port)->SSR & SPI_SSR_LTRIG_FLAG_Msk)>>SPI_SSR_LTRIG_FLAG_Pos)

/**
 * @details    Set the slave select control bit to 1
 */
#define _SPI_SET_SLAVE_SELECT_CNTRL_BIT(port) ((port)->SSR |= SPI_SSR_SSR_Msk)

/**
 * @details    Clear the slave select control bit to 0
 */
#define _SPI_CLR_SLAVE_SELECT_CNTRL_BIT(port) ((port)->SSR &= (~SPI_SSR_SSR_Msk))

/**
 * @details    Get the SPI busy status of the user-specified SPI port
 */
#define _SPI_GET_BUSY_STATUS(port) ((port)->CNTRL & SPI_CNTRL_GO_BUSY_Msk)

/**
 * @details    Set the SPI GO_BUSY bit of the user-specified SPI port
 */
#define _SPI_SET_GO(port) ((port)->CNTRL |= SPI_CNTRL_GO_BUSY_Msk)

/**
 * @details    Clear the SPI GO_BUSY bit of the user-specified SPI port
 */
#define _SPI_CLR_GO(port) ((port)->CNTRL &= (~SPI_CNTRL_GO_BUSY_Msk))

/**
 * @details    Set the SPI DIVIDER of the user-specified SPI port
 */
#define _SPI_SET_ENGINE_CLK_DIVIDER(port, x) ((port)->DIVIDER = (port)->DIVIDER & (~SPI_DIVIDER_DIVIDER_Msk) | (x))

/**
 * @details    Enable the SPI unit transfer interrupt
 */
#define _SPI_ENABLE_UNIT_TRANSFER_INT(port) ((port)->CNTRL |= SPI_CNTRL_IE_Msk)

/**
 * @details    Disable the SPI unit transfer interrupt
 */
#define _SPI_DISABLE_UNIT_TRANSFER_INT(port) ((port)->CNTRL &= (~SPI_CNTRL_IE_Msk))

/**
 * @details    Clear the SPI unit transfer interrupt flag.
 */
#define _SPI_CLR_UNIT_TRANS_INT_STS(port) ((port)->STATUS = SPI_STATUS_IF_Msk  )

/**
 * @details    Get the SPI unit transfer interrupt flag.
 */
#define _SPI_GET_UNIT_TRANS_INT_STS(port) (((port)->STATUS & SPI_STATUS_IF_Msk)>>SPI_STATUS_IF_Pos  )

/**
 * @details    Get the SPI Rx0 data
 */
#define _SPI_GET_RX0_DATA(port) ((port)->RX0)

/**
 * @details    Write data to the SPI Tx buffer0
 */
#define _SPI_WRITE_TX_BUFFER0(port, x) ((port)->TX0 = (x))

/**
 * @details    Enable the SPI 3-wire mode
 */
#define _SPI_ENABLE_3WIRE_MODE(port) ((port)->CNTRL2 |= SPI_CNTRL2_NOSLVSEL_Msk)

/**
 * @details    Disable the 3-wire mode
 */
#define _SPI_DISABLE_3WIRE_MODE(port) ((port)->CNTRL2 &= (~SPI_CNTRL2_NOSLVSEL_Msk))

/**
 * @details    Abort the SPI transfer when the 3-wire mode is enabled
 */
#define _SPI_ABORT_3WIRE_TRANSFER(port) ((port)->CNTRL2 |= SPI_CNTRL2_SLV_ABORT_Msk)

/**
 * @details    Enable the slave 3-wire mode start interrupt
 */
#define _SPI_ENABLE_3WIRE_START_INT(port) ((port)->CNTRL2 |= SPI_CNTRL2_SSTA_INTEN_Msk)

/**
 * @details    Disable the slave 3-wire mode start interrupt
 */
#define _SPI_DISABLE_3WIRE_START_INT(port) ((port)->CNTRL2 &= (~SPI_CNTRL2_SSTA_INTEN_Msk))

/**
 * @details    Get the slave 3-wire mode start interrupt flag
 */
#define _SPI_GET_3WIRE_START_INT_FLAG(port) (((port)->CNTRL2 & SPI_CNTRL2_SLV_START_INTSTS_Msk)>>SPI_CNTRL2_SLV_START_INTSTS_Pos)

/**
 * @details    Clear the slave 3-wire mode start interrupt flag
 */
#define _SPI_CLR_3WIRE_START_INT_FLAG(port) ((port)->CNTRL2 |= SPI_CNTRL2_SLV_START_INTSTS_Msk)

/**
 * @details    Clear BCn bit to 0 for clock configuration backward compatible.
 */
#define _SPI_SET_CLOCK_SETTING_BACKWARD_COMPATIBLE(port) ((port)->CNTRL2 &= (~SPI_CNTRL2_BCn_Msk))

/**
 * @details    Set BCn bit to 1 for more flexible clock configuration.
 */
#define _SPI_SET_CLOCK_SETTING_NOT_BACKWARD_COMPATIBLE(port) ((port)->CNTRL2 |= SPI_CNTRL2_BCn_Msk)

/**
 * @details    Enable the SPI FIFO mode.
 */
#define _SPI_ENABLE_FIFO_MODE(port) ((port)->CNTRL |= SPI_CNTRL_FIFO_Msk)

/**
 * @details    Disable the SPI FIFO mode.
 */
#define _SPI_DISABLE_FIFO_MODE(port) ((port)->CNTRL &= (~SPI_CNTRL_FIFO_Msk))

/**
 * @details    Configure the transmit FIFO threshold of the user-specified SPI port. The x setting can be 0-3.
 */
#define _SPI_SET_TX_THRESHOLD(port, x) ((port)->FIFO_CTL = ((port)->FIFO_CTL & (~SPI_FIFO_CTL_TX_THRESHOLD_Msk)) | (x)<<SPI_FIFO_CTL_TX_THRESHOLD_Pos)

/**
 * @details    Configure the receive FIFO threshold of the user-specified SPI port. The x setting can be 0-3.
 */
#define _SPI_SET_RX_THRESHOLD(port, x) ((port)->FIFO_CTL = ((port)->FIFO_CTL & (~SPI_FIFO_CTL_RX_THRESHOLD_Msk)) | (x)<<SPI_FIFO_CTL_RX_THRESHOLD_Pos)

/**
 * @details    Enable the SPI receive FIFO time-out interrupt.
 */
#define _SPI_ENABLE_RX_TIMEOUT_INT(port) ((port)->FIFO_CTL |= SPI_FIFO_CTL_TIMEOUT_INTEN_Msk)

/**
 * @details    Disable the SPI receive FIFO time-out interrupt.
 */
#define _SPI_DISABLE_RX_TIMEOUT_INT(port) ((port)->FIFO_CTL &= (~SPI_FIFO_CTL_TIMEOUT_INTEN_Msk))

/**
 * @details    Enable the SPI receive FIFO overrun interrupt.
 */
#define _SPI_ENABLE_RX_OVERRUN_INT(port) ((port)->FIFO_CTL |= SPI_FIFO_CTL_RXOV_INTEN_Msk)

/**
 * @details    Disable the SPI receive FIFO overrun interrupt.
 */
#define _SPI_DISABLE_RX_OVERRUN_INT(port) ((port)->FIFO_CTL &= (~SPI_FIFO_CTL_RXOV_INTEN_Msk))

/**
 * @details    Enable the SPI transmit FIFO threshold interrupt.
 */
#define _SPI_ENABLE_TX_THRESHOLD_INT(port) ((port)->FIFO_CTL |= SPI_FIFO_CTL_TX_INTEN_Msk)

/**
 * @details    Disable the SPI transmit FIFO threshold interrupt.
 */
#define _SPI_DISABLE_TX_THRESHOLD_INT(port) ((port)->FIFO_CTL &= (~SPI_FIFO_CTL_TX_INTEN_Msk))

/**
 * @details    Enable the SPI receive FIFO threshold interrupt.
 */
#define _SPI_ENABLE_RX_THRESHOLD_INT(port) ((port)->FIFO_CTL |= SPI_FIFO_CTL_RX_INTEN_Msk)

/**
 * @details    Disable the SPI receive FIFO threshold interrupt.
 */
#define _SPI_DISABLE_RX_THRESHOLD_INT(port) ((port)->FIFO_CTL &= (~SPI_FIFO_CTL_RX_INTEN_Msk))

/**
 * @details    Clear the SPI transmit FIFO buffer.
 */
#define _SPI_CLR_TX_FIFO(port) ((port)->FIFO_CTL |= SPI_FIFO_CTL_TX_CLR_Msk)

/**
 * @details    Clear the SPI receive FIFO buffer.
 */
#define _SPI_CLR_RX_FIFO(port) ((port)->FIFO_CTL |= SPI_FIFO_CTL_RX_CLR_Msk)

/**
 * @details    Enable the SPI slave select inactive interrupt.
 */
#define _SPI_ENABLE_SS_INACTIVE_INT(port) ((port)->CNTRL2 |= SPI_CNTRL2_SS_INT_OPT_Msk)

/**
 * @details    Disable the SPI slave select inactive interrupt.
 */
#define _SPI_DISABLE_SS_INACTIVE_INT(port) ((port)->CNTRL2 &= (~SPI_CNTRL2_SS_INT_OPT_Msk))

/**
 * @details    Get the Tx FIFO count.
 */
#define _SPI_GET_TX_FIFO_COUNT(port) (((port)->STATUS & SPI_STATUS_TX_FIFO_COUNT_Msk)>>SPI_STATUS_TX_FIFO_COUNT_Pos)

/**
 * @details    Get the Tx FIFO full flag.
 */
#define _SPI_GET_TX_FIFO_FULL_FLAG(port) (((port)->STATUS & SPI_STATUS_TX_FULL_Msk)>>SPI_STATUS_TX_FULL_Pos)

/**
 * @details    Get the Tx FIFO empty flag.
 */
#define _SPI_GET_TX_FIFO_EMPTY_FLAG(port) (((port)->STATUS & SPI_STATUS_TX_EMPTY_Msk)>>SPI_STATUS_TX_EMPTY_Pos)

/**
 * @details    Get the Rx FIFO full flag.
 */
#define _SPI_GET_RX_FIFO_FULL_FLAG(port) (((port)->STATUS & SPI_STATUS_RX_FULL_Msk)>>SPI_STATUS_RX_FULL_Pos)

/**
 * @details    Get the Rx FIFO empty flag.
 */
#define _SPI_GET_RX_FIFO_EMPTY_FLAG(port) (((port)->STATUS & SPI_STATUS_RX_EMPTY_Msk)>>SPI_STATUS_RX_EMPTY_Pos)

/**
 * @details    Get the Rx FIFO time-out flag.
 */
#define _SPI_GET_RX_FIFO_TIMEOUT_FLAG(port) (((port)->STATUS & SPI_STATUS_TIMEOUT_Msk)>>SPI_STATUS_TIMEOUT_Pos)

/**
 * @details    Get the Rx FIFO count.
 */
#define _SPI_GET_RX_FIFO_COUNT(port) (((port)->STATUS & SPI_STATUS_RX_FIFO_COUNT_Msk)>>SPI_STATUS_RX_FIFO_COUNT_Pos)

/**
 * @details    Get the Tx FIFO threshold interrupt flag.
 */
#define _SPI_GET_TX_FIFO_THRESHOLD_INT_FLAG(port) (((port)->STATUS & SPI_STATUS_TX_INTSTS_Msk)>>SPI_STATUS_TX_INTSTS_Pos)

/**
 * @details    Get the Rx FIFO overrun flag.
 */
#define _SPI_GET_RX_FIFO_OVERRUN_FLAG(port) (((port)->STATUS & SPI_STATUS_RX_OVERRUN_Msk)>>SPI_STATUS_RX_OVERRUN_Pos)

/**
 * @details    Get the Rx FIFO threshold interrupt flag.
 */
#define _SPI_GET_RX_FIFO_THRESHOLD_INT_FLAG(port) (((port)->STATUS & SPI_STATUS_RX_INTSTS_Msk)>>SPI_STATUS_RX_INTSTS_Pos)




/**
 * @brief      Get SPI0 peripheral clock frequency (Hz).
 *
 * @param      None
 *
 * @return     SPI0 peripheral clock frequency.
 *
 * @details    This function calculates the clock rate of SPI0 peripheral clock (SPI engine clock).
 *
 */
static __INLINE uint32_t SPI_GetSpi0ClockFreq()
{
    if(SPI0->CNTRL2 & SPI_CNTRL2_BCn_Msk)
    {
        /* Check the SPI peripheral clock source */
        if(SYSCLK->CLKSEL1 & SYSCLK_CLKSEL1_SPI0_S_Msk)
            return (SystemCoreClock/((SPI0->DIVIDER & 0xFF)+1)); /* SPI peripheral clock source: system clock */
        else
            return (PllClock/((SPI0->DIVIDER & 0xFF)+1)); /* SPI peripheral clock source: PLL */
    }
    else
    {
        /* Check the SPI peripheral clock source */
        if(SYSCLK->CLKSEL1 & SYSCLK_CLKSEL1_SPI0_S_Msk)
            return ((SystemCoreClock>>1)/((SPI0->DIVIDER & 0xFF)+1)); /* SPI peripheral clock source: system clock */
        else
            return ((PllClock>>1)/((SPI0->DIVIDER & 0xFF)+1)); /* SPI peripheral clock source: PLL */
    }
}

/**
 * @brief      Get SPI1 peripheral clock frequency (Hz).
 *
 * @param      None
 *
 * @return     SPI1 peripheral clock frequency.
 *
 * @details    This function calculates the clock rate of SPI1 peripheral clock (SPI engine clock).
 *
 */
static __INLINE uint32_t SPI_GetSpi1ClockFreq()
{
    if(SPI1->CNTRL2 & SPI_CNTRL2_BCn_Msk)
    {
        /* Check the SPI peripheral clock source */
        if(SYSCLK->CLKSEL1 & SYSCLK_CLKSEL1_SPI1_S_Msk)
            return (SystemCoreClock/((SPI1->DIVIDER & 0xFF)+1)); /* SPI peripheral clock source: system clock */
        else
            return (PllClock/((SPI1->DIVIDER & 0xFF)+1)); /* SPI peripheral clock source: PLL */
    }
    else
    {
        /* Check the SPI peripheral clock source */
        if(SYSCLK->CLKSEL1 & SYSCLK_CLKSEL1_SPI1_S_Msk)
            return ((SystemCoreClock>>1)/((SPI1->DIVIDER & 0xFF)+1)); /* SPI peripheral clock source: system clock */
        else
            return ((PllClock>>1)/((SPI1->DIVIDER & 0xFF)+1)); /* SPI peripheral clock source: PLL */
    }
}

/**
  * @} End of SPI Device Function Interface
  */ 

/**
  * @} End of M051 Function Interface
  */ 


#endif

