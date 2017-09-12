#ifndef __HAL_I2C_DRIVER_H
#define __HAL_I2C_DRIVER_H

#include "stm32f407xx.h"
#include <stdint.h>


/* I2C device base address */
#define I2C_1 I2C1
#define I2C_2 I2C2
#define I2C_3 I2C3

/* Macros to Enable Clock for different I2C devices */

#define _HAL_RCC_I2C1_CLK_ENABLE()        ( RCC->APB1ENR |= (1 << 21) )
#define _HAL_RCC_I2C2_CLK_ENABLE()        ( RCC->APB1ENR |= (1 << 22) )
#define _HAL_RCC_I2C3_CLK_ENABLE()        ( RCC->APB1ENR |= (1 << 23) )


/*************************************************************************/
/*                                                                       */
/*                                I2C                                    */
/*                        Register Bit Definition                        */
/*************************************************************************/

/******************  Bit definition for I2C_CR1 register  ****************/

#define I2C_REG_CR1_POS        ((uint32_t)1 << 11)

#define I2C_REG_CR1_ACK        ((uint32_t)1 << 10)
#define I2C_ACK_ENABLE          1
#define I2C_ACK_DISABLE         0

#define I2C_REG_CR1_STOP_GEN           ((uint32_t)1 << 9)
#define I2C_REG_CR1_START_GEN          ((uint32_t)1 << 8)

#define I2C_REG_CR1_NOSTRETCH          ((uint32_t)1 << 7)
#define I2C_ENABLE_CLK_STRETCH          0
#define I2C_DISABLE_CLK_STRETCH         1

#define I2C_REG_CR1_ENABLE_I2C         ((uint32_t) << 0)

/******************  Bit definition for I2C_CR2 register  ****************/
#define I2C_REG_CR2_BUF_INT_ENABLE     ((uint32_t) (1 << 10))
#define I2C_REG_CR2_EVT_INT_ENABLE     ((uint32_t) (1 << 9))
#define I2C_REG_CR2_ERR_INT_ENABLE     ((uint32_t) (1 << 8))

#define I2C_PERIPHERAL_CLK_FREQ_2MHZ   ((uint32_t) 2)
#define I2C_PERIPHERAL_CLK_FREQ_3MHZ   ((uint32_t) 3)
#define I2C_PERIPHERAL_CLK_FREQ_4MHZ   ((uint32_t) 4)
#define I2C_PERIPHERAL_CLK_FREQ_5MHZ   ((uint32_t) 5)
#define I2C_PERIPHERAL_CLK_FREQ_6MHZ   ((uint32_t) 6)
#define I2C_PERIPHERAL_CLK_FREQ_7MHZ   ((uint32_t) 7)
#define I2C_PERIPHERAL_CLK_FREQ_8MHZ   ((uint32_t) 8)
#define I2C_PERIPHERAL_CLK_FREQ_9MHZ   ((uint32_t) 9)
#define I2C_PERIPHERAL_CLK_FREQ_10MHZ  ((uint32_t) 10)

/******************  Bit definition for I2C_OAR1 register  ***************/
#define I2C_REG_OAR1_ADDRMODE          ((uint32_t) 1 << 15)
#define I2C_ADDRMODE_7BIT               0
#define I2C_ADDRMODE_10BIT              1

#define I2C_REG_OAR1_14TH_BIT          ((uint32_t) 1 << 14)
#define I2C_REG_OAR1_7BIT_ADDRESS_POS   1

/******************  Bit definition for I2C_SR1 register  ****************/
#define I2C_REG_SR1_TIMEOUT_FLAG        ((uint32_t) 1 << 14)
#define I2C_REG_SR1_OVR_FLAG            ((uint32_t) 1 << 11)
#define I2C_REG_SR1_AF_FAILURE_FLAG     ((uint32_t) 1 << 10)
#define I2C_REG_SR1_ARLO_FLAG           ((uint32_t) 1 << 9)
#define I2C_REG_SR1_BUS_ERROR_FLAG      ((uint32_t) 1 << 8)
#define I2C_REG_SR1_TXE_FLAG            ((uint32_t) 1 << 7)
#define I2C_REG_SR1_RXNE_FLAG           ((uint32_t) 1 << 6)
#define I2C_REG_SR1_STOP_DETECTION_FLAG ((uint32_t) 1 << 4) // For slave
#define I2C_REG_SR1_BTF_FLAG            ((uint32_t) 1 << 2)
#define I2C_REG_SR1_ADDR_FLAG           ((uint32_t) 1 << 1)
#define I2C_REG_SR1_ADDR_SENT_FLAG      ((uint32_t) 1 << 1) //For master
#define I2C_REG_SR1_ADDR_MATCHED_FLAG   ((uint32_t) 1 << 1) //For slave
#define I2C_REG_SR1_SB_FLAG             ((uint32_t) 1 << 0)

/******************  Bit definition for I2C_SR2 register  ****************/
#define I2C_REG_SR2_BUS_BUSY_FLAG       ((uint32_t) 1 << 1)
#define I2C_BUS_IS_BUSY                  1
#define I2C_BUS_IS_FREE                  0

#define I2C_REG_SR2_MSL_FLAG            ((uint32_t) 1 << 0)
#define I2C_MASTER_MODE                  1
#define I2C_SLAVE_MODE                   0

#define I2C_REG_SR2_TRA_FLAG            ((uint32_t) 1 << 2)
#define I2C_RX_MODE                      0
#define I2C_TX_MODE                      1

/******************  Bit definition for I2C_CCR register  ****************/
#define I2C_REG_CCR_ENABLE_FM           ((uint32_t) 1 << 15)
#define I2C_ENABLE_SM                    0
#define I2C_ENABLE_FM                    1

#define I2C_REG_CCR_DUTY                ((uint32_t) 1 << 14)
#define I2C_FM_DUTY_16BY9                1
#define I2C_FM_DUTY_2                    0

/*************************************************************************/
/*                                                                       */
/*                    Data Structures used by I2C Driver                 */
/*                                                                       */
/*************************************************************************/

/**
  * @brief HAL I2C State structure definition
	*/
typedef enum
{
	HAL_I2C_STATE_RESET           = 0x00,  /* I2C not yet initialized or disabled         */
	HAL_I2C_STATE_READY           = 0x01,  /* I2C initialized and ready for use           */
	HAL_I2C_STATE_BUSY            = 0x02,  /* I2C internal process is ongoing             */
	HAL_I2C_STATE_BUSY_TX         = 0x12,  /* Data Transmission process is ongoing        */
	HAL_I2C_STATE_BUSY_RX         = 0x22,  /* Data Reception process is ongoing           */
	HAL_I2C_STATE_MEM_BUSY_TX     = 0x32,  /* Memory Data Transmission process is ongoing */
	HAL_I2C_STATE_MEM_BUSY_RX     = 0x42,  /* Memory Data Reception process is ongoing    */
	HAL_I2C_STATE_TIMEOUT         = 0x03,  /* I2C timeout state                           */
	HAL_I2C_STATE_ERROR           = 0x04   /* I2C error state                             */
} hal_i2c_state_t;

/**
  * @brief I2C Configuration Structure definition
  */
typedef enum
{
	uint32_t ClockSpeed;
	uint32_t DutyCycle;
	uint32_t OwnAddress;
	uint32_t AddressingMode;
	uint32_t DualAddressMode;
	uint32_t OwnAddress2;
	uint32_t GeneralCallMode;
	uint32_t NoStretchMode;
	uint32_t ack_enable;
	uint32_t master;
	
};