#include "hal_i2c_driver.h"
#include "led.h"

/*************************************************************************/
/*                                                                       */
/*                          Helper functions                             */
/*                                                                       */
/*************************************************************************/

/*
 * @brief  Enables the given I2C module
 * @param  *i2cx : Base address of the I2C Peripheral
 * @retval None
 */
static void hal_i2c_enable_peripheral(I2C_TypeDef *i2cx)
{
	i2cx->CR1 |= I2C_REG_CR1_ENABLE_I2C;
}

/*
 * @brief  Disables the given I2C module
 * @param  *i2cx : Base address of the I2C Peripheral
 * @retval None
 */
static void hal_i2c_disable_peripheral(I2C_TypeDef *i2cx)
{
	i2cx->CR1 &= ~I2C_REG_CR1_ENABLE_I2C;
}

/*
 * @brief  Manages the clock stretching of the I2C device
 * @param  *i2cx : Base address of the I2C Peripheral
 * @param  no_stretch : clock stretching enable/disable
 * @retval None
 */
static void hal_i2c_manage_clock_stretch(I2C_TypeDef *i2cx, uint32_t no_stretch)
{
	if (no_stretch)
	{
		// clock stretching disabled
		i2cx->CR1 |= I2C_REG_CR1_NOSTRETCH;
	} else
	{
		i2cx->CR1 &= ~I2C_REG_CR1_NOSTRETCH;
	}
}

/*
 * @brief  Configures the own I2C device address
 * @param  *i2cx : Base address of the I2C Peripheral
 * @param  own_address : Address of the I2C Device to be configured
 * @retval None
 */
static void hal_i2c_set_own_address1(I2C_TypeDef *i2cx, uint32_t own_address)
{
	i2cx->OAR1 &= ~(0x7f << 1);
	i2cx->OAR1 |=  (own_address << 1);
}

/*
 * @brief  Configures I2C addressing mode either 7 or 10 bit
 * @param  *i2cx : Base address of the I2C Peripheral
 * @param  adr_mode : addressing mode to be configured
 * @retval None
 */
static void hal_i2c_set_addressing_mode(I2C_TypeDef *i2cx, uint32_t adr_mode)
{
	if (adr_mode == I2C_ADDRMODE_10BIT)
		i2cx->OAR1 |= I2C_REG_OAR1_ADDRMODE;
	else
		i2cx->OAR1 &= ~I2C_REG_OAR1_ADDRMODE;
}

/*
 * @brief  Configures I2C Clock duty cycle in FM mode
 * @param  *i2cx : Base address of the I2C Peripheral
 * @param  duty_cycle : duty cycle to be configured, could be either I2C_FM_DUTY_16Y9
           or I2C_FM_DUTY_2
 * @retval None
 */
static void hal_i2c_set_fm_mode_duty_cycle(I2C_TypeDef *i2cx, uint32_t duty_cycle)
{
	if (duty_cycle == I2C_FM_DUTY_16BY9)
	{
		i2cx->CCR |= I2C_REG_CCR_DUTY;
	} else
	{
		i2cx->CCR &= ~I2C_REG_CCR_DUTY;
	}
}

/**
  * @brief  Does I2C clock realted initialization
  * @param  *i2cx : Base address of the I2C peripheral
  * @param  clkspeed : I2C clock speed
  * @param  duty_cycle : I2C clock duty cycle
  * @retval None
  */
static void hal_i2c_clk_init(I2C_TypeDef *i2cx, uint32_t clkspeed, uint32_t duty_cycle)
{
	uint32_t pclk = I2C_PERIPHERAL_CLK_FREQ_10MHZ;
	i2cx->CR2 &= ~(0x3F);
	i2cx->CR2 |= (pclk & 0x3F);
	hal_i2c_configure_ccr(i2cx, pclk, clkspeed, duty_cycle);
	hal_i2c_rise_time_configuration(i2cx, pclk, clkspeed);
}

/**
  * @brief  Generate start condition, used only by master
  * @param  *i2cx : Base address of the I2C peripheral
  * @retval None
  */
static void hal_i2c_generate_start_condition(I2C_TypeDef *i2cx)
{
	i2cx->CR1 |= I2C_REG_CR1_START_GEN;
}

/**
  * @brief  Generate stop condition, used only by master
  * @param  *i2cx : Base address of the I2C peripheral
  * @retval None
  */
static void hal_i2c_generate_stop_condition(I2C_TypeDef *i2cx)
{
	i2cx->CR1 |= I2C_REG_CR1_STOP_GEN;
}

/**
  * @brief  Enables/disables TXE and RXNE(buffer) interrupt
  * @param  *i2cx : Base address of the I2C peripheral
  * @param  enable : if this is set to zero, then tx, rx interrupts will be disabled
  * @retval None
  */
static void hal_i2c_configure_buffer_interrupt(I2C_TypeDef *i2cx, uint32_t enable)
{
	if (enable)
	{
		i2cx->CR2 |= I2C_REG_CR2_BUF_INT_ENABLE;
	}
	else
	{
		i2cx->CR2 &= ~I2C_REG_CR2_BUF_INT_ENABLE;
	}
}

/**
  * @brief  Enables/disables error interrupt
  * @param  *i2cx : Base address of the I2C peripheral
  * @param  enable : if this is set to zero, then error interrupts will be disables
  * @retval None
  */
static void hal_i2c_configure_error_interrupt(I2C_TypeDef *i2cx, uint32_t enable)
{
	if (enable)
	{
		i2cx->CR2 |= I2C_REG_CR2_ERR_INT_ENABLE;
	}
	else
	{
		i2cx->CR2 &= ~I2C_REG_CR2_ERR_INT_ENABLE;
	}
}

/**
  * @brief  Enables/disables I2C EVT interrupt
  * @param  *i2cx : Base address of the I2C peripheral
  * @param  enable : if this is set to zero, then evt interrupts will be disables
  * @retval None
  */
static void hal_i2c_configure_evt_interrupt(I2C_TypeDef *i2cx, uint32_t enable)
{
	if (enable)
	{
		i2cx->CR2 |= I2C_REG_CR2_EVT_INT_ENABLE;
	}
	else
	{
		i2cx->CR2 &= ~I2C_REG_CR2_EVT_INT_ENABLE;
	}
}

/**
  * @brief  Check whether bus is free or busy
  * @param  *i2cx : Base address of the I2C peripheral
  * @retval : return 1, if the bus is busy
  */
static uint8_t is_bus_busy(I2C_TypeDef *i2cx)
{
	if (i2cx->SR2 & I2C_REG_SR2_BUS_BUSY_FLAG)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/**
  * @brief  Call this function to wait until SB(start byte) flag is set.
  *         It is set when master has successfully generated the start condition.
  * @param  *i2cx : Base address of the I2C peripheral
  * @retval None
  */
static void i2c_wait_until_sb_is_set(I2C_TypeDef *i2cx)
{
	// Wait until SB is successfully generated by the master.
  while (!(i2cx->SR1 & I2C_REG_SR1_SB_FLAG));
}

/**
  * @brief  Call this function to wait until ADDR flag is set.
  *         It is set when master has successfully finished sending
  * @param  *i2cx : Base address of the I2C peripheral
  * @retval None
  */
static void i2c_wait_until_addr_is_set(I2C_TypeDef *i2cx)
{
	while (!(i2cx->SR1 & I2C_REG_SR1_ADDR_SENT_FLAG));
}

static void hal_i2c_send_addr_first(I2C_TypeDef *i2cx, uint8_t address)
{
	i2cx->DR = address;
}

void clear_addr_flag(I2C_TypeDef *i2cx)
{
	uint16_t val;
	
	val = i2cx->SR1;
	val = i2cx->SR2;
}

/*************************************************************************/
/*                                                                       */
/*                         Driver exposed APIs                           */
/*                                                                       */
/*************************************************************************/

/**
  * @brief  Initializes the given I2C Peripheral
  * @param  *handle : Handle to the I2C Peripheral, which the application wants to initialize
  * @retval None
  */
void hal_i2c_init(i2c_handle_t *handle)
{
	// I2C Clock Init
	
	// Set I2C addressing mode
	
	// Enable the ACKing
	
	// Enable clock stretching
	
	// Configure the own address
	
	// Enable the I2C peripheral
}

void hal_i2c_manage_ack(I2C_TypeDef *i2cx, uint32_t ack_noack);

/**
  * @brief  API to do master data transmission
  * @param  *handle : pointer to the handle structure of the I2C Peripheral
  * @param  slave_address : addres to which we want to tx
  * @param  *buffer : holds the pointer to the tx buffer
  * @param  len : length of the data to be TXed
  * @retval None
  */
void hal_i2c_master_tx(i2c_handle_t *handle, uint8_t slave_address, uint8_t *buffer, uint32_t len)
{
	/* Populate the handle with tx buffer pointer and length information */
	handle->pBuffPtr = buffer;
	handle->XferCount = len;
	handle->XferSize = len;
	handle->State = HAL_I2C_STATE_BUSY_TX;
	
	/* Make sure that I2C is enables */
	hal_i2c_enable_peripheral(handle->Instance);
	
	/* First, generate the start condition by using helper function */
	hal_i2c_generate_start_condition(handle->Instance);
	
	/* Wait until SB is set */
	i2c_wait_until_sb_is_set(handle->Instance);
	
	/* Address phase; send the 8-bit slave address first(8th bit is r/w bit) */
	hal_i2c_send_addr_first(handle->Instance, slave_address);
	
	i2c_wait_until_addr_is_set(handle->Instance);
	
	/* If we are here, that means ADDR is set, and clock is stretched(wait state) */
	/* Clearing the ADDR flag make I2C come out of wait state */
	clear_addr_flag(handle->Instance);
	
	/* Enable the buff, err, and event interrupt */
	hal_i2c_configure_buffer_interrupt(handle->Instance, 1);
	hal_i2c_configure_error_interrupt(handle->Instance, 1);
	hal_i2c_configure_evt_interrupt(handle->Instance, 1);
}

/**
  * @brief  API to do master data reception
  * @param  *handle : pointer to the handle structure of the I2C Peripheral
  * @param  slave_address : address to who sends the data
  * @param  *buffer : holds the pointer to the RX buffer
  * @param  len : length of the data to be RXed
  * @retval None
  */
void hal_i2c_master_rx(i2c_handle_t *handle, uint8_t slave_address, uint8_t *buffer, uint32_t len)
{
	/* Populate the handle with rx buffer pointer and length information */
	handle->pBuffPtr = buffer;
	handle->XferCount = len;
	handle->XferSize = len;
	
	/* Make state is busy in RX */
	handle->State = HAL_I2C_STATE_BUSY_RX;
	
	/* Enable the I2C peripheral */
	hal_i2c_enable_peripheral(handle->Instance);
	
	/* Make sure that POS bit is disabled */
	handle->Instance->CR1 &= ~I2C_CR1_POS;
	
	/* Make sure ACKing is enabled */
	handle->Instance->CR1 |= I2C_CR1_ACK;
	
	/* First, generate the start condition by using our helper functions */
	hal_i2c_generate_start_condition(handle->Instance);
	
	/* Wait until SB is set */
	i2c_wait_until_sb_is_set(handle->Instance);
	
	/* Send the slave address */
	hal_i2c_send_addr_first(handle->Instance, slave_address);
	
	/* Wait until ADDR =1, that means address phase is completed successfully */
	i2c_wait_until_addr_is_set(handle->Instance);
	
	/* Clear the ADDR flag which is set */
	clear_addr_flag(handle->Instance);
	
	/* Enable the buff, err, and event interrupts */
	hal_i2c_configure_buffer_interrupt(handle->Instance, 1);
	hal_i2c_configure_error_interrupt(handle->Instance, 1);
	hal_i2c_configure_evt_interrupt(handle->Instance, 1);
}

/**
  * @brief  API to do slave data transmission
  * @param  *handle : pointer to the handle structure of the I2C Peripheral
  * @param  *buffer : holds the pointer to the tx buffer
  * @param  len : length of the data to be TXed
  * @retval None
  */
void hal_i2c_slave_tx(i2c_handle_t *handle, uint8_t *buffer, uint32_t len);

/**
  * @brief  API to do slave data reception
  * @param  *handle : pointer to the handle structure of the I2C Peripheral
  * @param  *buffer : holds the pointer to the RX buffer
  * @param  len : length of the data to be RXed
  * @retval None
	*/
void hal_i2c_slave_rx(i2c_handle_t *handle, uint8_t *buffer, uint32_t len);

/**
  * @brief  This function handles I2C error interrupt request
  * @param  hi2c : pointer to a i2c_handle_t structure that contains
                   the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_EV_IRQHandler(i2c_handle_t *hi2c);

/**
  * @brief  This function handles I2C event interrupt request
  * @param  hi2c : pointer to a i2c_handle_t structure that contains
  *                the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_ER_IRQHandler(i2c_handle_t *hi2c);