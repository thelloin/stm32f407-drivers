#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"
#include "hal_spi_driver.h"
#include "hal_gpio_driver.h"
#include "spi_main.h"
#include "led.h"
//#include "uart_debug.h"

/* SPI handle for our SPI device */
spi_handle_t SpiHandle;

int TestReady = 0;


/* Slave will reply this data, when master issues read command */
uint8_t slave_reply_data[4] = {0x55, 0xaa, 0x55, 0xaa};

/* Master read/write buffers */
uint8_t master_write_data[] = {0xa, 0xb, 0xc, 0xd};

uint8_t master_read_buffer[4];

/* configures gpio for spi functionality */
void spi_gpio_init(void)
{
	gpio_pin_conf_t spi_conf;
	
	
	_HAL_RCC_GPIOB_CLK_ENABLE();
	
	/* Configure GPIOB_PIN_13 for SPI CLK functionality */
	spi_conf.pin = SPI_CLK_PIN;
	spi_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	spi_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	spi_conf.pull = GPIO_PIN_PULL_DOWN;
	spi_conf.speed = GPIO_PIN_SPEED_MEDIUM;
	
	hal_gpio_set_alt_function(GPIOB, SPI_CLK_PIN, GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);
	
	/* Configure GPIOB_PIN_14 for SPI MISO functionality */
	spi_conf.pin = SPI_MISO_PIN;
	spi_conf.pull = GPIO_PIN_PULL_UP;
	hal_gpio_set_alt_function(GPIOB, SPI_MISO_PIN, GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);
	
	/* Configure GPIOB_PIN_15 for SPI MOSI functionality */
	spi_conf.pin = SPI_MOSI_PIN;
	spi_conf.pull = GPIO_PIN_PULL_UP;
	hal_gpio_set_alt_function(GPIOB, SPI_MOSI_PIN, GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);
	
}

/* Some delay generation */
void delay_gen()
{
	uint32_t cnt = 800000;
	while (cnt--);
}

// hang on here, if application can not proceed due to error

void assert_error(void)
{
	while(1)
	{
		led_toggle(GPIOD, LED_RED);
		delay_gen();
	}
}

/* function used to compare two buffers */
static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength)
{
	while (BufferLength--)
	{
		if ((*pBuffer1) != (*pBuffer2))
		{
			return BufferLength;
		}
		pBuffer1++;
		pBuffer2++;
	}
	return 0;
}

int main(void)
{
	uint32_t i=0;
	uint8_t addrcmd[CMD_LENGTH];
	uint8_t ack_buf[2];
	
	spi_gpio_init();
	
	/* To use LED */
	led_init();
	
	/* Configures USER Button interrupt */
	_HAL_RCC_GPIOA_CLK_ENABLE();
	hal_gpio_configure_interrupt(GPIO_BUTTON_PIN, INT_FALLING_EDGE);
	hal_gpio_enable_interrupt(GPIO_BUTTON_PIN, EXTI0_IRQn);
	
	/* Enable the clock to the SPI2 */
	_HAL_RCC_SPI2_CLK_ENABLE();
	
	/* Fill up the handle structure */
	SpiHandle.Instance               = SPI_2;
	SpiHandle.Init.BaudRatePrescaler = SPI_REG_CR1_BR_PCLK_DIV_32;
	SpiHandle.Init.Direction         = SPI_ENABLE_2_LINE_UNI_DIR;
	SpiHandle.Init.CLKPhase          = SPI_SECOND_CLOCK_TRANS;
	SpiHandle.Init.CLKPolarity       = SPI_CPOL_LOW;
	SpiHandle.Init.DataSize          = SPI_8BIT_DF_ENABLE;
	SpiHandle.Init.FirstBit          = SPI_TX_MSB_FIRST;
	SpiHandle.Init.NSS               = SPI_SSM_ENABLE;
	SpiHandle.Init.Mode              = SPI_MASTER_MODE_SEL;
	
	SpiHandle.State                  = HAL_SPI_STATE_READY;
	
	
	/* Call driver API to initialize the SPI device */
	hal_spi_init(&SpiHandle);
	
	/* Enable the IRQs in the NVIC */
	NVIC_EnableIRQ(SPI2_IRQn);
	
	/* First initialize the Debug UART */
	//hal_debug_uart_init( DEBUG_USART_BAUD_9600 );
	
	//uart_printf("SPI master Application Running ... \n");
	
	/* Wait for user Button press before starting the communication. Toggles LED_ORANGE until then */
	while (TestReady != SET)
	{
		led_toggle(GPIOD, LED_ORANGE);
		// LED3 (orange);
		delay_gen();
	}
	
	led_turn_off(GPIOD, LED_ORANGE);
	
	/*****************************************************************************/
	/*                                                                           */
	/*                     Master command sending code                           */
	/*                       Write and read command                              */
	/*****************************************************************************/
	
	while (1)
	{
		/* Check for state ready */
		while (SpiHandle.State != HAL_SPI_STATE_READY);
		
		/* Master write command */
		addrcmd[0] = (uint8_t) CMD_MASTER_WRITE;
		addrcmd[1] = (uint8_t) (CMD_MASTER_WRITE >> 8);
		
		/* First send the master write cmd to slave */
		hal_spi_master_tx(&SpiHandle, addrcmd, CMD_LENGTH);
		
		/* Application cal block here, or can do other task until above tx finishes */
		while (SpiHandle.State != HAL_SPI_STATE_READY);
		
		/* this delay helps for the slave to be ready with the ACK bytes */
		delay_gen();
		
		/* Read back the ACK bytes from the slave */
		hal_spi_master_rx(&SpiHandle, ack_buf, ACK_LEN);
		
		/* Wait until ACK reception finishes */
		while (SpiHandle.State != HAL_SPI_STATE_READY);
		
		/* Did we rcv the valid ACK from the slave ?? */
		if (ack_buf[1] == 0xE5 && ack_buf[0] == 0xD5)
		{
			// correct ack
			led_toggle(GPIOD, LED_GREEN);
			memset(ack_buf, 0, 2);
		}
		else
		{
			// invalid ack
			assert_error();
			memset(ack_buf, 0, 2);
		}
		
		/* Now send the data stream */
		hal_spi_master_tx(&SpiHandle, master_write_data, DATA_LENGTH);
		while (SpiHandle.State != HAL_SPI_STATE_READY);
		delay_gen();
		
		// Read from slave
		
		/* Master READ command */
		addrcmd[0] = (uint8_t) CMD_MASTER_READ;
		addrcmd[1] = (uint8_t) (CMD_MASTER_READ >> 8);
		
		/* first send the master write cmd to slave */
		hal_spi_master_tx(&SpiHandle, addrcmd, CMD_LENGTH);
		
		/* application can block here, or can do other tasks
		 * until above tc finishes 
		*/
		while (SpiHandle.State != HAL_SPI_STATE_READY);
		
		/* This delay helps for the slave to be ready with the ACK bytes */
		delay_gen();
		
		/* read back the ACK bytes from the slave */
		hal_spi_master_rx(&SpiHandle, ack_buf, ACK_LEN);
		
		while (SpiHandle.State != HAL_SPI_STATE_READY);
		
		if (ack_buf[1] == 0xE5 && ack_buf[0] == 0xE5)
		{
			// correct ack
			led_toggle(GPIOD, LED_GREEN);
			memset(ack_buf, 0, 2);
		}
		else
		{
			// invalide ack
			assert_error();
			memset(ack_buf, 0, 2);
		}
		
		/* start receiving from the slave */
		hal_spi_master_rx(&SpiHandle, master_read_buffer, DATA_LENGTH);
		
		while(SpiHandle.State != HAL_SPI_STATE_READY);
		
		/* Compare the data rcvd from slave, with what slave supposed to send */
		if (Buffercmp(master_read_buffer, slave_reply_data, DATA_LENGTH))
		{
			// We didn't rcv what needs to be rcvd !!! Error !
			led_toggle(GPIOD, LED_RED);
		}
		else
		{
			// Rcvd correct data
			led_toggle(GPIOD, LED_BLUE);
			for (i = 0; i < DATA_LENGTH; i++)
				; // uart_printf.....
		}
		
		delay_gen();
	}
	
	return 0;
}

/**
  * @brief  This function handles SPI2 interrupt request
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
	/* Call the driver api to process the interrupt */
	hal_spi_irq_handler(&SpiHandle);
}

/**
  * @brief  This function handles EXTI0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
	/* In the ISR, first clear out the sticky interrupt pending bit for this interrupt */
	hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	/* Do your task here */
	TestReady = SET;
}


