#include <stdint.h>
#include "stm32f4xx.h"
#include "hal_spi_driver.h"
#include "hal_gpio_driver.h"
#include "spi_main.h"
#include "led.h"


spi_handle_t SpiHandle;

int TestReady = 0;

uint8_t master_write_data[4] = { 0xA, 0xB, 0xC, 0xD };

uint8_t slave_tx_buffer[4] = { 0x55, 0xAA, 0x55, 0xAA };
uint8_t slave_rx_buffer[4];

/* Configure gpio for spi functionality */
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
	uint16_t ack_bytes = SPI_ACK_BYTES;
	uint8_t rcv_cmd[2];
	uint8_t ack_buf[2] = {0xD5, 0xE5};
	uint16_t master_cmd;
	
	/* Condigures the gpios for SPI functionality */
	spi_gpio_init();
	
	/* To use LED */
	led_init();
	
	/* Enable the clock for SPI2 */
	_HAL_RCC_SPI2_CLK_ENABLE();
	
	/* Fill up the handle structure */
	SpiHandle.Instance = SPI_2;
	SpiHandle.Init.BaudRatePrescaler = SPI_REG_CR1_BR_PCLK_DIV_32;
	SpiHandle.Init.Direction         = SPI_ENABLE_2_LINE_UNI_DIR;
	SpiHandle.Init.CLKPhase          = SPI_SECOND_CLOCK_TRANS;
	SpiHandle.Init.CLKPolarity       = SPI_CPOL_LOW;
	SpiHandle.Init.DataSize          = SPI_8BIT_DF_ENABLE;
	SpiHandle.Init.FirstBit          = SPI_TX_MSB_FIRST;
	SpiHandle.Init.NSS               = SPI_SSM_ENABLE;
	SpiHandle.Init.Mode              = SPI_SLAVE_MODE_SEL;
	
	SpiHandle.State = HAL_SPI_STATE_READY;
	
	
	/* Call driver API to initialize the SPI device */
	hal_spi_init(&SpiHandle);
	
	/* Enable the IRQs in the NVIC */
	NVIC_EnableIRQ(SPI2_IRQn);
	
	
	while(1)
	{
		/* Make sure that driver state is ready */
		while (SpiHandle.State != HAL_SPI_STATE_READY);
		
		/* Recieve the master command first */
		hal_spi_slave_rx(&SpiHandle, rcv_cmd, CMD_LENGTH);
		
		/* Wait until driver finishes RXing and state becomes ready again */
		while (SpiHandle.State != HAL_SPI_STATE_READY);
		
		/* This is the command slave got */
		master_cmd = (uint16_t) (rcv_cmd[1] << 8 | rcv_cmd[0]);
		
		/* Is it a valid command ? */
		if (master_cmd == CMD_MASTER_READ || master_cmd == CMD_MASTER_WRITE)
		{
			/* yes, send out the ACK bytes */
			hal_spi_slave_tx(&SpiHandle, (uint8_t *) &ack_buf, ACK_LEN);
			while (SpiHandle.State != HAL_SPI_STATE_READY);
		}
		else
		{
			/* No, Error! */
			led_toggle(GPIOD, LED_RED);
		}
		
	}
}