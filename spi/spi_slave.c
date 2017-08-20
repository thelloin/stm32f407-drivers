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
	
}
