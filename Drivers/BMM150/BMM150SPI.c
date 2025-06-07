#include "BMM150SPI.h"
#include "stm32g4xx_hal.h"
#include "bmm150_def.h"

#define BMM150_TIMEOUT_DURATION 10

struct spi_interface {
	SPI_HandleTypeDef* spi_handle;
	GPIO_TypeDef* chip_select_port;
	uint16_t chip_select_pin;
};

void BMM150_enable_chip_select(GPIO_TypeDef* cs_port, uint16_t cs_gpio_pin)
{
	HAL_GPIO_WritePin(cs_port, cs_gpio_pin, GPIO_PIN_RESET);
}

void BMM150_disable_chip_select(GPIO_TypeDef* cs_port, uint16_t cs_gpio_pin)
{
	HAL_GPIO_WritePin(cs_port, cs_gpio_pin, GPIO_PIN_SET);
}

/* Avoid declaring this in BMM150 driver
void BMM150_delay(int period_in_micro_s, void* intf_ptr)
{
	//tick is approximately 1 ms
	const int tick_amount = 1000;
	osDelay(period_in_micro_s / tick_amount);
}
*/

BMM150_INTF_RET_TYPE BMM150_SPI_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	HAL_StatusTypeDef status = HAL_OK;
	struct spi_interface *spi = (struct spi_interface *)intf_ptr;

	// First byte needs to have MSB set to indicate a read operation
	uint8_t tx_buf[1];
    tx_buf[0] = reg_addr | 0x80;
	// First byte received is dummy data
	uint8_t rx_buf[length + 1];

	BMM150_enable_chip_select(spi->chip_select_port, spi->chip_select_pin);

	status = HAL_SPI_TransmitReceive(spi->spi_handle, tx_buf, rx_buf, length + 1, BMM150_TIMEOUT_DURATION);

	BMM150_disable_chip_select(spi->chip_select_port, spi->chip_select_pin);

	if (status == HAL_OK)
		memcpy(reg_data, &rx_buf[1], length);

	return (BMM150_INTF_RET_TYPE)status;
}

BMM150_INTF_RET_TYPE BMM150_SPI_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
	HAL_StatusTypeDef status = HAL_OK;
	struct spi_interface *spi = (struct spi_interface *)intf_ptr;

	// Ensure that MSB is 0 to indicate a write operation
	uint8_t tx_buf[length + 1];
	tx_buf[0] = reg_addr & 0x7F;
	memcpy(&tx_buf[1], reg_data, length);

	BMM150_enable_chip_select(spi->chip_select_port, spi->chip_select_pin);

	status = HAL_SPI_Transmit(spi->spi_handle, tx_buf, length + 1, BMM150_TIMEOUT_DURATION);

	BMM150_disable_chip_select(spi->chip_select_port, spi->chip_select_pin);

	return (BMM150_INTF_RET_TYPE)status;
}

struct bmm150_dev BMM150_spi_init(SPI_HandleTypeDef *spi_handle, GPIO_TypeDef* cs_port, uint16_t cs_gpio_pin)
{
	struct bmm150_dev bmm150 = (struct bmm150_dev){ 0 };

	if (spi_handle == NULL)
	{
		bmm150.intf_rslt = BMM150_E_NULL_PTR;
		return bmm150;
	}

	struct spi_interface my_spi = {
		.spi_handle = &spi_handle,
		.chip_select_port = &cs_port,
		.chip_select_pin = cs_gpio_pin
	};

	bmm150.intf = BMM150_SPI_INTF;
	bmm150.read = BMM150_SPI_read;
	bmm150.write = BMM150_SPI_write;
	bmm150.delay_us = HAL_Delay;
	bmm150.intf_ptr = &my_spi;

	//NOTE: Potential error: is chip id initialized properly?
	bmm150.intf_rslt = bmm150_init(&bmm150);

	return bmm150;
}

//
BMM150_mag_data BMM150_read_mag_data(struct bmm150_dev* bmm150)
{
	BMM150_mag_data mag_data = (BMM150_mag_data) {0};

	if (bmm150->intf_rslt)
	{
		mag_data.error = BMM150_E_NULL_PTR;
		return mag_data;
	}

	struct bmm150_mag_data data = (struct bmm150_mag_data){ 0 };

	//int8_t bmm150_read_mag_data(struct bmm150_mag_data *mag_data, struct bmm150_dev *dev)
	mag_data.error = bmm150_read_mag_data(&data, bmm150);

	mag_data.x = data.x;
	mag_data.y = data.y;
	mag_data.z = data.z;

	return mag_data;
}
