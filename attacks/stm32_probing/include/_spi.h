#ifndef SPI_H_
#define SPI_H_

#include <stddef.h>

void spi_init();

void spi_transmit(uint8_t *data,uint32_t size);

void cs_low();

void cs_high();

#endif /* SPI_H_ */
