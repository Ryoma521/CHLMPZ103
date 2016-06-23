#ifndef SI4463_SPI_H
#define SI4463_SPI_H

#include "hal_types.h"

#define SPIWRITE 0x01
#define SPIREAD 0x00

U8 bSpi_ReadWriteSpi1(U8 biDataIn,U8 wrFlag);
void vSpi_ReadDataSpi1(U16 biDataOutLength, U8 *paboDataOut);
void vSpi_WriteDataSpi1(U16 biDataInLength, U8 *pabiDataIn);

#endif
