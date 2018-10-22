/*
 * i2c.c
 *
 *  Created on: Sep 20, 2018
 *      Author: satya
 */
#include "em_i2c.h"
#include "src/i2c.h"
#include "src/sleep.h"
#include "sleep.h"

#include "src/gpio.h"

/*I2C Initialisation*/

void i2cinit()
{
	//SCL- PortC 10, SDA- PortC 11

	  GPIO_PinModeSet(gpioPortC, 10, gpioModeWiredAnd, 1); // SCL Before
	  GPIO_PinModeSet(gpioPortC, 11, gpioModeWiredAnd, 1);//SDA
	  GPIO_PinModeSet(gpioPortD, 15, gpioModePushPull, 1);
	  GPIO_PinOutSet(gpioPortD, 15);


	 for (int k=0;k<200000;k++);


	I2C0 -> ROUTEPEN = I2C_ROUTEPEN_SCLPEN | I2C_ROUTEPEN_SDAPEN;
	I2C0 -> ROUTELOC0 = I2C_ROUTELOC0_SDALOC_LOC16 |I2C_ROUTELOC0_SCLLOC_LOC14;



		const I2C_Init_TypeDef i2cinitialization = I2C_INIT_DEFAULT;

		I2C_Init(I2C0, &i2cinitialization);

		I2C_Enable(I2C0,true);

		for (int i=0; i<9; i++)
			{
			GPIO_PinOutClear(gpioPortC,10);
			GPIO_PinOutSet(gpioPortC,10);

			}

	if(I2C0->STATE & I2C_STATE_BUSY)
			{
				I2C0->CMD = I2C_CMD_ABORT;
			}
		I2C0->IFC= 0x00000000;
		//I2C_IntEnable(I2C0, I2C_IF_ACK);
		//NVIC_EnableIRQ(I2C0_IRQn);


}

/*Talk to sensor using I2C*/

uint16_t tempread()
{

	uint16_t read;
	/*send write*/
	I2C0->TXDATA = (slaveAdd << 1)| 0x00 ;
	I2C0->CMD = I2C_CMD_START;  //send start bit

	while((I2C0->IF & I2C_IF_ACK) == 0);
		I2C0->IFC = I2C_IFC_ACK;

			//I2C0->CMD = I2C_CMD_START;
			I2C0->TXDATA = tempMeasure;

			while((I2C0->IF & I2C_IF_ACK) == 0);
			I2C0->IFC = I2C_IFC_ACK;


			I2C0->CMD = I2C_CMD_START;
			I2C0->TXDATA = (slaveAdd << 1) | 0x01;


			while((I2C0->IF & I2C_IF_ACK) == 0);
			while((I2C0->IF & I2C_IF_RXDATAV) == 0);
			read = I2C0->RXDATA;
			I2C0->CMD = I2C_CMD_ACK;
			I2C0->IFC = I2C_IFC_ACK;
			read = read << 8;


			//I2C0->CMD = 0x04;
			while((I2C0->IF & I2C_IF_RXDATAV) == 0);


			read |= I2C0->RXDATA;
			//read8 = read8 >> 2;
			//read = read | read8;


			I2C0->CMD = I2C_CMD_NACK;

			I2C0->CMD = I2C_CMD_STOP;


			return read;
		}



int degree(int j)
{
	j = j * 175.72;
	j = j/65536;
	j = j-46.85;
	return j;
}

/*Disable I2C*/

void i2cdisable()
{
	I2C0 -> ROUTEPEN &= ~I2C_ROUTEPEN_SCLPEN;
	I2C0 -> ROUTEPEN &=~ I2C_ROUTEPEN_SDAPEN;

	I2C_Enable(I2C0, false);
	GPIO_PinOutClear(gpioPortC, 10); // SCL Before
	GPIO_PinOutClear(gpioPortC, 11);//SDA
	GPIO_PinOutClear(gpioPortD, 15);

}



