

#include "i2c.h"
#include "src/gpio.h"
#include "stdio.h"
#include "lcd_driver.h"



void i2c_init(void)
{
		//blockSleepmode(sleepEM2);

	GPIO_PinModeSet(SCL_PORT, SCL_PIN ,gpioModeWiredAnd,1 ); //SCL Line

	GPIO_PinModeSet(SDA_PORT,SDA_PIN,gpioModeWiredAnd,1 ); //SDA Line

	I2C0->ROUTEPEN |= I2C_ROUTEPEN_SCLPEN | I2C_ROUTEPEN_SDAPEN; //Setting SCL and SDA

	I2C0->ROUTELOC0 |= (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SCLLOC_MASK))| I2C_ROUTELOC0_SCLLOC_LOC14;

	I2C0->ROUTELOC0 |= (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SDALOC_MASK))| I2C_ROUTELOC0_SDALOC_LOC16;

	const I2C_Init_TypeDef i2c_reg = I2C_INIT_SEN;
	I2C_Init(I2C0,&i2c_reg);

	I2C_Enable(I2C0,true);
//	GPIO_PinOutSet(sensor_port,sensor_pin);

	int i=0;
	while (i<400000)
	{
		i++;
	}

	//Reset I2C Slave if Required
	for (int i=0; i<9 ; i++)
	{
		GPIO_PinOutClear(SCL_PORT,SCL_PIN);
		GPIO_PinOutSet(SCL_PORT,SCL_PIN);
		//GPIO_PinOutSet(SDA_PORT,SDA_PIN);
	}

	if (I2C0->STATE & I2C_STATE_BUSY)
		{
			I2C0->CMD = I2C_CMD_ABORT;
		}

	//Clear all Interrupts
	//I2C0->IFC= 0x00000000;
}



void i2c_disable (void)
{
	//Disable Interrupts
	I2C0->ROUTEPEN &= ~(I2C_ROUTEPEN_SCLPEN | I2C_ROUTEPEN_SDAPEN);
	I2C_Enable(I2C0,false);

	GPIO_PinOutClear(SDA_PORT,SDA_PIN);
	GPIO_PinOutClear(SCL_PORT,SCL_PIN);
	//GPIO_PinOutClear(sensor_port,sensor_pin);

//	unblockSleepmode(2);
}

void i2c_driver (void)
{
	I2C0->TXDATA = (0x40<<1)|(0x00); 	//Initialize Tx Buffer

	I2C0->CMD= I2C_CMD_START; 	//Send Start

	while ((I2C0->IF & I2C_IF_ACK)==0){};

	I2C0->IFC=I2C_IFC_ACK; //Clear interrupt

	I2C0->TXDATA = 0xE3; //Initialize the TX Buffer

	while ((I2C0->IF & I2C_IF_ACK)==0){};  //Check ack

	I2C0->IFC=I2C_IFC_ACK; //Clear Interrupt

	I2C0->CMD= I2C_CMD_START; 	//Send Start

	I2C0->TXDATA =(0x40<<1)|(0x01);  //TX Buffer initialization Read

	while ((I2C0->IF & I2C_IF_ACK)==0){};  //Check ack

	I2C0->IFC =I2C_IFC_ACK;  //Clear Interrupt

	while ((I2C0->IF & I2C_IF_RXDATAV)==0); //Receive data from sensor

	temperature = I2C0-> RXDATA;	//Store as temperature

	temperature= temperature<<8;	//Only the MSB

	I2C0->CMD = I2C_CMD_ACK;

	while ((I2C0->IF & I2C_IF_RXDATAV)==0);

	temperature |= I2C0-> RXDATA;	//Only the LSB

	I2C0->CMD = I2C_CMD_NACK; //Send NACK

	I2C0->CMD = I2C_CMD_STOP; //Send Stop Command

}


void write_byte(uint8_t slave_address, uint8_t reg_address, uint8_t data_byte){
	I2C0->TXDATA =((slave_address << 1) & (0xFE)); //Set the slave address and write bit which is zero
	I2C0->CMD = I2C_CMD_START; //Start condition

	while((I2C0->IF & I2C_IF_ACK)==0); //Wait for acknowledge signal
	I2C0->IFC = (I2C_IFC_ACK );		//Clear the Acknowledge flag

	I2C0->TXDATA = reg_address;	//Send register address
	while((I2C0->IF & I2C_IF_ACK)==0); //Wait for Acknowledgement signal after sending data
	I2C0->IFC = (I2C_IFC_ACK );		//Clear the Acknowledge flag

	I2C0->TXDATA = data_byte;	//Send data
	while((I2C0->IF & I2C_IF_ACK)==0); //Wait for Acknowledgement signal after sending data
	I2C0->IFC = (I2C_IFC_ACK );		//Clear the Acknowledge flag

	I2C0->CMD = I2C_CMD_STOP;	//Send stop command

}

void read_byte(uint8_t slave_address, uint8_t reg_address, uint8_t* data_sixty_four){
	uint8_t i=0;	//Count to track how many bytes are read

	I2C0->CMD = I2C_CMD_START; //Start condition
	while(I2C0->CMD & I2C_STATUS_PSTART);

	I2C0->TXDATA = ((slave_address << 1) & (0xFE)); //Set the slave address and write bit which is zero

	while((I2C0->IF & I2C_IF_ACK)==0); //Wait for acknowledge signal
	I2C0->IFC = (I2C_IFC_ACK );		//Clear the Acknowledge flag

	I2C0->TXDATA = reg_address;	//Send register address


	while((I2C0->IF & I2C_IF_ACK)==0); //Wait for Acknowledgement signal after sending data
	I2C0->IFC = (I2C_IFC_ACK );		//Clear the Acknowledge flag

	I2C0->CMD = I2C_CMD_START; //Repeated Start condition
	while(I2C0->CMD & I2C_STATUS_PSTART);

	I2C0->TXDATA = ((slave_address << 1) | (0x01)); //Set the slave address and read bit which is one

	while((I2C0->IF & I2C_IF_ACK)==0); //Wait for acknowledge signal
	I2C0->IFC = (I2C_IFC_ACK );		//Clear the Acknowledge flag

	for(i=0;i<127;i++){
		while((I2C0->IF & I2C_IF_RXDATAV) ==0);	//Wait until Receive buffer contains some value
		*(data_sixty_four + i) = I2C0->RXDATA;	//The RXDATAV interrupt flag is cleared when we read data from receive Buffer
		//I2C0->IFC = I2C_IFC_ACK;				//Clear the acknowledgement flag
		I2C0->CMD = I2C_CMD_ACK;				//Transmit acknowledgement from master to slave to request next data

	}

	I2C0->IFC |= I2C_IFC_ACK;				//Clear the acknowledgement flag
	while((I2C0->IF & I2C_IF_RXDATAV) ==0);	//Wait until Receive buffer contains some value
	*(data_sixty_four + i) = I2C0->RXDATA;	//The RXDATAV interrupt flag is cleared when we read data from receive Buffer
	I2C0->CMD |= I2C_CMD_NACK;				//Transmit acknowledgement from master to slave to request next data
	while(I2C0->CMD & I2C_STATUS_PNACK);
	I2C0->CMD = I2C_CMD_STOP;					//Send stop bit
	while(I2C0->CMD & I2C_STATUS_PSTOP);


}

 int16_t temper(void)
 {
	 int16_t read= read_status(0x1E, 0x2F);
	 read = read<<8;
	 read |= read_status(0x1E, 0x2E);
	 read= read/8;
	 read +=25;
	 return read;
 }



uint8_t read_status(uint8_t slave_address, uint8_t reg_address){
		I2C0->IFC |= 0XFF;	//Clear all interrupts

	uint8_t data_byte=0;	//status to be read
	uint8_t i=0;
	I2C0->TXDATA = ((slave_address << 1)); //Set the slave address and write bit which is zero
	I2C0->CMD = I2C_CMD_START; //Start condition
	while((I2C0->IF & I2C_IF_ACK)==0); //Wait for acknowledge signal
	I2C0->IFC |= (I2C_IFC_ACK );		//Clear the Acknowledge flag

	I2C0->TXDATA = reg_address;	//Send register address

	while((I2C0->IF & I2C_IF_ACK)==0); //Wait for Acknowledgement signal after sending data
	I2C0->IFC |= (I2C_IFC_ACK );		//Clear the Acknowledge flag

	I2C0->CMD |= I2C_CMD_START; //Start condition
	I2C0->TXDATA = ((slave_address << 1) | (0x01)); //Set the slave address and read bit which is one
	while((I2C0->IF & I2C_IF_ACK)==0); //Wait for acknowledge signal
	I2C0->IFC |= (I2C_IFC_ACK );		//Clear the Acknowledge flag

	while((I2C0->IF & I2C_IF_RXDATAV) ==0);	//Wait until Receive buffer contains some value
	data_byte = I2C0->RXDATA;	//The RXDATAV interrupt flag is cleared when we read data from receive Buffer
	I2C0->IFC |= I2C_IFC_ACK;				//Clear the acknowledgement flag
	I2C0->CMD |= I2C_CMD_NACK;				//Transmit acknowledgement from master to slave to request next data


	I2C0->CMD |= I2C_CMD_STOP;					//Send stop bit
	for(i=0;i<253;i++);		//Insert some delay
	return data_byte;
}


float sensor_data(void)
	{
				data_receive=read_status(0x1E, WHO_AM_I);
		    	printf("\n\r My Address: %x ",data_receive);
		    	data_receive=read_status(0x1E, 0x29);
		    	xAxis=data_receive;
		    	xAxis=(xAxis<< 8);
		    	data_receive=read_status(0x1E, 0x28);//x low
		    	xAxis= xAxis|(data_receive);
		    	double xAxisf= (xAxis*SENSITIVITY);
		    	printf("\tX Final: %1.2f",xAxisf);
		    	data_receive=read_status(0x1E, 0x2B);
		    	yAxis= data_receive;
		    	data_receive=read_status(0x1E, 0x2A);
		    	yAxis=(yAxis<< 8);
		    	yAxis= yAxis|(data_receive);
		    	double yAxisf= yAxis*SENSITIVITY;
		    	printf("\tY Final: %1.2f ",yAxisf);
		    	data_receive=read_status(0x1E, 0x2D);
		    	zAxis= data_receive;
		    	data_receive=read_status(0x1E, 0x2C);
		    	zAxis=(zAxis<< 8);
		    	zAxis= zAxis|(data_receive);
		    	double zAxisf= zAxis*SENSITIVITY;
		    	printf("\tZ Final: %1.2f ",zAxisf);
		    	int16_t temp= temper();
		    	printf("\r\n Temperature is %d", temp);
		    	char Xaxis[20];
		    	sprintf(Xaxis, "XAXIS VALUE %2.2f",xAxisf);
		    	LCD_write(Xaxis, LCD_ROW_TEMPVALUE);
		    	return yAxisf;

	}

