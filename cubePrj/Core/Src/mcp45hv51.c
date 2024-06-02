
#include "mcp45hv51.h"
#include "ssd1306.h"

#define MCP_I2C_ADDR (MCP4551_DEFAULT_ADDRESS<<1)


extern I2C_HandleTypeDef hi2c1;

uint8_t mcp45hv51_Init(void) {
uint8_t ret = 0;
	/* Check if Electronic potentiometer is connected to I2C */
	if (HAL_I2C_IsDeviceReady(&hi2c1, MCP4551_DEFAULT_ADDRESS<<1, 1, 20000) != HAL_OK) {
		ret = 1;
	}
return ret;
}

/* Wiper Register..........................................................*/
void writeWiper(uint8_t wiperValue)
{

	uint8_t dt[2];
	dt[0] = MCP4551_RA_WIPER | MCP4551_CMD_WRITE;
	dt[1] = wiperValue;
	HAL_I2C_Master_Transmit(&hi2c1, MCP_I2C_ADDR, dt, 2, 1000);

}

uint8_t readWiper()
{
	uint8_t dt[2]= {0};
	if(HAL_I2C_Mem_Read(&hi2c1, MCP_I2C_ADDR, MCP4551_CMD_READ | MCP4551_RA_WIPER, I2C_MEMADD_SIZE_8BIT, dt, 2, 10000) == HAL_OK)
		return dt[1];

    return 0xff;
}

void incrementWiper(uint8_t incriments)
{
    uint8_t *data;
    if(incriments==0)
    	incriments=1;

    data = malloc(incriments);
    memset(data,MCP4551_RA_WIPER | MCP4551_CMD_INC, incriments);
    ssd1306_I2C_WriteMulti(MCP_I2C_ADDR, MCP4551_RA_WIPER | MCP4551_CMD_INC,  data, incriments);
    free(data);
}

void decrementWiper(uint8_t decriments)
{
    uint8_t *data;
    if(decriments==0)
    	decriments=1;

    data = malloc(decriments);
    memset(data,MCP4551_RA_WIPER | MCP4551_CMD_DEC,decriments);
    ssd1306_I2C_WriteMulti(MCP_I2C_ADDR, MCP4551_RA_WIPER | MCP4551_CMD_INC,  data, decriments);
    free(data);
}

/* TCON Register...........................................................*/

uint8_t readTCON()
{
	uint8_t dt[2]= {0};
	if(HAL_I2C_Mem_Read(&hi2c1, MCP_I2C_ADDR, MCP4551_CMD_READ | MCP4551_RA_TCON, I2C_MEMADD_SIZE_8BIT, dt, 2, 10000) == HAL_OK)
		return dt[1];

    return 0xff;
}

void defaultTCON()
{
	uint8_t tcon = MCP4551_TCON_R0_EN | MCP4551_TCON_A_EN | MCP4551_TCON_B_EN | MCP4551_TCON_W_EN;
    write_TCON_Register(tcon);
}

void write_TCON_R0HW(uint8_t isOn)
{
  uint8_t tcon = readTCON();
  if(isOn == 0 )
		tcon &= ~MCP4551_TCON_R0_EN;
  else
	  tcon &= MCP4551_TCON_R0_EN;

  write_TCON_Register(tcon);
}

void write_TCON_R0A(uint8_t isOn)
{
	  uint8_t tcon = readTCON();
	  if(isOn == 0 )
		  tcon &= ~MCP4551_TCON_A_EN;
	  else
		  tcon &= MCP4551_TCON_A_EN;

	  write_TCON_Register(tcon);
}

void write_TCON_R0W(uint8_t isOn)
{
	  uint8_t tcon = readTCON();
	  if(isOn == 0 )
		  tcon &= ~MCP4551_TCON_W_EN;
	  else
		  tcon &= MCP4551_TCON_W_EN;

	  write_TCON_Register(tcon);
}

void write_TCON_R0B(uint8_t isOn)
{
	  uint8_t tcon = readTCON();
	  if(isOn == 0 )
		  tcon &= ~MCP4551_TCON_B_EN;
	  else
		  tcon &= MCP4551_TCON_B_EN;

	  write_TCON_Register(tcon);
}

void write_TCON_Register(uint8_t tcon)
{

	uint8_t dt[1];
	dt[0] = tcon;
	HAL_I2C_Mem_Read(&hi2c1, MCP_I2C_ADDR, MCP4551_CMD_WRITE | MCP4551_RA_TCON, I2C_MEMADD_SIZE_8BIT, dt, 1, 10000);

}





