/*
 * mcp45hv51.h
 *
 *  Created on: Jan 5, 2024
 *      Author: Roberto
 */

#ifndef INC_MCP45HV51_H_
#define INC_MCP45HV51_H_

#include "main.h"

// I2C Address of device
#define MCP4551_DEFAULT_ADDRESS	0x3C	// A0 is connected to GND
#define MCP4551_ADDRESS_A0_VCC	0x3D	// A0 is connected to VCC
//#define MCP4551_ADDRESS_A0_GND	0x2E	// A0 is connected to GND

// Command definitions (sent to WIPER register) already shifted 2bit left to take in account the  "Data" bit unused
#define MCP4551_CMD_WRITE	0x00
#define MCP4551_CMD_INC		0x04
#define MCP4551_CMD_DEC		0x08
#define MCP4551_CMD_READ	0x0C

// Control bit definitions (sent to TCON register)
#define MCP4551_TCON_R0_EN	0x008
#define MCP4551_TCON_A_EN	0x004
#define MCP4551_TCON_W_EN	0x002
#define MCP4551_TCON_B_EN	0x001

// Register addresses already set as hi nibble
#define MCP4551_RA_WIPER	0x00
#define MCP4551_RA_TCON		0x40

// Common WIPER values
#define MCP4551_WIPER_MID	0x080
#define MCP4551_WIPER_A		0x100
#define MCP4551_WIPER_B		0x000

typedef struct {
	uint8_t R0HW;
	uint8_t R0A;
	uint8_t R0B;
	uint8_t R0W;
} TCON_Register;

uint8_t mcp45hv51_Init(void);
void writeWiper(uint8_t wiperValue);
uint8_t readWiper();
void incrementWiper(uint8_t incriments);
void decrementWiper(uint8_t decriments);
void defaultTCON();
uint8_t readTCON();
void write_TCON_R0HW(uint8_t isOn);
void write_TCON_R0A(uint8_t isOn);
void write_TCON_R0W(uint8_t isOn);
void write_TCON_R0B(uint8_t isOn);
void write_TCON_Register(uint8_t tcon);

#endif /* INC_MCP45HV51_H_ */
