#ifndef STMIPID02_H_
#define STMIPID02_H_

#include <stdint.h>
#include "settings.h"

/* Constants */
#define TIMEOUT_MAX      				10000

/* Camera I2C registers */
#define stmipid02_DEVICE_WRITE_ADDRESS    0x28
#define stmipid02_DEVICE_READ_ADDRESS     0x29

#define CLK_LANE_REG1                     0x02
#define CLK_LANE_REG3                     0x04
#define CLK_LANE_WR_REG1                  0x01 // clock lane status(Read Only)
#define DATA_LANE0_REG1                   0x05
#define DATA_LANE0_REG2                   0x06
#define DATA_LANE0_REG3                   0x07 // CSI controls of data lane 1.1(Read Only)
#define DATA_LANE0_REG4                   0x0C // error status registers(Read Only)

#define DATA_LANE1_REG1                   0x09
#define DATA_LANE1_REG2                   0x0A
//#define CCP_RX_REG1                       0x0D // data clock/data strobe and error control
//#define CCP_RX_REG2                       0x0E // ccp_rx module controls

#define CLK_LANE_REG1_C2                  0x31
#define DATA_LANE3_REG1                   0x34

//mode control registers
#define MODE_REG1                         0x14 // chip mode controls
#define MODE_REG2                         0x15 // output interface controls
#define MODE_REG3                         0x36 // output interface controls
#define CLOCK_CONTROL_REG1                0x16 // clear for INT & ERR
#define ERROR_REGS                        0x10 // error output registers(Read Only)

//data pipe information
#define DATA_ID_WREG                      0x11 // data type write registers(Read Only)
#define DATA_ID_RREG                      0x17 
#define DATA_ID_RREG_EMB                  0x18
#define DATA_SELECTION_CTRL               0x19

#define PIX_WIDTH_CTRL                    0x1E // pixel width control
#define PIX_WIDTH_CTRL_EMB                0x1F // no of active lines in image used for decompression

/* Functions */
/**
  * @brief  Configures the stmipid02 mipi to parallel bridge .
  */
void stmipid02_context_configuration(void);

uint8_t stmipid02_ReadReg8(uint8_t address);
uint8_t stmipid02_WriteReg8(uint8_t address, uint8_t Data);
uint8_t stmipid02_ReadReg(uint8_t Addr);
uint8_t stmipid02_WriteReg(uint8_t Addr, uint8_t Data);

#endif /* end of STMIPID02_H_ */
