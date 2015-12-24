
#ifndef OV7251_H_
#define OV7251_H_

#include <stdint.h>
#include "settings.h"

/* Constants */
#define TIMEOUT_MAX      				10000

/* Camera I2C registers */
#define ov7251_DEVICE_WRITE_ADDRESS    0xC0
#define ov7251_DEVICE_READ_ADDRESS     0xC1

#define SC_SW_RESET_REG                0x0103//0x01,
#define SC_MODE_SELECT                 0x0100//0x00,
#define SC_REG5                        0x3005//0x00,
#define SC_MIPI_PHY                    0x3012//0xc0,
#define SC_MIPI_PHY_2                  0x3013//0xd2,
#define SC_MIPI_SC_CTRL0               0x3014//0x04,
#define SC_CLKRST0                     0x3016//0x10,
#define SC_CLKRST1                     0x3017//0x00,
#define SC_CLKRST2                     0x3018//0x00,
#define SC_CLKRST4                     0x301A//0x00,
#define SC_CLKRST5                     0x301B//0x00,
#define SC_CLKRST6                     0x301C//0x00,
#define SC_LOW_PWR_CTRL                0x3023//0x05,
#define SC_R37                         0x3037//0xf0,
#define PLL_PLL18                      0x3098//0x04,
#define PLL_PLL19                      0x3099//0x28,
#define PLL_PLL1A                      0x309A//0x05,
#define PLL_PLL1B                      0x309B//0x04,
#define PLL_VT_PIX_CLK_DIV             0x30B0//0x0a,
#define PLL_VT_SYS_CLK_DIV             0x30B1//0x01,

#define PLL_MULTIPLIER                 0x30b3//0x64,
#define PLL_PLL1_PRE_PLL_DIV           0x30b4//0x03,
#define PLL_OP_PIX_CLK_DIV             0x30b5//0x05,
#define SB_SRB_CTRL                    0x3106//0xda,

#define AEC_EXPO                       0x3500//0x00
#define AEC_EXPO2                      0x3501//0x0a,
#define AEC_EXPO3                      0x3502//0x00,
#define AEC_MANUAL                     0x3503//0x07,
#define AEC_GAIN_CONVERT               0x3509//0x10,
#define AEC_AGC_ADJ                    0x350b//0x10,

#define ANALOG_REG                     0x3600//0x1c,
#define ANALOG_REG2                    0x3602//0x62,
#define ANALOG_REG3                    0x3620//0xb7,
#define ANALOG_REG4                    0x3622//0x04,
#define ANALOG_REG5                    0x3626//0x21,
#define ANALOG_REG6                    0x3627//0x30,
#define ANALOG_REG7                    0x3630//0x44,
#define ANALOG_REG8                    0x3631//0x35,
#define ANALOG_REG9                    0x3634//0x60,
#define ANALOG_REG10                   0x3636//0x00,
#define ANALOG_REG11                   0x3662//0x01,
#define ANALOG_REG12                   0x3663//0x70,
#define ANALOG_REG13                   0x3664//0xf0,

#define ANA_CORE6                      0x3666//0x0a,
#define ANALOG_REG14                   0x3669//0x1a,
#define ANALOG_REG15                   0x366a//0x00,
#define ANALOG_REG16                   0x366b//0x50,
#define ANALOG_REG17                   0x3673//0x01,
#define ANALOG_REG18                   0x3674//0xff,
#define ANALOG_REG19                   0x3675//0x03,

#define SENSOR_CONTROL_REG             0x3705//0x41,
#define SENSOR_CONTROL_REG2            0x3709//0x40,
#define SENSOR_CONTROL_REG3            0x373c//0x08,
#define SENSOR_CONTROL_REG4            0x3742//0xe0,
#define SENSOR_CONTROL_REG5            0x3757//0xb3,
#define SENSOR_CONTROL_REG6            0x3788//0x00,
#define FIFO_CTRL0_H                   0x37a8//0x02,
#define FIFO_CTRL0_L                   0x37a9//0x8c,

#define TIMING_X_ADDR_START            0x3800//0x00,
#define TIMING_X_ADDR_START2           0x3801//0x04,
#define TIMING_Y_ADDR_START            0x3802//0x00,
#define TIMING_Y_ADDR_START2           0x3803//0x00,
#define TIMING_X_ADDR_END              0x3804//0x02,
#define TIMING_X_ADDR_END2             0x3805//0x8b,
#define TIMING_Y_ADDR_END              0x3806//0x01,
#define TIMING_Y_ADDR_END2             0x3807//0xef,
#define TIMING_X_OUTPUT_SIZE           0x3808//0x00,
#define TIMING_X_OUTPUT_SIZE2          0x3809//0xa0,
#define TIMING_Y_OUTPUT_SIZE           0x380a//0x00,
#define TIMING_Y_OUTPUT_SIZE2          0x380b//0x78,
#define TIMING_HTS                     0x380c//0x03,
#define TIMING_HTS2                    0x380d//0x04,
#define TIMING_VTS                     0x380e//0x00,
#define TIMING_VTS2                    0x380f//0xad,

#define TIMING_ISP_X_WIN               0x3810//0x00,
#define TIMING_ISP_X_WIN2              0x3811//0x04,
#define TIMING_ISP_Y_WIN               0x3812//0x00,
#define TIMING_ISP_Y_WIN2              0x3813//0x03,
#define TIMING_X_INC                   0x3814//0x44,
#define TIMING_Y_INC                   0x3815//0x44,
#define TIMING_FORMAT1                 0x3820//0x40,
#define TIMING_FORMAT2                 0x3821//0x00,
#define TIMING_REG2F                   0x382f//0x0e,
#define TIMING_REG32                   0x3832//0x00,
#define TIMING_REG33                   0x3833//0x05,
#define TIMING_REG34                   0x3834//0x00,
#define TIMING_REG35                   0x3835//0x0c,
#define DIGITAL_BINNING_CTRL           0x3837//0x00,

/* 0x3B80 - 0x3B97 PWM & strobe
0x3b80//0x00,
0x3b81//0xa5,
0x3b82//0x10,
0x3b83//0x00,
0x3b84//0x08,
0x3b85//0x00,
0x3b86//0x01,
0x3b87//0x00,
0x3b88//0x00,
0x3b89//0x00,
0x3b8a//0x00,
0x3b8b//0x05,
0x3b8c//0x00,
0x3b8d//0x00,
0x3b8e//0x00,
0x3b8f//0x1a,
0x3b94//0x05,
0x3b95//0xf2,
0x3b96//0x40,
*/

#define LOWPWR00                       0x3c00//0x89,
#define LOWPWR01                       0x3c01//0x63,
#define LOWPWR02                       0x3c02//0x01,
#define LOWPWR03                       0x3c03//0x00,
#define LOWPWR04                       0x3c04//0x00,
#define LOWPWR05                       0x3c05//0x03,
#define LOWPWR06                       0x3c06//0x00,
#define LOWPWR07                       0x3c07//0x06,
#define LOWPWR0C                       0x3c0c//0x01,
#define LOWPWR0D                       0x3c0d//0x82,
#define LOWPWR0E                       0x3c0e//0x00,
#define LOWPWR0F                       0x3c0f//0xad,

#define BLC_CTRL01                     0x4001//0x40,
#define BLC_NUM                        0x4004//0x02,
#define BLC_MAN_CTRL                   0x4005//0x00,
#define BLC_AVG                        0x404e//0x01,
#define DATA_MAX_H                     0x4300//0xff,
#define DATA_MIN_H                     0x4301//0x00,
#define SC_REG1501                     0x4501//0x48,
#define READ_START_H                   0x4600//0x00,
#define READ_START_L                   0x4601//0x4e,

#define MIPI_CTRL01                    0x4801//0x0f,
#define MIPI_CTRL06                    0x4806//0x0f,
#define HS_ZERO_MIN                    0x4819//0xaa,
#define CLK_TRAIL_MIN                  0x4823//0x3e,
#define PCLK_PERIOD                    0x4837//0x19,
#define DEBUG_CTRL                     0x4a0d//0x00,
//#define LOWPWR_CTRL_REG                0x4a47//0x7f,
//#define LOWPWR_CTRL_REG2               0x4a49//0xf0,
//0x4a4b//0x30,
#define ISP_CTRL00                     0x5000//0x85,
#define ISP_CTRL01                     0x5001//0x80,
//#define SC_MODE_SELECT                 0x0100

/* Functions */
/**
  * @brief  Configures the ov7251 camera .
  */
void ov7251_context_configuration(void);

/*
 * Resolution:
 * ROW_SIZE * BINNING_ROW <= MAX_IMAGE_WIDTH
 * COLUMN_SIZE * BINNING_COLUMN <= MAX_IMAGE_HEIGHT
 */
#define OV7251_FULL_IMAGE_SIZE (188*120)
#define OV7251_FULL_IMAGE_ROW_SIZE (188)
#define OV7251_FULL_IMAGE_COLUMN_SIZE (120)

uint8_t ov7251_ReadReg16(uint16_t address);
uint8_t ov7251_WriteReg16(uint16_t address, uint8_t Data);
uint8_t ov7251_ReadReg(uint16_t Addr);
uint8_t ov7251_WriteReg(uint16_t Addr, uint8_t Data);


#endif /* OV7251_H_ */
