/* ---------------------------------------------------------------------------
 * this module implements the API to the BMP280
 * pressure / temperature sensor
 * -- f.m. 26.03.2024
 * ---------------------------------------------------------------------------
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32c0xx.h"
#include "bmp280.h"
#include "hal_spi.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

// the following pins are used to interface the KY051 board from the C031 Nucleo:
//  :  PB.3  =  SPI1_SCLK / SCL     = CN10.31
//  :  PB.4  =  SPI1_MISO / SDO     = CN10.27
//  :  PB.5  =  SPI1_MOSI / SDA/SDI = CN10.25
//  :  PB.7  =  SPI  CS   / SCB     = CN10.37
//    +3.0V  =                      = CN07.16
//     GND   =                      = CN07.20

#define RF_TRX_SCLK         0x0008   // GPIO_Pin_3
#define RF_TRX_MISO         0x0010   // GPIO_Pin_4
#define RF_TRX_MOSI         0x0020   // GPIO_Pin_5
#define RF_TRX_CS           0x0080   // GPIO_Pin_7

/* Private prototypes --------------------------------------------------------*/
extern void  tdelay (uint16_t ticks);


/* Code  ---------------------------------------------------------------------*/

/* initialize the sensor;
 * currently only one mode (MODE_0) is implemented,
 * a 150Hz pressure data read in normal mode, SPI, no filters;
 * return value is the chip ID, or 0xFF in case of error
 */
uint8_t  initSensor (uint8_t mode)
{
    uint8_t  ret;

    // reset the sensor, just in case
    writeReg (REG_RESET, RESET_VAL);
    tdelay (2);

    // read & check ID
    ret = getReg (REG_ID);
    if (ret != BMP280_ID)
        return (RET_SPI_ERR);

    // mean it ?
    if (mode != BMP280_CONFIG_MODE_0)
        return (RET_SPI_ERR);

    // write configuration
    writeReg (REG_CTRL, MODE_0_CTRL);
    tdelay (1);
    writeReg (REG_CONFIG, MODE_0_CONFIG);
    tdelay (1);

    // return chip ID
    return (ret);
}


/* read the current pressure sensor values;
 * returns the 16 bit value as is, comes in proper sequence
 */
uint16_t  readPSensor (void)
{
    uint32_t pval;

//    return (getHRegData (REG_DATA_P));
    pval = readData (REG_DATA_P, 2);
    return ((uint16_t) pval);
}
