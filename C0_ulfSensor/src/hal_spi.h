
#ifndef HAL_SPI_H
  #define HAL_SPI_H

#ifndef _MAIN_H_
  #include "main.h"
#endif

/** ---------------- definitions ---------------
 */
#define false                  0
#define true                   (!false)

#define RET_SPI_SUCCESS        0x00
#define RET_SPI_ERR            0xFF

/** ------------------ types -------------------
 */
typedef unsigned char  bool;


/* ------------ function prototypes ------------
 */
///> setup
void      delay         (uint32_t time);
///> pin functions
void      setSS         (uint8_t state);
void      setSCK        (uint8_t state);
void      setMOSI       (uint8_t state);
uint8_t   getMISO       (void);

///> SPI functions
uint8_t   spi_send      (uint8_t tx);

///> API functions
void      spi_setupPins (void);
uint8_t   getReg        (uint8_t regAddr);
void      writeReg      (uint8_t regAddr, uint8_t value);
uint32_t  readData      (uint8_t regAddr, uint8_t bytes);


#endif //  HAL_SPI_H
