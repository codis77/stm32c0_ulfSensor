#ifndef _MAIN_H_
 #define _MAIN_H_

/* ---------------- SPI interface ----------------
 * if "_SPI_BITBANG_IF_" is defined, a SW implementation for the SPI interface is used;
 * else, the STM32C03x peripheral unit SPI1 is used;
 * in both cases, the very same GPIO pins are used
 */
//#define _SPI_BITBANG_IF_


// --- types
typedef unsigned char    BYTE;
typedef unsigned char    UINT8;
typedef unsigned char    BOOL;

#define GPIO_PIN_6                0x00000040  // GPIO mask for P<x>.6
#define PB6_EXTICR2_MSK           0x00010000  // mask to set PB6 as EXTI input in EXTICR2
#define EXTI_PEND_CLR_MASK        0x00FFFF    // mask for clearing the "pending" bit registers

#define SER_FMT_HEX               "%hX\n"
#define SER_FMT_DEC               "%hd\n"


#define _HW_TEST_

#endif // _MAIN_H_
