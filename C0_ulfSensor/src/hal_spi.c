
#include "stm32c0xx.h"
#include "main.h"
#include "hal_spi.h"
#include "bmp280.h"
#include <stdio.h>


#define GPIO_RF_TRX_PORT    GPIOB
#define RF_TRX_SCLK         0x0008   // GPIO_Pin_3
#define RF_TRX_MISO         0x0010   // GPIO_Pin_4
#define RF_TRX_MOSI         0x0020   // GPIO_Pin_5
#define RF_TRX_CS           0x0080   // GPIO_Pin_7


/* ---------------- internal prototypes ----------------
 */
void  udelay (uint32_t time);



/* set the Slave Select signal
 */
void  setSS (uint8_t state)
{
    if (state)
        GPIO_RF_TRX_PORT->BSRR = RF_TRX_CS;
    else
        GPIO_RF_TRX_PORT->BRR  = RF_TRX_CS;
}


/* set the SPI clock signal
 */
void  setSCK (uint8_t state)
{
    if (state)
        GPIO_RF_TRX_PORT->BSRR = RF_TRX_SCLK;
    else
        GPIO_RF_TRX_PORT->BRR  = RF_TRX_SCLK;
}


/* set the MOSI pin signal
 */
void  setMOSI (uint8_t state)
{
    if (state)
        GPIO_RF_TRX_PORT->BSRR = RF_TRX_MOSI;
    else
        GPIO_RF_TRX_PORT->BRR  = RF_TRX_MOSI;
}


/* read the MISO input pin back
 */
uint8_t  getMISO (void)
{
    return (GPIO_RF_TRX_PORT->IDR & RF_TRX_MISO);
}



/* ein Byte-Register per SPI einlesen;
 * Parameter ist die Registeraddresse, Rueckgabewert der ausgelesene Inhalt;
 */
uint8_t  getReg (uint8_t regAddr)
{
    uint8_t   result;

    setSS (0);

    regAddr |= REG_READ_MASK;  /* set address MSB       */

    spi_send (regAddr);        /* send register address */
    result = spi_send (0);     /* read register value   */

    setSS (1);                 /* deselect */

    return (result);
}


/* ein Byte-Register per SPI beschreiben;
 * Parameter ist die Registeraddresse und der zu schreibende Wert
 */
void  writeReg (uint8_t regAddr, uint8_t value)
{
    setSS (0);                  /* select */

    regAddr &= REG_WRITE_MASK;  /* reset MSB of address  */ 

    spi_send (regAddr);         /* send register address */
    spi_send (value);           /* and value             */

    setSS (1);                 /* deselect */
}

/* read successive register from the sensor via auto-increment;
 * maximal 4 bytes can be read at once;
 * <bytes> successive reads starting at <regAddr> are done,
 * written to the return value MSB first
 */
uint32_t  readData (uint8_t regAddr, uint8_t bytes)
{
    uint32_t  readval;
    uint8_t   bread;

    readval = 0;

    if ((bytes > 4) || (!bytes))
        return 0;

    setSS (0);

    /* send first register address */
    spi_send (regAddr);

    while (bytes)
    {
        bread = spi_send (0);
        readval  = (readval << 8);  // make room ...
        readval += bread;           // ... and insert into 32-bit word
        bytes--;
    }

    setSS (1);
    return (readval);
}


// --- debug ---
extern volatile uint8_t     devStatus;


/* this function sets up SPI 1 (SPI & GPIO);
 *   -  PB.3  =  SPI SCLK    -->  O
 *   -  PB.4  =  SPI MISO    <--  i
 *   -  PB.5  =  SPI MOSI    -->  O
 *   -  PB.7  =  SPI CS      -->  O
 *  PB6 & PB7 (GDO0 + CS) remain GPIO, i.e. manual SW control
 */
void  spi_setupPins (void)
{
    uint32_t  temp, mask;

    // enable power to GPIOB and the SPI peripheral
    RCC->IOPENR  |= (1 << RCC_IOPENR_GPIOBEN_Pos);
    RCC->APBENR2 |= (1 << RCC_APBENR2_SPI1EN_Pos);

    // set MODE register bits;
    // the SPI pins are set to 0b10 (alt. function), CS and GDO0 remain GPIOs
    //  PB6 (GDO0) is set to DIN
    temp  = GPIOB->MODER;
    mask  = ~(GPIO_MODER_MODE3_Msk + GPIO_MODER_MODE4_Msk + GPIO_MODER_MODE5_Msk + GPIO_MODER_MODE7_Msk);
    temp &= mask;
    temp |= (GPIO_MODER_MODE3_1 + GPIO_MODER_MODE4_1 + GPIO_MODER_MODE5_1 + GPIO_MODER_MODE7_0); // set 3, 4 and 5 to AF, 7 t output
    GPIOB->MODER = temp;

    // AF mode for SPI pins; all pins (PB3, PB4, PB5) are AF0,
    // i.e. the register default value; this means no need to change anything ...

    // setup the SPI unit; CPOL & CPHA = 0, manual SS, MSB first
    // BR is set to PCLK / 32 (with PCLK = CoreClk / 1 as default value !) = 0b100
    // this equals 48MHz / 32 = 1.5 MHz (max would be 10MHz at most, 6.5MHz for 
    mask  = (SPI_CR1_BIDIOE_Msk + SPI_CR1_MSTR_Msk + SPI_CR1_SSM_Msk + SPI_CR1_SSI_Msk);
//  mask  = (SPI_CR1_MSTR_Msk + SPI_CR1_SSM_Msk + SPI_CR1_SSI_Msk);
    mask |= (SPI_CR1_BR_2);
    SPI1->CR1 = mask;

    // set 8-bit transfers; 8 bits = 0b0111
    mask = (SPI_CR2_DS_0 + SPI_CR2_DS_1 + SPI_CR2_DS_2); 
    SPI1->CR2 = mask;

    // enable the unit
    SPI1->CR1 |= SPI_CR1_SPE_Msk;
}

/* send uint8_t via HW - SPI;
 * does only the SPI byte transfer, slave select handling
 * must be done in the context of the calling routine !
 * 'value'  value to be sent
 * Return:
 *  response received from SPI slave
 */
uint8_t  spi_send (uint8_t tx)
{
    uint8_t   rx  = 0;
    uint8_t  *drb = (uint8_t *) &(SPI1->DR);

    *drb = tx;                           // write Tx byte
//  while (!(SPI1->SR & SPI_SR_RXNE));   // wait till finished (sync'ed. Rx in)
    while ((SPI1->SR & SPI_SR_BSY));     // wait till finished
    rx = *drb;                           // read MISO value

    return (rx);
}



#define UDELAY_LOOP_CNTR      64
void  udelay (uint32_t n)
{
    volatile uint32_t  cnt;

    do
    {
        for (cnt=UDELAY_LOOP_CNTR; cnt>0; cnt--);
        n--;
    }
    while (n);
}


#if 0
void  delay (uint32_t time)
{
    volatile uint32_t  i;

    do
    {
        for (i=0; i<16; )
            i++;
    }
    while (--time);
}
#endif
