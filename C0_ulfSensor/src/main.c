/***********************************************************
*              BMP280 test driver application              *
*                 for the STM32C031 MCU                    *
************************************************************
*
* main.c
* V1.0.0, 3-2024
*  # -- (C) f.m. #
*
* This module is a test implementation for the KY052 sensor
* board with the BMP280 air pressure / temperature sensor.
* The aim is an application to sample the air pressure at
* about 150..200Hz sampling rate, and transfer the sample
* data to the host for evaluation.
* Transmissions are indicated via the blue onboard LED of
* the module. 
*
************************************************************
*
* peripheral pins used in the application:
*  -  PB.3  =  SPI SCLK    -->  O
*  -  PB.4  =  SPI MISO    <--  i
*  -  PB.5  =  SPI MOSI    -->  O
*  -  PB.7  =  SPI CS      -->  O
* --------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include "stm32c0xx.h"
#include "bmp280.h"
#include "hal_spi.h"
#include "main.h"
#include "stm32c0xx_ll_system.h"
#include "stm32c0xx_ll_bus.h"
#include "stm32c0xx_ll_utils.h"
#include "stm32c0xx_ll_rcc.h"
#include "stm32c0xx_ll_gpio.h"
#include "stm32c0xx_ll_usart.h"

#define BIT_USER_BTN                 0x2000  // PC13

#define DEV_STATUS_NONE              0x00
#define DEV_STATUS_INIT              0x01
#define DEV_STATUS_CALIBRATE         0x02
#define DEV_STATUS_RUN               0x03
#define DEV_STATUS_ERROR             0x80
#define BUFFER_0                     0       // transmit definitions ...
#define BUFFER_1                     1
#define DB_SIZE                      16

volatile uint32_t   toDelay          = 0;   /* Timeout Delay, in ms */
volatile uint32_t   runChain         = 0;
volatile uint32_t   txTimer          = 0;
uint16_t            calValue         = 0;   /* calibration value */
uint8_t             sysMode          = 0;   /* system state      */
uint8_t             btnState         = 0;

uint16_t            buffer0[DB_SIZE] = {0};
uint16_t            buffer1[DB_SIZE] = {0};
uint8_t             txBuffer         = 0;
uint8_t             SmplBuffer       = 0;
uint8_t             currentBufIndex  = 0;
uint8_t             currentSmpIndex  = 0;
volatile uint16_t  *bufPtr           = buffer0;  /* transmit buffer pointer */
volatile uint16_t  *smpPtr           = buffer0;  /* sample buffer pointer   */
volatile uint8_t    sBuffer[32]      = {0};
volatile uint8_t    sBufIndex        = 0;
volatile uint8_t    sBufChars        = 0;

///> ==================== DEBUG ====================
volatile uint8_t    devStatus = DEV_STATUS_NONE;

///> ---------------- external data ----------------
extern uint32_t     SystemCoreClock;

/* -------------- internal prototypes --------------
 */
static void        initN44C_LED        (void);
static void        setCoreClock        (void);
static void        sysDelay            (uint16_t delaytime);
static void        setupUserButton     (void);
static uint32_t    getUserButton       (void);
static void        setLED              (uint8_t state);
static void        initUart            (void);
static void        eLoop               (void);
void               tdelay              (uint16_t ticks);
void               putItem             (uint16_t);
void               sendItem            (void);
void               sSendBuffer         (uint8_t *str, uint8_t size);
static uint16_t    getCalibrationValue (uint16_t *pBuffer, uint16_t items);
static void        txTestSequence      (void);

#ifdef _HW_TEST_
  #warning "hardware test code enabled !"
#endif


/* -------------------------------------------------
 * ---------------------- CODE ---------------------
 * -------------------------------------------------
 */

/* ------------ main() ------------
 */
int  main (void)
{
    uint32_t   i, temp, count;
    uint8_t    ret, len, id = 0;
    uint16_t   data;

    devStatus = DEV_STATUS_INIT;
    setCoreClock ();
    SystemCoreClockUpdate ();
    i = SystemCoreClock;

    /* set a 6.6ms cycle, to create a 150Hz sample frequency */
    SysTick_Config (i/150);

    // some hardware/system initialisations
    initN44C_LED ();
    setupUserButton ();
    initUart ();
    spi_setupPins ();

#ifdef _HW_TEST_
    txTestSequence ();
    txTestSequence ();
#endif

    // check user button at startup;
    btnState = getUserButton ();
    if (btnState)
        sysMode = DEV_STATUS_CALIBRATE;

    // init the SPI interface and the sensor
    ret = initSensor (BMP280_CONFIG_MODE_0);
    if (ret != BMP280_ID)
    {
        printf ("sensor init failure (ID read) !\n");
        devStatus = DEV_STATUS_ERROR;
        eLoop ();
    }

#ifdef _HW_TEST_
    ret = getReg (REG_STATUS);
    printf ("STAT = 0x%02x\n", ret);
    ret = getReg (REG_CTRL);
    printf ("CTRL = 0x%02x\n", ret);
    ret = getReg (REG_CONFIG);
    printf ("CFG  = 0x%02x\n", ret);
#endif

    temp  = 0;
    count = 0;
    devStatus = DEV_STATUS_RUN;

    ///> main loop; read pressure value regularly
    do
    {
        if (runChain)
        {
            runChain = 0;
            setLED (1);
            data = readPSensor ();
            putItem (data);
            setLED (0);
        }
#if 0
        if (getUserButton())  // add a LED switch flag derived from the sender's user button
            sendBuffer[0] = 0xFF;
#endif
    }
    while (1);
}



static void  setupUserButton (void)
{
    uint32_t  temp;

    // enable power to GPIOC
    RCC->IOPENR |= (1 << RCC_IOPENR_GPIOCEN_Pos);

    //  set the GPIOC MODE register bits for pin 13
    // in this case, just clear the two bits
    temp  = GPIOC->MODER;
    temp &= ~(GPIO_MODER_MODE13_Msk);
    GPIOC->MODER = temp;
}



/* get the user button state;
 * button logic is inverted, reads H if not pressed
 * return value is "1" for button pressed, "0" otherwise
 */
static uint32_t  getUserButton (void)
{
    if (GPIOC->IDR & BIT_USER_BTN)
        return 0;
    else 
        return 1;
}



/* set the user LED at PA5 on the Nucleo board */
static void  setLED (uint8_t state)
{
    if (state)
        GPIOA->BSRR = 0x0020;
    else
        GPIOA->BRR  = 0x0020;
}


static uint32_t  counter = 0;

/* SysTick handler; toggle bit to measure clock cycle
 * @@@> measured cycle is with SystemCoreClockUpdate() reporting 12.0 MHz
 *      and a SysTick divider of 1000 is 1.0ms/1kHz, i.e. 12 MHz
 */
void  SysTick_Handler (void)
{

    if (devStatus == DEV_STATUS_RUN)
        runChain = 1;

    // delay timer
    if (toDelay)
        toDelay--;
}



/* handler for the EXTI interrupt;
 * check the EXTI_pending registers and update the status flag
 */
void  EXTI4_15_IRQHandler (void)
{
}



/* init PA5 as digital out (for an attached LED);
 * the MODER register is set appropriately;
 * OTYPER, OSPEEDR and PUPDR are not touched, no change required;
 */
static void   initN44C_LED (void)
{
    uint32_t  temp;

    // enable power to GPIOA
    RCC->IOPENR |= (1 << RCC_IOPENR_GPIOAEN_Pos);

    //  set the GPIOA MODE register for pin 5
    temp          = GPIOA->MODER;
    temp         &= ~(GPIO_MODER_MODE5_Msk);
    GPIOA->MODER  = temp;
    GPIOA->MODER |= GPIO_MODER_MODE5_0;
}


/* set the core clock to the external quartz frequency;
 * assumes >24MHz and thus increases Flash latency/wait states to 1;
 * the different peripheral clock dividers are not changed !
 */
static void  setCoreClock (void)
#ifdef _meins_
{
    FLASH->ACR |= (1 << FLASH_ACR_LATENCY_Pos);    // set latency (Flash) to 1 WS

    RCC->CR |= (1 << RCC_CR_HSEON_Pos);            // enable ext. quartz oscillator
    while (!(RCC->CR & (1 << RCC_CR_HSERDY_Pos))); // wait till ready
    RCC->CFGR |= 0x0001;                           // now, switch over to HSE
}
#else // Cube_LL code
{
    LL_FLASH_SetLatency (LL_FLASH_LATENCY_1);

    LL_RCC_HSI_Enable ();                           /* HSI config & activation */
    while (LL_RCC_HSI_IsReady () != 1);
    LL_RCC_HSI_SetCalibTrimming (64);
    LL_RCC_SetHSIDiv (LL_RCC_HSI_DIV_1);
    LL_RCC_SetAHBPrescaler (LL_RCC_HCLK_DIV_1);     /* Set AHB prescaler */

    /* Sysclk activation on the HSI */
    LL_RCC_SetSysClkSource (LL_RCC_SYS_CLKSOURCE_HSI);
    while (LL_RCC_GetSysClkSource () != LL_RCC_SYS_CLKSOURCE_STATUS_HSI);

    LL_RCC_SetAPB1Prescaler (LL_RCC_APB1_DIV_1);    /* Set APB1 prescaler */
    LL_SetSystemCoreClock (48000000);    /* Update CMSIS variable */
}
#endif


/* use SysTick and a dedicated counter variable for a delay;
 * parameter is interpreted as ms, but SysTick based upon 10ms cycle;
 * thus it is divided by 10
 */
static void  sysDelay (uint16_t delaytime)
{
    toDelay = delaytime / 10;
    while (toDelay > 0);
}


/* a simpler delay function version, based on ticks instead of milliseconds;
 */
void  tdelay (uint16_t ticks)
{
    toDelay = ticks;
    while (toDelay > 0);
}



/* prepare a data item for UART transmission, and initiate the sending;
 * 
 */
void  sendItem (void)
{
    int       len;
    uint16_t  data;
    char     *fmt;

#ifdef _SER_OUT_FORMAT_HEX
    fmt = SER_FMT_HEX;
#else
    fmt = SER_FMT_DEC;
#endif
    data = *bufPtr++;
    currentBufIndex++;

    /* double buffer management */
    if (currentBufIndex >= DB_SIZE)
    {
        currentBufIndex = 0;
        if (txBuffer == 0)
        {
            bufPtr   = buffer1;
            txBuffer = 1;
        }
        else
        {
            bufPtr   = buffer0;
            txBuffer = 0;
        }
    }

    /* create tx string, and init transmission */
    len = sprintf ((char *)sBuffer, fmt, data);
    sSendBuffer ((uint8_t *) sBuffer, len);
}



static uint8_t  initLatency = 1;

/* put sample item into the double-buffer;
 * consequently, trigger the next states of the sample event chain:
 * either calculate the calibration data and transmit them,
 * or send the next data item once one buffer is full;
 */
void  putItem (uint16_t data)
{
    uint8_t  bufFull = 0;
#if 0
    *smpPtr++ = data;
    currentSmpIndex++;

    /* double buffer management */
    if (currentSmpIndex > DB_SIZE)
    {
        currentSmpIndex = 0;
        if (SmplBuffer == 0)
        {
            smpPtr      = buffer1;
            SmplBuffer  = 1;
            bufFull     = 1;
            initLatency = 0;  // reset the delay marker
        }
        else
        {
            smpPtr     = buffer0;
            SmplBuffer = 0;
        }
    }
#endif
    int              len;
    static uint16_t  cbuf[DB_SIZE];
    static uint16_t  cbindex = 0;

    if (sysMode == DEV_STATUS_CALIBRATE)
    {
        cbuf[cbindex++] = data;
        if (cbindex >= DB_SIZE)
            bufFull = 1;

        if (bufFull)
        {
            sysMode  = DEV_STATUS_RUN;
            calValue = getCalibrationValue (buffer0, DB_SIZE);
            /* create tx string, and init transmission */
            len = sprintf ((char *)sBuffer, "#XC=%hd\n", data);
            sSendBuffer ((uint8_t *) sBuffer, len);
        }
    }
    else  // run mode, interleave data sampling with transmission
    {
#if 0
        sendItem();
#endif
        len = sprintf ((char *)sBuffer, "%hd\n", data);
        sSendBuffer ((uint8_t *) sBuffer, len);
    }
}


/* initiate the transmission of a string via UART
 */
void  sSendBuffer (uint8_t *str, uint8_t size)
{
#if 0
//  LL_USART_EnableIT_TXE (USART1);
    sBufIndex    = 1;                        // set index for first interrupt
    sBufChars    = size - 1;                 // and string length
    USART1->TDR  = str[0];                   // push first character
    USART1->CR1 |= USART_CR1_TXEIE_TXFNFIE;  // enable TXE interrupt
#else
    sBufIndex = 0;
    while (str[sBufIndex] != '\0')
    {
        USART1->TDR = str[sBufIndex++];
        while (! (USART1->ISR & USART_ISR_TC_Msk));  // busy-wait
    }
#endif
}



/* endless error loop;
 * cannot init sensor; blink LED
 */
static void  eLoop (void)
{
    while (1)
    {
        setLED (1);
        sysDelay (150);
        setLED (0);
        sysDelay (850);
    }
}

/* USART1 init
 * USART1, PA9 -> Tx
 *        PA10 -> Rx
 */
static void  initUart (void)
{
    LL_USART_InitTypeDef  USART_InitStruct = {0};
    LL_GPIO_InitTypeDef   GPIO_InitStruct  = {0};

    /* Peripheral clock enable */
    LL_RCC_SetUSARTClockSource (LL_RCC_USART1_CLKSOURCE_PCLK1);
    LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_USART1);
    LL_IOP_GRP1_EnableClock (LL_IOP_GRP1_PERIPH_GPIOA);


    GPIO_InitStruct.Pin        = LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_1;
    LL_GPIO_Init (GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin        = LL_GPIO_PIN_10;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_1;
    LL_GPIO_Init (GPIOA, &GPIO_InitStruct);

    /* interrupt setup */
    NVIC_SetPriority (USART1_IRQn, 0);
    NVIC_EnableIRQ (USART1_IRQn);

    USART_InitStruct.PrescalerValue      = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate            = 115200;
    USART_InitStruct.DataWidth           = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits            = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity              = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection   = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling        = LL_USART_OVERSAMPLING_16;
    LL_USART_Init (USART1, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold (USART1, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold (USART1, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO (USART1);
    LL_USART_ConfigAsyncMode (USART1);
    LL_USART_Enable (USART1);

    /* Polling USART1 initialisation */
//  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))));
}


/* use one sample buffer as calibration data;
 * using the simple average as calibration value;
 */
static uint16_t  getCalibrationValue (uint16_t *pBuffer, uint16_t items)
{
    int           i;
    unsigned int  sum = 0;
 
    for (i=0; i<items; i++)
        sum += pBuffer[i];
 
    return (uint16_t)(sum / items);
}


static void  txTestSequence (void)
{
    USART1->TDR = 0xAA;
    sysDelay (2);
    USART1->TDR = 0x55;
    sysDelay (2);
    USART1->TDR = 0x00;
    sysDelay (2);
    USART1->TDR = 0xFF;
    sysDelay (2);
    USART1->TDR = 0x81;
    sysDelay (2);
    USART1->TDR = 0x7E;
    sysDelay (2);
}

/********************* End of file **********************/
