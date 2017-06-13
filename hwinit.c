#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "hwinit.h"
#include "main.h" // For callback functions

/* I2C interface #1 - goes to Camera - Master on the BUS */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    100000,
    FAST_DUTY_CYCLE_2,
};

/* UART 2 interface - debug and comms */
SerialConfig uart2_config = {
        38400,
        0,
        USART_CR2_STOP1_BITS | USART_CR2_LINEN,
        0
};

/* PWM config - XCLK */
static const PWMConfig pwmcfg = {
    20000000,
    4,
    NULL,
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_DISABLED, NULL},
        {PWM_OUTPUT_DISABLED, NULL},
        {PWM_OUTPUT_DISABLED, NULL}
    },
    0,
    0
};



void hwInit(void) {
  /* Setup pins for I2C interface 1 - Camera - AF(4) is I2C1 function */
  palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN); // SCL
  palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN); // SDA

  /* Starts I2C */
  i2cStart(&I2CD1, &i2cfg1);

  /* Set RESET and POWERDOWN pins */
  palSetPadMode(GPIOA, 0, PAL_MODE_OUTPUT_PUSHPULL); // RESET
  palSetPadMode(GPIOA, 1, PAL_MODE_OUTPUT_PUSHPULL); // PWDN
  /* Reset initially LOW (keeps cam in reset state) */
  palClearPad(GPIOA, 0);
  /* Power-down initially HIGH (keeps Cam in power-down) */
  palSetPad(GPIOA, 1);

  /* Setup MCO1 pin - XCLK */
  palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(0));
  /* Setup MCO1 pin - XCLK - PWM TIM1 CH2 */
  //palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(1) | PAL_STM32_OSPEED_HIGHEST);

  /* Start XCLK */
  pwmStart(&PWMD1, &pwmcfg);

  /* Setup USART pins */
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));   /* USART2 TX */
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));   /* USART2 RX */
  /* initialize serial output */
  sdStart(&SD2, &uart2_config);

  /* Camera button input */
  palSetPadMode(GPIOD, 2, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOB, 3, PAL_MODE_OUTPUT_PUSHPULL);

  /* Setup alternate function for DCMI pins - DCMI is AF13 */
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(13)); // D0
  palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(13)); // D1
  palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(13)); // D2
  palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(13)); // D3
  palSetPadMode(GPIOE, 4, PAL_MODE_ALTERNATE(13)); // D4
  palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(13)); // D5
  palSetPadMode(GPIOE, 5, PAL_MODE_ALTERNATE(13)); // D6
  palSetPadMode(GPIOE, 6, PAL_MODE_ALTERNATE(13)); // D7
  palSetPadMode(GPIOA, 4, PAL_MODE_ALTERNATE(13)); // HSYNC
  palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(13)); // VSYNC
  palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(13)); // PCLK

  /* Start DCMI Driver */
  // Now in cmd_cam_capture call
  //dcmiStart(&DCMID1, &dcmicfg);

  /* Setup LEDs pins, used for debug */
  palSetPadMode(GPIOD, 12, PAL_MODE_OUTPUT_PUSHPULL); // LED GREEN
  palSetPadMode(GPIOD, 13, PAL_MODE_OUTPUT_PUSHPULL); // LED ORANGE
  palSetPadMode(GPIOD, 14, PAL_MODE_OUTPUT_PUSHPULL); // LED RED
  palSetPadMode(GPIOD, 15, PAL_MODE_OUTPUT_PUSHPULL); // LED BLUE
  palClearPad(GPIOD, 12);
  palClearPad(GPIOD, 13);
  palClearPad(GPIOD, 14);
  palClearPad(GPIOD, 15);
  /* Set PD4 (CS43L22 IC RESET Signal) LOW - this way the IC should */
  /* Stay in power down state and not interfere with DCMI lines     */
  palSetPadMode(GPIOD, 4, PAL_MODE_OUTPUT_PUSHPULL);
  palClearPad(GPIOD, 4);
}



