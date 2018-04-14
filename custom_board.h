#ifndef __CUSTOM_BOARD_DEF_H__
#define __CUSTOM_BOARD_DEF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define LED_START      31
#define LED_0          31
#define LED_1          30
#define LED_STOP       30

#define LEDS_ACTIVE_STATE 1

#define LEDS_INV_MASK  0x00000000

#define BSP_LED_0      LED_0
#define BSP_LED_1      LED_1

#define BUTTON_START   4
#define BUTTON_0       4
#define BUTTON_1       27
#define BUTTON_STOP    27
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BSP_BUTTON_0   BUTTON_0
#define BSP_BUTTON_1   BUTTON_1

#define BUTTONS_NUMBER 2
#define LEDS_NUMBER    2
#define LEDS_LIST { LED_0, LED_1 }

#define BUTTONS_LIST { BUTTON_0, BUTTON_1 }

#if 1
#define RX_PIN_NUMBER  7
#define TX_PIN_NUMBER  6
#endif

#if 1
#define CTS_PIN_NUMBER 8
#define RTS_PIN_NUMBER 9
#define HWFC           false
#endif

// Low frequency clock source to be used by the SoftDevice
#ifdef S210
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
#else
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CUSTOM_BOARD_DEF_H__ */
