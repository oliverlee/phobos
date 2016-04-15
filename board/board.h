/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for Olimex STM32-H405 board.
 */

/*
 * Board identifier.
 */
#define BOARD_OLIMEX_STM32_H405
#define BOARD_NAME                  "Olimex STM32-H405"

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   330U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F405xx

/*
 * IO pins assignments.
 */
#define GPIOA_BUTTON                0U
#define GPIOA_TIM5_CH1              0U /* cut trace at R19 for GPIOA_BUTTON if necessary */
#define GPIOA_TIM5_CH2              1U
#define GPIOA_PIN2                  2U
#define GPIOA_SPI1_ENC2_NSS         3U
#define GPIOA_SPI1_ENC1_NSS         4U
#define GPIOA_SPI1_SCK              5U
#define GPIOA_SPI1_MISO             6U
#define GPIOA_SPI1_MOSI             7U
#define GPIOA_TIM1_CH1              8U
#define GPIOA_TIM1_CH2              9U
#define GPIOA_SPI1_PWR              10U
#define GPIOA_USB_DM                11U
#define GPIOA_USB_DP                12U
#define GPIOA_JTAG_TMS              13U
#define GPIOA_JTAG_TCK              14U
#define GPIOA_JTAG_TDI              15U

#define GPIOB_SPI1_IMU_NSS          0U
#define GPIOB_SDIO_CD               1U
#define GPIOB_BOOT1                 2U
#define GPIOB_JTAG_TDO              3U
#define GPIOB_JTAG_TRST             4U
#define GPIOB_IMU_EXTI              5U
#define GPIOB_USART1_TX             6U
#define GPIOB_TIM4_CH1              6U /* set PCB S3 to enable */
#define GPIOB_USART1_RX             7U
#define GPIOB_TIM4_CH2              7U /* set PCB S3 to enable */
#define GPIOB_USART3_DE             8U
#define GPIOB_USART3_NRE            9U
#define GPIOB_USART3_TX             10U
#define GPIOB_USART3_RX             11U
#define GPIOB_MOTOR1_EN             12U
#define GPIOB_MOTOR2_EN             13U
#define GPIOB_MOTOR1_RDY            14U
#define GPIOB_MOTOR2_RDY            15U

#define GPIOC_ADC10                 0U
#define GPIOC_ADC11                 1U
#define GPIOC_ADC12                 2U
#define GPIOC_ADC13                 3U
#define GPIOC_USB_P                 4U
#define GPIOC_USART1_DE             4U /* open H405 SJ USBP_E to enable */
#define GPIOC_ADC14                 4U /* open H405 SJ USBP_E AND set PCB S1 to enable */
#define GPIOC_USART1_NRE            5U
#define GPIOC_ADC15                 5U /* set PCB S2 to enable */
#define GPIOC_TIM3_CH1              6U
#define GPIOC_TIM3_CH2              7U
#define GPIOC_SDIO_D0               8U
#define GPIOC_SDIO_D1               9U
#define GPIOC_SDIO_D2               10U
#define GPIOC_USB_DISC              11U
#define GPIOC_SDIO_D3               11U /* cut trace at R27 for GPIOC_USB_DISC to enable */
#define GPIOC_LED                   12U
#define GPIOC_SDIO_SCK              12U /* open H405 SJ LED_E to enable */
#define GPIOC_TORQUE_MEAS_NEN       13U
#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U

#define GPIOD_OSC_IN                0U
#define GPIOD_OSC_OUT               1U
#define GPIOD_SDIO_CMD              2U
#define GPIOD_PIN3                  3U
#define GPIOD_PIN4                  4U
#define GPIOD_PIN5                  5U
#define GPIOD_PIN6                  6U
#define GPIOD_PIN7                  7U
#define GPIOD_PIN8                  8U
#define GPIOD_PIN9                  9U
#define GPIOD_PIN10                 10U
#define GPIOD_PIN11                 11U
#define GPIOD_PIN12                 12U
#define GPIOD_PIN13                 13U
#define GPIOD_PIN14                 14U
#define GPIOD_PIN15                 15U

#define GPIOE_PIN0                  0U
#define GPIOE_PIN1                  1U
#define GPIOE_PIN2                  2U
#define GPIOE_PIN3                  3U
#define GPIOE_PIN4                  4U
#define GPIOE_PIN5                  5U
#define GPIOE_PIN6                  6U
#define GPIOE_PIN7                  7U
#define GPIOE_PIN8                  8U
#define GPIOE_PIN9                  9U
#define GPIOE_PIN10                 10U
#define GPIOE_PIN11                 11U
#define GPIOE_PIN12                 12U
#define GPIOE_PIN13                 13U
#define GPIOE_PIN14                 14U
#define GPIOE_PIN15                 15U

#define GPIOF_PIN0                  0U
#define GPIOF_PIN1                  1U
#define GPIOF_PIN2                  2U
#define GPIOF_PIN3                  3U
#define GPIOF_PIN4                  4U
#define GPIOF_PIN5                  5U
#define GPIOF_PIN6                  6U
#define GPIOF_PIN7                  7U
#define GPIOF_PIN8                  8U
#define GPIOF_PIN9                  9U
#define GPIOF_PIN10                 10U
#define GPIOF_PIN11                 11U
#define GPIOF_PIN12                 12U
#define GPIOF_PIN13                 13U
#define GPIOF_PIN14                 14U
#define GPIOF_PIN15                 15U

#define GPIOG_PIN0                  0U
#define GPIOG_PIN1                  1U
#define GPIOG_PIN2                  2U
#define GPIOG_PIN3                  3U
#define GPIOG_PIN4                  4U
#define GPIOG_PIN5                  5U
#define GPIOG_PIN6                  6U
#define GPIOG_PIN7                  7U
#define GPIOG_PIN8                  8U
#define GPIOG_PIN9                  9U
#define GPIOG_PIN10                 10U
#define GPIOG_PIN11                 11U
#define GPIOG_PIN12                 12U
#define GPIOG_PIN13                 13U
#define GPIOG_PIN14                 14U
#define GPIOG_PIN15                 15U

#define GPIOH_PIN0                  0U
#define GPIOH_PIN1                  1U
#define GPIOH_PIN2                  2U
#define GPIOH_PIN3                  3U
#define GPIOH_PIN4                  4U
#define GPIOH_PIN5                  5U
#define GPIOH_PIN6                  6U
#define GPIOH_PIN7                  7U
#define GPIOH_PIN8                  8U
#define GPIOH_PIN9                  9U
#define GPIOH_PIN10                 10U
#define GPIOH_PIN11                 11U
#define GPIOH_PIN12                 12U
#define GPIOH_PIN13                 13U
#define GPIOH_PIN14                 14U
#define GPIOH_PIN15                 15U

#define GPIOI_PIN0                  0U
#define GPIOI_PIN1                  1U
#define GPIOI_PIN2                  2U
#define GPIOI_PIN3                  3U
#define GPIOI_PIN4                  4U
#define GPIOI_PIN5                  5U
#define GPIOI_PIN6                  6U
#define GPIOI_PIN7                  7U
#define GPIOI_PIN8                  8U
#define GPIOI_PIN9                  9U
#define GPIOI_PIN10                 10U
#define GPIOI_PIN11                 11U
#define GPIOI_PIN12                 12U
#define GPIOI_PIN13                 13U
#define GPIOI_PIN14                 14U
#define GPIOI_PIN15                 15U

/*
 * IO lines assignments.
 */
#define LINE_BUTTON                 PAL_LINE(GPIOA, 0U)
#define LINE_TIM5_CH1               PAL_LINE(GPIOA, 0U) /* cut trace at R19 for GPIOA_BUTTON if necessary */
#define LINE_TIM5_CH2               PAL_LINE(GPIOA, 1U)
#define LINE_SPI1_ENC2_NSS          PAL_LINE(GPIOA, 3U)
#define LINE_SPI1_ENC1_NSS          PAL_LINE(GPIOA, 4U)
#define LINE_SPI1_SCK               PAL_LINE(GPIOA, 5U)
#define LINE_SPI1_MISO              PAL_LINE(GPIOA, 6U)
#define LINE_SPI1_MOSI              PAL_LINE(GPIOA, 7U)
#define LINE_TIM1_CH1               PAL_LINE(GPIOA, 8U)
#define LINE_TIM1_CH2               PAL_LINE(GPIOA, 9U)
#define LINE_SPI1_PWR               PAL_LINE(GPIOA, 10U)
#define LINE_USB_DM                 PAL_LINE(GPIOA, 11U)
#define LINE_USB_DP                 PAL_LINE(GPIOA, 12U)
#define LINE_JTAG_TMS               PAL_LINE(GPIOA, 13U)
#define LINE_JTAG_TCK               PAL_LINE(GPIOA, 14U)
#define LINE_JTAG_TDI               PAL_LINE(GPIOA, 15U)

#define LINE_SPI1_IMU_NSS           PAL_LINE(GPIOB, 0U)
#define LINE_SDIO_CD                PAL_LINE(GPIOB, 1U)
#define LINE_BOOT1                  PAL_LINE(GPIOB, 2U)
#define LINE_JTAG_TDO               PAL_LINE(GPIOB, 3U)
#define LINE_JTAG_TRST              PAL_LINE(GPIOB, 4U)
#define LINE_IMU_EXTI               PAL_LINE(GPIOB, 4U)
#define LINE_USART1_TX              PAL_LINE(GPIOB, 6U)
#define LINE_TIM4_CH1               PAL_LINE(GPIOB, 6U) /* set PCB S3 to enable */
#define LINE_USART1_RX              PAL_LINE(GPIOB, 7U)
#define LINE_TIM4_CH2               PAL_LINE(GPIOB, 7U) /* set PCB S3 to enable */
#define LINE_USART3_DE              PAL_LINE(GPIOB, 8U)
#define LINE_USART3_NRE             PAL_LINE(GPIOB, 9U)
#define LINE_USART3_TX              PAL_LINE(GPIOB, 10U)
#define LINE_USART3_RX              PAL_LINE(GPIOB, 11U)
#define LINE_MOTOR1_EN              PAL_LINE(GPIOB, 12U)
#define LINE_MOTOR2_EN              PAL_LINE(GPIOB, 13U)
#define LINE_MOTOR1_RDY             PAL_LINE(GPIOB, 14U)
#define LINE_MOTOR2_RDY             PAL_LINE(GPIOB, 15U)

#define LINE_ADC10                  PAL_LINE(GPIOC, 0U)
#define LINE_ADC11                  PAL_LINE(GPIOC, 1U)
#define LINE_ADC12                  PAL_LINE(GPIOC, 2U)
#define LINE_ADC13                  PAL_LINE(GPIOC, 3U)
#define LINE_USB_P                  PAL_LINE(GPIOC, 4U)
#define LINE_USART1_DE              PAL_LINE(GPIOC, 4U) /* open H405 SJ USBP_E to enable */
#define LINE_ADC14                  PAL_LINE(GPIOC, 4U) /* open H405 SJ USBP_E AND set PCB S1 to enable */
#define LINE_USART1_NRE             PAL_LINE(GPIOC, 5U)
#define LINE_ADC15                  PAL_LINE(GPIOC, 5U) /* set PCB S2 to enable */
#define LINE_TIM3_CH1               PAL_LINE(GPIOC, 6U)
#define LINE_TIM3_CH2               PAL_LINE(GPIOC, 7U)
#define LINE_SDIO_D0                PAL_LINE(GPIOC, 8U)
#define LINE_SDIO_D1                PAL_LINE(GPIOC, 9U)
#define LINE_SDIO_D2                PAL_LINE(GPIOC, 10U)
#define LINE_USB_DISC               PAL_LINE(GPIOC, 11U)
#define LINE_SDIO_D3                PAL_LINE(GPIOC, 11U) /* cut trace at R27 for GPIOC_USB_DISC to enable */
#define LINE_LED                    PAL_LINE(GPIOC, 12U)
#define LINE_SDIO_SCK               PAL_LINE(GPIOC, 12U) /* open H405 SJ LED_E to enable */
#define LINE_TORQUE_MEAS_NEN        PAL_LINE(GPIOC, 13U)
#define LINE_OSC32_IN               PAL_LINE(GPIOC, 14U)
#define LINE_OSC32_OUT              PAL_LINE(GPIOC, 15U)

#define LINE_OSC_IN                 PAL_LINE(GPIOD, 0U)
#define LINE_OSC_OUT                PAL_LINE(GPIOD, 1U)
#define LINE_SDIO_CMD               PAL_LINE(GPIOC, 2U)


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 * PA0  - BUTTON                    (input floating - TIM5_CH1 not enabled[alternate 2]).
 * PA1  - TIM5_CH2                  (input pullup - TIM5_CH2 not enabled[alternate 2]).
 * PA2  - PIN2                      (input pullup).
 * PA3  - SPI1_ENC2_NSS             (output opendrain pullup maximum).
 * PA4  - SPI1_ENC1_NSS             (output opendrain pullup maximum).
 * PA5  - SPI1_SCK                  (alternate 5).
 * PA6  - SPI1_MISO                 (alternate 5).
 * PA7  - SPI1_MOSI                 (alternate 5).
 * PA8  - TIM1_CH1                  (alternate 1).
 * PA9  - TIM1_CH2                  (alternate 1).
 * PA10 - SPI1_PWR                  (output pushpull maximum).
 * PA11 - USB_DM                    (alternate 10).
 * PA12 - USB_DP                    (alternate 10).
 * PA13 - JTAG_TMS                  (alternate 0).
 * PA14 - JTAG_TCK                  (alternate 0).
 * PA15 - JTAG_TDI                  (alternate 0).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_BUTTON) |             \
                                     PIN_MODE_INPUT(GPIOA_TIM5_CH2) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN2) |               \
                                     PIN_MODE_OUTPUT(GPIOA_SPI1_ENC2_NSS) |     \
                                     PIN_MODE_OUTPUT(GPIOA_SPI1_ENC1_NSS) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_SCK) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MISO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MOSI) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_TIM1_CH1) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_TIM1_CH2) |       \
                                     PIN_MODE_OUTPUT(GPIOA_SPI1_PWR) |          \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_DM) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_DP) |         \
                                     PIN_MODE_ALTERNATE(GPIOA_JTAG_TMS) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_JTAG_TCK) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_JTAG_TDI))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_BUTTON) |             \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TIM5_CH2) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN2) |               \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_SPI1_ENC2_NSS) |     \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_SPI1_ENC1_NSS) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_SCK) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MISO) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MOSI) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TIM1_CH1) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TIM1_CH2) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_PWR) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DM) |             \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DP) |             \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTAG_TMS) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTAG_TCK) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTAG_TDI))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_HIGH(GPIOA_BUTTON) |            \
                                     PIN_OSPEED_HIGH(GPIOA_TIM5_CH2) |          \
                                     PIN_OSPEED_HIGH(GPIOA_PIN2) |              \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_ENC2_NSS) |     \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_ENC1_NSS) |     \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_SCK) |          \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MISO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_MOSI) |         \
                                     PIN_OSPEED_HIGH(GPIOA_TIM1_CH1) |          \
                                     PIN_OSPEED_HIGH(GPIOA_TIM1_CH2) |          \
                                     PIN_OSPEED_HIGH(GPIOA_SPI1_PWR) |          \
                                     PIN_OSPEED_HIGH(GPIOA_USB_DM) |            \
                                     PIN_OSPEED_HIGH(GPIOA_USB_DP) |            \
                                     PIN_OSPEED_HIGH(GPIOA_JTAG_TMS) |          \
                                     PIN_OSPEED_HIGH(GPIOA_JTAG_TCK) |          \
                                     PIN_OSPEED_HIGH(GPIOA_JTAG_TDI))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_BUTTON) |             \
                                     PIN_PUPDR_PULLUP(GPIOA_TIM5_CH2) |             \
                                     PIN_PUPDR_PULLUP(GPIOA_PIN2) |                 \
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_ENC2_NSS) |        \
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_ENC1_NSS) |        \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_SCK) |           \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_MISO) |          \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_MOSI) |          \
                                     PIN_PUPDR_FLOATING(GPIOA_TIM1_CH1) |           \
                                     PIN_PUPDR_FLOATING(GPIOA_TIM1_CH2) |           \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_PWR) |           \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DM) |             \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DP) |             \
                                     PIN_PUPDR_FLOATING(GPIOA_JTAG_TMS) |           \
                                     PIN_PUPDR_PULLDOWN(GPIOA_JTAG_TCK) |           \
                                     PIN_PUPDR_FLOATING(GPIOA_JTAG_TDI))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_BUTTON) |           \
                                     PIN_ODR_HIGH(GPIOA_TIM5_CH2) |         \
                                     PIN_ODR_HIGH(GPIOA_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOA_SPI1_ENC2_NSS) |    \
                                     PIN_ODR_HIGH(GPIOA_SPI1_ENC1_NSS) |    \
                                     PIN_ODR_HIGH(GPIOA_SPI1_SCK) |         \
                                     PIN_ODR_HIGH(GPIOA_SPI1_MISO) |        \
                                     PIN_ODR_HIGH(GPIOA_SPI1_MOSI) |        \
                                     PIN_ODR_HIGH(GPIOA_TIM1_CH1) |         \
                                     PIN_ODR_HIGH(GPIOA_TIM1_CH2) |         \
                                     PIN_ODR_LOW(GPIOA_SPI1_PWR) |          \
                                     PIN_ODR_HIGH(GPIOA_USB_DM) |           \
                                     PIN_ODR_HIGH(GPIOA_USB_DP) |           \
                                     PIN_ODR_HIGH(GPIOA_JTAG_TMS) |         \
                                     PIN_ODR_HIGH(GPIOA_JTAG_TCK) |         \
                                     PIN_ODR_HIGH(GPIOA_JTAG_TDI))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_BUTTON, 0) |             \
                                     PIN_AFIO_AF(GPIOA_TIM5_CH2, 0) |           \
                                     PIN_AFIO_AF(GPIOA_PIN2, 0) |               \
                                     PIN_AFIO_AF(GPIOA_SPI1_ENC2_NSS, 0) |      \
                                     PIN_AFIO_AF(GPIOA_SPI1_ENC1_NSS, 0) |      \
                                     PIN_AFIO_AF(GPIOA_SPI1_SCK, 5) |           \
                                     PIN_AFIO_AF(GPIOA_SPI1_MISO, 5) |          \
                                     PIN_AFIO_AF(GPIOA_SPI1_MOSI, 5))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_TIM1_CH1, 1) |           \
                                     PIN_AFIO_AF(GPIOA_TIM1_CH2, 1) |           \
                                     PIN_AFIO_AF(GPIOA_SPI1_PWR, 0) |           \
                                     PIN_AFIO_AF(GPIOA_USB_DM, 10) |            \
                                     PIN_AFIO_AF(GPIOA_USB_DP, 10) |            \
                                     PIN_AFIO_AF(GPIOA_JTAG_TMS, 0) |           \
                                     PIN_AFIO_AF(GPIOA_JTAG_TCK, 0) |           \
                                     PIN_AFIO_AF(GPIOA_JTAG_TDI, 0))

/*
 * GPIOB setup:
 *
 * PB0  - SPI1_IMU_NSS              (output opendrain pullup maxiumum).
 * PB1  - SDIO_CD                   (input floating).
 * PB2  - BOOT1                     (input floating).
 * PB3  - JTAG_TDO                  (alternate 0).
 * PB4  - JTAG_TRST                 (alternate 0).
 * PB5  - IMU_EXTI                  (input floating).
 * PB6  - USART1_TX                 (alternate 7 - TIM4_CH1 not enabled[alternate 2]).
 * PB7  - USART1_RX                 (alternate 7 - TIM4_CH2 not enabled[alternate 2]).
 * PB8  - USART3_DE                 (output pushpull maximum).
 * PB9  - USART3_NRE                (output pushpull maximum).
 * PB10 - USART3_TX                 (alternate 7).
 * PB11 - USART3_RX                 (alternate 7).
 * PB12 - MOTOR1_EN                 (output pushpull maximum).
 * PB13 - MOTOR2_EN                 (output pushpull maximum).
 * PB14 - MOTOR1_RDY                (input floating).
 * PB15 - MOTOR2_RDY                (input floating).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIOB_SPI1_IMU_NSS) |      \
                                     PIN_MODE_INPUT(GPIOB_SDIO_CD) |            \
                                     PIN_MODE_INPUT(GPIOB_BOOT1) |              \
                                     PIN_MODE_ALTERNATE(GPIOB_JTAG_TDO) |       \
                                     PIN_MODE_ALTERNATE(GPIOB_JTAG_TRST) |      \
                                     PIN_MODE_INPUT(GPIOB_IMU_EXTI) |           \
                                     PIN_MODE_ALTERNATE(GPIOB_USART1_TX) |      \
                                     PIN_MODE_ALTERNATE(GPIOB_USART1_RX) |      \
                                     PIN_MODE_OUTPUT(GPIOB_USART3_DE) |         \
                                     PIN_MODE_OUTPUT(GPIOB_USART3_NRE) |        \
                                     PIN_MODE_ALTERNATE(GPIOB_USART3_TX) |      \
                                     PIN_MODE_ALTERNATE(GPIOB_USART3_RX) |      \
                                     PIN_MODE_OUTPUT(GPIOB_MOTOR1_EN) |         \
                                     PIN_MODE_OUTPUT(GPIOB_MOTOR2_EN) |         \
                                     PIN_MODE_INPUT(GPIOB_MOTOR1_RDY) |         \
                                     PIN_MODE_INPUT(GPIOB_MOTOR2_RDY))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOB_SPI1_IMU_NSS) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SDIO_CD) |            \
                                     PIN_OTYPE_PUSHPULL(GPIOB_BOOT1) |              \
                                     PIN_OTYPE_PUSHPULL(GPIOB_JTAG_TDO) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOB_JTAG_TRST) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOB_IMU_EXTI) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOB_USART1_TX) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOB_USART1_RX) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOB_USART3_DE) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOB_USART3_NRE) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_USART3_TX) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOB_USART3_RX) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MOTOR1_EN) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MOTOR2_EN) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MOTOR1_RDY) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_MOTOR2_RDY))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_SPI1_IMU_NSS) |      \
                                     PIN_OSPEED_HIGH(GPIOB_SDIO_CD) |           \
                                     PIN_OSPEED_HIGH(GPIOB_BOOT1) |             \
                                     PIN_OSPEED_HIGH(GPIOB_JTAG_TDO) |          \
                                     PIN_OSPEED_HIGH(GPIOB_JTAG_TRST) |         \
                                     PIN_OSPEED_HIGH(GPIOB_IMU_EXTI) |          \
                                     PIN_OSPEED_HIGH(GPIOB_USART1_TX) |         \
                                     PIN_OSPEED_HIGH(GPIOB_USART1_RX) |         \
                                     PIN_OSPEED_HIGH(GPIOB_USART3_DE) |         \
                                     PIN_OSPEED_HIGH(GPIOB_USART3_NRE) |        \
                                     PIN_OSPEED_HIGH(GPIOB_USART3_TX) |         \
                                     PIN_OSPEED_HIGH(GPIOB_USART3_RX) |         \
                                     PIN_OSPEED_HIGH(GPIOB_MOTOR1_EN) |         \
                                     PIN_OSPEED_HIGH(GPIOB_MOTOR2_EN) |         \
                                     PIN_OSPEED_HIGH(GPIOB_MOTOR1_RDY) |        \
                                     PIN_OSPEED_HIGH(GPIOB_MOTOR2_RDY))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_SPI1_IMU_NSS) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_SDIO_CD) |            \
                                     PIN_PUPDR_FLOATING(GPIOB_BOOT1) |              \
                                     PIN_PUPDR_FLOATING(GPIOB_JTAG_TDO) |           \
                                     PIN_PUPDR_FLOATING(GPIOB_JTAG_TRST) |          \
                                     PIN_PUPDR_FLOATING(GPIOB_IMU_EXTI) |           \
                                     PIN_PUPDR_FLOATING(GPIOB_USART1_TX) |          \
                                     PIN_PUPDR_FLOATING(GPIOB_USART1_RX) |          \
                                     PIN_PUPDR_FLOATING(GPIOB_USART3_DE) |          \
                                     PIN_PUPDR_FLOATING(GPIOB_USART3_NRE) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_USART3_TX) |          \
                                     PIN_PUPDR_FLOATING(GPIOB_USART3_RX) |          \
                                     PIN_PUPDR_FLOATING(GPIOB_MOTOR1_EN) |          \
                                     PIN_PUPDR_FLOATING(GPIOB_MOTOR2_EN) |          \
                                     PIN_PUPDR_FLOATING(GPIOB_MOTOR1_RDY) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_MOTOR2_RDY))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_SPI1_IMU_NSS) |     \
                                     PIN_ODR_HIGH(GPIOB_SDIO_CD) |          \
                                     PIN_ODR_HIGH(GPIOB_BOOT1) |            \
                                     PIN_ODR_HIGH(GPIOB_JTAG_TDO) |         \
                                     PIN_ODR_HIGH(GPIOB_JTAG_TRST) |        \
                                     PIN_ODR_HIGH(GPIOB_IMU_EXTI) |         \
                                     PIN_ODR_HIGH(GPIOB_USART1_TX) |        \
                                     PIN_ODR_HIGH(GPIOB_USART1_RX) |        \
                                     PIN_ODR_LOW(GPIOB_USART3_DE) |         \
                                     PIN_ODR_LOW(GPIOB_USART3_NRE) |        \
                                     PIN_ODR_HIGH(GPIOB_USART3_TX) |        \
                                     PIN_ODR_HIGH(GPIOB_USART3_RX) |        \
                                     PIN_ODR_LOW(GPIOB_MOTOR1_EN) |         \
                                     PIN_ODR_LOW(GPIOB_MOTOR2_EN) |         \
                                     PIN_ODR_HIGH(GPIOB_MOTOR1_RDY) |       \
                                     PIN_ODR_HIGH(GPIOB_MOTOR2_RDY))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_SPI1_IMU_NSS, 0) |       \
                                     PIN_AFIO_AF(GPIOB_SDIO_CD, 0) |            \
                                     PIN_AFIO_AF(GPIOB_BOOT1, 0) |              \
                                     PIN_AFIO_AF(GPIOB_JTAG_TDO, 0) |           \
                                     PIN_AFIO_AF(GPIOB_JTAG_TRST, 0) |          \
                                     PIN_AFIO_AF(GPIOB_IMU_EXTI, 0) |           \
                                     PIN_AFIO_AF(GPIOB_USART1_TX, 7) |          \
                                     PIN_AFIO_AF(GPIOB_USART1_RX, 7))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_USART3_DE, 0) |          \
                                     PIN_AFIO_AF(GPIOB_USART3_NRE, 0) |         \
                                     PIN_AFIO_AF(GPIOB_USART3_TX, 7) |          \
                                     PIN_AFIO_AF(GPIOB_USART3_RX, 7) |          \
                                     PIN_AFIO_AF(GPIOB_MOTOR1_EN, 0) |          \
                                     PIN_AFIO_AF(GPIOB_MOTOR2_EN, 0) |          \
                                     PIN_AFIO_AF(GPIOB_MOTOR1_RDY, 0) |         \
                                     PIN_AFIO_AF(GPIOB_MOTOR2_RDY, 0))

/*
 * GPIOC setup:
 *
 * PC0  - ADC10                     (analog).
 * PC1  - ADC11                     (analog).
 * PC2  - ADC12                     (analog).
 * PC3  - ADC13                     (analog).
 * PC4  - USART1_DE                 (output pushpull maximum - USB_P, ADC14 not enabled[analog]).
 * PC5  - USART1_NRE                (output pushpull maximum - ADC15 not enabled[analog]).
 * PC6  - TIM3_CH1                  (alternate 2).
 * PC7  - TIM3_CH2                  (alternate 2).
 * PC8  - SDIO_D0                   (input pullup - SDIO_D0 not enabled[alternate 12]).
 * PC9  - SDIO_D1                   (input pullup - SDIO_D1 not enabled[alternate 12]).
 * PC10 - SDIO_D2                   (input pullup - SDIO_D2 not enabled[alternate 12]).
 * PC11 - USB_DISC                  (output pushpull maximum - SDIO_D3 not enabled[alternate 12]).
 * PC12 - LED                       (output pushpull maximum - SDIO_SCK not enabled[alternate 12]).
 * PC13 - TORQUE_MEAS_NEN           (output pushpull maximum).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG(GPIOC_ADC10) |             \
                                     PIN_MODE_ANALOG(GPIOC_ADC11) |             \
                                     PIN_MODE_ANALOG(GPIOC_ADC12) |             \
                                     PIN_MODE_ANALOG(GPIOC_ADC13) |             \
                                     PIN_MODE_OUTPUT(GPIOC_USART1_DE) |         \
                                     PIN_MODE_OUTPUT(GPIOC_USART1_NRE) |        \
                                     PIN_MODE_ALTERNATE(GPIOC_TIM3_CH1) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_TIM3_CH2) |       \
                                     PIN_MODE_INPUT(GPIOC_SDIO_D0) |            \
                                     PIN_MODE_INPUT(GPIOC_SDIO_D1) |            \
                                     PIN_MODE_INPUT(GPIOC_SDIO_D2) |            \
                                     PIN_MODE_OUTPUT(GPIOC_USB_DISC) |          \
                                     PIN_MODE_OUTPUT(GPIOC_LED) |               \
                                     PIN_MODE_OUTPUT(GPIOC_TORQUE_MEAS_NEN) |   \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |           \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_ADC10) |              \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC11) |              \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC12) |              \
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC13) |              \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USART1_DE) |          \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USART1_NRE) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOC_TIM3_CH1) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOC_TIM3_CH2) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D0) |            \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D1) |            \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D2) |            \
                                     PIN_OTYPE_PUSHPULL(GPIOC_USB_DISC) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOC_LED) |                \
                                     PIN_OTYPE_PUSHPULL(GPIOC_TORQUE_MEAS_NEN) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |           \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_HIGH(GPIOC_ADC10) |             \
                                     PIN_OSPEED_HIGH(GPIOC_ADC11) |             \
                                     PIN_OSPEED_HIGH(GPIOC_ADC12) |             \
                                     PIN_OSPEED_HIGH(GPIOC_ADC13) |             \
                                     PIN_OSPEED_HIGH(GPIOC_USART1_DE) |         \
                                     PIN_OSPEED_HIGH(GPIOC_USART1_NRE) |        \
                                     PIN_OSPEED_HIGH(GPIOC_TIM3_CH1) |          \
                                     PIN_OSPEED_HIGH(GPIOC_TIM3_CH2) |          \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D0) |           \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D1) |           \
                                     PIN_OSPEED_HIGH(GPIOC_SDIO_D2) |           \
                                     PIN_OSPEED_HIGH(GPIOC_USB_DISC) |          \
                                     PIN_OSPEED_HIGH(GPIOC_LED) |               \
                                     PIN_OSPEED_HIGH(GPIOC_TORQUE_MEAS_NEN) |   \
                                     PIN_OSPEED_HIGH(GPIOC_OSC32_IN) |          \
                                     PIN_OSPEED_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_ADC10) |              \
                                     PIN_PUPDR_FLOATING(GPIOC_ADC11) |              \
                                     PIN_PUPDR_FLOATING(GPIOC_ADC12) |              \
                                     PIN_PUPDR_FLOATING(GPIOC_ADC13) |              \
                                     PIN_PUPDR_FLOATING(GPIOC_USART1_DE) |          \
                                     PIN_PUPDR_FLOATING(GPIOC_USART1_NRE) |         \
                                     PIN_PUPDR_FLOATING(GPIOC_TIM3_CH1) |           \
                                     PIN_PUPDR_FLOATING(GPIOC_TIM3_CH2) |           \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D0) |              \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D1) |              \
                                     PIN_PUPDR_PULLUP(GPIOC_SDIO_D2) |              \
                                     PIN_PUPDR_FLOATING(GPIOC_USB_DISC) |           \
                                     PIN_PUPDR_FLOATING(GPIOC_LED) |                \
                                     PIN_PUPDR_FLOATING(GPIOC_TORQUE_MEAS_NEN) |    \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |           \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_ADC10) |            \
                                     PIN_ODR_HIGH(GPIOC_ADC11) |            \
                                     PIN_ODR_HIGH(GPIOC_ADC12) |            \
                                     PIN_ODR_HIGH(GPIOC_ADC13) |            \
                                     PIN_ODR_HIGH(GPIOC_USART1_DE) |        \
                                     PIN_ODR_HIGH(GPIOC_USART1_NRE) |       \
                                     PIN_ODR_HIGH(GPIOC_TIM3_CH1) |         \
                                     PIN_ODR_HIGH(GPIOC_TIM3_CH2) |         \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D0) |          \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D1) |          \
                                     PIN_ODR_HIGH(GPIOC_SDIO_D2) |          \
                                     PIN_ODR_HIGH(GPIOC_USB_DISC) |         \
                                     PIN_ODR_HIGH(GPIOC_LED) |              \
                                     PIN_ODR_HIGH(GPIOC_TORQUE_MEAS_NEN) |   \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_ADC10, 0) |              \
                                     PIN_AFIO_AF(GPIOC_ADC11, 0) |              \
                                     PIN_AFIO_AF(GPIOC_ADC12, 0) |              \
                                     PIN_AFIO_AF(GPIOC_ADC13, 0) |              \
                                     PIN_AFIO_AF(GPIOC_USART1_DE, 0) |          \
                                     PIN_AFIO_AF(GPIOC_USART1_NRE, 0) |         \
                                     PIN_AFIO_AF(GPIOC_TIM3_CH1, 2) |           \
                                     PIN_AFIO_AF(GPIOC_TIM3_CH2, 2))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_SDIO_D0, 0) |            \
                                     PIN_AFIO_AF(GPIOC_SDIO_D1, 0) |            \
                                     PIN_AFIO_AF(GPIOC_SDIO_D2, 0) |            \
                                     PIN_AFIO_AF(GPIOC_USB_DISC, 0) |           \
                                     PIN_AFIO_AF(GPIOC_LED, 0) |                \
                                     PIN_AFIO_AF(GPIOC_TORQUE_MEAS_NEN, 0) |    \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0) |           \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0))

/*
 * GPIOD setup:
 *
 * PD0  - OSC_IN                    (input floating).
 * PD1  - OSC_OUT                   (input floating).
 * PD2  - SDIO_CMD                  (input floating - SDIO_CMD not enabled [alternate 12]).
 * PD3  - PIN3                      (input pullup).
 * PD4  - PIN4                      (input pullup).
 * PD5  - PIN5                      (input pullup).
 * PD6  - PIN6                      (input pullup).
 * PD7  - PIN7                      (input pullup).
 * PD8  - PIN8                      (input pullup).
 * PD9  - PIN9                      (input pullup).
 * PD10 - PIN10                     (input pullup).
 * PD11 - PIN11                     (input pullup).
 * PD12 - PIN12                     (input pullup).
 * PD13 - PIN13                     (input pullup).
 * PD14 - PIN14                     (input pullup).
 * PD15 - PIN15                     (input pullup).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOD_OSC_OUT) |        \
                                     PIN_MODE_INPUT(GPIOD_SDIO_CMD) |       \
                                     PIN_MODE_INPUT(GPIOD_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOD_SDIO_CMD) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_HIGH(GPIOD_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOD_OSC_OUT) |       \
                                     PIN_OSPEED_HIGH(GPIOD_SDIO_CMD) |      \
                                     PIN_OSPEED_HIGH(GPIOD_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOD_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(GPIOD_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_OSC_OUT) |    \
                                     PIN_PUPDR_PULLUP(GPIOD_SDIO_CMD) |     \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOD_OSC_OUT) |          \
                                     PIN_ODR_HIGH(GPIOD_SDIO_CMD) |         \
                                     PIN_ODR_HIGH(GPIOD_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_OSC_IN, 0) |         \
                                     PIN_AFIO_AF(GPIOD_OSC_OUT, 0) |        \
                                     PIN_AFIO_AF(GPIOD_SDIO_CMD, 0) |       \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOD_PIN15, 0))

/*
 * GPIOE setup:
 *
 * PE0  - PIN0                      (input pullup).
 * PE1  - PIN1                      (input pullup).
 * PE2  - PIN2                      (input pullup).
 * PE3  - PIN3                      (input pullup).
 * PE4  - PIN4                      (input pullup).
 * PE5  - PIN5                      (input pullup).
 * PE6  - PIN6                      (input pullup).
 * PE7  - PIN7                      (input pullup).
 * PE8  - PIN8                      (input pullup).
 * PE9  - PIN9                      (input pullup).
 * PE10 - PIN10                     (input pullup).
 * PE11 - PIN11                     (input pullup).
 * PE12 - PIN12                     (input pullup).
 * PE13 - PIN13                     (input pullup).
 * PE14 - PIN14                     (input pullup).
 * PE15 - PIN15                     (input pullup).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_HIGH(GPIOE_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOE_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOE_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOE_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOE_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOE_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLUP(GPIOE_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN7, 0))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0))

/*
 * GPIOF setup:
 *
 * PF0  - PIN0                      (input pullup).
 * PF1  - PIN1                      (input pullup).
 * PF2  - PIN2                      (input pullup).
 * PF3  - PIN3                      (input pullup).
 * PF4  - PIN4                      (input pullup).
 * PF5  - PIN5                      (input pullup).
 * PF6  - PIN6                      (input pullup).
 * PF7  - PIN7                      (input pullup).
 * PF8  - PIN8                      (input pullup).
 * PF9  - PIN9                      (input pullup).
 * PF10 - PIN10                     (input pullup).
 * PF11 - PIN11                     (input pullup).
 * PF12 - PIN12                     (input pullup).
 * PF13 - PIN13                     (input pullup).
 * PF14 - PIN14                     (input pullup).
 * PF15 - PIN15                     (input pullup).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_PULLUP(GPIOF_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0))

/*
 * GPIOG setup:
 *
 * PG0  - PIN0                      (input pullup).
 * PG1  - PIN1                      (input pullup).
 * PG2  - PIN2                      (input pullup).
 * PG3  - PIN3                      (input pullup).
 * PG4  - PIN4                      (input pullup).
 * PG5  - PIN5                      (input pullup).
 * PG6  - PIN6                      (input pullup).
 * PG7  - PIN7                      (input pullup).
 * PG8  - PIN8                      (input pullup).
 * PG9  - PIN9                      (input pullup).
 * PG10 - PIN10                     (input pullup).
 * PG11 - PIN11                     (input pullup).
 * PG12 - PIN12                     (input pullup).
 * PG13 - PIN13                     (input pullup).
 * PG14 - PIN14                     (input pullup).
 * PG15 - PIN15                     (input pullup).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_INPUT(GPIOG_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_HIGH(GPIOG_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOG_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_PULLUP(GPIOG_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOG_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0))

/*
 * GPIOH setup:
 *
 * PH0  - PIN0                      (input pullup).
 * PH1  - PIN1                      (input pullup).
 * PH2  - PIN2                      (input pullup).
 * PH3  - PIN3                      (input pullup).
 * PH4  - PIN4                      (input pullup).
 * PH5  - PIN5                      (input pullup).
 * PH6  - PIN6                      (input pullup).
 * PH7  - PIN7                      (input pullup).
 * PH8  - PIN8                      (input pullup).
 * PH9  - PIN9                      (input pullup).
 * PH10 - PIN10                     (input pullup).
 * PH11 - PIN11                     (input pullup).
 * PH12 - PIN12                     (input pullup).
 * PH13 - PIN13                     (input pullup).
 * PH14 - PIN14                     (input pullup).
 * PH15 - PIN15                     (input pullup).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOH_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_PULLUP(GPIOH_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0))

/*
 * GPIOI setup:
 *
 * PI0  - PIN0                      (input pullup).
 * PI1  - PIN1                      (input pullup).
 * PI2  - PIN2                      (input pullup).
 * PI3  - PIN3                      (input pullup).
 * PI4  - PIN4                      (input pullup).
 * PI5  - PIN5                      (input pullup).
 * PI6  - PIN6                      (input pullup).
 * PI7  - PIN7                      (input pullup).
 * PI8  - PIN8                      (input pullup).
 * PI9  - PIN9                      (input pullup).
 * PI10 - PIN10                     (input pullup).
 * PI11 - PIN11                     (input pullup).
 * PI12 - PIN12                     (input pullup).
 * PI13 - PIN13                     (input pullup).
 * PI14 - PIN14                     (input pullup).
 * PI15 - PIN15                     (input pullup).
 */
#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(GPIOI_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_HIGH(GPIOI_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOI_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_PULLUP(GPIOI_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN4, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN6, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN7, 0))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0) |           \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0) |          \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0) |          \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0) |          \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0) |          \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0) |          \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0))

/*
 * Board specific USB bus activation macro. Replaces use of usbConnectBus() in the USB driver.
 */
#define board_usb_lld_connect_bus() palClearPad(GPIOC, GPIOC_USB_DISC)

/*
 * Board specific USB bus de-activation macro. Replaces use of usbDisconnectBus() in the USB driver.
 */
#define board_usb_lld_disconnect_bus() palSetPad(GPIOC, GPIOC_USB_DISC)


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
