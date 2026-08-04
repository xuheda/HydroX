/****************************************************************************
 * HydroX FMUv6C board definition for Pixhawk 6C / 6C Mini.
 *
 * Hardware values are derived from the PX4 fmu-v6c board port (BSD-3-Clause)
 * and adapted to Apache NuttX 13.0.0. See boards/fmu_v6c/README.md.
 ****************************************************************************/

#ifndef __HYDROX_BOARDS_FMU_V6C_INCLUDE_BOARD_H
#define __HYDROX_BOARDS_FMU_V6C_INCLUDE_BOARD_H

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* FMUv6C uses a 16 MHz HSE and runs the STM32H743 at 480 MHz. */

#define STM32_BOARD_XTAL                 16000000ul
#define STM32_HSI_FREQUENCY              16000000ul
#define STM32_LSI_FREQUENCY              32000ul
#define STM32_HSE_FREQUENCY              STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY              32768ul
#define STM32_BOARD_USEHSE
#define STM32_PLLCFG_PLLSRC               RCC_PLLCKSELR_PLLSRC_HSE

#define STM32_PLLCFG_PLL1CFG              (RCC_PLLCFGR_PLL1VCOSEL_WIDE | \
                                           RCC_PLLCFGR_PLL1RGE_4_8_MHZ | \
                                           RCC_PLLCFGR_DIVP1EN | \
                                           RCC_PLLCFGR_DIVQ1EN | \
                                           RCC_PLLCFGR_DIVR1EN)
#define STM32_PLLCFG_PLL1M                RCC_PLLCKSELR_DIVM1(1)
#define STM32_PLLCFG_PLL1N                RCC_PLL1DIVR_N1(60)
#define STM32_PLLCFG_PLL1P                RCC_PLL1DIVR_P1(2)
#define STM32_PLLCFG_PLL1Q                RCC_PLL1DIVR_Q1(4)
#define STM32_PLLCFG_PLL1R                RCC_PLL1DIVR_R1(8)
#define STM32_VCO1_FREQUENCY              ((STM32_HSE_FREQUENCY / 1) * 60)
#define STM32_PLL1P_FREQUENCY             (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1Q_FREQUENCY             (STM32_VCO1_FREQUENCY / 4)
#define STM32_PLL1R_FREQUENCY             (STM32_VCO1_FREQUENCY / 8)

#define STM32_PLLCFG_PLL2CFG              (RCC_PLLCFGR_PLL2VCOSEL_WIDE | \
                                           RCC_PLLCFGR_PLL2RGE_4_8_MHZ | \
                                           RCC_PLLCFGR_DIVP2EN | \
                                           RCC_PLLCFGR_DIVQ2EN | \
                                           RCC_PLLCFGR_DIVR2EN)
#define STM32_PLLCFG_PLL2M                RCC_PLLCKSELR_DIVM2(4)
#define STM32_PLLCFG_PLL2N                RCC_PLL2DIVR_N2(48)
#define STM32_PLLCFG_PLL2P                RCC_PLL2DIVR_P2(2)
#define STM32_PLLCFG_PLL2Q                RCC_PLL2DIVR_Q2(2)
#define STM32_PLLCFG_PLL2R                RCC_PLL2DIVR_R2(2)
#define STM32_VCO2_FREQUENCY              ((STM32_HSE_FREQUENCY / 4) * 48)
#define STM32_PLL2P_FREQUENCY             (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL2Q_FREQUENCY             (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL2R_FREQUENCY             (STM32_VCO2_FREQUENCY / 2)

#define STM32_PLLCFG_PLL3CFG              (RCC_PLLCFGR_PLL3VCOSEL_WIDE | \
                                           RCC_PLLCFGR_PLL3RGE_4_8_MHZ | \
                                           RCC_PLLCFGR_DIVQ3EN)
#define STM32_PLLCFG_PLL3M                RCC_PLLCKSELR_DIVM3(4)
#define STM32_PLLCFG_PLL3N                RCC_PLL3DIVR_N3(48)
#define STM32_PLLCFG_PLL3P                RCC_PLL3DIVR_P3(2)
#define STM32_PLLCFG_PLL3Q                RCC_PLL3DIVR_Q3(4)
#define STM32_PLLCFG_PLL3R                RCC_PLL3DIVR_R3(2)
#define STM32_VCO3_FREQUENCY              ((STM32_HSE_FREQUENCY / 4) * 48)
#define STM32_PLL3P_FREQUENCY             (STM32_VCO3_FREQUENCY / 2)
#define STM32_PLL3Q_FREQUENCY             (STM32_VCO3_FREQUENCY / 4)
#define STM32_PLL3R_FREQUENCY             (STM32_VCO3_FREQUENCY / 2)

#define STM32_RCC_D1CFGR_D1CPRE           RCC_D1CFGR_D1CPRE_SYSCLK
#define STM32_RCC_D1CFGR_HPRE             RCC_D1CFGR_HPRE_SYSCLKd2
#define STM32_SYSCLK_FREQUENCY            STM32_PLL1P_FREQUENCY
#define STM32_CPUCLK_FREQUENCY            STM32_SYSCLK_FREQUENCY
#define STM32_ACLK_FREQUENCY              (STM32_CPUCLK_FREQUENCY / 2)
#define STM32_HCLK_FREQUENCY              (STM32_CPUCLK_FREQUENCY / 2)
#define STM32_BOARD_HCLK                  STM32_HCLK_FREQUENCY
#define STM32_RCC_D2CFGR_D2PPRE1          RCC_D2CFGR_D2PPRE1_HCLKd2
#define STM32_RCC_D2CFGR_D2PPRE2          RCC_D2CFGR_D2PPRE2_HCLKd2
#define STM32_RCC_D1CFGR_D1PPRE           RCC_D1CFGR_D1PPRE_HCLKd2
#define STM32_RCC_D3CFGR_D3PPRE           RCC_D3CFGR_D3PPRE_HCLKd2
#define STM32_PCLK1_FREQUENCY             (STM32_HCLK_FREQUENCY / 2)
#define STM32_PCLK2_FREQUENCY             (STM32_HCLK_FREQUENCY / 2)
#define STM32_PCLK3_FREQUENCY             (STM32_HCLK_FREQUENCY / 2)
#define STM32_PCLK4_FREQUENCY             (STM32_HCLK_FREQUENCY / 2)

#define STM32_APB1_TIM2_CLKIN             (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN             (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN             (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN             (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN             (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN             (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN            (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN            (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN            (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB2_TIM1_CLKIN             (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN             (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN            (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN            (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN            (2 * STM32_PCLK2_FREQUENCY)

#define STM32_RCC_D2CCIP2R_I2C123SRC      RCC_D2CCIP2R_I2C123SEL_HSI
#define STM32_RCC_D3CCIPR_I2C4SRC         RCC_D3CCIPR_I2C4SEL_HSI
#define STM32_RCC_D2CCIP1R_SPI123SRC      RCC_D2CCIP1R_SPI123SEL_PLL2
#define STM32_RCC_D2CCIP1R_SPI45SRC       RCC_D2CCIP1R_SPI45SEL_PLL2
#define STM32_RCC_D3CCIPR_SPI6SRC         RCC_D3CCIPR_SPI6SEL_PLL2
#define STM32_RCC_D2CCIP2R_USBSRC         RCC_D2CCIP2R_USBSEL_PLL3
#define STM32_RCC_D2CCIP2R_USART234578_SEL RCC_D2CCIP2R_USART234578SEL_RCC
#define STM32_RCC_D2CCIP2R_USART16_SEL    RCC_D2CCIP2R_USART16SEL_RCC
#define STM32_RCC_D3CCIPR_ADCSRC          RCC_D3CCIPR_ADCSEL_PLL2
#define STM32_RCC_D2CCIP1R_FDCANSEL       RCC_D2CCIP1R_FDCANSEL_HSE
#define STM32_FDCANCLK                    STM32_HSE_FREQUENCY
#define BOARD_FLASH_WAITSTATES            2

/* Keep the FMUv6C serial registration order stable. Unused ports are not
 * opened by HydroX, but enabling them preserves the documented tty mapping. */
#define GPIO_USART1_RX                    GPIO_USART1_RX_2 /* PA10 */
#define GPIO_USART1_TX                    GPIO_USART1_TX_3 /* PB6 */
#define GPIO_USART2_RX                    GPIO_USART2_RX_1 /* PA3 */
#define GPIO_USART2_TX                    GPIO_USART2_TX_2 /* PD5 */
#define GPIO_USART6_RX                    GPIO_USART6_RX_1 /* PC7, PX4IO left disabled */
#define GPIO_USART6_TX                    GPIO_USART6_TX_1 /* PC6, PX4IO left disabled */
#define GPIO_UART8_RX                     GPIO_UART8_RX_1  /* PE0 */
#define GPIO_UART8_TX                     GPIO_UART8_TX_1  /* PE1 */
/* Debug console: USART3 on the FMU debug connector. */
#define GPIO_USART3_RX                    GPIO_USART3_RX_3 /* PD9 */
#define GPIO_USART3_TX                    GPIO_USART3_TX_3 /* PD8 */

/* TELEM2: UART5, /dev/ttyS3 in the locked defconfig. */
#define GPIO_UART5_RX                     GPIO_UART5_RX_3  /* PD2 */
#define GPIO_UART5_TX                     GPIO_UART5_TX_3  /* PC12 */
#define GPIO_UART5_RTS                    GPIO_UART5_RTS_0 /* PC8 */
#define GPIO_UART5_CTS                    (GPIO_UART5_CTS_0 | GPIO_PULLDOWN) /* PC9 */

/* TELEM1: UART7, /dev/ttyS5 in the FMUv6C serial order. */
#define GPIO_UART7_RX                     GPIO_UART7_RX_3  /* PE7 */
#define GPIO_UART7_TX                     GPIO_UART7_TX_3  /* PE8 */
#define GPIO_UART7_RTS                    GPIO_UART7_RTS_1 /* PE9 */
#define GPIO_UART7_CTS                    (GPIO_UART7_CTS_1 | GPIO_PULLDOWN) /* PE10 */

#define GPIO_OTGFS_DM                     (GPIO_OTGFS_DM_0 | GPIO_SPEED_100MHz)
#define GPIO_OTGFS_DP                     (GPIO_OTGFS_DP_0 | GPIO_SPEED_100MHz)

/* Active-low status LEDs. */
#define GPIO_nLED_RED                     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN10)
#define GPIO_nLED_BLUE                    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTD | GPIO_PIN11)
#define BOARD_LED1                        0
#define BOARD_LED2                        1
#define BOARD_NLEDS                       2
#define BOARD_LED_RED                     BOARD_LED1
#define BOARD_LED_BLUE                    BOARD_LED2
#define BOARD_LED1_BIT                    (1 << BOARD_LED1)
#define BOARD_LED2_BIT                    (1 << BOARD_LED2)
#define LED_STARTED                       0
#define LED_HEAPALLOCATE                  1
#define LED_IRQSENABLED                   2
#define LED_STACKCREATED                  3
#define LED_INIRQ                         4
#define LED_SIGNAL                        5
#define LED_ASSERTION                     6
#define LED_PANIC                         7
#define LED_IDLE                          8

/* FMU PWM pins are forced to input/pulldown before drivers start. */
#define GPIO_FMU_PWM1_SAFE                (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTA | GPIO_PIN8)
#define GPIO_FMU_PWM2_SAFE                (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTE | GPIO_PIN11)
#define GPIO_FMU_PWM3_SAFE                (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTE | GPIO_PIN13)
#define GPIO_FMU_PWM4_SAFE                (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTE | GPIO_PIN14)
#define GPIO_FMU_PWM5_SAFE                (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTD | GPIO_PIN14)
#define GPIO_FMU_PWM6_SAFE                (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTD | GPIO_PIN15)
#define GPIO_FMU_PWM7_SAFE                (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTA | GPIO_PIN0)
#define GPIO_FMU_PWM8_SAFE                (GPIO_INPUT | GPIO_PULLDOWN | GPIO_PORTA | GPIO_PIN2)

#endif /* __HYDROX_BOARDS_FMU_V6C_INCLUDE_BOARD_H */