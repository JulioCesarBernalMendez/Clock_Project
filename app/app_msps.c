#include <stdint.h>
#include "stm32g0xx.h"
#include "queue.h"
#include "app_bsp.h"
#include "lcd.h"
#include "temp.h"
#include "app_clock.h"
#include "app_serial.h"

/**
  * @brief  Configures the System Frequency to 48MHz.
  * @param  None.
  * @retval None.
  */
void HAL_MspInit( void )
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 }; // RCC oscillator configuration structure
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 }; // RCC clock configuration structure

    // configure the main internal regulator output voltage
    HAL_PWREx_ControlVoltageScaling( PWR_REGULATOR_VOLTAGE_SCALE1 );

    /* configure PLLRCLK frequency 
    FPLLRCLK = ( FPLLIN * N ) / ( M * R ) = ( 16 MHz * 12 ) / ( 2 * 2 ) = 48 MHz */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;          // HSI is the oscillator to be configured
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;                      // turn on HSI
    RCC_OscInitStruct.HSIDiv         = RCC_HSI_DIV1;                    // HSI prescaler = 1
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // default HSI calibration trimming value
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;                      // turn on PLL
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSI;               // HSI (16 MHz) selected as PLL source
    RCC_OscInitStruct.PLL.PLLN       = 12u;                             // N = 12
    RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV2;                   // M = 2
    RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;                   // R = 2
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV7;                   // P = 7
    RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;                   // Q = 2
    HAL_RCC_OscConfig( &RCC_OscInitStruct );                            // initialize RCC oscillator

    // select PLLRCLK as system clock source
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK; // PLLRCLK selected as SYSCLK source
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;         // AHB = 1
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;           // APB = 1
    HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_2 ); // initialize RCC buses clocks
}


/**
  * @brief  UART low level initialization.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains the configuration information for the specified UART.
  * @retval None.
  */
void HAL_UART_MspInit( UART_HandleTypeDef *huart )
{   
    GPIO_InitTypeDef GPIO_InitStruct = { 0 }; // GPIO configuration structure

    // if USART2
    if( huart->Instance == USART2 ) {

        __HAL_RCC_USART2_CLK_ENABLE(); // enable UART2 clock
        __HAL_RCC_GPIOA_CLK_ENABLE();  // enable GPIOA clock

        // configure GPIOA2 (UART2 TX) and GPIOA3 (UART2 RX) in alternate mode
        GPIO_InitStruct.Pin       = GPIO_PIN_UART2_TX | GPIO_PIN_UART2_RX; // uart2 pins
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;                       // pins as alternate functions push pull
        GPIO_InitStruct.Pull      = GPIO_PULLUP;                           // pull up
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;                  // high speed pins
        GPIO_InitStruct.Alternate = GPIO_AF1_USART2;                       // pins as uart2 alternate function
        HAL_GPIO_Init( GPIOUART2, &GPIO_InitStruct );                      // initialize GPIO

        // enable UART2 interruption with priority 3
        HAL_NVIC_SetPriority( USART2_LPUART2_IRQn, 3, 0 );
        HAL_NVIC_EnableIRQ( USART2_LPUART2_IRQn );
    }
}


/**
  * @brief  RTC low level initialization.
  * @param  hrtc Pointer to an RTC_HandleTypeDef structure that contains the configuration information for the specified RTC.
  * @retval None.
  */
void HAL_RTC_MspInit( RTC_HandleTypeDef* hrtc )
{   
    RCC_OscInitTypeDef       RCC_OscInitStruct = { 0 }; // RCC oscillator configuration structure
    RCC_PeriphCLKInitTypeDef RCC_ClkInitStruct = { 0 }; // RCC clock configuration structure

    // if RTC
    if( hrtc->Instance == RTC ) {

        // turn on LSE clock
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE; // LSE is the oscillator to be configured
        RCC_OscInitStruct.LSEState       = RCC_LSE_ON;             // turn on LSE
        RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE;           // no PLL
        HAL_RCC_OscConfig( &RCC_OscInitStruct );                   // initialize RCC oscillator

        // select LSE as RTC clock source
        RCC_ClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;    // RTC is the extended clock to be configured
        RCC_ClkInitStruct.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE; // LSE as RTC source
        HAL_RCCEx_PeriphCLKConfig( &RCC_ClkInitStruct );               // initialize RCC extended clock

        __HAL_RCC_RTC_ENABLE();        // enable RTC clock
        __HAL_RCC_RTCAPB_CLK_ENABLE(); // enable APB clock

        // enable RTC interruption with priority 1
        HAL_NVIC_SetPriority( RTC_TAMP_IRQn, 1, 0 );
        HAL_NVIC_EnableIRQ( RTC_TAMP_IRQn );
    }
}


/**
  * @brief  SPI low level initialization.
  * @param  hspi Pointer to an SPI_HandleTypeDef structure that contains the configuration information for the specified SPI.
  * @retval None.
  */
void HAL_SPI_MspInit( SPI_HandleTypeDef *hspi )
{      
    GPIO_InitTypeDef GPIO_InitStruct = { 0 }; // GPIO configuration structure

    // if SPI3
    if( hspi->Instance == SPI3 ) {

        __HAL_RCC_SPI3_CLK_ENABLE();  // enable SPI3  clock
        __HAL_RCC_GPIOC_CLK_ENABLE(); // enable GPIOC clock

        // configure GPIOC10 (SPI3 SCK) and GPIOC12 (SPI3 MOSI) in alternate mode
        GPIO_InitStruct.Pin       = GPIO_PIN_SCK | GPIO_PIN_MOSI; // spi3 pins
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;              // pins as alternate function push pull
        GPIO_InitStruct.Pull      = GPIO_PULLUP;                  // pull up
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;         // high speed pins
        GPIO_InitStruct.Alternate = GPIO_AF4_SPI3;                // pins as spi3 alternate function
        HAL_GPIO_Init( GPIOSPI3, &GPIO_InitStruct );              // initialize GPIO
    }
}


/**
  * @brief  TIMER low level initialization.
  * @param  htim Pointer to a TIM_HandleTypeDef structure that contains the configuration information for the specified TIM.
  * @retval None.
  */
void HAL_TIM_Base_MspInit( TIM_HandleTypeDef *htim )
{   
    // if TIM6
    if( htim->Instance == TIM6 ) {
        __HAL_RCC_TIM6_CLK_ENABLE(); // enable TIM6 clock
    }
}


/**
  * @brief  WWDG low level initialization.
  * @param  hwwdg Pointer to a WWDG_HandleTypeDef structure that contains the configuration information for the specified WWDG.
  * @retval None.
  */
void HAL_WWDG_MspInit( WWDG_HandleTypeDef *hwwdg )
{   
    // if WWDG
    if( hwwdg->Instance == WWDG ) {

        __HAL_RCC_WWDG_CLK_ENABLE(); // enable WWDG clock

        // freeze wwdg counter while core is halted
        __HAL_RCC_DBGMCU_CLK_ENABLE();
        __HAL_DBGMCU_FREEZE_WWDG();
    }
}


/**
  * @brief  I2C low level initialization.
  * @param  hi2c Pointer to an I2C_HandleTypeDef structure that contains the configuration information for the specified I2C.
  * @retval None.
  */
void HAL_I2C_MspInit( I2C_HandleTypeDef *hi2c )
{
    RCC_PeriphCLKInitTypeDef RCC_ClkInitStruct = { 0 }; // RCC clock configuration structure
    GPIO_InitTypeDef         GPIO_InitStruct   = { 0 }; // GPIO configuration structure

    // if I2C1
    if( hi2c->Instance == I2C1 ) {
    
        __HAL_RCC_I2C1_CLK_ENABLE();  // enable I2C1 clock
        __HAL_RCC_GPIOB_CLK_ENABLE(); // enable GPIOB clock

        // select HSI as I2C1 clock source
        RCC_ClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;      // I2C1 is the extended clock to be configured
        RCC_ClkInitStruct.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_HSI;   // HSI as I2C1 source
        HAL_RCCEx_PeriphCLKConfig( &RCC_ClkInitStruct );                  // initialize RCC extended clock

        // configure GPIOB6 (I2C1 SCK) and GPIOB7 (I2C1 SDA) in alternate mode
        GPIO_InitStruct.Pin       = GPIO_PIN_I2C1_SCK | GPIO_PIN_I2C1_SDA; // i2c1 pins
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;                       // pins as alternate function open drain
        GPIO_InitStruct.Pull      = GPIO_PULLUP;                           // pull up
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;                   // low speed pins
        GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;                         // pins as i2c1 alternate function
        HAL_GPIO_Init( GPIOI2C1, &GPIO_InitStruct );                       // initialize GPIO
    }
}


/**
  * @brief  Temperature Sensor low level initialization.
  * @param  htemp Pointer to a TEMP_HandleTypeDef structure that contains the configuration information for the specified Temperature Sensor.
  * @retval None.
  */
void MOD_TEMP_MspInit( TEMP_HandleTypeDef *htemp )
{      
    GPIO_InitTypeDef GPIO_InitStruct = { 0 }; // GPIO configuration structure

    __HAL_RCC_GPIOB_CLK_ENABLE(); // enable GPIOB clock

    // initialize GPIO for sensor alert pin
    GPIO_InitStruct.Pin   = htemp->AlertPin;             // alert pin
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;             // pin as input
    GPIO_InitStruct.Pull  = GPIO_NOPULL;                 // no pull
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;         // low speed pin
    HAL_GPIO_Init( htemp->AlertPort, &GPIO_InitStruct ); // initialize GPIO
}