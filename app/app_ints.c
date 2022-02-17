#include <stdint.h>
#include "stm32g0xx.h"
#include "queue.h"
#include "app_bsp.h"

/**
  * @brief  This is the code that gets called when the processor receives a SysTick Timer interrupt.
  * @param  None.
  * @retval None.
  */
void SysTick_Handler( void )
{
    HAL_IncTick();
}


/**
  * @brief  This is the code that gets called when the processor receives a UART2 interrupt.
  * @param  None.
  * @retval None.
  */
void USART2_LPUART2_IRQHandler( void )
{   
    HAL_UART_IRQHandler( &Uart2Handler );
}


/**
  * @brief  This is the code that gets called when the processor receives an RTC interrupt.
  * @param  None.
  * @retval None.
  */
void RTC_TAMP_IRQHandler( void )
{
    HAL_RTC_AlarmIRQHandler( &RtcHandler );
}


/**
  * @brief  This is the code that gets called when the processor receives an EXTI4_15 (board button) interrupt.
  * @param  None.
  * @retval None.
  */
void EXTI4_15_IRQHandler( void )
{
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_13 );
}