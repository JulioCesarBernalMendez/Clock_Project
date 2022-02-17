#ifndef BSP_H
#define BSP_H

    // serial message type definitions
    #define NONE   0u
    #define TIME   1u
    #define DATE   2u
    #define ALARM  3u
    #define HEART  4u
    #define TEMP   5u
    
    // serial message structure
    typedef struct
    {
        uint8_t  msg;     // serial message type
        uint16_t param1;  // hours or day or miliseconds or lower temperature
        uint8_t  param2;  // minutes or month or upper temperature
        uint16_t param3;  // seconds or year
    } SERIAL_MsgTypeDef;

    // extern functions
    extern void disable_interrupts( void );
    extern void enable_interrupts( void );
    extern void HAL_RTC_AlarmAEventCallback( RTC_HandleTypeDef *hrtc );
    extern void HAL_GPIO_EXTI_Rising_Callback( uint16_t GPIO_Pin );
    extern void HAL_GPIO_EXTI_Falling_Callback( uint16_t GPIO_Pin );
    extern void SysTick_Handler( void );
    extern void USART2_LPUART2_IRQHandler( void );
    extern void RTC_TAMP_IRQHandler( void );
    extern void EXTI4_15_IRQHandler( void );
    extern void HAL_MspInit( void );
    extern void HAL_UART_MspInit( UART_HandleTypeDef *huart );
    extern void HAL_RTC_MspInit( RTC_HandleTypeDef* hrtc );
    extern void HAL_SPI_MspInit( SPI_HandleTypeDef *hspi );
    extern void HAL_TIM_Base_MspInit( TIM_HandleTypeDef *htim );
    extern void HAL_WWDG_MspInit( WWDG_HandleTypeDef *hwwdg );
    extern void HAL_I2C_MspInit( I2C_HandleTypeDef *hi2c );
    extern void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart );

    // extern variables
    extern UART_HandleTypeDef  Uart2Handler;       // UART handler structure
    extern RTC_HandleTypeDef   RtcHandler;         // RTC handler structure
    extern QUEUE_HandleTypeDef QueueHandler_Clock; // clock queue handler queue
    extern uint16_t            HeartRead;          // current heartbeat period

#endif