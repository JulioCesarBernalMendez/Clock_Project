#include "stm32g0xx.h"
#include <stdint.h>
#include "app_bsp.h"


/**------------------------------------------------------------------------------------------------
Brief.- Punto de entrada del programa
-------------------------------------------------------------------------------------------------*/
int main( void )
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    HAL_Init( );

    __HAL_RCC_GPIOA_CLK_ENABLE( );

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin   = GPIO_PIN_5;
    HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );
    

    for( ; ; )
    {
        HAL_GPIO_TogglePin( GPIOA, GPIO_PIN_5 );
        HAL_Delay( 100u );
    }

    return 0u;
}

