#include <stdint.h>
#include "stm32g0xx.h"
#include "queue.h"
#include "app_bsp.h"
#include "lcd.h"
#include "temp.h"
#include "app_clock.h"
#include "app_serial.h"

// heartbeat definitions
#define GPIOHEART       GPIOA
#define GPIO_PIN_HEART  GPIO_PIN_5

// function prototypes
void dog_init( void );
void pet_the_dog( void );
void heart_init( void );
void heart_beat( void );
void disable_interrupts( void );
void enable_interrupts( void );

// main.c global variables
static WWDG_HandleTypeDef WwdgHandler; // WWDG handler structure
uint16_t HeartRead = 300u;             // current heartbeat period

int main( void )
{   
    uint32_t TickClock;  // timer counter for clock tasks
    uint32_t TickSerial; // timer counter for serial tasks
    uint32_t TickPet;    // timer counter for resetting wwdg
    uint32_t TickHeart;  // timer counter for heartbeat

    HAL_Init();    // HAL libraries initialization
    clock_init();  // clock processes initialization
    serial_init(); // serial processes initialization
    heart_init();  // heartbeat initialization
    dog_init();    // WWDG initialization

    TickClock  = HAL_GetTick(); // initialize timer for clock state machine
    TickSerial = HAL_GetTick(); // initialize timer for serial state machine
    TickPet    = HAL_GetTick(); // initialize timer for wwdg reset
    TickHeart  = HAL_GetTick(); // initialize timer for heartbeat

    while( 1 ) {

        // if 9 miliseconds have passed
        if( ( HAL_GetTick() - TickClock ) >= 9u ) {
            clock_task();              // do one state from clock state machine
            TickClock = HAL_GetTick(); // update clock state machine timer
        }

        // if 10 miliseconds have passed
        if( ( HAL_GetTick() - TickSerial ) >= 10u ) {
            serial_task();              // do one state from serial state machine
            TickSerial = HAL_GetTick(); // update clock state machine timer
        }

        // if 25 miliseconds have passed
        if( ( HAL_GetTick() - TickPet ) >= 25u ) {
            pet_the_dog();           // reset wwdg counter
            TickPet = HAL_GetTick(); // update wwdg reset timer
        }

        // if 'heartRead' miliseconds have passed
        if( ( HAL_GetTick() - TickHeart ) >= HeartRead ) {
            heart_beat();              // beat the heart
            TickHeart = HAL_GetTick(); // update heartbeat timer
        }
    }

    return 0;
}


/**
  * @brief  Initializes the Window Watchdog down-counter timer used to reset the microcontroller.
  * @param  None.
  * @retval None.
  */
void dog_init( void )
{   
    // twwdg = ( 4096 * 64 ) / fapb = ( 4096 * 64 ) / 48 MHz = 5.46 ms
    WwdgHandler.Instance       = WWDG;
    WwdgHandler.Init.Prescaler = WWDG_PRESCALER_64; // wwdg prescaler
    WwdgHandler.Init.Counter   = 0x49u;             // 73 to 64 = 9 = 49.15 ms until wwdg timeout
    WwdgHandler.Init.Window    = 0x45u;             // 73 to 69 = 4 = 21.84 ms must pass for a valid wwdg reset
    WwdgHandler.Init.EWIMode   = WWDG_EWI_DISABLE;  // disable wwdg early wake up interrupt
    HAL_WWDG_Init( &WwdgHandler );                  // initialize WWDG

    // wwdg counter reset must be done at:
    // wwdg window ( = 21.84 ms ) <= wwdg counter <= wwdg timeout ( = 49.15 ms )
}


/**
  * @brief  Refreshes the Window Watchdog down-counter timer to its previously initialized value.
  * @param  None.
  * @retval None.
  */
void pet_the_dog( void )
{   
    // reset wwdg down-counter
    HAL_WWDG_Refresh( &WwdgHandler );
}


/**
  * @brief  Initializes the Board Led.
  * @param  None.
  * @retval None.
  */
void heart_init( void )
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 }; // GPIO configuration structure

    // enable GPIOA clock
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // configure GPIOA5 (board led) as output
    GPIO_InitStruct.Pin   = GPIO_PIN_HEART;       // heartbeat pin
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;  // pin as push pull output
    GPIO_InitStruct.Pull  = GPIO_NOPULL;          // no pull
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // low speed pin
    HAL_GPIO_Init( GPIOHEART, &GPIO_InitStruct ); // initialize GPIO
}


/**
  * @brief  Toggles the Board Led.
  * @param  None.
  * @retval None.
  */
void heart_beat( void )
{   
    // toggle board led
    HAL_GPIO_TogglePin( GPIOHEART, GPIO_PIN_HEART );
}


/**
  * @brief  Disables the necessary interrupts to stop receiving UART2 serial data.
  * @param  None.
  * @retval None.
  */
void disable_interrupts( void )
{
    // disable UART2 interruption
    HAL_NVIC_DisableIRQ( USART2_LPUART2_IRQn );
}


/**
  * @brief  Enables the necessary interrupts to start receiving UART2 serial data.
  * @param  None.
  * @retval None.
  */
void enable_interrupts( void )
{
    // re-enable UART2 interruption with priority 3
    HAL_NVIC_SetPriority( USART2_LPUART2_IRQn, 3, 0 );
    HAL_NVIC_EnableIRQ( USART2_LPUART2_IRQn );
}