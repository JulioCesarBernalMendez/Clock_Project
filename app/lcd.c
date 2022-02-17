#include <stdint.h>
#include "stm32g0xx.h"
#include "lcd.h"

// lcd.c global variables
static TIM_HandleTypeDef Tim6Handler; // TIMER handler structure

/**
  * @brief  Initializes the LCD to the specified parameters in the LCD_HandleTypeDef.
  * @param  hlcd Pointer to an LCD_HandleTypeDef structure that contains the configuration information for the specified LCD.
  * @retval None.
  */
void MOD_LCD_Init( LCD_HandleTypeDef *hlcd )
{   
    GPIO_InitTypeDef GPIO_InitStruct = { 0 }; // GPIO configuration structure
    uint32_t tick;                            // timer counter for waiting after sending instruction
    
    tim_6_init(); // timer6 initialization

    // configure GPIO as output for RS pin
    GPIO_InitStruct.Pin   = hlcd->RsPin;              // RS pin
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;      // pin as push pull output
    GPIO_InitStruct.Pull  = GPIO_NOPULL;              // no pull
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;     // high speed pin
    HAL_GPIO_Init( hlcd->RsPort, &GPIO_InitStruct );  // initialize GPIO

    // configure GPIO as output for RST pin
    GPIO_InitStruct.Pin   = hlcd->RstPin;             // RST pin
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;      // pin as push pull output
    GPIO_InitStruct.Pull  = GPIO_NOPULL;              // no pull
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;     // high speed pin
    HAL_GPIO_Init( hlcd->RstPort, &GPIO_InitStruct ); // initialize GPIO

    // initialize GPIO as output for CS pin
    GPIO_InitStruct.Pin   = hlcd->CsPin;              // CS pin
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;      // pin as push pull output
    GPIO_InitStruct.Pull  = GPIO_PULLUP;              // no pull
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;     // high speed pin
    HAL_GPIO_Init( hlcd->CsPort, &GPIO_InitStruct );  // initialize GPIO

    // user extra initialization code
    MOD_LCD_MspInit( hlcd );
    
    HAL_GPIO_WritePin( hlcd->CsPort, hlcd->CsPin, GPIO_PIN_SET );     // CS on
    HAL_GPIO_WritePin( hlcd->RstPort, hlcd->RstPin, GPIO_PIN_RESET ); // RST on (external reset)
    
    // wait 2 miliseconds
    tick = HAL_GetTick();
    while( ( HAL_GetTick() - tick ) < 2u ) {
        // no action required
    }
    tick = HAL_GetTick();

    HAL_GPIO_WritePin( hlcd->RstPort, hlcd->RstPin, GPIO_PIN_SET );   // RST off
    
    // wait 40 miliseconds
    tick = HAL_GetTick();
    while( ( HAL_GetTick() - tick ) < 40u ) {
        // no action required
    }
    tick = HAL_GetTick();
    
    MOD_LCD_Command( hlcd, FUNCTION_SET );      // function set
    MOD_LCD_Command( hlcd, FUNCTION_SET );      // function set
    MOD_LCD_Command( hlcd, INTERNAL_OSC_FREQ ); // internal osc frequency
    MOD_LCD_Command( hlcd, CONTRAST );          // contrast
    MOD_LCD_Command( hlcd, POWER_CONTROL );     // power control
    MOD_LCD_Command( hlcd, FOLLOWER_CONTROL );  // follower control
    
    // wait 200 ms
    tick = HAL_GetTick();
    while( ( HAL_GetTick() - tick ) < 200u ) {
        // no action required
    }
    tick = HAL_GetTick();

    MOD_LCD_Command( hlcd, DISPLAY_ON );        // display on
    MOD_LCD_Command( hlcd, ENTRY_MODE );        // entry mode
    MOD_LCD_Command( hlcd, CLEAR_DISPLAY );     // clear display
    
    // wait 2 ms
    tick = HAL_GetTick();
    while( ( HAL_GetTick() - tick ) < 2u ) {
        // no action required
    }
}


/**
  * @brief  Initializes the LCD MSP.
  * @param  hlcd Pointer to an LCD_HandleTypeDef structure that contains the configuration information for the specified LCD.
  * @retval None.
  */
__weak void MOD_LCD_MspInit( LCD_HandleTypeDef *hlcd )
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED( hlcd );

  /* NOTE : This function should not be modified, when the callback is needed,
            the MOD_LCD_MspInit should be implemented in the user file */
}


/**
  * @brief  Sends a command to the LCD by using the specified SPI in the LCD_HandleTypeDef.
  * @param  hlcd Pointer to an LCD_HandleTypeDef structure that contains the configuration information for the specified LCD.
  * @param  cmd  Command to send.
  * @retval None.
  */
void MOD_LCD_Command( LCD_HandleTypeDef *hlcd, uint8_t cmd )
{   
    HAL_GPIO_WritePin( hlcd->CsPort, hlcd->CsPin, GPIO_PIN_RESET ); // CS on
    HAL_GPIO_WritePin( hlcd->RsPort, hlcd->RsPin, GPIO_PIN_RESET ); // instruction register
    HAL_SPI_Transmit( hlcd->SpiLcd, &cmd, sizeof( cmd ), 5000 );    // send instruction
    HAL_GPIO_WritePin( hlcd->CsPort, hlcd->CsPin, GPIO_PIN_SET );   // CS off
    delay_us( 27 );                                                 // wait 27 microseconds
}


/**
  * @brief  Sends a character to the LCD by using the specified SPI in the LCD_HandleTypeDef.
  * @param  hlcd Pointer to an LCD_HandleTypeDef structure that contains the configuration information for the specified LCD.
  * @param  data Character to print.
  * @retval None.
  */
void MOD_LCD_Data( LCD_HandleTypeDef *hlcd, uint8_t data )
{   
    HAL_GPIO_WritePin( hlcd->CsPort, hlcd->CsPin, GPIO_PIN_RESET ); // CS on
    HAL_GPIO_WritePin( hlcd->RsPort, hlcd->RsPin, GPIO_PIN_SET );   // data register
    HAL_SPI_Transmit( hlcd->SpiLcd, &data, sizeof( data ), 5000 );  // send data
    HAL_GPIO_WritePin( hlcd->CsPort, hlcd->CsPin, GPIO_PIN_SET );   // CS off
    delay_us( 27 );                                                 // wait 27 microseconds
}


/**
  * @brief  Sends a string to the LCD by using the specified SPI in the LCD_HandleTypeDef.
  * @param  hlcd Pointer to an LCD_HandleTypeDef structure that contains the configuration information for the specified LCD.
  * @param  str  String to print.
  * @retval None.
  */
void MOD_LCD_String( LCD_HandleTypeDef *hlcd, const uint8_t *str )
{   
    uint8_t CharCtr; // character counter
    
    // loop through all elements in character array
    for( CharCtr = 0u; str[ CharCtr ] != ( uint8_t ) '\0'; CharCtr++ ) {
       MOD_LCD_Data( hlcd, str[ CharCtr ] ); // send current character to LCD
    }
}


/**
  * @brief  Sets the LCD Cursor by using the specified SPI in the LCD_HandleTypeDef.
  * @param  hlcd Pointer to an LCD_HandleTypeDef structure that contains the configuration information for the specified LCD.
  * @param  row  Row position on the LCD (0-1).
  * @param  col  Column position on the LCD (0-15).
  * @retval None.
  */
void MOD_LCD_SetCursor( LCD_HandleTypeDef *hlcd, uint8_t row, uint8_t col )
{   
    uint8_t ddram = 0x80u; // DDRAM address for LCD first row and first column

    // if row and column are valid
    if( ( row >= 0u ) && ( row <= 1u ) && ( col >= 0u ) && ( col <= 15u ) ) {
        
        // get ddram address for user specified cursor position
        if( row == 0u ) {
            ddram += col;
        }
        else {
            ddram = ddram + 0x40u + col;
        }

        // send instruction to LCD
        MOD_LCD_Command( hlcd, ddram );
    }
}


/**
  * @brief  Initializes TIM6 used to get a microsecond time base.
  * @param  None.
  * @retval None.
  */
void tim_6_init( void )
{
    Tim6Handler.Instance         = TIM6;
    Tim6Handler.Init.CounterMode = TIM_COUNTERMODE_UP; // up-counter mode
    Tim6Handler.Init.Prescaler   = 0u;                 // no preescaler
    Tim6Handler.Init.Period      = 47u;                // counter period
    HAL_TIM_Base_Init( &Tim6Handler );                 // initialize TIM6
}


/**
  * @brief  Generates the specified microseconds time delay.
  * @param  us Microseconds delay.
  * @retval None.
  */
void delay_us( uint32_t us )
{   
    uint32_t usctr = 0u; // microseconds counter

    // start TIM6 counting
    HAL_TIM_Base_Start( &Tim6Handler );

    // wait 'us' microseconds
    while( usctr < us ) {
        
        // wait one tim6 period (1 microsecond)
        while( ( TIM6->SR & TIM_SR_UIF ) == 0u ) {
            // no action required
        }

        TIM6->SR &= ~TIM_SR_UIF; // reset timer6 event flag
        usctr++;                 // increase microseconds counter
    }
}