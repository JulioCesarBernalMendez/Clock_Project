#ifndef LCD_H
#define LCD_H

    // lcd definitions
    #define FUNCTION_SET       0x39u
    #define INTERNAL_OSC_FREQ  0x14u
    #define CONTRAST           0x7Au
    #define POWER_CONTROL      0x56u
    #define FOLLOWER_CONTROL   0x6Du
    #define DISPLAY_ON         0x0Cu
    #define ENTRY_MODE         0x06u
    #define CLEAR_DISPLAY      0x01u

    // lcd configuration structure
    typedef struct {
        SPI_HandleTypeDef *SpiLcd;  // SPI handler pointer
        GPIO_TypeDef      *RstPort; // reset port
        uint32_t           RstPin;  // reset pin
        GPIO_TypeDef      *RsPort;  // register selection port
        uint32_t           RsPin;   // register selection pin
        GPIO_TypeDef      *CsPort;  // chip select port
        uint32_t           CsPin;   // chip select pin
    } LCD_HandleTypeDef;

    // lcd function prototypes
    void MOD_LCD_Init( LCD_HandleTypeDef *hlcd );
    void MOD_LCD_MspInit( LCD_HandleTypeDef *hlcd );
    void MOD_LCD_Command( LCD_HandleTypeDef *hlcd, uint8_t cmd );
    void MOD_LCD_Data( LCD_HandleTypeDef *hlcd, uint8_t data );
    void MOD_LCD_String( LCD_HandleTypeDef *hlcd, const uint8_t *str );
    void MOD_LCD_SetCursor( LCD_HandleTypeDef *hlcd, uint8_t row, uint8_t col );
    void tim_6_init( void );
    void delay_us( uint32_t wait );
    
#endif