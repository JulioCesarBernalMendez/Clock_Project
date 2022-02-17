#ifndef CLOCK_H
#define CLOCK_H

    // clock state machine definitions
    #define IDLE               0u
    #define MSG_TYPE           1u
    #define PRINT_ALARM_CONF   2u
    #define ALARM_STATE        3u
    #define SET_TIME           4u
    #define SET_DATE           5u
    #define SET_ALARM          6u
    #define SET_HEART          7u
    #define SET_TEMP           8u
    #define PRINT_DATE         9u
    #define PRINT_TIME_TEMP    10u
    #define RING               11u
    #define DEACTIVATE_ALARM   12u

    // clock queue definitions
    #define CLOCK_QUEUE_SIZE   8u

    // board button definition
    #define GPIOBUTTON         GPIOC
    #define GPIO_PIN_BUTTON    GPIO_PIN_13

    // SPI3 definitions
    #define GPIOSPI3           GPIOC
    #define GPIO_PIN_SCK       GPIO_PIN_10
    #define GPIO_PIN_MOSI      GPIO_PIN_12

    // lcd definitions
    #define GPIORS             GPIOC
    #define GPIORST            GPIOC
    #define GPIOCS             GPIOC
    #define GPIO_PIN_RS        GPIO_PIN_4
    #define GPIO_PIN_RST       GPIO_PIN_5
    #define GPIO_PIN_CS        GPIO_PIN_7

    // I2C1 definitions
    #define GPIOI2C1           GPIOB
    #define GPIO_PIN_I2C1_SCK  GPIO_PIN_6
    #define GPIO_PIN_I2C1_SDA  GPIO_PIN_7

    // sensor alert output definitions
    #define GPIOALERT          GPIOB
    #define GPIO_PIN_ALERT     GPIO_PIN_1

    // sensor alert output temperature window structure
    typedef struct {
        uint8_t lower;
        uint8_t upper;
    } TEMP_AlarmWindowTypeDef;

    // clock initialization function prototypes
    void clock_init( void );
    void rtc_init( void );
    void button_init( void );
    void spi_init( void );
    void lcd_init( void );
    void queue_clock_init( void );
    void i2c_init( void );
    void temp_init( void );

    // clock state machine function prototype
    void clock_task( void );
    
    // real time clock function prototypes
    void update_clock_time( RTC_HandleTypeDef *hrtc, SERIAL_MsgTypeDef *message );
    void update_clock_date( RTC_HandleTypeDef *hrtc, SERIAL_MsgTypeDef *message );
    void update_clock_alarm( RTC_HandleTypeDef *hrtc, SERIAL_MsgTypeDef *message );
    void read_clock_calendar( RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *timeStruct, RTC_DateTypeDef *dateStruct );
    void read_clock_alarm( RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *clockAlarmStruct );
    void deactivate_clock_alarm( RTC_HandleTypeDef *hrtc );

    // lcd function prototypes
    void lcd_print_date( LCD_HandleTypeDef *hlcd, RTC_DateTypeDef *dateStruct );
    void lcd_print_time( LCD_HandleTypeDef *hlcd, RTC_TimeTypeDef *timeStruct );
    void lcd_print_temperature( LCD_HandleTypeDef *hlcd, uint8_t temp );
    void lcd_print_alarm_conf( LCD_HandleTypeDef *hlcd, RTC_AlarmTypeDef *clockAlarmStruct, TEMP_AlarmWindowTypeDef *tempAlarmStruct );
    void lcd_print_alarm( LCD_HandleTypeDef *hlcd );
    uint8_t *uint_to_string( uint16_t decimal );
    uint8_t day_of_week( uint8_t day, uint8_t month, uint16_t year );

    // temperature sensor function prototypes
    uint8_t read_temperature( TEMP_HandleTypeDef *tempStruct, uint8_t tempRegister );
    void read_temperature_alarm( TEMP_HandleTypeDef *tempStruct, TEMP_AlarmWindowTypeDef *tempAlarmStruct );
    void deactivate_temperature_alarm( void );
    
#endif