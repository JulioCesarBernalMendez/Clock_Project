#include <stdint.h>
#include <string.h>
#include "stm32g0xx.h"
#include "queue.h"
#include "app_bsp.h"
#include "lcd.h"
#include "temp.h"
#include "app_clock.h"

// app_clock.c global variables
RTC_HandleTypeDef         RtcHandler;    // RTC handler structure
QUEUE_HandleTypeDef QueueHandler_Clock;  // clock queue handler structure
static RTC_AlarmTypeDef   RTC_AlarmInit; // RTC Alarm initialization structure
static SPI_HandleTypeDef  Spi3Handler;   // SPI handler structure
static LCD_HandleTypeDef  LcdHandler;    // LCD handler structure
static I2C_HandleTypeDef  I2c1Handler;   // I2C  handler structure
static TEMP_HandleTypeDef TempHandler;   // TEMP handler structure
__IO ITStatus ButtonPressed = RESET;     // board button pressed flag
__IO ITStatus AlarmClockSet = RESET;     // clock alarm set flag
__IO ITStatus AlarmTempSet  = RESET;     // temperature alarm set flag
__IO ITStatus RingClock     = RESET;     // clock alarm active flag
__IO ITStatus RingTemp      = RESET;     // temperature alarm active flag


/**
  * @brief  Initializes the necessary resources for the clock state machine.
  * @param  None.
  * @retval None.
  */
void clock_init( void )
{
    rtc_init();         // RTC initialization
    button_init();      // board button initialization
    lcd_init();         // SPI3 and LCD initialization
    queue_clock_init(); // clock queue initialization
    temp_init();        // I2C1 and temperature sensor initialization
}


/**
  * @brief  Initializes the Real Time Clock peripheral.
  * @param  None.
  * @retval None.
  */
void rtc_init( void )
{   
    RTC_DateTypeDef  RTC_DateInit  = { 0 }; // RTC Date configuration structure
    RTC_TimeTypeDef  RTC_TimeInit  = { 0 }; // RTC Time configuration structure

    // configure RTC
    RtcHandler.Instance            = RTC;
    RtcHandler.Init.HourFormat     = RTC_HOURFORMAT_24;        // 24 four hour format
    RtcHandler.Init.AsynchPrediv   = 0x7Fu;                     // Asynchronous predivider = 0x7F
    RtcHandler.Init.SynchPrediv    = 0xFFu;                     // Synchronous  predivider = 0xFF
    RtcHandler.Init.OutPut         = RTC_OUTPUT_DISABLE;       // no signal routed to rtc output
    RtcHandler.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_LOW;  // rtc low polarity output
    RtcHandler.Init.OutPutType     = RTC_OUTPUT_TYPE_PUSHPULL; // rtc output as push pull
    HAL_RTC_Init( &RtcHandler );                               // initialize RTC

    // configure RTC initial clock
    RTC_TimeInit.Hours   = 0u;                                    // 00 hours
    RTC_TimeInit.Minutes = 0u;                                    // 00 mins
    RTC_TimeInit.Seconds = 0u;                                    // 00 seconds
    HAL_RTC_SetTime( &RtcHandler, &RTC_TimeInit, RTC_FORMAT_BIN ); // set RTC time

    // configure RTC initial date
    RTC_DateInit.Month   = RTC_MONTH_JANUARY;                      // january
    RTC_DateInit.Date    = 1u;                                     // 1
    RTC_DateInit.Year    = 0u;                                     // 2000
    RTC_DateInit.WeekDay = RTC_WEEKDAY_SATURDAY;                   // saturday
    HAL_RTC_SetDate( &RtcHandler, &RTC_DateInit, RTC_FORMAT_BIN ); // set RTC date

    // configure RTC initial alarm (disabled)
    RTC_AlarmInit.Alarm                = RTC_ALARM_A;                                       // Alarm A is the alarm to be configured
    RTC_AlarmInit.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;                               // 24 hours format
    RTC_AlarmInit.AlarmMask            = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_SECONDS; // don't care days nor seconds
    RTC_AlarmInit.AlarmSubSecondMask   = RTC_ALARMSUBSECONDMASK_ALL;                        // don't care subseconds
    HAL_RTC_DeactivateAlarm( &RtcHandler, RTC_ALARM_A );                                    // deactivate Alarm A
}


/**
  * @brief  Initializes the Board Button.
  * @param  None.
  * @retval None.
  */
void button_init( void )
{   
    GPIO_InitTypeDef GPIO_InitStruct = { 0 }; // GPIO configuration structure

    // enable GPIOC clock
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // configure GPIOC13 (board button) as rising and falling interupt
    GPIO_InitStruct.Pin   = GPIO_PIN_BUTTON;             // button pin
    GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING_FALLING; // pin as rising and falling interrupt
    GPIO_InitStruct.Pull  = GPIO_NOPULL;                 // no pull
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;         // low speed pin
    HAL_GPIO_Init( GPIOBUTTON, &GPIO_InitStruct );       // initialize GPIO

    // enable EXTI13 interrupt with priority 2
    HAL_NVIC_SetPriority( EXTI4_15_IRQn, 2, 0 );
    HAL_NVIC_EnableIRQ( EXTI4_15_IRQn );
}


/**
  * @brief  Initializes the SPI3 peripheral used to send data to the LCD.
  * @param  None.
  * @retval None.
  */
void spi_init( void )
{
    // configure SPI3 in Master mode
    Spi3Handler.Instance               = SPI3;                     
    Spi3Handler.Init.Mode              = SPI_MODE_MASTER;            // Master mode
    Spi3Handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;   // SPI prescaler = 16
    Spi3Handler.Init.Direction         = SPI_DIRECTION_2LINES;       // direction mode
    Spi3Handler.Init.CLKPhase          = SPI_PHASE_1EDGE;            // clock phase = first edge
    Spi3Handler.Init.CLKPolarity       = SPI_POLARITY_LOW;           // clock polarity = low
    Spi3Handler.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE; // no CRC calculation
    Spi3Handler.Init.CRCPolynomial     = 0x07;                       // CRC polynomial reset value
    Spi3Handler.Init.DataSize          = SPI_DATASIZE_8BIT;          // 8 bits data word
    Spi3Handler.Init.FirstBit          = SPI_FIRSTBIT_MSB;           // first bit is msb
    Spi3Handler.Init.NSS               = SPI_NSS_SOFT;               // slave select handled by software
    Spi3Handler.Init.TIMode            = SPI_TIMODE_DISABLE;         // TI mode disabled
    HAL_SPI_Init( &Spi3Handler );                                    // initialize SPI3
}


/**
  * @brief  Initializes the LCD.
  * @param  None.
  * @retval None.
  */
void lcd_init( void )
{
    spi_init();                        // SPI3 initialization

    // configure LCD
    LcdHandler.SpiLcd  = &Spi3Handler; // SPI used for LCD
    LcdHandler.RsPort  = GPIORS;       // register selection port
    LcdHandler.RsPin   = GPIO_PIN_RS;  // register selection pin
    LcdHandler.RstPort = GPIORST;      // reset port 
    LcdHandler.RstPin  = GPIO_PIN_RST; // reset pin
    LcdHandler.CsPort  = GPIOCS;       // chip select port
    LcdHandler.CsPin   = GPIO_PIN_CS;  // chip select pin
    MOD_LCD_Init( &LcdHandler );       // initialize LCD
}


/**
  * @brief  Initializes the serial data queue used to store CLOCK data.
  * @param  None.
  * @retval None.
  */
void queue_clock_init( void )
{
    static SERIAL_MsgTypeDef   QueueClock[ CLOCK_QUEUE_SIZE ]; // clock data buffer
    
    QueueHandler_Clock.Buffer   = QueueClock;                  // buffer used for clock queue
    QueueHandler_Clock.Elements = CLOCK_QUEUE_SIZE;            // clock queue size
    QueueHandler_Clock.Size     = sizeof( SERIAL_MsgTypeDef ); // queue elements size
    HIL_QUEUE_Init( &QueueHandler_Clock );                     // initialize clock queue
}


/**
  * @brief  Initializes the I2C1 peripheral used to send and receive data to and from the Temperature Sensor.
  * @param  None.
  * @retval None.
  */
void i2c_init( void )
{   
    // configure I2C1
    I2c1Handler.Instance              = I2C1;
    I2c1Handler.Init.Timing           = 0x20303E5Du;             // timing = 100 KHz
    I2c1Handler.Init.OwnAddress1      = 0u;                      // first device own address
    I2c1Handler.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT; // 7 bits addressing mode
    I2c1Handler.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE; // dual addressing mode disabled
    I2c1Handler.Init.OwnAddress2      = 0u;                      // second device own address
    I2c1Handler.Init.OwnAddress2Masks = I2C_OA2_NOMASK;          // no acknowledge mask address second device own address
    I2c1Handler.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE; // general call mode disabled
    I2c1Handler.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;   // slave clock stretching disabled
    HAL_I2C_Init( &I2c1Handler );                                // initialize I2C1
}


/**
  * @brief  Initializes the Temperature Sensor.
  * @param  None.
  * @retval None.
  */
void temp_init( void )
{
    i2c_init();                             // I2C1 initialization
    
    TempHandler.I2cSensor = &I2c1Handler;   // I2C used for temperature sensor
    TempHandler.AlertPort = GPIOALERT;      // temperature alert output port
    TempHandler.AlertPin  = GPIO_PIN_ALERT; // temperature alert output pint
    MOD_TEMP_Init( &TempHandler );          // initialize temperature sensor
}


/**
  * @brief  This function is called by the RTC Alarm A interrupt.
  * @param  hrtc Pointer to an RTC_HandleTypeDef structure that contains the configuration information for the specified RTC.
  * @retval None.
  */
void HAL_RTC_AlarmAEventCallback( RTC_HandleTypeDef *hrtc )
{      
    // if RTC
    if( hrtc->Instance == RTC ) {
        RingClock = SET; // set clock alarm active flag
    }
}


/**
  * @brief  This function is called by the External GPIO (board button) Rising interrupt (button released).
  * @param  GPIO_Pin GPIO Pin which caused the interrupt.
  * @retval None.
  */
void HAL_GPIO_EXTI_Rising_Callback( uint16_t GPIO_Pin )
{   
    // if button released is board button
    if( GPIO_Pin == GPIO_PIN_BUTTON ) {
        ButtonPressed = RESET; // reset board button pressed flag
    }
}


/**
  * @brief  This function is called by the External GPIO (board button) Faling interrupt (button pressed).
  * @param  GPIO_Pin GPIO Pin which caused the interrupt.
  * @retval None.
  */
void HAL_GPIO_EXTI_Falling_Callback( uint16_t GPIO_Pin )
{   
    // if button pressed is board button
    if( GPIO_Pin == GPIO_PIN_BUTTON ) {
        
        ButtonPressed = SET; // set board button pressed flag

        // if clock alarm is active
        if( RingClock == SET ) {
            deactivate_clock_alarm( &RtcHandler ); // deactivate clock alarm
        }

        // if temperature alarm is active
        if( RingTemp == SET ) {
            deactivate_temperature_alarm();        // deactivate temperature alarm
        }
    }
}


/**
  * @brief  Executes the different Clock State Machine processes.
  * @param  None.
  * @retval None.
  */
void clock_task( void )
{   
    static SERIAL_MsgTypeDef MsgToRead;           // serial clock structure (contains new clock parameters)
    static uint32_t  tick1sec;                    // timer counter for printing on LCD (idle state)
    static uint32_t  tickButton;                  // timer counter for printing on LCD (board button pressed)
    static uint32_t  tick1min;                    // timer counter for disabling temperature alarm after one minute
    static uint8_t   c_state = IDLE;              // clock state machine current state
    RTC_DateTypeDef  RTC_DateRead;                // RTC Date  configuration structure
    RTC_TimeTypeDef  RTC_TimeRead;                // RTC Time  configuration structure
    RTC_AlarmTypeDef RTC_AlarmRead;               // RTC Alarm configuration structure
    TEMP_AlarmWindowTypeDef Temp_AlarmWindowRead; // TEMP Alarm Window read structure
    GPIO_PinState    alertRead;                   // sensor temperature alert output read

    switch( c_state ) {
        
        case IDLE:
            
            // if board button is pressed
            if( ButtonPressed == SET ) {
                tickButton = HAL_GetTick();    // update timer counter for printing on LCD (board button pressed)
                c_state    = PRINT_ALARM_CONF; // update clock state
            }
            // if one clock second and one second after the button was pressed have passed
            else if( ( ( HAL_GetTick() - tick1sec ) >= 1000u ) && ( ( HAL_GetTick() - tickButton ) >= 1000u ) ) {
                tick1sec = HAL_GetTick();  // update timer counter for printing on LCD (idle state)
                c_state  = ALARM_STATE;    // update clock state
            }
            else {
                
                // while clock queue is not empty
                while( HIL_QUEUE_IsEmpty( &QueueHandler_Clock ) == QUEUE_NOT_EMPTY ) {
                    
                    // if succesfull clock queue reading (read 1 element from clock queue and store it in MsgToRead)
                    if( HIL_QUEUE_Read( &QueueHandler_Clock, &MsgToRead ) == QUEUE_READ_SUCCESS ) {

                        // if message read is valid
                        if( MsgToRead.msg != NONE ) {
                            c_state = MSG_TYPE;     // update clock state
                            break;                  // break while
                        }
                    }
                }
            }
            break;

        case MSG_TYPE:

            // if message read is a time message
            if( MsgToRead.msg == TIME ) {
                c_state = SET_TIME;  // update clock state
            }
            else if( MsgToRead.msg == DATE ) { // if message read is a date message
                c_state = SET_DATE;  // update clock state
            }
            else if( MsgToRead.msg == ALARM ) { // if message read is an alarm message
                c_state = SET_ALARM; // update clock state
            }
            else if( MsgToRead.msg == HEART ) { // if message read is a heartbeat message
                c_state = SET_HEART; // update clock state
            }
            else if( MsgToRead.msg == TEMP ) {  // if message read is a temperature message
                c_state = SET_TEMP;  // update clock state
            }
            else {
                c_state = IDLE;      // update clock state
            }
            break;

        case SET_TIME:
            update_clock_time( &RtcHandler, &MsgToRead );  // update clock time
            MsgToRead.msg = NONE;                          // set message type to none
            c_state       = ALARM_STATE;                   // update clock state
            break;

        case SET_DATE:
            update_clock_date( &RtcHandler, &MsgToRead );  // update clock date
            MsgToRead.msg = NONE;                          // set message type to none
            c_state       = PRINT_DATE;                    // update clock state
            break;
        
        case SET_ALARM:
            update_clock_alarm( &RtcHandler, &MsgToRead ); // update clock alarm
            MsgToRead.msg = NONE;                          // set message type to none
            c_state       = ALARM_STATE;                   // update clock state
            break;

        case SET_HEART:
            HeartRead = MsgToRead.param1;                  // update heartbeat period
            c_state   = IDLE;                              // update clock state
            break;

        case SET_TEMP:
            MOD_TEMP_SetAlarms( &TempHandler, MsgToRead.param1, MsgToRead.param2 );      // update temperature alarm window
            MsgToRead.msg = NONE;                                                        // set messate type to none
            RingTemp      = RESET;                                                       // reset temperature alarm active flag
            AlarmTempSet  = SET;                                                         // set temperature alarm set flag
            c_state       = ALARM_STATE;                                                 // update clock state
            break;

        case PRINT_ALARM_CONF:
            read_clock_alarm( &RtcHandler, &RTC_AlarmRead );                             // read clock alarm
            read_temperature_alarm( &TempHandler, &Temp_AlarmWindowRead );               // read temperature alarm
            lcd_print_alarm_conf( &LcdHandler, &RTC_AlarmRead, &Temp_AlarmWindowRead );  // print alarm configuration
            ButtonPressed = RESET;                                                       // reset board button pressed flag
            c_state = IDLE;                                                              // update clock state
            break;
        
        case ALARM_STATE:
            alertRead = HAL_GPIO_ReadPin( TempHandler.AlertPort, TempHandler.AlertPin ); // read sensor temperature alert output

            // if temperature alarm is set and sensor temperature alert output is active
            if( ( AlarmTempSet == SET ) && ( alertRead == GPIO_PIN_RESET ) ) {
                
                // if temperature alarm is not active
                if( RingTemp == RESET ) {
                    RingTemp = SET;           // set temperature alarm active flag
                    tick1min = HAL_GetTick(); // update timer counter for disabling temperature alarm after one minute
                }
            }

            // if no alarm active
            if( ( RingClock == RESET ) && ( RingTemp == RESET ) ) {
                c_state = PRINT_DATE; // update clock state
            }
            else { // if alarm active (clock, temperature or both)
                c_state = RING;       // update clock state
            }
            break;

        case PRINT_DATE:
            read_clock_calendar( &RtcHandler, &RTC_TimeRead, &RTC_DateRead ); // read clock time and date
                    
            // print date
            MOD_LCD_SetCursor( &LcdHandler, 0, 1 );
            lcd_print_date( &LcdHandler, &RTC_DateRead );

            c_state = PRINT_TIME_TEMP; // update clock state
            break;

        case PRINT_TIME_TEMP:
            read_clock_calendar( &RtcHandler, &RTC_TimeRead, &RTC_DateRead ); // read clock time and date

            lcd_print_time( &LcdHandler, &RTC_TimeRead ); // print time

            // print temperature
            MOD_LCD_Data( &LcdHandler, ' ' );
            lcd_print_temperature( &LcdHandler, MOD_TEMP_Read( &TempHandler ) );
            
            // if no alarm active
            if( ( RingClock == RESET ) && ( RingTemp == RESET ) ) {

                MOD_LCD_SetCursor( &LcdHandler, 1, 12 );
                MOD_LCD_String( &LcdHandler, ( const uint8_t* ) "  " );

                // if clock alarm is set
                if( AlarmClockSet == SET ) {
                    MOD_LCD_Data( &LcdHandler, 'A' ); // print 'A' to indicate clock alarm is set
                }
                else {
                    MOD_LCD_Data( &LcdHandler, ' ' );
                }

                // if temperature alarm is set
                if( AlarmTempSet == SET ) {
                    MOD_LCD_Data( &LcdHandler, 'T' ); // print 'T' to indicate temperature alarm is set
                }
                else {
                    MOD_LCD_Data( &LcdHandler, ' ' );
                }
            }

            c_state = IDLE; // update clock state
            break;

        case RING:
            read_clock_calendar( &RtcHandler, &RTC_TimeRead, &RTC_DateRead ); // read clock time and date
            read_clock_alarm( &RtcHandler, &RTC_AlarmRead );                  // read clock alarm

            // if { ( clock alarm is active and 1 minute has passed ) or ( temperature alarm is active and 1 minute has passed ) }
            if( ( ( RingClock == SET ) && ( ( RTC_TimeRead.Hours != RTC_AlarmRead.AlarmTime.Hours ) || ( RTC_TimeRead.Minutes != RTC_AlarmRead.AlarmTime.Minutes ) ) ) ||
                ( ( RingTemp  == SET ) && ( ( HAL_GetTick() - tick1min ) >= 60000 ) )  ) {
                    
                c_state = DEACTIVATE_ALARM;     // update clock state
            }
            else { // ... else
                lcd_print_alarm( &LcdHandler ); // print alarm
                c_state = PRINT_DATE;           // update clock state
            }
            break;

        case DEACTIVATE_ALARM:

            // if clock alarm is active
            if( RingClock == SET ) {
                deactivate_clock_alarm( &RtcHandler ); // deactivate clock alarm
            }

            // if temperature alarm is active
            if( RingTemp == SET ) {
                deactivate_temperature_alarm();        // deactivate temperature alarm
            }

            c_state = PRINT_DATE; // update clock state
            break;

        default:
            c_state = IDLE; // update clock state
            break;

    } // end switch
}


/**
  * @brief  Updates the RTC clock time.
  * @param  hrtc    Pointer to an RTC_HandleTypeDef structure that contains the configuration information for the specified RTC.
  * @param  message Pointer to a SERIAL_MsgTypeDef structure that contains the clock parameters.
  * @retval None.
  */
void update_clock_time( RTC_HandleTypeDef *hrtc, SERIAL_MsgTypeDef *message )
{   
    RTC_TimeTypeDef RTC_TimeConf = { 0 }; // RTC Time configuration structure

    // reconfigure RTC time
    RTC_TimeConf.Hours   = message->param1;
    RTC_TimeConf.Minutes = message->param2;
    RTC_TimeConf.Seconds = message->param3;
    HAL_RTC_SetTime( hrtc, &RTC_TimeConf, RTC_FORMAT_BIN );
}


/**
  * @brief  Updates the RTC date.
  * @param  hrtc    Pointer to an RTC_HandleTypeDef structure that contains the configuration information for the specified RTC.
  * @param  message Pointer to a SERIAL_MsgTypeDef structure that contains the date parameters.
  * @retval None.
  */
void update_clock_date( RTC_HandleTypeDef *hrtc, SERIAL_MsgTypeDef *message )
{   
    RTC_DateTypeDef RTC_DateConf = { 0 }; // RTC Date configuration structure

    // reconfigure RTC date
    RTC_DateConf.Date    = message->param1;
    RTC_DateConf.Month   = message->param2;
    RTC_DateConf.Year    = message->param3 % 2000u;
    RTC_DateConf.WeekDay = day_of_week( message->param1, message->param2, message->param3 );
    HAL_RTC_SetDate( hrtc, &RTC_DateConf, RTC_FORMAT_BIN );
}


/**
  * @brief  Updates the RTC Alarm.
  * @param  hrtc    Pointer to an RTC_HandleTypeDef structure that contains the configuration information for the specified RTC.
  * @param  message Pointer to a SERIAL_MsgTypeDef structure that contains the alarm parameters.
  * @retval None.
  */
void update_clock_alarm( RTC_HandleTypeDef *hrtc, SERIAL_MsgTypeDef *message )
{   
    RingClock = RESET; // reset clock alarm active flag

    // reconfigure RTC alarm
    RTC_AlarmInit.AlarmTime.Hours   = message->param1;
    RTC_AlarmInit.AlarmTime.Minutes = message->param2;
    HAL_RTC_SetAlarm_IT( hrtc, &RTC_AlarmInit, RTC_FORMAT_BIN );

    AlarmClockSet  = SET; // set clock alarm set flag
}


/**
  * @brief  Reads the RTC Clock and Date.
  * @param  hrtc       Pointer to an RTC_HandleTypeDef structure that contains the configuration information for the specified RTC.
  * @param  timeStruct Pointer to an RTC_TimeTypeDef structure that will be used to store the read RTC clock.
  * @param  dateStruct Pointer to an RTC_DateTypeDef structure that will be used to store the read RTC date.
  * @retval None.
  */
void read_clock_calendar( RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *timeStruct, RTC_DateTypeDef *dateStruct )
{   
    HAL_RTC_GetTime( hrtc, timeStruct, RTC_FORMAT_BIN );
    HAL_RTC_GetDate( hrtc, dateStruct, RTC_FORMAT_BIN );
}


/**
  * @brief  Reads the RTC Alarm.
  * @param  hrtc             Pointer to an RTC_HandleTypeDef structure that contains the configuration information for the specified RTC.
  * @param  clockAlarmStruct Pointer to an RTC_AlarmTypeDef structure that will be used to store the read RTC alarm.
  * @retval None.
  */
void read_clock_alarm( RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *clockAlarmStruct )
{   
    HAL_RTC_GetAlarm( hrtc, clockAlarmStruct, RTC_ALARM_A, RTC_FORMAT_BIN );
}


/**
  * @brief  Deactivates the RTC Alarm.
  * @param  hrtc Pointer to an RTC_HandleTypeDef structure that contains the configuration information for the specified RTC.
  * @retval None.
  */
void deactivate_clock_alarm( RTC_HandleTypeDef *hrtc )
{
    HAL_RTC_DeactivateAlarm( hrtc, RTC_ALARM_A ); // deactivate alarm A
    AlarmClockSet = RESET;                        // reset clock alarm set flag
    RingClock     = RESET;                        // reset clocl alarm active flag
}


/**
  * @brief  Prints the date on the LCD.
  * @param  hlcd       Pointer to an LCD_HandleTypeDef structure that contains the configuration information for the specified LCD.
  * @param  dateStruct Pointer to an RTC_DateTypeDef structure that contains the RTC date.
  * @retval None.
  */
void lcd_print_date( LCD_HandleTypeDef *hlcd, RTC_DateTypeDef *dateStruct )
{   
    // print month
    switch( dateStruct->Month ) {
        case 1:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "JAN" );
            break;

        case 2:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "FEB" );
            break;

        case 3:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "MAR" );
            break;

        case 4:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "APR" );
            break;

        case 5:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "MAY" );
            break;

        case 6:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "JUN" );
            break;

        case 7:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "JUL" );
            break;

        case 8:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "AUG" );
            break;

        case 9:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "SEP" );
            break;

        case 10:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "OCT" );
            break;

        case 11:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "NOV" );
            break;

        case 12:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "DEC" );
            break;

        default:
            // no action required
            break;
    }
    
    MOD_LCD_Data( hlcd, ',' ); // print comma
    
    // print day of the month
    if( dateStruct->Date < 10 ) {
        MOD_LCD_Data( hlcd, '0' );
    }
    MOD_LCD_String( hlcd, uint_to_string( dateStruct->Date ) );

    MOD_LCD_Data( hlcd, ' ' ); // print space

    // print year
    MOD_LCD_String( hlcd, uint_to_string( 2000 + dateStruct->Year ) );

    MOD_LCD_Data( hlcd, ' ' ); // print space
    

    // print day of the week
    switch( dateStruct->WeekDay ) {
        case 1:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "Mo" );
            break;

        case 2:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "Tu" );
            break;

        case 3:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "We" );
            break;

        case 4:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "Th" );
            break;

        case 5:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "Fr" );
            break;

        case 6:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "Sa" );
            break;

        case 7:
            MOD_LCD_String( hlcd, ( const uint8_t* ) "Su" );
            break;

        default:
            // no action required
            break;
    }
}


/**
  * @brief  Prints the clock time on the LCD.
  * @param  hlcd       Pointer to an LCD_HandleTypeDef structure that contains the configuration information for the specified LCD.
  * @param  timeStruct Pointer to an RTC_TimeTypeDef structure that contains the RTC clock.
  * @retval None.
  */
void lcd_print_time( LCD_HandleTypeDef *hlcd, RTC_TimeTypeDef *timeStruct )
{   
    // if no alarm active
    if( ( RingClock == RESET ) && ( RingTemp == RESET ) ) {
        MOD_LCD_SetCursor( hlcd, 1, 0 );
    }
    else { // if alarm active (clock, temperature or both)
        MOD_LCD_SetCursor( hlcd, 1, 4 );
    }
    
    // print hours
    if( timeStruct->Hours < 10 ) {
        MOD_LCD_Data( hlcd, '0' );
    }
    MOD_LCD_String( hlcd, uint_to_string( timeStruct->Hours ) );
    
    MOD_LCD_Data( hlcd, ':' ); // print colon
    
    // print minutes
    if( timeStruct->Minutes < 10 ) {
        MOD_LCD_Data( hlcd, '0' );
    }
    MOD_LCD_String( hlcd, uint_to_string( timeStruct->Minutes ) );
    
    // if no alarm active
    if( ( RingClock == RESET ) && ( RingTemp == RESET ) ) {
        
        MOD_LCD_Data( hlcd, ':' ); // print colon
    
        // print seconds
        if( timeStruct->Seconds < 10 ) {
            MOD_LCD_Data( hlcd, '0' );
        }
        MOD_LCD_String( hlcd, uint_to_string( timeStruct->Seconds ) );
    }
}


/**
  * @brief  Prints the ambient temperature on the LCD.
  * @param  hlcd Pointer to an LCD_HandleTypeDef structure that contains the configuration information for the specified LCD.
  * @param  temp Ambient Temperature to be printed.
  * @retval None.
  */
void lcd_print_temperature( LCD_HandleTypeDef *hlcd, uint8_t temp )
{   
    // if no alarm active
    if( ( RingClock == RESET ) && ( RingTemp == RESET ) ) {
        MOD_LCD_SetCursor( hlcd, 1, 9 );
    }
    else { // if alarm active (clock, temperature or both)
        MOD_LCD_SetCursor( hlcd, 1, 10 );
    }

    // print temperature
    if( temp < 10u ) {
        MOD_LCD_Data( hlcd, '0' );
    }
    MOD_LCD_String( hlcd, uint_to_string( temp ) );
    
    // if no alarm active
    if( ( RingClock == RESET ) && ( RingTemp == RESET ) ) {
        MOD_LCD_SetCursor( hlcd, 1, 11 );
    }
    else { // if alarm active (clock, temperature or both)
        MOD_LCD_SetCursor( hlcd, 1, 12 );    
    }

    MOD_LCD_Data( hlcd, 'C' ); // print 'C' indicating Celsius temperature
}


/**
  * @brief  Prints both clock and temperature alarm configuration on the LCD.
  * @param  hlcd             Pointer to an LCD_HandleTypeDef structure that contains the configuration information for the specified LCD.
  * @param  clockAlarmStruct Pointer to an RTC_AlarmTypeDef structure that contains the RTC alarm.
  * @param  tempAlarmStruct  Pointer to an TEMP_AlarmWindowTypeDef structure that contains the Temperature Sensor window limits.
  * @retval None.
  */
void lcd_print_alarm_conf( LCD_HandleTypeDef *hlcd, RTC_AlarmTypeDef *clockAlarmStruct, TEMP_AlarmWindowTypeDef *tempAlarmStruct )
{   
    // if temperature alarm is set
    if( AlarmTempSet == SET ) {
        MOD_LCD_SetCursor( hlcd, 1, 0 );
        MOD_LCD_Data( hlcd, 'T' );  // print 'T' indicating temperature alarm is set

        MOD_LCD_SetCursor( hlcd, 1, 9 );

        // print lower temperature
        if( tempAlarmStruct->lower < 10u ) {
            MOD_LCD_Data( hlcd, '0' );
        }
        MOD_LCD_String( hlcd, uint_to_string( tempAlarmStruct->lower ) );

        MOD_LCD_Data( hlcd, '-' ); // print hyphen

        // print upper temperature
        if( tempAlarmStruct->upper < 10u ) {
            MOD_LCD_Data( hlcd, '0' );
        }
        MOD_LCD_String( hlcd, uint_to_string( tempAlarmStruct->upper ) );
        
        MOD_LCD_String( hlcd, ( const uint8_t* ) "C " ); // print 'C' indicating Celsius temperature
    }
    else { // if temperature alarm is no set
        
        // clear first character on second row
        MOD_LCD_SetCursor( hlcd, 1, 0 );
        MOD_LCD_Data( hlcd, ' ' );

        // clear last 7 characters on second row
        MOD_LCD_SetCursor( hlcd, 1, 9 );
        MOD_LCD_String( hlcd, ( const uint8_t* ) "       " );
    }

    // if clock alarm is set
    if( AlarmClockSet == SET ) {
        MOD_LCD_SetCursor( hlcd, 1, 1 );
        MOD_LCD_String( hlcd, ( const uint8_t* ) "A " ); // print 'A' indicating clock alarm is set
        
        // print hours
        if( clockAlarmStruct->AlarmTime.Hours < 10u ) {
            MOD_LCD_Data( hlcd, '0' );
        }
        MOD_LCD_String( hlcd, uint_to_string( clockAlarmStruct->AlarmTime.Hours ) );

        MOD_LCD_Data( hlcd, ':' ); // print colon

        // print minutes 
        if( clockAlarmStruct->AlarmTime.Minutes < 10u ) {
            MOD_LCD_Data( hlcd, '0' );
        }
        MOD_LCD_String( hlcd, uint_to_string( clockAlarmStruct->AlarmTime.Minutes ) );
        
        MOD_LCD_Data( hlcd, ' ' );
    }
    else { // if clock alarm is not set

        // clear 8 characters on second row starting from second column
        MOD_LCD_SetCursor( hlcd, 1, 1 );
        MOD_LCD_String( hlcd, ( const uint8_t* ) "        " );
    }

    // if no alarm is set
    if( ( AlarmClockSet == RESET ) && ( AlarmTempSet == RESET ) ) {
        MOD_LCD_SetCursor( hlcd, 1, 0 );
        MOD_LCD_String( hlcd, ( const uint8_t* ) "NO ALARMS CONFIG" );
    }
}


/**
  * @brief  Prints asterisks on the LCD indicating an alarm (clock, temperature or both) is active.
  * @param  hlcd Pointer to an LCD_HandleTypeDef structure that contains the configuration information for the specified LCD.
  * @retval None.
  */
void lcd_print_alarm( LCD_HandleTypeDef *hlcd )
{
    static uint8_t toggle = 0u; // variable used to print or not on LCD

    // if toggle is true
    if( toggle != 0u ) {
        
        // print asterisks on LCD left side
        MOD_LCD_SetCursor( hlcd, 1, 0 );
        MOD_LCD_String( hlcd, ( const uint8_t* ) "*** " );

        // print asterisks on LCD right side
        MOD_LCD_SetCursor( hlcd, 1, 13 );
        MOD_LCD_String( hlcd, ( const uint8_t* ) "***" );

        toggle = 0u; // "invert" toggle variable
    }
    else { // if toggle is false

        // clear asterisks on LCD left side
        MOD_LCD_SetCursor( hlcd, 1, 0 );
        MOD_LCD_String( hlcd, ( const uint8_t* ) "    " );

        // clear asterisks on LCD right side
        MOD_LCD_SetCursor( hlcd, 1, 12 );
        MOD_LCD_String( hlcd, ( const uint8_t* ) "    " );

        toggle = 1u; // "invert" toggle variable
    }
}


/**
  * @brief  Reads the Temperature Window Limits from the TLOWER or TUPPER registers from the temperature sensor.
  * @param  tempStruct   Pointer to a TEMP_HandleTypeDef structure that contains the configuration information for the specified Temperature Sensor.
  * @param  tempRegister Temperature register address (TUPPER or TLOWER).
  * @retval Temperature read in decimal format.
  */
uint8_t read_temperature( TEMP_HandleTypeDef *tempStruct, uint8_t tempRegister )
{
    uint8_t readBuffer[ 2 ];  // i2c read buffer
    uint8_t temperature = 0u; // temperature read from sensor

    // if temperature sensor register is TLOWER or TUPPER register
    if( ( tempRegister == TLOWER_REGISTER ) || ( tempRegister == TUPPER_REGISTER ) ) {
        
        // read from TLOWER or TUPPER register
        HAL_I2C_Master_Transmit( tempStruct->I2cSensor, SENSOR_ADRESS, &tempRegister, 1, 5000 );
        HAL_I2C_Master_Receive( tempStruct->I2cSensor, SENSOR_ADRESS, readBuffer, 2, 5000 );

        // if temperature is negative
        if( ( readBuffer[ 0 ] & ( 1u << 4u ) ) == 0x10u ) {

            // convert temperature data to decimal value
            readBuffer[ 0 ] *= 16u;
            temperature = 256u - readBuffer[ 0 ] + readBuffer[ 1 ];
            temperature |= ( 1u << 7u ); // add minus sign
        }
        else { // if temperature is positive
            
            // convert temperature data to decimal value
            readBuffer[ 0 ] *= 16u;
            temperature = readBuffer[ 0 ] + ( readBuffer[ 1 ] * 0.0625 );
        }
    }
    
    return temperature;
}


/**
  * @brief  Reads the Temperature Window Limits from both TLOWER or TUPPER registers from the temperature sensor and store it
  *         into a TEMP_AlarmWindowTypeDef structure.
  * @param  tempStruct      Pointer to a TEMP_HandleTypeDef structure that contains the configuration information for the specified
  *                         Temperature Sensor.
  * @param  tempAlarmStruct Pointer to a TEMP_AlarmWindowTypeDef structure used to store the values read from both TLOWER and TUPPER
  *                         registers.
  * @retval None.
  */
void read_temperature_alarm( TEMP_HandleTypeDef *tempStruct, TEMP_AlarmWindowTypeDef *tempAlarmStruct )
{
    tempAlarmStruct->lower = read_temperature( tempStruct, TLOWER_REGISTER ); // read lower temperature from temperature sensor
    tempAlarmStruct->upper = read_temperature( tempStruct, TUPPER_REGISTER ); // read upper temperature from temperature sensor
}


/**
  * @brief  Deactivates the Temperature Sensor Alarm by resetting the correspoding alarm flag values.
  * @param  None.
  * @retval None.
  */
void deactivate_temperature_alarm( void )
{
    AlarmTempSet = RESET; // reset temperature alarm set flag
    RingTemp     = RESET; // reset temperature alarm active flag
}


/**
  * @brief  Converts an unsigned integer to its equivalent character string.
  * @param  decimal Unsiged Integer to be converted.
  * @retval Pointer to the converted string.
  */
uint8_t *uint_to_string( uint16_t decimal )
{
    static uint8_t str[ 5 ]; // return string
    uint16_t temp;           // temporal value
    uint8_t CharCtr = 0u;    // character counter
    uint8_t first   = 0u;    // first character index
    uint8_t last;            // last  character index

    temp = decimal;

    // if temporal value is different from zero
    if( temp != 0u ) {

        /* while temp value is greater than zero ...
        ... convert temp (=decimal) to string (reverse order) */
        while( temp > 0u ) {
            str[ CharCtr ] = ( temp % 10u ) + ( uint8_t ) '0'; // convert last digit from temp value into a character and store it into character array
            temp /= 10; // divide temp over 10
            CharCtr++;  // increase character counter
        }

        last = CharCtr - 1u; // get last character index

        /* while first character index is less than last character index ...
        ... invert string order (decimal reversed string) */
        while( first < last ) {

            // swap 'first' and 'last' characters
            temp = str[ first ]; 
            str[ first ] = str[ last ];
            str[ last  ] = temp;

            // update 'first' and 'last' indexes for next character swap
            first++;
            last--;
        }

        str[ CharCtr ] = ( uint8_t ) '\0'; // add terminating null character
    }
    else{ // if decimal value is zero
        
        // write "0" into str
        str[ 0 ] = ( uint8_t ) '0';
        str[ 1 ] = ( uint8_t ) '\0';
    }

    return str;
}


/**
  * @brief  Returns the day of the week for the specified date.
  * @param  day   Day of the month.
  * @param  month Month in the year.
  * @param  year  Year.
  * @retval Day of the week.
  *         Monday  = 1.
  *         Tuesday = 2.
  *         ...
  *         Sunday  = 7.
  */
uint8_t day_of_week( uint8_t day, uint8_t month, uint16_t year )
{
    uint8_t  DayWeek;           // day of the week
    uint8_t  MonthCpy = month;  // month
    uint16_t YearCpy  = year;   // year
    uint8_t  CentYear;          // year in the century
    uint8_t  ZeroBasedCent;     // zero-based century

    // execute zeller's congruence algorithm (basic modification)
    if( month < 3u ) {
        MonthCpy = month + 12u;
        YearCpy  = year  - 1u;
    }

    ZeroBasedCent = ( uint8_t ) ( YearCpy / 100u );
    CentYear      = ( uint8_t ) ( YearCpy % 100u );

    DayWeek  = ( day + ( 13u * ( MonthCpy + 1u ) / 5u ) + CentYear + ( CentYear / 4u )
               + ( ZeroBasedCent / 4u ) + ( 5u * ZeroBasedCent ) ) % 7u;
    
    // change day order so Monday = 1, Tuesday = 2, ... , Sunday = 7
    if( DayWeek <= 1u ) {
        DayWeek += 6u;
    }
    else {
        DayWeek -= 1u;
    }   

    return DayWeek;
}