#include <stdint.h>
#include <string.h>
#include "stm32g0xx.h"
#include "queue.h"
#include "app_bsp.h"
#include "app_serial.h"

// app_serial.c global variables
UART_HandleTypeDef Uart2Handler; // UART handler structure
static uint8_t     UartRxByte;   // UART reception byte
static QUEUE_HandleTypeDef QueueHandler_Serial;                // serial data queue handler structure
static const uint8_t WriteErrorMessage[] = "-WRITE ERROR\r\n"; // writing error serial message

/**
  * @brief  Initializes the necessary resources for the serial state machine.
  * @param  None.
  * @retval None.
  */
void serial_init( void )
{
    uart_2_init();       // UART2 initialization
    queue_serial_init(); // serial queue initialization
    
    // enable 1 byte reception interrupt for UART2
    HAL_UART_Receive_IT( &Uart2Handler, &UartRxByte, 1 );
}


/**
  * @brief  Initializes the UART2 peripheral.
  * @param  None.
  * @retval None.
  */
void uart_2_init( void )
{   
    // configure UART2 in TX-RX mode
    Uart2Handler.Instance            = USART2;               
    Uart2Handler.Init.ClockPrescaler = UART_PRESCALER_DIV1;  // UART prescaler = 1
    Uart2Handler.Init.BaudRate       = 115200;               // 115200 bauds
    Uart2Handler.Init.WordLength     = UART_WORDLENGTH_8B;   // 8 bits data word
    Uart2Handler.Init.StopBits       = UART_STOPBITS_1;      // 1 stop bit
    Uart2Handler.Init.Parity         = UART_PARITY_NONE;     // no parity bit
    Uart2Handler.Init.HwFlowCtl      = UART_HWCONTROL_NONE;  // no hardware control
    Uart2Handler.Init.Mode           = UART_MODE_TX_RX;      // TX and RX mode
    Uart2Handler.Init.OverSampling   = UART_OVERSAMPLING_16; // oversampling by 16
    HAL_UART_Init( &Uart2Handler );                          // initialize UART2
}


/**
  * @brief  Initializes the serial data queue used to store UART data.
  * @param  None.
  * @retval None.
  */
void queue_serial_init( void )
{   
    static uint8_t QueueSerial[ SERIAL_QUEUE_SIZE ];  // serial data buffer

    // configure serial queue
    QueueHandler_Serial.Buffer   = QueueSerial;       // buffer used for serial queue
    QueueHandler_Serial.Elements = SERIAL_QUEUE_SIZE; // serial queue size
    QueueHandler_Serial.Size     = sizeof( uint8_t ); // queue elements size
    HIL_QUEUE_Init( &QueueHandler_Serial );           // initialize serial queue
}


/**
  * @brief  This function is called by the UART one byte reception interrupt.
  * @param  huart Pointer to a UART_HandleTypeDef structure that contains the configuration information for the specified UART.
  * @retval None.
  */
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
{   
    // if USART2
    if( huart->Instance == USART2 ) {

        // if unsuccesfull serial queue writing (write received byte into serial queue)
        if( HIL_QUEUE_Write( &QueueHandler_Serial, &UartRxByte ) == QUEUE_WRITE_ERROR ) {
            // transmit write error message over UART2
            HAL_UART_Transmit( &Uart2Handler, WriteErrorMessage, sizeof( WriteErrorMessage ), 5000 );
        }
    
        // re-enable UART2 1 byte reception interrupt
        HAL_UART_Receive_IT( &Uart2Handler, &UartRxByte, 1 );
    }
}


/**
  * @brief  Executes the different Serial State Machine processes.
  * @param  None.
  * @retval None.
  */
void serial_task( void )
{   
    static SERIAL_MsgTypeDef MsgToSend;              // serial message structure (contains new clock parameters)
    static uint8_t  SerialMsgBuffer[ MESSAGE_SIZE ]; // serial message to process (extracted from serial queue)
    static uint8_t  index = 0;                       // serial message character index
    static uint8_t *param1Ptr;                       // serial message first parameter pointer
    static uint8_t *param2Ptr;                       // serial message second parameter pointer
    static uint8_t *param3Ptr;                       // serial message third parameter
    static uint8_t  MsgType;                         // serial message type
    static uint8_t  s_state = IDLE;                  // serial state machine current state
    const uint8_t   OkMessage[]    = "-OK\r\n";      // ok serial message
    const uint8_t   ErrorMessage[] = "-ERROR\r\n";   // error serial message
    uint8_t         byteRead;                        // byte read from serial queue

    switch( s_state ) {

        case IDLE:

            // while serial queue is not empty
            while( HIL_QUEUE_IsEmpty( &QueueHandler_Serial ) == QUEUE_NOT_EMPTY ) {

                disable_interrupts(); // disable interrupts to avoid receiving serial data while reading from serial queue

                // read 1 byte from serial queue, store it in byteRead and check if read was succesfull
                if( HIL_QUEUE_Read( &QueueHandler_Serial, &byteRead ) == QUEUE_READ_SUCCESS ) {

                    // if index is less than maximum message size
                    if( index < MESSAGE_SIZE ) {
                        SerialMsgBuffer[ index ] = byteRead; // store byte read from serial queue into SerialMsgBuffer
                        index++;                             // increase index
                    }

                    // ... otherwise ignore byte read
                }
                    
                enable_interrupts(); // re-enable interrupts to allow serial data receiving

                // if byte read is enter key
                if( byteRead == ( uint8_t ) '\r' ) {
                    index   = 0;              // reset SerialMsgBuffer character index
                    s_state = PROCESS_BUFFER; // update serial state
                    break;                    // break while
                }
            }
            break;
        
        case PROCESS_BUFFER:
            // validate serial message
            MsgType = validate_message( SerialMsgBuffer, &param1Ptr, &param2Ptr, &param3Ptr );

            // if serial message type is valid
            if( MsgType != NONE ) {
                s_state = EVAL_PARAMETERS; // update serial state
            }
            else { // if serial message type is not valid
                s_state = SERIAL_ERROR;    // update serial state
            }
            break;
        
        case EVAL_PARAMETERS:
            // extract parameters from serial message and store it into the serial message structure
            MsgToSend.param1 = string_to_uint( param1Ptr );
            MsgToSend.param2 = string_to_uint( param2Ptr );
            MsgToSend.param3 = string_to_uint( param3Ptr );

            // if parameters are valid
            if( validate_msg_parameters( &MsgToSend, MsgType ) == VALID_PARAMETERS ) {
                s_state = SERIAL_OK;    // update serial state
            }
            else { // if parameters are not valid
                s_state = SERIAL_ERROR; // update serial state
            }
            break;

        case SERIAL_ERROR:
            // transmit error message over UART2
            HAL_UART_Transmit( &Uart2Handler, ErrorMessage, sizeof( ErrorMessage ), 5000 );
            s_state = IDLE;
            break;

        case SERIAL_OK:
            // update message type received in serial message structure
            MsgToSend.msg = MsgType;

            // if succesfull serial queue writing (write serial message structure into clock queue)
            if( HIL_QUEUE_Write( &QueueHandler_Clock, &MsgToSend ) == QUEUE_WRITE_SUCCESS ) {
                // transmit ok message over UART2
                HAL_UART_Transmit( &Uart2Handler, OkMessage, sizeof( OkMessage ), 5000 );
            }
            else {
                // transmit write error message over UART2
                HAL_UART_Transmit( &Uart2Handler, WriteErrorMessage, sizeof( WriteErrorMessage ), 5000 );
            }

            s_state = IDLE; // update serial state
            break;

        default:
            s_state = IDLE; // update serial state
            break;
            
    } // end switch
}


/**
  * @brief  Validates the message type received over UART2:
  *         "AT+TIME=<hh>,<mm>,<ss>\r"
  *         "AT+DATE=<dd>,<mm>,<aaaa>\r"
  *         "AT+ALARM=<hh>,<mm>\r"
  *         "AT+HEARTBEAT=<xxxx>\r"
  *         "AT+TEMP=<lower>,<upper>\r"
  * @param  SerialMsg Pointer to the received serial message.
  * @param  param1Ptr Pointer to the Pointer used to store the message parameter 1.
  * @param  param2Ptr Pointer to the Pointer used to store the message parameter 2.
  * @param  param3Ptr Pointer to the Pointer used to store the message parameter 3.
  * @retval Message Type received.
  */
uint8_t validate_message( uint8_t *SerialMsg, uint8_t **param1Ptr, uint8_t **param2Ptr, uint8_t **param3Ptr )
{   
    uint8_t MsgType = NONE; // return message type
    char *tokenPtr;

    // tokenize serial message (equal sign is the token)
    tokenPtr = strtok( ( char* ) SerialMsg, "=" );

    // if tokenPtr points to "AT+TIME" or "AT+DATE"
    if( ( strcmp( tokenPtr, "AT+TIME" ) == 0 ) || ( strcmp( tokenPtr, "AT+DATE" ) == 0 ) ) {

        // get parameter pointers
        *param1Ptr = ( uint8_t* ) strtok( NULL, "," );  // pointer to hours or day
        *param2Ptr = ( uint8_t* ) strtok( NULL, "," );  // pointer to minutes or month
        *param3Ptr = ( uint8_t* ) strtok( NULL, "\r" ); // pointer to seconds or year
        
        // if tokenPtr points to "AT+TIME" and parameter pointers point to "something"
        if( ( strcmp( tokenPtr, "AT+TIME" ) == 0 ) && ( *param1Ptr != NULL ) && ( *param2Ptr != NULL ) && ( *param3Ptr != NULL ) ) {
            MsgType = TIME; // time message
        }
        // if tokenPtr points to "AT+DATE" and parameter pointers point to "something"
        else if( ( strcmp( tokenPtr, "AT+DATE" ) == 0 ) && ( *param1Ptr != NULL ) && ( *param2Ptr != NULL ) && ( *param3Ptr != NULL ) ) {
            MsgType = DATE; // date message
        }
        else {
            // no action required
        }
    }
    else if( strcmp( tokenPtr, "AT+ALARM" ) == 0 ) { // if tokenPtr points to "AT+ALARM"

        // get parameter pointers
        *param1Ptr = ( uint8_t* ) strtok( NULL, "," );  // pointer to hours
        *param2Ptr = ( uint8_t* ) strtok( NULL, "\r" ); // pointer to minutes

        // if parameter pointers point to "something"
        if( ( *param1Ptr != NULL ) && ( *param2Ptr != NULL ) ) {
            MsgType = ALARM; // alarm message
        }
    }
    else if( strcmp( tokenPtr, "AT+HEARTBEAT" ) == 0 ) { // if tokenPtr points to "AT+HEARTBEAT"

        // get paremeter pointer
        *param1Ptr = ( uint8_t* ) strtok( NULL, "\r" ); // pointer to miliseconds

        // if parameter pointer points to "something"
        if( *param1Ptr != NULL ) {
            MsgType = HEART; // heartbeat message
        }
    }
    else if( strcmp( tokenPtr, "AT+TEMP" ) == 0 ) { // if tokenPtr points to "AT+TEMP"

        *param1Ptr = ( uint8_t* ) strtok( NULL, "," );  // pointer to lower temperature
        *param2Ptr = ( uint8_t* ) strtok( NULL, "\r" ); // pointer to upper temperature

        if( ( *param1Ptr != NULL ) && ( *param2Ptr != NULL ) ) {
            MsgType = TEMP; // temperature message
        }
    }
    else {
        // no action required
    }
    
    return MsgType; // return message type
}


/**
  * @brief  Validates the serial message parameters according to the received message type.
  * @param  MsgStruct Pointer to a SERIAL_MsgTypeDef structure that will be used to store the message parameters if valid.
  * @param  MsgType   Serial Message Type.
  * @retval Parameters Status.
  */
uint8_t validate_msg_parameters( SERIAL_MsgTypeDef *MsgStruct, uint8_t MsgType )
{   
    uint8_t ParamStatus = INVALID_PARAMETERS; // return parameter status

    switch( MsgType ) {

        // time message
        case TIME:
            // if hours, minutes and seconds are valid
            if( ( MsgStruct->param1 <= 23u ) && ( MsgStruct->param2 <= 59u ) && ( MsgStruct->param3 <= 59u ) ) {
                ParamStatus = VALID_PARAMETERS; // valid parameters
            }
            break;
        
        // date message
        case DATE:
            // if day, month and year are valid
            if( ( MsgStruct->param1 <= 31u ) && ( MsgStruct->param2 <= 12u ) && ( MsgStruct->param3 >= 2000u ) && ( MsgStruct->param3 <= 2099u ) ) {
                
                // if date is valid
                if( validate_date( MsgStruct->param1, MsgStruct->param2, MsgStruct->param3 ) == VALID_DATE ) {
                    ParamStatus = VALID_PARAMETERS; // valid parameters
                }
            }
            break;

        // alarm message
        case ALARM:
            // if hours and minutes are valid
            if( ( MsgStruct->param1 <= 23u ) && ( MsgStruct->param2 <= 59u ) ) {
                ParamStatus = VALID_PARAMETERS; // valid parameters
            }
            break;

        // heartbeat message
        case HEART:
            // if miliseconds are valid and multiple of 50
            if( ( MsgStruct->param1 >= 50u ) && ( MsgStruct->param1 <= 1000u ) && ( ( MsgStruct->param1 % 50u ) == 0u ) ) {
                ParamStatus = VALID_PARAMETERS; // valid parameters
            }
            break;

        case TEMP:
            // if lower temperature and upper temperature are valid and lower is less than upper
            if( ( MsgStruct->param1 <= 99u ) && ( MsgStruct->param2 <= 99u ) && ( MsgStruct->param1 < MsgStruct->param2 ) ) {
                ParamStatus = VALID_PARAMETERS; // valid parameters
            }
            break;
        
        default:
            // no action required
            break;

    } // end switch 

    return ParamStatus; // return parameter status
}


/**
  * @brief  Validates the date for the DATE message type.
  * @param  day    Day in the month.
  * @param  month  Month in the year.
  * @param  year   Year.
  * @retval Date Status.
  */
uint8_t validate_date( uint8_t day, uint8_t month, uint16_t year )
{   
    uint8_t DateStatus = INVALID_DATE; // return date status

    // if month has 31 days
    if( ( month == 1u ) || ( month == 3u ) || ( month == 5u ) || ( month == 7u ) || ( month == 8u ) || ( month == 10u)  || ( month == 12u ) ) {
        
        // if day is valid
        if( ( day >= 1u ) && ( day <= 31u ) ) {
            DateStatus = VALID_DATE;
        }
    }
    // if month has 30 days
    else if( ( month == 4u ) || ( month == 6u ) || ( month == 9u ) || ( month == 11u ) ) {
        
        // if day is valid
        if( ( day >= 1u ) && ( day <= 30u ) ) {
            DateStatus = VALID_DATE;
        }
    }
    // if month is february
    else if( month == 2u ) {
        
        // if day is valid
        if( ( day >= 1u ) && ( day <= 28u ) ) {
            DateStatus = VALID_DATE;
        }

        // if day is 29 and year is leap
        else if( ( day == 29u ) && ( ( year % 4u ) == 0u ) && ( ( year % 100u ) == 0u ) && ( ( year % 400u ) == 0u ) ) {
            DateStatus = VALID_DATE;
        }
        else if( ( day == 29u ) && ( ( year % 4u ) == 0u ) && ( ( year % 100u ) != 0u ) ) {
            DateStatus = VALID_DATE;
        }
        else {
            // no action required
        }
    }
    else {
        // no action required
    }
    
    return DateStatus; // return date status
}


/**
  * @brief  Converts a string containing only decimal characters into its corresponding decimal value.
  * @param  str Pointer to the decimal string.
  * @retval Decimal value if valid, otherwise INVALID_STRING.
  */
int16_t string_to_uint( uint8_t *str )
{   
    uint16_t decimal = 0; // decimal conversion
    uint8_t  CharCtr;     // character counter

    // loop through all elements in character array
    for( CharCtr = 0u; str[ CharCtr ] != ( uint8_t ) '\0'; CharCtr++ ) {

        // if character array is less than 5 characters and current character is decimal character
        if( ( CharCtr < 4u ) && ( str[ CharCtr ] >= ( uint8_t ) '0' ) && ( str[ CharCtr ] <= ( uint8_t ) '9' ) ) {
            decimal = ( decimal * 10u ) + ( str[ CharCtr ] - ( uint8_t ) '0' ); // update decimal value
        }
        else { // if current character is greater than 5 characters or current character is not decimal character
            decimal = INVALID_STRING;
            break; // break for
        }
    }
    
    return decimal;
}