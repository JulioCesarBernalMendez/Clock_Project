#ifndef SERIAL_H
#define SERIAL_H

    // serial state machine definitions
    #define IDLE                 0u
    #define PROCESS_BUFFER       1u
    #define EVAL_PARAMETERS      2u
    #define SERIAL_ERROR         3u
    #define SERIAL_OK            4u

    // serial data queue definitions
    #define MESSAGE_SIZE         19u
    #define SERIAL_QUEUE_SIZE    128
    
    // serial message validation definitions
    #define INVALID_PARAMETERS   0u
    #define VALID_PARAMETERS     1u
    #define INVALID_DATE         0u
    #define VALID_DATE           1u
    #define INVALID_STRING      -1

    // UART2 definitions
    #define GPIOUART2            GPIOA
    #define GPIO_PIN_UART2_TX    GPIO_PIN_2
    #define GPIO_PIN_UART2_RX    GPIO_PIN_3
    
    // serial initialization function prototypes
    void serial_init( void );
    void uart_2_init( void );
    void queue_serial_init( void );

    // serial state machine function prototype
    void serial_task( void );
    
    // serial message validation function prototypes
    uint8_t validate_message( uint8_t *SerialMsg, uint8_t **param1Ptr, uint8_t **param2Ptr, uint8_t **param3Ptr );
    uint8_t validate_msg_parameters( SERIAL_MsgTypeDef *MsgStruct, uint8_t MsgType );
    uint8_t validate_date( uint8_t day, uint8_t month, uint16_t year );
    int16_t string_to_uint( uint8_t *str );

#endif