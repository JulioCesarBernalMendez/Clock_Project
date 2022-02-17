#ifndef QUEUE_H
#define QUEUE_H

    // queue definitions
    #define QUEUE_NOT_EMPTY      0u
    #define QUEUE_EMPTY          1u
    #define QUEUE_NOT_FULL       0u
    #define QUEUE_FULL           1u
    #define QUEUE_WRITE_ERROR    0u
    #define QUEUE_WRITE_SUCCESS  1u
    #define QUEUE_READ_ERROR     0u
    #define QUEUE_READ_SUCCESS   1u

    // queue configuration structure
    typedef struct
    {
        void     *Buffer;   // array of any data type
        uint32_t  Elements; // maximum number of elements the buffer can store
        uint8_t   Size;     // size of each element in buffer array
        uint32_t  Head;     // current buffer element to write on
        uint32_t  Tail;     // current buffer element to be read
        uint8_t   Empty;    // buffer empty flag
        uint8_t   Full;     // buffer full flag
    } QUEUE_HandleTypeDef;

    // queue function prototypes
    void HIL_QUEUE_Init( QUEUE_HandleTypeDef *hqueue );
    uint8_t HIL_QUEUE_IsEmpty( QUEUE_HandleTypeDef *hqueue );
    uint8_t HIL_QUEUE_IsFull( QUEUE_HandleTypeDef *hqueue );
    uint8_t HIL_QUEUE_Write( QUEUE_HandleTypeDef *hqueue, void *data );
    uint8_t HIL_QUEUE_Read( QUEUE_HandleTypeDef *hqueue, void *data );

#endif