#include <stdint.h>
#include <string.h>
#include "queue.h"

/**
  * @brief  Initializes the Queue to the specified parameters in the QUEUE_HandleTypeDef.
  * @param  hqueue Pointer to a QUEUE_HandleTypeDef structure that contains
  *                the configuration information for the specified Queue.
  * @retval None.
  */
void HIL_QUEUE_Init( QUEUE_HandleTypeDef *hqueue )
{
    hqueue->Head  = 0u;              // head is at zero element
    hqueue->Tail  = 0u;              // tail is at zero element
    hqueue->Empty = QUEUE_EMPTY;     // buffer empty
    hqueue->Full  = QUEUE_NOT_FULL;  // buffer not full
}


/**
  * @brief  Checks if the specified Queue is empty.
  * @param  hqueue Pointer to a QUEUE_HandleTypeDef structure that contains
  *                the configuration information for the specified Queue.
  * @retval Queue Empty flag status.
  */
uint8_t HIL_QUEUE_IsEmpty( QUEUE_HandleTypeDef *hqueue )
{
    return hqueue->Empty; // return 'Empty' flag status
}


/**
  * @brief  Checks if the specified Queue is full.
  * @param  hqueue Pointer to a QUEUE_HandleTypeDef structure that contains
  *                the configuration information for the specified Queue.
  * @retval Queue Full flag status.
  */
uint8_t HIL_QUEUE_IsFull( QUEUE_HandleTypeDef *hqueue )
{
    return hqueue->Full; // return 'Full' flag status
}


/**
  * @brief  Writes a data value at the Queue Head whose size is specified by the QUEUE_HandleTypeDef.
  * @param  data   Pointer to the value to copied.
  * @param  hqueue Pointer to a QUEUE_HandleTypeDef structure that contains
  *                the configuration information for the specified Queue.
  * @retval Write Operation Status.
  */
uint8_t HIL_QUEUE_Write( QUEUE_HandleTypeDef *hqueue, void *data )
{   
    uint8_t WriteStatus = QUEUE_WRITE_ERROR; // return status

    // if queue is not full
    if( HIL_QUEUE_IsFull( hqueue ) == QUEUE_NOT_FULL ) {
        
        // copy 'Size' bytes from data into queue starting at position ( Head * Size )
        /* cppcheck-suppress misra-c2012-11.5 - since arithmetic on void pointer is undefined behaviour, to cast to a known type is necessary */
        /* cppcheck-suppress misra-c2012-18.4 - in order to read data from queue starting at the tail, this is the appropiate way to do so */
        ( void ) memcpy( ( uint8_t* ) ( hqueue->Buffer ) + ( hqueue->Head * hqueue->Size ), data, hqueue->Size );

        // move head to next position
        ++( hqueue->Head );

        // if 'Head' is off buffer, move it to start position
        hqueue->Head %= hqueue->Elements;

        // check if queue is full now
        if( hqueue->Head == hqueue->Tail ) {
            hqueue->Full = QUEUE_FULL; // set queue full flag
        }

        // if queue was empty
        if( HIL_QUEUE_IsEmpty( hqueue ) == QUEUE_EMPTY ) {
            hqueue->Empty = QUEUE_NOT_EMPTY; // queue not empty anymore
        }

        WriteStatus = QUEUE_WRITE_SUCCESS; // successful writting
    }

    return WriteStatus; // return write status
}


/**
  * @brief  Reads a data value at the Queue Tail whose size is specified by the QUEUE_HandleTypeDef.
  * @param  hqueue Pointer to a QUEUE_HandleTypeDef structure that contains
  *                the configuration information for the specified Queue.
  * @param  data   Pointer to the variable used to store the read value.
  * @retval Read Operation Status.
  */
uint8_t HIL_QUEUE_Read( QUEUE_HandleTypeDef *hqueue, void *data )
{   
    uint8_t ReadStatus = QUEUE_READ_ERROR; // return status

    // if queue is not empty
    if( HIL_QUEUE_IsEmpty( hqueue ) == QUEUE_NOT_EMPTY ) {
        
        // copy 'Size' bytes from queue starting at position ( Tail * Size ) into data
        /* cppcheck-suppress misra-c2012-11.5 - since arithmetic on void pointer is undefined behaviour, to cast to a known type is necessary */
        /* cppcheck-suppress misra-c2012-18.4 - in order to read data from queue starting at the tail, this is the appropiate way to do so */
        ( void ) memcpy( data, ( uint8_t* ) ( hqueue->Buffer ) + ( hqueue->Tail * hqueue->Size ), hqueue->Size );

        // move tail to next position
        ++( hqueue->Tail );

        // if 'Tail' is off buffer, move it to start position
        hqueue->Tail %= hqueue->Elements;

        // check if queue is empty now
        if( hqueue->Tail == hqueue->Head ) {
            hqueue->Empty = QUEUE_EMPTY; // set queue empty flag
        }

        // if queue was full
        if( HIL_QUEUE_IsFull( hqueue ) == QUEUE_FULL ) {
            hqueue->Full = QUEUE_NOT_FULL; // queue not full anymore
        }

        ReadStatus = QUEUE_READ_SUCCESS; // succesfull reading
    }

    return ReadStatus; // return read status
}
