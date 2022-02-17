#include <stdint.h>
#include "stm32g0xx.h"
#include "temp.h"

/**
  * @brief  Initializes the Temperature Sensor to the specified parameters in the TEMP_HandleTypeDef.
  * @param  htemp Pointer to a TEMP_HandleTypeDef structure that contains the configuration information for the specified Temperature Sensor.
  * @retval None.
  */
void MOD_TEMP_Init( TEMP_HandleTypeDef *htemp )
{
    uint8_t writeBuffer[ 3 ]; // i2c write buffer

    // write 125°C to TCRIT register
    writeBuffer[ 0 ] = TCRIT_REGISTER;
    writeBuffer[ 1 ] = 0x07u;
    writeBuffer[ 2 ] = 0xD0u;
    HAL_I2C_Master_Transmit( htemp->I2cSensor, SENSOR_ADRESS, writeBuffer, 3, 5000 );

    /* write to CONFIG register:
    - THYST     = 0°C
    - SHDN      = continuous conversion
    - Alert Cnt = alert disabled
    - Alert Sel = alert output for TUPPER and TLOWER
    - Alert Pol = active-low
    - Alert Mod = comparator */
    MOD_TEMP_DisableAlarm( htemp );

    // user extra initialization code
    MOD_TEMP_MspInit( htemp );
}


/**
  * @brief  Initializes the Temperature Sensor MSP.
  * @param  htemp Pointer to a TEMP_HandleTypeDef structure that contains the configuration information for the specified Temperature Sensor.
  * @retval None.
  */
__weak void MOD_TEMP_MspInit( TEMP_HandleTypeDef *htemp )
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED( htemp );

  /* NOTE : This function should not be modified, when the callback is needed,
            the MOD_TEMP_MspInit should be implemented in the user file */
}


/**
  * @brief  Reads the Temperature Ambient Register from the Temperature Sensor 
  *         by using the specified I2C in the TEMP_HandleTypeDef.
  * @param  htemp Pointer to a TEMP_HandleTypeDef structure that contains the configuration information for the specified Temperature Sensor.
  * @retval Celsius Temperature in decimal format.
  */
int8_t MOD_TEMP_Read( TEMP_HandleTypeDef *htemp )
{
    uint8_t writeBuffer;     // i2c write buffer
    uint8_t readBuffer[ 2 ]; // i2c read buffer
    int8_t  temperature;     // temperature read from sensor

    // read from TA (temperature) register
    writeBuffer = TA_REGISTER;
    HAL_I2C_Master_Transmit( htemp->I2cSensor, SENSOR_ADRESS, &writeBuffer, 1, 5000 );
    HAL_I2C_Master_Receive( htemp->I2cSensor, SENSOR_ADRESS, readBuffer, 2, 5000 );

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
    
    return temperature;
}


/**
  * @brief  Sets the Temperature Alert Window Limits by writing to the Temperature Limit Registers
  *         by using the specified I2C in the TEMP_HandleTypeDef.
  * @param  htemp Pointer to a TEMP_HandleTypeDef structure that contains the configuration information for the specified Temperature Sensor.
  * @param  lower Lower Temperature Window Limit (0-99).
  * @param  upper Upper Temperature Window Limit (0-99).
  * @note   upper temperature should be a value grater than lower temperature.
  * @retval None.
  */
void MOD_TEMP_SetAlarms( TEMP_HandleTypeDef *htemp, uint8_t lower, uint8_t upper )
{   
    uint8_t writeBuffer[ 3 ]; // i2c write buffer

    // convert lower temperature to TLOWER register format
    writeBuffer[ 1 ] = ( lower & ~0x0Fu ) >> 4u;
    writeBuffer[ 2 ] = lower << 4u;

    // write lower temperature to TLOWER register
    writeBuffer[ 0 ] = TLOWER_REGISTER;
    HAL_I2C_Master_Transmit( htemp->I2cSensor, SENSOR_ADRESS, writeBuffer, 3, 5000 );

    // convert upper temperature to TUPPER register format
    writeBuffer[ 1 ] = ( upper & ~0x0Fu ) >> 4u;
    writeBuffer[ 2 ] = upper << 4u;

    // write upper temperature to TUPPER register
    writeBuffer[ 0 ] = TUPPER_REGISTER;
    HAL_I2C_Master_Transmit( htemp->I2cSensor, SENSOR_ADRESS, writeBuffer, 3, 5000 );

    // write to CONFIG register (enable alert output)
    writeBuffer[ 0 ] = CONFIG_REGISTER;
    writeBuffer[ 1 ] = 0x00u;
    writeBuffer[ 2 ] = 0x08u;
    HAL_I2C_Master_Transmit( htemp->I2cSensor, SENSOR_ADRESS, writeBuffer, 3, 5000 );
}


/**
  * @brief  Disables the Temperature Alert Window Limits by writing to the Configuration register
  *         by using the specified I2C in the TEMP_HandleTypeDef.
  * @param  htemp Pointer to a TEMP_HandleTypeDef structure that contains the configuration information for the specified Temperature Sensor.
  * @retval None.
  */
void MOD_TEMP_DisableAlarm( TEMP_HandleTypeDef *htemp )
{
    uint8_t writeBuffer[ 3 ]; // i2c write buffer

    // write to CONFIG register (disable alert output)
    writeBuffer[ 0 ] = CONFIG_REGISTER;
    writeBuffer[ 1 ] = 0x00u;
    writeBuffer[ 2 ] = 0x00u;
    HAL_I2C_Master_Transmit( htemp->I2cSensor, SENSOR_ADRESS, writeBuffer, 3, 5000 );
}