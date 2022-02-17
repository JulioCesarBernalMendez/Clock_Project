#ifndef TEMP_H
#define TEMP_H

    // temperature sensor configuration structure
    typedef struct {
        I2C_HandleTypeDef *I2cSensor; // I2C Handler pointer
        GPIO_TypeDef      *AlertPort; // alert output port
        uint32_t           AlertPin;  // alert output pin
    } TEMP_HandleTypeDef;

    // temperature sensor definitions
    #define SENSOR_ADRESS        ( 0x1Fu << 1u )
    #define CONFIG_REGISTER      0x01u
    #define TUPPER_REGISTER      0x02u
    #define TLOWER_REGISTER      0x03u
    #define TCRIT_REGISTER       0x04u
    #define TA_REGISTER          0x05u
    #define RESOLUTION_REGISTER  0x08u

    // temperature sensor functions
    void   MOD_TEMP_Init( TEMP_HandleTypeDef *htemp );
    void   MOD_TEMP_MspInit( TEMP_HandleTypeDef *htemp );
    int8_t MOD_TEMP_Read( TEMP_HandleTypeDef *htemp );
    void   MOD_TEMP_SetAlarms( TEMP_HandleTypeDef *htemp, uint8_t lower, uint8_t upper );
    void   MOD_TEMP_DisableAlarm( TEMP_HandleTypeDef *htemp );

#endif