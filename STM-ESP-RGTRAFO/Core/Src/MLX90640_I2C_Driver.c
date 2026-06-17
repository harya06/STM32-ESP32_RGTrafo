#include "MLX90640_I2C_Driver.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define MLX90640_I2C_TIMEOUT  100  // 100ms timeout

void MLX90640_I2CInit()
{
    // Optional: Force I2C reset
    __HAL_I2C_DISABLE(&hi2c1);
    HAL_Delay(10);
    __HAL_I2C_ENABLE(&hi2c1);
}

int MLX90640_I2CGeneralReset(void)
{
    return 0;
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t* data)
{
    uint8_t* p = (uint8_t*)data;
    HAL_StatusTypeDef status;

    //Timeout terbatas (bukan HAL_MAX_DELAY)
    status = HAL_I2C_Mem_Read(&hi2c1, (slaveAddr << 1), startAddress,
                              I2C_MEMADD_SIZE_16BIT, p, nMemAddressRead * 2,
                              MLX90640_I2C_TIMEOUT);

    if (status != HAL_OK)
    {
        //Clear error state
        if (hi2c1.ErrorCode != HAL_I2C_ERROR_NONE)
        {
            __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF);  // Clear NACK
            hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
        }
        return -1;
    }

    // Byte swap (endianness correction)
    for (int i = 0; i < nMemAddressRead * 2; i += 2)
    {
        uint8_t temp = p[i + 1];
        p[i + 1] = p[i];
        p[i] = temp;
    }

    return 0;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    uint8_t cmd[2];
    uint16_t datacheck;
    HAL_StatusTypeDef status;

    cmd[0] = data >> 8;
    cmd[1] = data & 0x00FF;

    //Timeout terbatas
    status = HAL_I2C_Mem_Write(&hi2c1, slaveAddr << 1, writeAddress,
                               I2C_MEMADD_SIZE_16BIT, cmd, 2,
                               MLX90640_I2C_TIMEOUT);

    if (status != HAL_OK)
    {
        //Clear error state
        if (hi2c1.ErrorCode != HAL_I2C_ERROR_NONE)
        {
            __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF);
            hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
        }
        return -1;
    }

    // Verify write
    if (MLX90640_I2CRead(slaveAddr, writeAddress, 1, &datacheck) != 0)
        return -2;

    if (datacheck != data)
        return -2;

    return 0;
}
