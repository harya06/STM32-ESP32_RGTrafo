#include "MLX90640_I2C_Driver.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

#define MLX90640_I2C_TIMEOUT  100

/* Handle aktif — diset sebelum setiap operasi MLX */
static I2C_HandleTypeDef *_hi2c = NULL;

void MLX90640_I2CSelectBus(I2C_HandleTypeDef *hi2c)
{
    _hi2c = hi2c;
}

void MLX90640_I2CInit()
{
    if (_hi2c == NULL) return;
    __HAL_I2C_DISABLE(_hi2c);
    HAL_Delay(10);
    __HAL_I2C_ENABLE(_hi2c);
}

int MLX90640_I2CGeneralReset(void)
{
    return 0;
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t* data)
{
    if (_hi2c == NULL) return -1;
    uint8_t* p = (uint8_t*)data;
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(_hi2c, (slaveAddr << 1), startAddress,
                              I2C_MEMADD_SIZE_16BIT, p, nMemAddressRead * 2,
                              MLX90640_I2C_TIMEOUT);

    if (status != HAL_OK)
    {
        if (_hi2c->ErrorCode != HAL_I2C_ERROR_NONE)
        {
            __HAL_I2C_CLEAR_FLAG(_hi2c, I2C_FLAG_AF);
            _hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
        }
        return -1;
    }

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
    if (_hi2c == NULL) return -1;
    uint8_t cmd[2];
    uint16_t datacheck;
    HAL_StatusTypeDef status;

    cmd[0] = data >> 8;
    cmd[1] = data & 0x00FF;

    status = HAL_I2C_Mem_Write(_hi2c, slaveAddr << 1, writeAddress,
                               I2C_MEMADD_SIZE_16BIT, cmd, 2,
                               MLX90640_I2C_TIMEOUT);

    if (status != HAL_OK)
    {
        if (_hi2c->ErrorCode != HAL_I2C_ERROR_NONE)
        {
            __HAL_I2C_CLEAR_FLAG(_hi2c, I2C_FLAG_AF);
            _hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
        }
        return -1;
    }

    if (MLX90640_I2CRead(slaveAddr, writeAddress, 1, &datacheck) != 0)
        return -2;

    if (datacheck != data)
        return -2;

    return 0;
}
