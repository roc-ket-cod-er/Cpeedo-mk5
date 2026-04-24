#include "CSE_CST328.h"
#include <string.h>

void CST328_InitHandle(CST328_HandleTypeDef* dev, I2C_HandleTypeDef* hi2c, uint16_t w, uint16_t h) {
    memset(dev, 0, sizeof(CST328_HandleTypeDef));
    dev->hi2c = hi2c;
    dev->defWidth = w;
    dev->defHeight = h;
    dev->width = w;
    dev->height = h;
    dev->rotation = 0;
    dev->inited = false;
}

bool CST328_Begin(CST328_HandleTypeDef* dev) {
    if (dev->inited) return true;

    // 1. Hardware Reset
    if (dev->rst_port != NULL) {
        HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_RESET);
        HAL_Delay(20);
        HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_SET);
        HAL_Delay(100);
    }

    // 2. Put into Normal Mode
    uint8_t mode_cmd[2] = { (uint8_t)(REG_MODE_NORMAL >> 8), (uint8_t)(REG_MODE_NORMAL & 0xFF) };
    HAL_I2C_Master_Transmit(dev->hi2c, CST328_I2C_ADDR, mode_cmd, 2, 100);
    HAL_Delay(50);

    dev->inited = true;
    return true;
}

void CST328_ReadData(CST328_HandleTypeDef* dev) {
    uint8_t data[6];

    // Read 6 bytes starting at 0xD000
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c, CST328_I2C_ADDR,
                                               REG_TOUCH_INFO, I2C_MEMADD_SIZE_16BIT,
                                               data, 6, 20);

    if (status != HAL_OK) return;

    // Based on your RAW dump: 0x06 = Press, 0x00 = Release
    if (data[0] == 0x06) {
        dev->touchPoints[0].state = 1;

        // Assembly: X = Byte1:Byte3_High, Y = Byte2:Byte3_Low
        uint16_t raw_x = (data[1] << 4) | ((data[3] >> 4) & 0x0F);
        uint16_t raw_y = (data[2] << 4) | (data[3] & 0x0F);

        // Apply Rotation
        switch (dev->rotation) {
            case 1:
                dev->touchPoints[0].x = raw_y;
                dev->touchPoints[0].y = dev->defWidth - raw_x - 1;
                break;
            case 2:
                dev->touchPoints[0].x = dev->defWidth - raw_x - 1;
                dev->touchPoints[0].y = dev->defHeight - raw_y - 1;
                break;
            case 3:
                dev->touchPoints[0].x = dev->defHeight - raw_y - 1;
                dev->touchPoints[0].y = raw_x;
                break;
            default: // Rotation 0
                dev->touchPoints[0].x = raw_x;
                dev->touchPoints[0].y = raw_y;
                break;
        }
        dev->touchPoints[0].z = data[4];
    } else {
        dev->touchPoints[0].state = 0;
    }
}

bool CST328_IsTouched(CST328_HandleTypeDef* dev) {
    CST328_ReadData(dev);
    return (dev->touchPoints[0].state == 1);
}

uint8_t CST328_SetRotation(CST328_HandleTypeDef* dev, uint8_t r) {
    dev->rotation = r % 4;
    if (dev->rotation % 2 == 0) {
        dev->width = dev->defWidth; dev->height = dev->defHeight;
    } else {
        dev->width = dev->defHeight; dev->height = dev->defWidth;
    }
    return dev->rotation;
}
