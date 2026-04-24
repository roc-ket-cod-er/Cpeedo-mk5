#ifndef CSE_CST328_H
#define CSE_CST328_H

#include "main.h"
#include <stdbool.h>

#define CST328_I2C_ADDR     (0x1A << 1)
#define REG_TOUCH_INFO      0xD000
#define REG_MODE_NORMAL     0xD109
#define REG_MODE_DEBUG_INFO 0xD101

typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t z;
    uint8_t state;
} CST328_TouchPoint;

typedef struct {
    I2C_HandleTypeDef* hi2c;
    GPIO_TypeDef* rst_port;
    uint16_t rst_pin;
    uint16_t width;
    uint16_t height;
    uint16_t defWidth;
    uint16_t defHeight;
    uint8_t rotation;
    bool inited;
    CST328_TouchPoint touchPoints[5];
} CST328_HandleTypeDef;

void CST328_InitHandle(CST328_HandleTypeDef* dev, I2C_HandleTypeDef* hi2c, uint16_t w, uint16_t h);
bool CST328_Begin(CST328_HandleTypeDef* dev);
void CST328_ReadData(CST328_HandleTypeDef* dev);
bool CST328_IsTouched(CST328_HandleTypeDef* dev);
uint8_t CST328_SetRotation(CST328_HandleTypeDef* dev, uint8_t r);

#endif
