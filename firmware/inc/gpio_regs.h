// Created with Corsair v1.0.4
#ifndef __GREGS_H
#define __GREGS_H

#define __I  volatile const // 'read only' permissions
#define __O  volatile       // 'write only' permissions
#define __IO volatile       // 'read / write' permissions


#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

#define GCSR_BASE_ADDR 0x40000000

// GPIO_0 - General Purpose Register
#define GCSR_GPIO_0_ADDR 0x0
#define GCSR_GPIO_0_RESET 0x0
typedef struct {
    uint32_t DATA : 32; // Data To Send To Leds
} gcsr_gpio_0_t;

// GPIO_0.DATA - Data To Send To Leds
#define GCSR_GPIO_0_DATA_WIDTH 32
#define GCSR_GPIO_0_DATA_LSB 0
#define GCSR_GPIO_0_DATA_MASK 0xffffffff
#define GCSR_GPIO_0_DATA_RESET 0x0


// Register map structure
typedef struct {
    union {
        __IO uint32_t GPIO_0; // General Purpose Register
        __IO gcsr_gpio_0_t GPIO_0_bf; // Bit access for GPIO_0 register
    };
} gcsr_t;

#define GCSR ((gcsr_t*)(GCSR_BASE_ADDR))

#ifdef __cplusplus
}
#endif

#endif /* __REGS_H */