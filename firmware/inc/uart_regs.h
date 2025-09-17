// Created with Corsair v1.0.4
#ifndef __UREGS_H
#define __UREGS_H

#define __I  volatile const // 'read only' permissions
#define __O  volatile       // 'write only' permissions
#define __IO volatile       // 'read / write' permissions


#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

#define UCSR_BASE_ADDR 0x50000000

// U_DATA - UART Data register
#define UCSR_U_DATA_ADDR 0x0
#define UCSR_U_DATA_RESET 0x0
typedef struct {
    uint32_t DATA : 8; // Data To Send Via UART TX
    uint32_t : 24; // reserved
} ucsr_u_data_t;

// U_DATA.DATA - Data To Send Via UART TX
#define UCSR_U_DATA_DATA_WIDTH 8
#define UCSR_U_DATA_DATA_LSB 0
#define UCSR_U_DATA_DATA_MASK 0xff
#define UCSR_U_DATA_DATA_RESET 0x0

// U_STAT - UART Status register
#define UCSR_U_STAT_ADDR 0x4
#define UCSR_U_STAT_RESET 0x20
typedef struct {
    uint32_t : 5; // reserved
    uint32_t READY : 1; // UART is Ready
    uint32_t : 7; // reserved
    uint32_t TX_DONE : 1; // Done Transmitting The Last Char (8-bit)
    uint32_t : 18; // reserved
} ucsr_u_stat_t;

// U_STAT.READY - UART is Ready
#define UCSR_U_STAT_READY_WIDTH 1
#define UCSR_U_STAT_READY_LSB 5
#define UCSR_U_STAT_READY_MASK 0x20
#define UCSR_U_STAT_READY_RESET 0x1

// U_STAT.TX_DONE - Done Transmitting The Last Char (8-bit)
#define UCSR_U_STAT_TX_DONE_WIDTH 1
#define UCSR_U_STAT_TX_DONE_LSB 13
#define UCSR_U_STAT_TX_DONE_MASK 0x2000
#define UCSR_U_STAT_TX_DONE_RESET 0x0

// U_CTRL - UART Control register
#define UCSR_U_CTRL_ADDR 0x8
#define UCSR_U_CTRL_RESET 0x0
typedef struct {
    uint32_t : 9; // reserved
    uint32_t START : 1; // TX Begin Signal, Valid For Only One Cycle
    uint32_t : 22; // reserved
} ucsr_u_ctrl_t;

// U_CTRL.START - TX Begin Signal, Valid For Only One Cycle
#define UCSR_U_CTRL_START_WIDTH 1
#define UCSR_U_CTRL_START_LSB 9
#define UCSR_U_CTRL_START_MASK 0x200
#define UCSR_U_CTRL_START_RESET 0x0


// Register map structure
typedef struct {
    union {
        __IO uint32_t U_DATA; // UART Data register
        __IO ucsr_u_data_t U_DATA_bf; // Bit access for U_DATA register
    };
    union {
        __IO uint32_t U_STAT; // UART Status register
        __IO ucsr_u_stat_t U_STAT_bf; // Bit access for U_STAT register
    };
    union {
        __O uint32_t U_CTRL; // UART Control register
        __O ucsr_u_ctrl_t U_CTRL_bf; // Bit access for U_CTRL register
    };
} ucsr_t;

#define UCSR ((ucsr_t*)(UCSR_BASE_ADDR))

#ifdef __cplusplus
}
#endif

#endif /* __REGS_H */