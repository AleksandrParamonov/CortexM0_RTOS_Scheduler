/*
 * Kernel.h
 *
 *  Created on: 11 но€б. 2019 г.
 *      Author: paramonov.ext
 */

#ifndef KERNEL_H_
#define KERNEL_H_

#include "stdint.h"

#define MAX_NUMBER_OF_THREADS 10
#define THREADS_BEGINNING_OF_STACK 0x20004000
#define THREADS_END_OF_STACK 0x20007000

typedef struct __attribute__((__packed__))
{
    uint32_t SP;
    uint32_t PC;
    uint32_t LR;
    uint32_t xPSR;
    uint32_t registers[16];
} ThreadState ;

/**
 * @brief Adds thread to scheduler.
 * @param ptr Pointer to thread function.
 */
void AddToScheduler(void(*ptr)(), uint16_t stackSize);

/**
 * @brief Starts scheduler which performs context switch on every timer interrupt.
 */
void StartScheduler(void);

#endif /* KERNEL_H_ */
