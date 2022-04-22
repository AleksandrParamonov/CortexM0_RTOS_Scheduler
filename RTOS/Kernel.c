/*
 * Kernel.c
 *
 *  Created on: 11 но€б. 2019 г.
 *      Author: paramonov.ext
 */

#include "Kernel.h"
#include "stm32f030xc.h"

static ThreadState threads[MAX_NUMBER_OF_THREADS] = { 0 };
static int threadsAmount = 0;
static int currentThread = 0;

void StartScheduler(void)
{
    currentThread = -1;
    __asm volatile ("MOV r11, %0\n" : : "r" (threads[0].SP));
    // preparing PSP for first thread
    __asm volatile ("MSR PSP, r11");
    while (1)
        ;
}

/**
 * @brief This function handles global timer interrupt which performs context switching.
 * @details Function is naked because custom return sequence is used.
 */
__attribute__((naked))  void SysTick_Handler(void)
{
    __disable_irq();

    // getting current stack pointer
    int SP;
    __asm volatile ("MRS r0, PSP");
    __asm volatile ("MOV %0, r0\n" : "=r" (SP) );
    /* USER CODE BEGIN TIM6_IRQn 0 */

    // no need to save context before first context switching because its context of scheduler
    if (currentThread != -1)
    {
        // registers R0...R3, R12 are pushed onto the stack by processor itself, here they are extracted from stack and stored in scheduler
        uint32_t* stackRegisters = (uint32_t*) SP;
        threads[currentThread].registers[0] = stackRegisters[0];
        threads[currentThread].registers[1] = stackRegisters[1];
        threads[currentThread].registers[2] = stackRegisters[2];
        threads[currentThread].registers[3] = stackRegisters[3];
        threads[currentThread].registers[12] = stackRegisters[4];
        threads[currentThread].LR = stackRegisters[5];
        threads[currentThread].PC = stackRegisters[6];
        threads[currentThread].xPSR = stackRegisters[7];

        // reading other registers
        __asm volatile ("MOV %0, r4\n" : "=r" (threads[currentThread].registers[4]) );
        __asm volatile ("MOV %0, r5\n" : "=r" (threads[currentThread].registers[5]) );
        __asm volatile ("MOV %0, r6\n" : "=r" (threads[currentThread].registers[6]) );
        __asm volatile ("MOV %0, r7\n" : "=r" (threads[currentThread].registers[7]) );
        __asm volatile ("MOV %0, r8\n" : "=r" (threads[currentThread].registers[8]) );
        __asm volatile ("MOV %0, r9\n" : "=r" (threads[currentThread].registers[9]) );
        __asm volatile ("MOV %0, r10\n" : "=r" (threads[currentThread].registers[10]) );
        __asm volatile ("MOV %0, r11\n" : "=r" (threads[currentThread].registers[11]) );

        // moving PSP value to r11
        __asm volatile ("MRS r11, PSP");

        // reading extracted PSP value
        __asm volatile ("MOV %0, r11\n" : "=r" (threads[currentThread].SP) );
    }

    // simple round-robin to select next thread
    currentThread = (currentThread + 1) % threadsAmount;

    uint32_t* stackRegisters = (uint32_t*) threads[currentThread].SP;

    // restoring register values in stack
    stackRegisters[0] = threads[currentThread].registers[0];
    stackRegisters[1] = threads[currentThread].registers[1];
    stackRegisters[2] = threads[currentThread].registers[2];
    stackRegisters[3] = threads[currentThread].registers[3];
    stackRegisters[4] = threads[currentThread].registers[12];
    stackRegisters[5] = threads[currentThread].LR;
    stackRegisters[6] = threads[currentThread].PC;
    stackRegisters[7] = threads[currentThread].xPSR;

    __asm volatile ("MOV r11, %0\n" : : "r" (threads[currentThread].SP));
    __asm volatile ("MSR PSP, r11");

    __asm volatile ("MOV r4, %0\n" : : "r" (threads[currentThread].registers[4]));
    __asm volatile ("MOV r5, %0\n" : : "r" (threads[currentThread].registers[5]));
    __asm volatile ("MOV r6, %0\n" : : "r" (threads[currentThread].registers[6]));
    __asm volatile ("MOV r7, %0\n" : : "r" (threads[currentThread].registers[7]));
    __asm volatile ("MOV r8, %0\n" : : "r" (threads[currentThread].registers[8]));
    __asm volatile ("MOV r9, %0\n" : : "r" (threads[currentThread].registers[9]));
    __asm volatile ("MOV r10, %0\n" : : "r" (threads[currentThread].registers[10]));
    __asm volatile ("MOV r11, %0\n" : : "r" (threads[currentThread].registers[11]));

    // preparing to return to thread mode. PSP will be used as stack pointer
    __asm volatile ("MOV r0, %0\n" : : "r" (0xFFFFFFFD));

    __enable_irq();
    // time to begin execution of thread
    __asm volatile ("bx r0");
}

void AddToScheduler(void (*ptr)(), uint16_t stackSize)
{
    static int stackPointer = THREADS_BEGINNING_OF_STACK;

    // checking whether there is sufficient memory for thread stack
    if (stackPointer + stackSize >= THREADS_END_OF_STACK)
        return;
    // ignore if there are too many threads added already.
    if (threadsAmount >= MAX_NUMBER_OF_THREADS)
        return;

    // thread will begin execution from the beginning of assigned function.
    threads[threadsAmount].PC = (int) ptr;

    // T-bit (1 << 24) is set because otherwise there is HardFault.
    threads[threadsAmount].xPSR = 0x01000000;

    // arbitrary funny value
    threads[threadsAmount].LR = 0xdeadbeef;

    // stack pointer of thread
    stackPointer += stackSize;
    threads[threadsAmount].SP = stackPointer;
    threadsAmount++;

}
