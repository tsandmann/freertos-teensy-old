/*
 * This file is part of the FreeRTOS port to Teensy boards.
 * Copyright (c) 2018 Timo Sandmann
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    teensy.cpp
 * @brief   FreeRTOS support implementations for Teensy boards with newlib
 * @author  Timo Sandmann
 * @date    26.05.2018
 */

#include "teensy.h"


/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers. That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#include <private/portable.h>

#if( configSUPPORT_DYNAMIC_ALLOCATION == 0 )
#error This file must not be used if configSUPPORT_DYNAMIC_ALLOCATION is 0
#endif

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include <Arduino.h>
#include <kinetis.h>

#include <new>
#include <cstdlib>
#include <cerrno>
#include <unistd.h>


extern "C" {
uint8_t* stack_top { nullptr }; /**< Pointer to top of (initial) stack, necessary for _sbrk() */

void serial_puts(const char* str) {
    Serial.println(str);
    Serial.flush();
}

/**
 * @brief Print assert message and blink one short pulse every two seconds
 * @param[in] file: Filename as C-string
 * @param[in] line: Line number
 * @param[in] func: Function name as C-string
 * @param[in] expr: Expression that failed as C-string
 */
void assert_blink(const char* file, int line, const char* func, const char* expr) {
    Serial.print("ASSERT in [");
    Serial.print(file);
    Serial.print(':');
    Serial.print(line, 10);
    Serial.print("] ");
    Serial.print(func);
    Serial.print(": ");
    Serial.println(expr);
    Serial.flush();

    freertos::error_blink(1);
}

struct __sFILE_fake {
    unsigned char* _p; /* current position in (some) buffer */
    int _r; /* read space left for getc() */
    int _w; /* write space left for putc() */
    short _flags; /* flags, below; this FILE is free if 0 */
    short _file; /* fileno, if Unix descriptor, else -1 */
    struct __sbuf _bf; /* the buffer (at least 1 byte, if !NULL) */
    int _lbfsize; /* 0 or -_bf._size, for inline putc */
    struct _reent* _data;
} __sf_fake_stdin, __sf_fake_stdout, __sf_fake_stderr; // FIXME: check this
} // extern C


void* operator new(size_t n, const std::nothrow_t&) noexcept {
    return malloc(n);
}

void* operator new[](size_t n, const std::nothrow_t&) noexcept {
    return malloc(n);
}

void operator delete(void* p, const std::nothrow_t&) noexcept {
    free(p);
}

void operator delete[](void* p, const std::nothrow_t&) noexcept {
    free(p);
}


namespace freertos {
/**
 * @brief Delay between led error flashes
 * @param[in] ms: Milliseconds to delay
 * @note Doesn't use a timer to work with interrupts disabled
 */
void delay_ms(const uint32_t ms) {
    const uint32_t iterations { ms * (F_CPU / 7000UL) };
    for (uint32_t i { 0 }; i < iterations; ++i) {
        asm volatile("nop");
    }
}

void error_blink(const uint8_t n) {
    __disable_irq();
    ::pinMode(LED_BUILTIN, OUTPUT);

    while (true) {
        for (uint8_t i { 0 }; i < n; ++i) {
            ::digitalWrite(LED_BUILTIN, true);
            delay_ms(300UL);
            ::digitalWrite(LED_BUILTIN, false);
            delay_ms(300UL);
        }
        delay_ms(2000UL);
    }
}

long free_ram() {
    return stack_top - reinterpret_cast<uint8_t*>(sbrk(0));
}

void print_free_ram() {
    Serial.print("free RAM: ");
    Serial.print(freertos::free_ram() / 1024UL);
    Serial.println(" KB");
}
} // namespace freertos


extern "C" {

extern unsigned long _ebss;
extern uint8_t* stack_top;

#ifndef STACK_MARGIN
#if defined(__MKL26Z64__)
#define STACK_MARGIN 512
#elif defined(__MK20DX128__)
#define STACK_MARGIN 1024
#elif defined(__MK20DX256__)
#define STACK_MARGIN 4096
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define STACK_MARGIN 8192
#endif
#endif // STACK_MARGIN

void* _sbrk(ptrdiff_t);
// override _sbrk() to make it thread-safe - you have to link with gcc option: "-Wl,--wrap=_sbrk"
void* __wrap__sbrk(ptrdiff_t incr) {
    static uint8_t* currentHeapEnd = reinterpret_cast<uint8_t*>(&_ebss);

    vTaskSuspendAll(); // Note: safe to use before FreeRTOS scheduler started
    void* const previousHeapEnd = currentHeapEnd;
    if (((intptr_t) currentHeapEnd + incr >= (intptr_t) stack_top - STACK_MARGIN) && stack_top) { // FIXME: double check this
#if( configUSE_MALLOC_FAILED_HOOK == 1 )
        {
            extern void vApplicationMallocFailedHook(void);
            vApplicationMallocFailedHook();
        }
#else
        // If you prefer to believe your application will gracefully trap out-of-memory...
        _impure_ptr->_errno = ENOMEM; // newlib's thread-specific errno
        xTaskResumeAll();
#endif
        return (void*) -1; // the malloc-family routine that called sbrk will return 0
    }

    currentHeapEnd += incr;
    xTaskResumeAll();

    return previousHeapEnd;
}

void* sbrk(ptrdiff_t incr) {
    return _sbrk(incr);
};

void* pvPortMalloc(size_t xWantedSize) {
    return malloc(xWantedSize);
}

void vPortFree(void* pv) {
    free(pv);
}


void vApplicationMallocFailedHook();
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char* pcTaskName);

void vApplicationMallocFailedHook() {
    /* Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap. pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores. */

    freertos::error_blink(2);
}

void vApplicationStackOverflowHook(TaskHandle_t, char*) {
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.
    This hook function is called if a stack overflow is detected. */

    freertos::error_blink(3);
}


#if ( configUSE_IDLE_HOOK == 1 )
void vApplicationIdleHook();
void vApplicationIdleHook() {
    yield();
}
#endif // configUSE_IDLE_HOOK

#if( configUSE_TICK_HOOK > 0 )
void vApplicationTickHook();
void vApplicationTickHook() {
    systick_isr();
}
#endif // configUSE_TICK_HOOK

#if( configSUPPORT_STATIC_ALLOCATION == 1 )
/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer, StackType_t** ppxIdleTaskStackBuffer, uint32_t* pulIdleTaskStackSize) {
    /* If the buffers to be provided to the Idle task are declared inside this
    function then they must be declared static - otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer, StackType_t** ppxTimerTaskStackBuffer, uint32_t* pulTimerTaskStackSize) {
    /* If the buffers to be provided to the Timer task are declared inside this
    function then they must be declared static - otherwise they will be allocated on
    the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif // configSUPPORT_STATIC_ALLOCATION

} // extern C
