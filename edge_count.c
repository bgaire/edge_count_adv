//*****************************************************************************
//
// edge_count.c - Timer edge count mode example.
//
// Copyright (c) 2019-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.2.0.295 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
//! \addtogroup timer_examples_list
//! <h1>Timer Edge Count Demo (edge_count)</h1>
//!
//! This example application demonstrates the use of a general purpose timer
//! in down edge count mode.  Timer 0 is configured to decrement each time
//! a rising edge is seen on PF0/CCP0.  The count is initialized to 9 and the
//! match is set to 0, causing an interrupt to fire after 10 positive edges
//! detected on the CCP pin.
//!
//! PF0/CCP0 was chosen for this example due to being a push button available
//! on the EK-TM4C123GXL LaunchPad.
//!
//! UART0, connected to the Virtual Serial Port and running at 115,200, 8-N-1,
//! is used to display messages from this application.
//
//*****************************************************************************

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART
//
//*****************************************************************************
volatile bool g_bFlags = false;

//*****************************************************************************
//
// An interrupt counter indicating how often the timer down counter reached 0.
//
//*****************************************************************************
uint32_t g_ui32RawCount = 0;
volatile uint32_t g_ui32Count = 0;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// In this particular example, this function merely updates an interrupt
// count and sets a flag which tells the main loop to update the display.
//
//*****************************************************************************
void
ProcessInterrupt(void)
{
    //
    // Update our counter.
    //
    uint32_t oldCount = g_ui32RawCount;
    g_ui32RawCount = MAP_TimerValueGet(WTIMER5_BASE, TIMER_A);
    g_ui32Count = g_ui32RawCount - oldCount;
    /*MAP_TimerDisable(WTIMER5_BASE, TIMER_A);
    MAP_TimerEnable(WTIMER5_BASE, TIMER_A);*/

    //
    // Toggle the interrupt flag telling the main loop to update the display.
    //
    g_bFlags = true;
}

//*****************************************************************************
//
// The interrupt handler for Timer 0.  This will be called whenever the timer
// count reaches the match value (10 in this example).
//
//*****************************************************************************
void
WTimer5BIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(WTIMER5_BASE, TIMER_TIMB_TIMEOUT);

    ProcessInterrupt();
}


//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    MAP_UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
int
main(void)
{

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    MAP_FPULazyStackingEnable();

    //
    // Set the clocking to run at 80 MHz from PLL
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);
    uint32_t ui32SysClock = MAP_SysCtlClockGet();

    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for Systick operation.
    //
    ConfigureUART();

    //
    // Enable the peripherals used by this example.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    
    MAP_GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_6);
    MAP_GPIOPinConfigure(GPIO_PD6_WT5CCP0);

    //
    // Enable processor interrupts.
    //
    MAP_IntMasterEnable();

    //
    // Configure the timers in downward edge count mode.
    //
    MAP_TimerConfigure(WTIMER5_BASE, (TIMER_CFG_SPLIT_PAIR |
                       TIMER_CFG_A_CAP_COUNT_UP | TIMER_CFG_B_PERIODIC));
    MAP_TimerControlEvent(WTIMER5_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);
    MAP_TimerLoadSet(WTIMER5_BASE, TIMER_B, ui32SysClock);

    //
    // Setup the interrupt for the edge capture timer.  Note that we
    // use the capture match interrupt and NOT the timeout interrupt!
    //
    MAP_IntEnable(INT_WTIMER5B);
    MAP_TimerIntEnable(WTIMER5_BASE, TIMER_TIMB_TIMEOUT);

    //
    // Enable the timer.
    //
    MAP_TimerEnable(WTIMER5_BASE, TIMER_BOTH);


    //
    // Print the initial count.
    //
    UARTprintf("Count: %2d\r", g_ui32Count);

    //
    // At this point, the timer will count every time a positive
    // edge is detected on the relevant pin.
    //
    while(1)
    {
        //
        // Has there been an interrupt since last we checked?
        //
        if(g_bFlags)
        {
            //
            // Clear the bit.
            //
            g_bFlags = false;

            //
            // Update the interrupt count.
            //
            UARTprintf("Count: %08d\n\r", g_ui32Count);
        }
    }
}
