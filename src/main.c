/*
 * FreeRTOS V202112.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */


/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "semphr.h"
#include "queue.h"
#include "string.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

/*-----------------------------------------------------------*/
int but1_in = 0,  but1_out = 0,  but1_total = 0;
int but2_in = 0,  but2_out = 0,  but2_total = 0;
int periodic_in = 0, periodic_out = 0, periodic_total = 0;
int uart_in = 0,     uart_out = 0,     uart_total = 0;
int load1_in = 0,    load1_out = 0,    load1_total = 0;
int load2_in = 0,    load2_out = 0,    load2_total = 0;
int total_time = 0, cpu_load = 0;
QueueHandle_t eventQueue;
TaskHandle_t but1_handler            = NULL;
TaskHandle_t but2_handler            = NULL;
TaskHandle_t periodic_handler        = NULL;
TaskHandle_t uartRecv_handler        = NULL;
TaskHandle_t load1_handler           = NULL;
TaskHandle_t load2_handler           = NULL;


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );

/*-----------------------------------------------------------*/

void Button_1_Monitor( void * pvParameters )
{
	pinState_t but1_prevState = PIN_IS_HIGH;  
	pinState_t but1_nextState;            
	char* but1_event = NULL;   
	TickType_t prevWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
	for( ;; )
	{
		but1_nextState = GPIO_read(PORT_0, PIN1);

		if ( but1_nextState != but1_prevState )
		{
			
			if ( but1_nextState == PIN_IS_HIGH )
			{
				if ( eventQueue != 0 )  /* Make sure handler points to a queue */
				{
					but1_event = "Rising Edge on Button 1 \n";
					xQueueSend( eventQueue, ( void * ) &but1_event, ( TickType_t ) 10 );   /* Can wait 10 ticks for space to be available */
				}
			}
			else
			{

				if ( eventQueue != 0 )
				{
					but1_event = "Falling Edge on Button 1 \n";
					xQueueSend( eventQueue, ( void * ) &but1_event, ( TickType_t ) 10 );    /* Can wait 10 ticks for space to be available */
				}
			}
		}
		but1_prevState = but1_nextState;
		GPIO_write (PORT_0, PIN4, PIN_IS_LOW);
		vTaskDelayUntil( &prevWakeTime, 50 ); /* Periodicity is 50 */
		GPIO_write (PORT_0, PIN4, PIN_IS_HIGH);
	}
}

void Button_2_Monitor( void * pvParameters )
{
	pinState_t but2_prevState = PIN_IS_HIGH;  
	pinState_t but2_nextState;            
	char* but2_event = NULL;   
	TickType_t prevWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 2 );

	for( ;; )
	{
		but2_nextState = GPIO_read(PORT_0, PIN2);

		if ( but2_nextState != but2_prevState )
		{
			
			if ( but2_nextState == PIN_IS_HIGH )
			{
				if ( eventQueue != 0 )  /* Make sure handler points to a queue */
				{
					but2_event = "Rising Edge on Button 2 \n";
					xQueueSend( eventQueue, ( void * ) &but2_event, ( TickType_t ) 10 );   /* Can wait 10 ticks for space to be available */
				}
			}
			else
			{

				if ( eventQueue != 0 )
				{
					but2_event = "Falling Edge on Button 2 \n";
					xQueueSend( eventQueue, ( void * ) &but2_event, ( TickType_t ) 10 );    /* Can wait 10 ticks for space to be available */
				}
			}
		}
		but2_prevState = but2_nextState;
		GPIO_write (PORT_0, PIN5, PIN_IS_LOW);
		vTaskDelayUntil( &prevWakeTime, 50 ); /* Periodicity is 50 */
		GPIO_write (PORT_0, PIN5, PIN_IS_HIGH);
	}
}



void Periodic_Transmitter( void * pvParameters )
{
	char* msg = NULL;   
	TickType_t prevWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 3 );
	for( ;; )
	{

		if ( eventQueue != 0 )
		{
			msg = "Periodic String \n";
			xQueueSend( eventQueue, ( void * ) &msg, ( TickType_t ) 10 );
		}
		GPIO_write (PORT_0, PIN6, PIN_IS_LOW);
		vTaskDelayUntil( &prevWakeTime, 100 );
		GPIO_write (PORT_0, PIN6, PIN_IS_HIGH);
	}
}

void Uart_Receiver( void * pvParameters )
{   
	char* recv_msg = NULL;   
	TickType_t prevWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 4 );
	for( ;; )
	{

		if( xQueueReceive( eventQueue,&(recv_msg),( TickType_t ) 10 ) == pdPASS )
		{
			vSerialPutString((const signed char * const)(recv_msg), strlen(recv_msg) );
		}
		GPIO_write (PORT_0, PIN7, PIN_IS_LOW);
		vTaskDelayUntil( &prevWakeTime, 20 );
		GPIO_write (PORT_0, PIN7, PIN_IS_HIGH);
	}
}



void Load_1_Simulation( void * pvParameters )
{
	int i;
	TickType_t prevWakeTime = xTaskGetTickCount();
  vTaskSetApplicationTaskTag( NULL, ( void * ) 5 );
	for( ;; )
	{
		GPIO_write(PORT_0, PIN1, PIN_IS_HIGH);
		for ( i=0; i < 32000; i++ )
		{
			i=i;
		}
		GPIO_write (PORT_0, PIN8, PIN_IS_LOW);
		vTaskDelayUntil( &prevWakeTime, 10 );
		GPIO_write (PORT_0, PIN8, PIN_IS_HIGH);
	}
}


void Load_2_Simulation( void * pvParameters )
{
	int i;
	TickType_t prevWakeTime = xTaskGetTickCount();
	vTaskSetApplicationTaskTag( NULL, ( void * ) 6 );
	for( ;; )
	{
		for ( i=0; i < 91000; i++ )
		{
			i=i;
		}
		GPIO_write (PORT_0, PIN9, PIN_IS_LOW);
		vTaskDelayUntil( &prevWakeTime, 100 );
		GPIO_write (PORT_0, PIN9, PIN_IS_HIGH);
	}
}


/* Implement tick hook */
void vApplicationTickHook(void)
{
	GPIO_write(PORT_0, PIN0, PIN_IS_HIGH);
	GPIO_write(PORT_0, PIN0, PIN_IS_LOW);
	
}

/* Implement idle hook */
void vApplicationIdleHook(void)
{
	
	
}


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	eventQueue = xQueueCreate( 20, sizeof( char* ) );

	/* Create Tasks here */

	/* Create the task, storing the handle. */
	xTaskPeriodicCreate(
			Button_1_Monitor,                  /* Function that implements the task. */
			"BUTTON 1 MONITOR",                /* Text name for the task. */
			100,                               /* Stack size in words, not bytes. */
			( void * ) 0,                      /* Parameter passed into the task. */
			1,                                 /* Priority at which the task is created. */
			&but1_handler,       /* Used to pass out the created task's handle. */
			50);     /* Period for the task */

	/* Create the task, storing the handle. */
	xTaskPeriodicCreate(
			Button_2_Monitor,                  /* Function that implements the task. */
			"BUTTON 2 MONITOR",                /* Text name for the task. */
			100,                               /* Stack size in words, not bytes. */
			( void * ) 0,                      /* Parameter passed into the task. */
			1,                                 /* Priority at which the task is created. */
			&but2_handler,       /* Used to pass out the created task's handle. */
			50);     /* Period for the task */

	/* Create the task, storing the handle. */
	xTaskPeriodicCreate(
			Periodic_Transmitter,               /* Function that implements the task. */
			"PERIODIC TRANSMITTER",             /* Text name for the task. */
			100,                                /* Stack size in womain.crds, not bytes. */
			( void * ) 0,                       /* Parameter passed into the task. */
			1,                                  /* Priority at which the task is created. */
			&periodic_handler,   /* Used to pass out the created task's handle. */
			100);  /* Period for the task */

	/* Create the task, storing the handle. */
	xTaskPeriodicCreate(
			Uart_Receiver,                      /* Function that implements the task. */
			"UART RECEIVER",                    /* Text name for the task. */
			100,                                /* Stack size in words, not bytes. */
			( void * ) 0,                       /* Parameter passed into the task. */
			1,                                  /* Priority at which the task is created. */
			&uartRecv_handler,          /* Used to pass out the created task's handle. */
			20);         /* Period for the task */

			
	/* Create the task, storing the handle. */
	xTaskPeriodicCreate(
			Load_1_Simulation,                 /* Function that implements the task. */
			"LOAD 1 SIMULATION",               /* Text name for the task. */
			100,                               /* Stack size in words, not bytes. */
			( void * ) 0,                      /* Parameter passed into the task. */
			1,                                 /* Priority at which the task is created. */
			&load1_handler,      /* Used to pass out the created task's handle. */
			10);	   /* Period for the task */

	/* Create the task, storing the handle. */
	xTaskPeriodicCreate(
			Load_2_Simulation,                 /* Function that implements the task. */
			"LOAD 2 SIMULATION",               /* Text name for the task. */
			100,                               /* Stack size in words, not bytes. */
			( void * ) 0,                      /* Parameter passed into the task. */
			1,                                 /* Priority at which the task is created. */
			&load2_handler,      /* Used to pass out the created task's handle. */
				100); 	 /* Period for the task */

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

static void ConfigTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);
	/* Configure Timer */
	ConfigTimer1();
	/* Configure GPIO */
	GPIO_init();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/

