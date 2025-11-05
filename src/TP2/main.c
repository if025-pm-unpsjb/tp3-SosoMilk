/*************************************************************************
 * Programa de prueba basado para LM3S6965.
 *
 * Basado en el demo provisto por FreeRTOS, para ejecutar sobre QEMU.
 *
 * Please ensure to read http://www.freertos.org/portlm3sx965.html
 * which provides information on configuring and running this demo for the
 * various Luminary Micro EKs.
 *************************************************************************/

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Hardware library includes. */
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_sysctl.h"
#include "hw_uart.h"
#include "sysctl.h"
#include "gpio.h"
#include "grlib.h"
#include "osram128x64x4.h"
#include "uart.h"
#include "bitmap.h"

/*-----------------------------------------------------------*/

/* Dimensions the buffer for text messages. */
#define mainMAX_MSG_LEN                     25

/* Constants used when writing strings to the display. */
#define mainCHARACTER_HEIGHT                ( 9 )
#define mainMAX_ROWS_128                    ( mainCHARACTER_HEIGHT * 14 )
#define mainMAX_ROWS_96                     ( mainCHARACTER_HEIGHT * 10 )
#define mainMAX_ROWS_64                     ( mainCHARACTER_HEIGHT * 7 )
#define mainFULL_SCALE                      ( 15 )
#define ulSSI_FREQUENCY                     ( 3500000UL )
#define restTiempo							100

/* Tasks periods. SISTEMA 1*/
#define SYS1_TASK1_PERIOD 	4000
#define SYS1_TASK2_PERIOD 	5000
#define SYS1_TASK3_PERIOD 	8000

/* Tasks WCETs. */
#define SYS1_TASK1_WCET		1000
#define SYS1_TASK2_WCET		1000
#define SYS1_TASK3_WCET		2000

//-------------------------------------------------------------

/* Tasks periods. SISTEMA 2*/
#define SYS2_TASK1_PERIOD 	4000
#define SYS2_TASK2_PERIOD 	8000
#define SYS2_TASK3_PERIOD 	9000

/* Tasks WCETs. */
#define SYS2_TASK1_WCET		1000
#define SYS2_TASK2_WCET		1000
#define SYS2_TASK3_WCET		2000

/*-----------------------------------------------------------*/

/* Tasks periods. SISTEMA 3*/
#define SYS3_TASK1_PERIOD 	4000
#define SYS3_TASK2_PERIOD 	6000
#define SYS3_TASK3_PERIOD 	8000
#define SYS3_TASK4_PERIOD 	13000

/* Tasks WCETs. */
#define SYS3_TASK1_WCET		1000
#define SYS3_TASK2_WCET		1000
#define SYS3_TASK3_WCET		1000
#define SYS3_TASK4_WCET		3000

/*-----------------------------------------------------------*/

/* Tasks periods. SISTEMA 4*/
#define SYS4_TASK1_PERIOD 	4000
#define SYS4_TASK2_PERIOD 	7000
#define SYS4_TASK3_PERIOD 	12000
#define SYS4_TASK4_PERIOD 	14000

/* Tasks WCETs. */
#define SYS4_TASK1_WCET		1000
#define SYS4_TASK2_WCET		2000
#define SYS4_TASK3_WCET		1000
#define SYS4_TASK4_WCET		2000

/*-----------------------------------------------------------*/

/* Tasks periods. SISTEMA 5*/
#define SYS5_TASK1_PERIOD 	5000
#define SYS5_TASK2_PERIOD 	9000
#define SYS5_TASK3_PERIOD 	10000
#define SYS5_TASK4_PERIOD 	15000
#define SYS5_TASK5_PERIOD 	16000

/* Tasks WCETs. */
#define SYS5_TASK1_WCET		1000
#define SYS5_TASK2_WCET		1000
#define SYS5_TASK3_WCET		1000
#define SYS5_TASK4_WCET		2000
#define SYS5_TASK5_WCET		2000

/*-----------------------------------------------------------*/

/* Tasks periods. SISTEMA 6*/
#define SYS6_TASK1_PERIOD 	6000
#define SYS6_TASK2_PERIOD 	8000
#define SYS6_TASK3_PERIOD 	11000
#define SYS6_TASK4_PERIOD 	15000
#define SYS6_TASK5_PERIOD 	17000

/* Tasks WCETs. */
#define SYS6_TASK1_WCET		1000
#define SYS6_TASK2_WCET		1000
#define SYS6_TASK3_WCET		2000
#define SYS6_TASK4_WCET		1000
#define SYS6_TASK5_WCET		2000

/*-----------------------------------------------------------*/

/* Tasks periods. SISTEMA 7*/
#define SYS7_TASK1_PERIOD 	4000
#define SYS7_TASK2_PERIOD 	7000
#define SYS7_TASK3_PERIOD 	10000
#define SYS7_TASK4_PERIOD 	14000

/* Tasks WCETs. */
#define SYS7_TASK1_WCET		1000
#define SYS7_TASK2_WCET		1000
#define SYS7_TASK3_WCET		2000
#define SYS7_TASK4_WCET		2000

/*-----------------------------------------------------------*/

/* Tasks periods. SISTEMA 8*/
#define SYS8_TASK1_PERIOD 	4000
#define SYS8_TASK2_PERIOD 	8000
#define SYS8_TASK3_PERIOD 	10000
#define SYS8_TASK4_PERIOD 	14000

/* Tasks WCETs. */
#define SYS8_TASK1_WCET		1000
#define SYS8_TASK2_WCET		1000
#define SYS8_TASK3_WCET		2000
#define SYS8_TASK4_WCET		2000

/*-----------------------------------------------------------*/

/* Tasks periods. SISTEMA 9*/
#define SYS9_TASK1_PERIOD 	5000
#define SYS9_TASK2_PERIOD 	8000
#define SYS9_TASK3_PERIOD 	12000
#define SYS9_TASK4_PERIOD 	15000
#define SYS9_TASK5_PERIOD 	16000

/* Tasks WCETs. */
#define SYS9_TASK1_WCET		1000
#define SYS9_TASK2_WCET		1000
#define SYS9_TASK3_WCET		2000
#define SYS9_TASK4_WCET		1000
#define SYS9_TASK5_WCET		1000

/*-----------------------------------------------------------*/

/* Tasks periods. SISTEMA 10*/
#define SYS10_TASK1_PERIOD 	5000
#define SYS10_TASK2_PERIOD 	8000
#define SYS10_TASK3_PERIOD 	12000
#define SYS10_TASK4_PERIOD 	13000
#define SYS10_TASK5_PERIOD 	16000

/* Tasks WCETs. */
#define SYS10_TASK1_WCET	1000
#define SYS10_TASK2_WCET	1000
#define SYS10_TASK3_WCET	1000
#define SYS10_TASK4_WCET	1000
#define SYS10_TASK5_WCET	2000

/*-----------------------------------------------------------*/

#define seleccion system10

/*
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );

/*
 * Basic polling UART write function.
 */
static void prvPrintString( const char * pcString );

/*
 * Busy wait the specified number of ticks.
 */
static void vBusyWait( TickType_t ticks );

/*
 * Periodic task.
 */
static void prvTask( void* pvParameters );

/*-----------------------------------------------------------*/

/* Functions to access the OLED.  The one used depends on the dev kit
being used. */
void ( *vOLEDInit )( uint32_t ) = NULL;
void ( *vOLEDStringDraw )( const char *, uint32_t, uint32_t, unsigned char ) = NULL;
void ( *vOLEDImageDraw )( const unsigned char *, uint32_t, uint32_t, uint32_t, uint32_t ) = NULL;
void ( *vOLEDClear )( void ) = NULL;

/*-----------------------------------------------------------*/

struct xTaskStruct {
	TickType_t wcet;
	TickType_t period;
};

typedef struct xTaskStruct xTask;

xTask task1 = { SYS1_TASK1_WCET, SYS1_TASK1_PERIOD };
xTask task2 = { SYS1_TASK2_WCET, SYS1_TASK2_PERIOD };
xTask task3 = { SYS1_TASK3_WCET, SYS1_TASK3_PERIOD };

xTask system1[] = {
    { SYS1_TASK1_WCET, SYS1_TASK1_PERIOD },
    { SYS1_TASK2_WCET, SYS1_TASK2_PERIOD },
    { SYS1_TASK3_WCET, SYS1_TASK3_PERIOD }
};

xTask system2[] = {
    { SYS2_TASK1_WCET, SYS2_TASK1_PERIOD },
    { SYS2_TASK2_WCET, SYS2_TASK2_PERIOD },
    { SYS2_TASK3_WCET, SYS2_TASK3_PERIOD }
};

xTask system3[] = {
    { SYS3_TASK1_WCET, SYS3_TASK1_PERIOD },
    { SYS3_TASK2_WCET, SYS3_TASK2_PERIOD },
    { SYS3_TASK3_WCET, SYS3_TASK3_PERIOD },
    { SYS3_TASK4_WCET, SYS3_TASK4_PERIOD }
};

xTask system4[] = {
    { SYS4_TASK1_WCET, SYS4_TASK1_PERIOD },
    { SYS4_TASK2_WCET, SYS4_TASK2_PERIOD },
    { SYS4_TASK3_WCET, SYS4_TASK3_PERIOD },
    { SYS4_TASK4_WCET, SYS4_TASK4_PERIOD }
};

xTask system5[] = {
    { SYS5_TASK1_WCET, SYS5_TASK1_PERIOD },
    { SYS5_TASK2_WCET, SYS5_TASK2_PERIOD },
    { SYS5_TASK3_WCET, SYS5_TASK3_PERIOD },
    { SYS5_TASK4_WCET, SYS5_TASK4_PERIOD },
    { SYS5_TASK5_WCET, SYS5_TASK5_PERIOD }
};

xTask system6[] = {
    { SYS6_TASK1_WCET, SYS6_TASK1_PERIOD },
    { SYS6_TASK2_WCET, SYS6_TASK2_PERIOD },
    { SYS6_TASK3_WCET, SYS6_TASK3_PERIOD },
    { SYS6_TASK4_WCET, SYS6_TASK4_PERIOD },
    { SYS6_TASK5_WCET, SYS6_TASK5_PERIOD }
};

xTask system7[] = {
    { SYS7_TASK1_WCET, SYS7_TASK1_PERIOD },
    { SYS7_TASK2_WCET, SYS7_TASK2_PERIOD },
    { SYS7_TASK3_WCET, SYS7_TASK3_PERIOD },
    { SYS7_TASK4_WCET, SYS7_TASK4_PERIOD }
};

xTask system8[] = {
    { SYS8_TASK1_WCET, SYS8_TASK1_PERIOD },
    { SYS8_TASK2_WCET, SYS8_TASK2_PERIOD },
    { SYS8_TASK3_WCET, SYS8_TASK3_PERIOD },
    { SYS8_TASK4_WCET, SYS8_TASK4_PERIOD }
};

xTask system9[] = {
    { SYS9_TASK1_WCET, SYS9_TASK1_PERIOD },
    { SYS9_TASK2_WCET, SYS9_TASK2_PERIOD },
    { SYS9_TASK3_WCET, SYS9_TASK3_PERIOD },
    { SYS9_TASK4_WCET, SYS9_TASK4_PERIOD },
    { SYS9_TASK5_WCET, SYS9_TASK5_PERIOD }
};

xTask system10[] = {
    { SYS10_TASK1_WCET, SYS10_TASK1_PERIOD },
    { SYS10_TASK2_WCET, SYS10_TASK2_PERIOD },
    { SYS10_TASK3_WCET, SYS10_TASK3_PERIOD },
    { SYS10_TASK4_WCET, SYS10_TASK4_PERIOD },
    { SYS10_TASK5_WCET, SYS10_TASK5_PERIOD }
};

/*************************************************************************
 * Main
 *************************************************************************/
int main( void )
{
	/* Initialise the trace recorder. */
	vTraceEnable( TRC_INIT );

    prvSetupHardware();

    /* Map the OLED access functions to the driver functions that are appropriate
    for the evaluation kit being used. */
    //configASSERT( ( HWREG( SYSCTL_DID1 ) & SYSCTL_DID1_PRTNO_MASK ) == SYSCTL_DID1_PRTNO_6965 );
    vOLEDInit = OSRAM128x64x4Init;
    vOLEDStringDraw = OSRAM128x64x4StringDraw;
    vOLEDImageDraw = OSRAM128x64x4ImageDraw;
    vOLEDClear = OSRAM128x64x4Clear;

    /* Initialise the OLED and display a startup message. */
    vOLEDInit( ulSSI_FREQUENCY );

    /* Print Hello World! to the OLED display. */
    static char cMessage[ mainMAX_MSG_LEN ];
    sprintf(cMessage, "Hello World!");
    vOLEDStringDraw( cMessage, 0, 0, mainFULL_SCALE );

    /* Print "Start!" to the UART. */
    prvPrintString("Start!\n\r");

    xTask* selectedSystem = seleccion;
	int systemSize = sizeof(seleccion) / sizeof(xTask);

	for (int i = 0; i < systemSize; i++) {
		char name[10];
		int n = i + 1;
		sprintf(name, "T%d", i + 1);
		xTaskCreate(prvTask, name, configMINIMAL_STACK_SIZE + 50, &selectedSystem[i],
					configMAX_PRIORITIES - n, NULL);
	}

    /* Creates the periodic tasks. */
    //funcion que va a ejecutar, nombre, pila, puntero del c y el periodo, 6 prioridad mas alta y le va restando, no lo usamos aun es un puntero que permite referenciar
    //xTaskCreate( prvTask, "T1", configMINIMAL_STACK_SIZE + 50, (void*) &task1, configMAX_PRIORITIES - 1, NULL );
    //xTaskCreate( prvTask, "T2", configMINIMAL_STACK_SIZE + 50, (void*) &task2, configMAX_PRIORITIES - 2, NULL );
    //xTaskCreate( prvTask, "T3", configMINIMAL_STACK_SIZE + 50, (void*) &task3, configMAX_PRIORITIES - 3, NULL );

    vTraceEnable( TRC_START );

    /* Launch the scheduler. */
    vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task. */
    for( ;; );
}
/*-----------------------------------------------------------*/

void prvSetupHardware( void )
{
    /* If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.  This is
    a workaround to allow the PLL to operate reliably. */
    if( DEVICE_IS_REVA2 )
    {
        SysCtlLDOSet( SYSCTL_LDO_2_75V );
    }

    /* Set the clocking to run from the PLL at 50 MHz */
    SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ );

    /* Initialise the UART - QEMU usage does not seem to require this
    initialisation. */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_UART0 );
    UARTEnable( UART0_BASE );
}
/*-----------------------------------------------------------*/

static void prvPrintString( const char * pcString )
{
    while( *pcString != 0x00 )
    {
        UARTCharPut( UART0_BASE, *pcString );
        pcString++;
    }
}
/*-----------------------------------------------------------*/

static void vBusyWait( TickType_t ticks )
{
    TickType_t elapsedTicks = 0;
    TickType_t currentTick = 0;
    while ( elapsedTicks < ticks ) {
        currentTick = xTaskGetTickCount();
        while ( currentTick == xTaskGetTickCount() ) {
            asm("nop");
        }
        elapsedTicks++;
    }
}
/*-----------------------------------------------------------*/

void prvTask( void *pvParameters )
{
	char cMessage[ mainMAX_MSG_LEN ];
	unsigned int uxReleaseCount = 0;
	TickType_t pxPreviousWakeTime = 0; //0
	xTask *task = (xTask*) pvParameters;

	for( ;; )
	{

		snprintf(cMessage, sizeof(cMessage), "S %s - %u - %u\n\r", pcTaskGetTaskName(NULL), uxReleaseCount, (unsigned) xTaskGetTickCount());
		prvPrintString(cMessage); //imprime por pantalla

        vBusyWait( task->wcet - restTiempo); //imprimir de vuelta antes de terminar

        snprintf(cMessage, sizeof(cMessage), "E %s - %u - %u\n\r", pcTaskGetTaskName(NULL), uxReleaseCount, (unsigned) xTaskGetTickCount());
        prvPrintString(cMessage);

		vTaskDelayUntil( &pxPreviousWakeTime, task->period );

		uxReleaseCount += 1;
	}

	vTaskDelete( NULL );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char *pcFile, uint32_t ulLine )
{
    volatile uint32_t ulSetTo1InDebuggerToExit = 0;
    {
        while( ulSetTo1InDebuggerToExit == 0 )
        {
            /* Nothing to do here.  Set the loop variable to a non zero value in
            the debugger to step out of this function to the point that caused
            the assertion. */
            ( void ) pcFile;
            ( void ) ulLine;
        }
    }
}

char* _sbrk_r (struct _reent *r, int incr)
{
    /* Just to keep the linker quiet. */
    ( void ) r;
    ( void ) incr;

    /* Check this function is never called by forcing an assert() if it is. */
    //configASSERT( incr == -1 );

    return NULL;
}

int __error__(char *pcFilename, unsigned long ulLine) {
    return 0;
}

