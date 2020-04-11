//*****************************************************************************
//
// Codigo de partida comunicacion TIVA-QT (Marzo2020)
// Autores: Eva Gonzalez, Ignacio Herrero, Jose Manuel Cano
//
//  Estructura de aplicacion basica para el desarrollo de aplicaciones genericas
//  basada en la TIVA, en las que existe un intercambio de comandos con un interfaz
//  gráfico (GUI) Qt.
//  La aplicacion se basa en un intercambio de comandos con ordenes e informacion, a traves  de la
//  configuracion de un perfil CDC de USB (emulacion de puerto serie) y un protocolo
//  de comunicacion con el PC que permite recibir ciertas ordenes y enviar determinados datos en respuesta.
//   En el ejemplo basico de partida se implementara la recepcion de un comando
//  generico que permite el apagado y encendido de los LEDs de la placa; asi como un segundo
//  comando enviado desde la placa al GUI, para mostrar el estado de los botones.
//
//*****************************************************************************
#include<stdbool.h>
#include<stdint.h>
#include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
#include "inc/hw_types.h"        // TIVA: Definiciones API
#include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
#include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip
#include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/rom_map.h"   // TIVA: Mapeo automatico de funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
#include "driverlib/uart.h"      // TIVA: Funciones API manejo UART
#include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
#include "driverlib/adc.h"       // TIVA: Funciones API manejo de ADC
#include "driverlib/timer.h"     // TIVA: Funciones API manejo de timers
#include <utils/uartstdioMod.h>     // TIVA: Funciones API UARTSTDIO (printf)
#include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
#include "drivers/rgb.h"         // TIVA: Funciones API manejo de leds con PWM
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
#include "semphr.h"              // FreeRTOS: definiciones relacionadas con semaforos
#include "queue.h"               // FreeRTOS: definiciones relacionadas con colas de mensajes
#include "utils/cpu_usage.h"
#include "commands.h"
#include <serial2USBprotocol.h>
#include <usb_dev_serial.h>
#include <usb_commands_table.h>


#define LED1TASKPRIO 1
#define LED1TASKSTACKSIZE 128

// Variables globales "main"
uint32_t g_ui32CPUUsage;
uint32_t g_ui32SystemClock;
SemaphoreHandle_t mutexUSB;
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static uint8_t ui8Count = 0;

	if (++ui8Count == 10)
	{
		g_ui32CPUUsage = CPUUsageTick();
		ui8Count = 0;
	}
	//return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
	SysCtlSleep();
}


//Esto se ejecuta cada vez que se produce un fallo de asignacio de heap
void vApplicationMallocFailedHook (void)
{
	while(1);
}



//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************

//// Codigo para procesar los comandos recibidos a traves del canal USB

static portTASK_FUNCTION( CommandProcessingTask, pvParameters ){

    uint8_t pui8Frame[MAX_FRAME_SIZE];	//Ojo, esto hace que esta tarea necesite bastante pila
    int32_t i32Numdatos;
    uint8_t ui8Command;
    void *ptrtoreceivedparam; // <---?
	uint32_t ui32Errors=0;

	/* The parameters are not used. */
	( void ) pvParameters;

	//
	// Mensaje de bienvenida inicial.
	//
	UARTprintf("\n\nBienvenido a la aplicacion XXXXX (curso 2019/20)!\n");
	UARTprintf("\nAutores: XXXXXX y XXXXX ");

	for(;;)
	{
        //Espera hasta que se reciba una trama con datos serializados por el interfaz USB
		i32Numdatos=receive_frame(pui8Frame,MAX_FRAME_SIZE); //Esta funcion es bloqueante
		if (i32Numdatos>0)
		{	//Si no hay error, proceso la trama que ha llegado.
			i32Numdatos=destuff_and_check_checksum(pui8Frame,i32Numdatos); // Primero, "destuffing" y comprobación checksum
			if (i32Numdatos<0)
			{
				//Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
				ui32Errors++;
                // Procesamiento del error (TODO, POR HACER!!)
			}
			else
			{
				//El paquete esta bien, luego procedo a tratarlo.
                //Obtiene el valor del campo comando
				ui8Command=decode_command_type(pui8Frame);
                //Obtiene un puntero al campo de parametros y su tamanio.
                i32Numdatos=get_command_param_pointer(pui8Frame,i32Numdatos,&ptrtoreceivedparam);
				switch(ui8Command)
				{
				case COMANDO_PING :
					//A un comando de ping se responde con el propio comando
					i32Numdatos=create_frame(pui8Frame,ui8Command,0,0,MAX_FRAME_SIZE);
					if (i32Numdatos>=0)
					{
					    xSemaphoreTake(mutexUSB,portMAX_DELAY);
						send_frame(pui8Frame,i32Numdatos);
						xSemaphoreGive(mutexUSB);
					}else{
						//Error de creacion de trama: determinar el error y abortar operacion
						ui32Errors++;
						// Procesamiento del error (TODO)
//						// Esto de aqui abajo podria ir en una funcion "createFrameError(numdatos)  para evitar
//						// tener que copiar y pegar todo en cada operacion de creacion de paquete
						switch(i32Numdatos){
						case PROT_ERROR_NOMEM:
							// Procesamiento del error NO MEMORY (TODO)
							break;
						case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
//							// Procesamiento del error STUFFED_FRAME_TOO_LONG (TODO)
							break;
						case PROT_ERROR_COMMAND_TOO_LONG:
//							// Procesamiento del error COMMAND TOO LONG (TODO)
							break;
						}
                        case PROT_ERROR_INCORRECT_PARAM_SIZE:
                        {
                            // Procesamiento del error INCORRECT PARAM SIZE (TODO)
                        }
                        break;
					}
					break;

				default:
				 {
					PARAM_COMANDO_NO_IMPLEMENTADO parametro;
					parametro.command=ui8Command;
					//El comando esta bien pero no esta implementado
					i32Numdatos=create_frame(pui8Frame,COMANDO_NO_IMPLEMENTADO,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
					if (i32Numdatos>=0)
					{
						send_frame(pui8Frame,i32Numdatos);
					}
					break;
				 }
				}// switch
			}
		}else{ // if (ui32Numdatos >0)
			//Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
			ui32Errors++;
			// Procesamiento del error (TODO)
		}
	}
}

// Codigo de tarea de ejemplo: eliminar para la aplicacion final
static portTASK_FUNCTION(LEDTask,pvParameters)
						{

	bool  fLedOn=false;

	//
	// Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
	//
	while(1)
	{
		fLedOn=!fLedOn;

		if (fLedOn)
		{
			MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 , GPIO_PIN_1);
			vTaskDelay(0.1*configTICK_RATE_HZ);        //Espera del RTOS (eficiente, no gasta CPU)
			//Esta espera es de unos 100ms aproximadamente.
		}
		else
		{
		    MAP_GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1,0);
			vTaskDelay(2*configTICK_RATE_HZ);        //Espera del RTOS (eficiente, no gasta CPU)
			//Esta espera es de unos 2s aproximadamente.
		}
	}
						}

//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{

	//
	// Set the clocking to run at 40 MHz from the PLL.
	//
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
			SYSCTL_OSC_MAIN);	//Ponermos el reloj principal a 50 MHz (200 Mhz del Pll dividido por 4)


	// Get the system clock speed.
	g_ui32SystemClock = SysCtlClockGet();


	//Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
	//                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
	MAP_SysCtlPeripheralClockGating(true);

	// Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
	// Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
	// (y por tanto este no se deberia utilizar para otra cosa).
	CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ/10, 3);

	//Inicializa el puerto F (LEDs)
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);	//LEDS APAGADOS

	//Inicializa la biblioteca RGB (sin configurar las salidas como RGB)
	RGBInit(0);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);  //Esto es necesario para que el timer0 siga funcionando en bajo consumo
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);  //Esto es necesario para que el timer1 siga funcionando en bajo consumo

	/**                                              Creacion de tareas 									**/

	// Inicializa el sistema de depuraci�n por terminal UART
	if (initCommandLine(512,tskIDLE_PRIORITY + 1) != pdTRUE)
    {
        while(1);
    }

	USBSerialInit(32,32);	//Inicializo el  sistema USB
	//
	// Crea la tarea que gestiona los comandos USB (definidos en CommandProcessingTask)
	//
	if(xTaskCreate(CommandProcessingTask, (portCHAR *)"usbser",512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE)
	{
		while(1);
	}

	//
	// Ejemplo de creacion de una tarea que parpadea el LED ROJO -> quitar en la aplicacion final
	//
	if((xTaskCreate(LEDTask, "Led1", LED1TASKSTACKSIZE,NULL,tskIDLE_PRIORITY + LED1TASKPRIO, NULL) != pdTRUE))
	{
		while(1);
	}

    /**                                             Creacion de recursos IPC                                            **/
    // Mutex para proteccion del canal USB (por si 2 tareas deciden usarlo a la vez!)
    mutexUSB=xSemaphoreCreateMutex();
    if(mutexUSB==NULL) // Error de creacion de semaforos
        while(1);


	//
	// Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
	//
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

	//De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}
