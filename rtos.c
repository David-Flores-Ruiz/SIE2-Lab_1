/**
 * @file rtos.c
 * @author ITESO
 * @date Feb 2018
 * @brief Implementation of rtos API
 *
 * This is the implementation of the rtos module for the
 * embedded systems II course at ITESO
 */

#include "rtos.h"
#include "rtos_config.h"
#include "clock_config.h"

#ifdef RTOS_ENABLE_IS_ALIVE
#include "fsl_gpio.h"
#include "fsl_port.h"
#endif
/**********************************************************************************/
// Module defines
/**********************************************************************************/

#define FORCE_INLINE 	__attribute__((always_inline)) inline

#define STACK_FRAME_SIZE			8
#define STACK_LR_OFFSET				2
#define STACK_PSR_OFFSET			1
#define STACK_PSR_DEFAULT			0x01000000

/**********************************************************************************/
// IS ALIVE definitions
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
#define CAT_STRING(x,y)  		x##y
#define alive_GPIO(x)			CAT_STRING(GPIO,x)
#define alive_PORT(x)			CAT_STRING(PORT,x)
#define alive_CLOCK(x)			CAT_STRING(kCLOCK_Port,x)
static void init_is_alive(void);
static void refresh_is_alive(void);
#endif

/**********************************************************************************/
// Type definitions
/**********************************************************************************/

typedef enum
{
	S_READY = 0, S_RUNNING, S_WAITING, S_SUSPENDED
} task_state_e;

typedef enum
{
	kFromISR = 0, kFromNormalExec
} task_switch_type_e;

typedef struct
{
	uint8_t priority;
	task_state_e state;
	uint32_t *sp;
	void (*task_body)();
	rtos_tick_t local_tick;
	uint32_t reserved[10];
	uint32_t stack[RTOS_STACK_SIZE];
} rtos_tcb_t;

/**********************************************************************************/
// Global (static) task list
/**********************************************************************************/

struct
{
	uint8_t nTasks;
	rtos_task_handle_t current_task;
	rtos_task_handle_t next_task;
	rtos_tcb_t tasks[RTOS_MAX_NUMBER_OF_TASKS + 1];
	rtos_tick_t global_tick;
} task_list =
{ 0 };

/**********************************************************************************/
// Local methods prototypes
/**********************************************************************************/

static void reload_systick(void);
static void dispatcher(task_switch_type_e type);
static void activate_waiting_tasks();
FORCE_INLINE static void context_switch(task_switch_type_e type);
static void idle_task(void);

/**********************************************************************************/
// API implementation
/**********************************************************************************/

void rtos_start_scheduler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	init_is_alive();
#endif
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
	        | SysTick_CTRL_ENABLE_Msk;
	reload_systick();

	task_list.global_tick = 0;			 		/* Poner el reloj global en 0 */
	rtos_create_task(idle_task, 0, kAutoStart);	/* Crear tarea IDLE */

	for (;;)
		;
}

rtos_task_handle_t rtos_create_task(void (*task_body)(), uint8_t priority,
		rtos_autostart_e autostart)
{
	rtos_tcb_t new_task;
	rtos_task_handle_t task_handle;

	if (task_list.nTasks < RTOS_MAX_NUMBER_OF_TASKS)
	{
		if (autostart == kAutoStart) {
			new_task.state = S_READY;
		}

		if (autostart == kStartSuspended) {
			new_task.state = S_SUSPENDED;
		}

		new_task.priority = priority;
		new_task.sp = &(task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE - STACK_FRAME_SIZE - 1]);
		new_task.task_body = task_body;
		new_task.local_tick = 0;	   /* Reloj local de la tarea en 0 */
//		new_task.reserved =
		new_task.stack[RTOS_STACK_SIZE - STACK_PSR_OFFSET] = STACK_PSR_DEFAULT;
		new_task.stack[RTOS_STACK_SIZE - STACK_LR_OFFSET] = (uint32_t) task_body;

		task_list.tasks[task_list.nTasks] = new_task;  /* Insertamos la nueva tarea a la lista*/

		task_handle = task_list.nTasks;/* Para retornar el contador actual de la tarea creada */
		task_list.nTasks++;			   /* Incrementa indice para insertar la siguiente tarea */

		return task_handle;	/* Retornamos el indice de la tarea nueva */
	}
	return -1;				/* Tarea invalida */
}

rtos_tick_t rtos_get_clock(void)
{
	rtos_tick_t global_tick = task_list.global_tick;
	return global_tick;		/* Retorna el reloj del sistema */
}

void rtos_delay(rtos_tick_t ticks)
{
	task_list.tasks[task_list.current_task].state = S_WAITING;		/* La actual tarea en ESPERA */
	task_list.tasks[task_list.current_task].local_tick = ticks; 	/* Asigna tick al reloj local de la tarea */
	dispatcher(kFromNormalExec);	// kFromISR, kFromNormalExec 	/* Llama dispatcher (desde la tarea) */
}

void rtos_suspend_task(void)
{
	task_list.tasks[task_list.current_task].state = S_SUSPENDED;	/* La actual tarea en SUSPENDIDA */
	dispatcher(kFromNormalExec);	// kFromISR, kFromNormalExec 	/* Llama dispatcher (desde la tarea) */
}

void rtos_activate_task(rtos_task_handle_t task)
{
	task_list.tasks[task_list.current_task].state = S_READY;		/* La actual tarea en LISTO */
	dispatcher(kFromNormalExec);	// kFromISR, kFromNormalExec 	/* Llama dispatcher (desde la tarea) */
}

/**********************************************************************************/
// Local methods implementation
/**********************************************************************************/

static void reload_systick(void)
{
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
}

static void dispatcher(task_switch_type_e type)
 {

	uint8_t siguiente_tarea = task_list.nTasks - 1; /* Siguiente Tarea = IDLE creada en el MAIN: rtos_start_scheduler(); */
	int8_t prioridad_mas_alta = -1;

	uint8_t i = 0;
	for (i=0; i < task_list.nTasks; i++) /* Todas las tareas creadas + IDLE(prioridad = 0) */
	{
		if (task_list.tasks[i].priority > prioridad_mas_alta		/* ¿Prioridad mas alta? "Y"... ¿Tarea en estado liso "O" corriendo? */
				&& (task_list.tasks[i].state == S_READY || task_list.tasks[i].state == S_RUNNING))
		{
			prioridad_mas_alta = task_list.tasks[i].priority;// > prioridad_mas_alta;	//prioridad_mas_alta = prioridad_de_tarea
			next_task_handler = i;			//siguiente_tarea = tarea
		}			//end if
	}			//end for
				//if siguiente tarea diferente de tarea actual then
	task_list.next_task = next_task_handler;
	if (task_list.next_task != task_list.current_task) {
		context_switch(type);	//context switch (desde la tarea)
	}	//end if
}

FORCE_INLINE static void context_switch(task_switch_type_e type)
{

}

static void activate_waiting_tasks()
{
	uint8_t i = 0;
	for (i = 0; i < task_list.nTasks; i++) 		  	/* Recorremos la lista total de tareas */
	{
		if (task_list.tasks[i].state = S_WAITING) 	/* ¿Tarea en estado de ESPERA? */
		{
			task_list.tasks[i].local_tick = task_list.tasks[i].local_tick - 1; 	 /* Disminuye en 1 el reloj local de la tarea */

			if (task_list.tasks[i].local_tick == 0)	/* ¿Reloj local de tarea es igual a 0? */
			{
				task_list.tasks[i].state = S_READY;	/* Ponemos la tarea en estado LISTO */
			}
		}
	}
}

/**********************************************************************************/
// IDLE TASK
/**********************************************************************************/

static void idle_task(void)
{
	for (;;)
	{

	}
}

/****************************************************/
// ISR implementation
/****************************************************/

void SysTick_Handler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	refresh_is_alive();
#endif
	activate_waiting_tasks();
	reload_systick();
}

void PendSV_Handler(void)
{

}

/**********************************************************************************/
// IS ALIVE SIGNAL IMPLEMENTATION
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
static void init_is_alive(void)
{
	gpio_pin_config_t gpio_config =
	{ kGPIO_DigitalOutput, 1, };

	port_pin_config_t port_config =
	{ kPORT_PullDisable, kPORT_FastSlewRate, kPORT_PassiveFilterDisable,
	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
	        kPORT_UnlockRegister, };
	CLOCK_EnableClock(alive_CLOCK(RTOS_IS_ALIVE_PORT));
	PORT_SetPinConfig(alive_PORT(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &port_config);
	GPIO_PinInit(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &gpio_config);
}

static void refresh_is_alive(void)
{
	static uint8_t state = 0;
	static uint32_t count = 0;
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
	if (RTOS_IS_ALIVE_PERIOD_IN_US / RTOS_TIC_PERIOD_IN_US - 1 == count)
	{
		GPIO_PinWrite(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
		        state);
		state = state == 0 ? 1 : 0;
		count = 0;
	} else //
	{
		count++;
	}
}
#endif
///
