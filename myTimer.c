/*
 * myTimer.c
 *
 *  Created on: Jun 23, 2015
 *      Author: NhatTan
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "inc/hw_memmap.h"
#include "driverlib/systick.h"

#include "myIO.h"
#include  "myTimer.h"
//#include "PPM.h"


volatile uint32_t seconds=0; //variable for capture second
extern volatile struct{
	unsigned char RF_module:1;
	unsigned char MaxSonar:1;
	unsigned char System:1;
} Flag_Safe;

extern struct{
	unsigned char run_controller:1;
	//	unsigned char motor:1;
	unsigned char test_motor:1;
	unsigned char display:1;
	unsigned char set_parameter_roll:1;
	unsigned char set_parameter_pitch:1;
	unsigned char set_parameter_yaw:1;
	unsigned char set_parameter_y:1;
	unsigned char set_parameter_x:1;
	unsigned char set_parameter_z:1;
	unsigned char PD_controller:1;
	unsigned char PID_controller:1;
	unsigned char PD_y_controller:1;
	unsigned char PID_y_controller:1;
	unsigned char Position_controller:1;
	volatile unsigned char sonar:1;
}Flag;

//extern uint8_t toggle_led[];
extern uint16_t PositionSensor_count;

extern volatile struct{
	uint32_t rise_time;
	uint32_t pulse_width;
	unsigned char flag_update;
	unsigned char fail_signal_count;
	float attitude;
}MaxSonar;

volatile uint8_t safe_flag=0; // using for RF signal, Sonar signal

/* timer is used for setting costant frequency
 *
 */
void SysTick_Init()
{
	SysTickPeriodSet(200000);//200Hz interrupt
	SysTickIntEnable();
	SysTickEnable();
}

volatile struct{
	unsigned char Hz_1;
	unsigned char Hz_5;
	unsigned char Hz_10;
	unsigned char Hz_100;
	unsigned char Hz_50;
	unsigned char Hz_20;
}SysTick_Int_Count;
/*
 * COUNT is defined for systick interrupt each 1/200 ms
 */
#define COUNT_5_HZ_		40
#define COUNT_10_HZ_	20
#define COUNT_20_HZ_	10
#define COUNT_50_HZ_	4
#define COUNT_100_HZ_	2


volatile unsigned char SycTick_Int_Count=0;

void SycTick_Interrupt_Handler(void)
{
	SysTick_Int_Count.Hz_100++;
	SysTick_Int_Count.Hz_50++;
	SysTick_Int_Count.Hz_20++;
	SysTick_Int_Count.Hz_10++;
	SysTick_Int_Count.Hz_5++;

	FlagTimer.Hz_200=1;
	if(Flag.sonar==1)
	{
//		Flag.sonar=0;
//		led(LED_BLUE,0);
	}

	if(SysTick_Int_Count.Hz_100==COUNT_100_HZ_)
	{
		SysTick_Int_Count.Hz_100=0;
		FlagTimer.Hz_100=1;

	}
	if(SysTick_Int_Count.Hz_50==COUNT_50_HZ_)
	{
		SysTick_Int_Count.Hz_50=0;
		FlagTimer.Hz_50=1;
	}

	if(SysTick_Int_Count.Hz_20==COUNT_20_HZ_)
	{
		SysTick_Int_Count.Hz_20=0;
		FlagTimer.Hz_20=1;
	}
	if(SysTick_Int_Count.Hz_10==COUNT_10_HZ_)
	{
		SysTick_Int_Count.Hz_10=0;
		FlagTimer.Hz_10=1;

	}

	if(SysTick_Int_Count.Hz_5==COUNT_5_HZ_)
	{
		SysTick_Int_Count.Hz_5=0;
		FlagTimer.Hz_5=1;
		/*
		 * imediately check for safety, not wait for any task
		 */
		//	if((RF_pulse_count>=40)&&(PositionSensor_count>=15);


//		if(MaxSonar.fail_signal_count<=1)
//		{
//			Flag_Safe.MaxSonar=1;
//		}
//		else
//		{
//			Flag_Safe.MaxSonar=0;
//		}

//		if(Flag_Safe.MaxSonar & Flag_Safe.RF_module)
//		{
//			Flag_Safe.System=1;
//		}
//		else
//		{
//			Flag_Safe.System=0;
//		}

//		MaxSonar.fail_signal_count=0;
	}
}

// timer is used for real time clock
void Timer0_init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC_UP);
	TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());

	IntMasterEnable(); //enable processor interrupts
	IntEnable(INT_TIMER0A); //enable the GPIOD interrupt
	IntPrioritySet(INT_TIMER0A,0x00);// configurated in Timer0_init()

	TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);

	TimerEnable(TIMER0_BASE,TIMER_A);
}

/*
 * Interrupt at every 1 second
 */
void Timer0_Interrupt_Handler(void)
{
	FlagTimer.Hz_1=1;
	TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	seconds++;
}

uint32_t getMicroSecond()
{
	uint32_t time;
	time=seconds*1000000+TimerValueGet(TIMER0_BASE,TIMER_A)/40;
	return time; //change to micro second unit when use 40Mhz clock system
}
