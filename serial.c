/*-----system include----------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "utils/ringbuf.h"

/*-----my include-------------------*/
#include "serial.h"
#include "myIO.h"
//#include "myFIFO.h"

/* -------define variables----------*/
MyUart Uart;
MyUart UartKinect;
/*
 * ====================UART6_INIT=====================================
 */

//void  UART6_Interrupt_Handler(void)
//{
//	// return: true-the masked interrupt status | false -the current interrupt status
//	UARTIntClear(UART6_BASE,UART_INT_RT|UART_INT_RX);//|UART_INT_RX
//	unsigned char flag_valid_data=0;
//	char charData;
//
////	toggle_led[2]^=1;
////	led(LED_RED,toggle_led[2]);
//	while(UARTCharsAvail(UART6_BASE))
//	{
//		charData=(char)UARTCharGet(UART6_BASE);
//		if(charData==TERMINATOR_)
//			flag_valid_data=1;
//		FIFO_Rx_CharPut(&kinectFIFO,charData);
//	}
//
//	if(flag_valid_data==1)
//	{
//		flag_valid_data=0;
//		FIFO_Rx_StrGet(&kinectFIFO,UartKinect.Command_Data);
//		UartKinect.Flag_receive=1;
//		//			UartPutStr(UART1_BASE,Uart.Command_Data);
//	}
//
//
//	//		UartGetStr(UART1_BASE,Uart.Command_Data);
//	//		Uart.Flag_receive=1;
//}

//void UART1_Interrupt_Handler(void)
//{
//	// return: true-the masked interrupt status | false -the current interrupt status
//	uint32_t interrupt_status;
//	interrupt_status=UARTIntStatus(UART1_BASE,true);
//
//	if((((interrupt_status&UART_INT_RT)==UART_INT_RT)|((interrupt_status&UART_INT_RX)==UART_INT_RX))==1)
//	{
//		UARTIntClear(UART1_BASE,UART_INT_RT|UART_INT_RX);//|UART_INT_RX
//		unsigned char flag_valid_data=0;
//		char charData;
//
//		while(UARTCharsAvail(UART1_BASE))
//		{
//			charData=(char)UARTCharGet(UART1_BASE);
//			if(charData==TERMINATOR_)
//				flag_valid_data=1;
//			FIFO_Rx_CharPut(&communicationFIFO,charData);
//		}
//
//		if(flag_valid_data==1)
//		{
//			flag_valid_data=0;
//			FIFO_Rx_StrGet(&communicationFIFO,Uart.Command_Data);
//			Uart.Flag_receive=1;
//			//			UartPutStr(UART1_BASE,Uart.Command_Data);
//		}
//
//		//		UartGetStr(UART1_BASE,Uart.Command_Data);
//		//		Uart.Flag_receive=1;
//	}
//
//	else if((interrupt_status&UART_INT_TX)==UART_INT_TX)
//	{
//		UARTIntClear(UART1_BASE,UART_INT_TX);
//		//		toggle_led[2]^=1;
//		//		led(LED_RED,toggle_led[2]);
//		update_hardwareFIFO(&communicationFIFO,UART1_BASE);
//	}
//}


//void UART1_Interrupt_Handler(void)
//{
//	UARTIntClear(UART1_BASE,UART_INT_RT|UART_INT_RX);//|UART_INT_RX
//	UartGetStr(UART1_BASE,Uart.Command_Data);
//
//	if(!(UARTBusy(UART1_BASE))){
//		UartPutStr(UART1_BASE,Uart.Command_Data);
//	}
//	Uart.Flag_receive=1;
//}

void UART0_Interrupt_Handler(void)
{
	UARTIntClear(UART0_BASE,UART_INT_RT|UART_INT_RX);//|UART_INT_RX
	UartGetStr(UART0_BASE,Uart.Command_Data);

	if(!(UARTBusy(UART0_BASE))){
		UartPutStr(UART0_BASE,Uart.Command_Data);
	}
	Uart.Flag_receive=1;

	led(LED_GREEN,1);

	// return: true-the masked interrupt status | false -the current interrupt status
	//		uint32_t interrupt_status;
	//		interrupt_status=UARTIntStatus(UART0_BASE,true);
	//
	//		if((((interrupt_status&UART_INT_RT)==UART_INT_RT)|((interrupt_status&UART_INT_RX)==UART_INT_RX))==1)
	//		{
	//			UARTIntClear(UART0_BASE,UART_INT_RT|UART_INT_RX);//|UART_INT_RX
	//			unsigned char flag_valid_data=0;
	//			char charData;
	//
	//			while(UARTCharsAvail(UART0_BASE))
	//			{
	//				charData=(char)UARTCharGet(UART0_BASE);
	//				if(charData==TERMINATOR_)
	//					flag_valid_data=1;
	//				FIFO_Rx_CharPut(&communicationFIFO,charData);
	//			}
	//
	//			if(flag_valid_data==1)
	//			{
	//				flag_valid_data=0;
	//				FIFO_Rx_StrGet(&communicationFIFO,Uart.Command_Data);
	//				Uart.Flag_receive=1;
	//				//			UartPutStr(UART1_BASE,Uart.Command_Data);
	//			}
	//
	//			//		UartGetStr(UART1_BASE,Uart.Command_Data);
	//			//		Uart.Flag_receive=1;
	//		}
	//
	//		else if((interrupt_status&UART_INT_TX)==UART_INT_TX)
	//		{
	//			UARTIntClear(UART0_BASE,UART_INT_TX);
	//			//		toggle_led[2]^=1;
	//			//		led(LED_RED,toggle_led[2]);
	//			update_hardwareFIFO(&communicationFIFO,UART0_BASE);
	//		}
}

void UART6_Init()
{
	//
	// Enable Peripheral Clocks
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	//
	// Enable pin PD5 for UART6 U6TX
	//
	GPIOPinConfigure(GPIO_PD5_U6TX);
	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_5);

	//
	// Enable pin PD4 for UART6 U6RX
	//
	GPIOPinConfigure(GPIO_PD4_U6RX);
	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4);

	UARTConfigSetExpClk(UART6_BASE, SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));


	IntMasterEnable(); //enable processor interrupts
	IntEnable(INT_UART6); //enable the UART interrupt

	IntPrioritySet(INT_UART6,0xE0);

	UARTIntEnable(UART6_BASE,  UART_INT_RT|UART_INT_RX); //only enable RX and RTinterrupts
}

void UART0_Init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	IntMasterEnable(); //enable processor interrupts
	IntEnable(INT_UART0); //enable the UART interrupt
	IntPrioritySet(INT_UART0,0xE0);

	//	UARTTxIntModeSet(UART0_BASE,UART_TXINT_MODE_EOT);
	//	UARTIntEnable(UART0_BASE, UART_INT_RX|UART_INT_RT|UART_INT_TX);
	UARTIntEnable(UART0_BASE, UART_INT_RX|UART_INT_RT);

}

//uart5: PE4-Rx PE5 -Tx
void UART5_Init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	GPIOPinConfigure(GPIO_PE4_U5RX);
	GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4);

	GPIOPinConfigure(GPIO_PE5_U5TX);
	GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_5);

	UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

}

// UART1: PB0-RX | PB1-TX
void UART1_Init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0);

	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_1);

	/*
	 * setting the baud clock source from the system clock
	 */
	UARTClockSourceSet(UART1_BASE,UART_CLOCK_SYSTEM);

	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));


	/*
	 * Enable the UART.
	 */
	UARTEnable(UART1_BASE);

	IntMasterEnable(); //enable processor interrupts
	IntEnable(INT_UART1); //enable the UART interrupt

	IntPrioritySet(INT_UART1,0xE0);

	//	UARTTxIntModeSet(UART1_BASE,UART_TXINT_MODE_EOT);
	//	UARTIntEnable(UART1_BASE,  UART_INT_RT|UART_INT_RX|UART_INT_TX); //only enable time out interrupts
	UARTIntEnable(UART1_BASE,  UART_INT_RT|UART_INT_RX);
}


void UartPutStr(uint32_t ui32Base,char *uart_str)
{
	while(*uart_str != '\0') {UARTCharPut(ui32Base,*uart_str++ );}
	UARTCharPut(ui32Base,TERMINATOR_);
}

void UartPutStrLn(uint32_t ui32Base,char *uart_str)
{
	while(*uart_str != '\0') {UARTCharPut(ui32Base,*uart_str++ );}
	UARTCharPut(ui32Base,'\r');
	UARTCharPut(ui32Base,'\n');
}

void  UartPutStr_NonTer(uint32_t ui32Base,char *uart_str)
{
	while(*uart_str != '\0') {UARTCharPut(ui32Base,*uart_str++ );}
}

void UartGetStr(uint32_t ui32Base,char *uart_str)
{
	*uart_str= (char)UARTCharGet(ui32Base);
	while (*uart_str != '\n')
	{
		uart_str++;
		*uart_str= (char)UARTCharGet(ui32Base);
	}
	*(uart_str+1)='\0';
}


void float2num(float num, char *str){
	unsigned char numNeg,numID;
	long numDem;
	long numInt;

	numID=0;

	if (num<0)
	{
		num=-num;
		numNeg=1;
	}
	else
		numNeg=0;



	char a[10];

	numInt=num*100;

	a[numID++]=numInt%10+48;
	a[numID++]=numInt/10%10+48;
	a[numID++]='.';

	numInt=num;
	if(num<10)
	{
		a[numID++]=numInt+48;
	}
	else
	{
		a[numID++]=numInt%10+48;

		numDem=10;
		while(num>=numDem)
		{
			numInt=num/numDem;
			numDem*=10;
			a[numID++]=numInt%10+48;
		}
	}

	if(numNeg==1)
		a[numID++]='-';
	a[numID]='\0';

	//	char b[10];
	//	signed char i;
	//	char numInx=0;
	//	for (i=numID-1;i>=0;i--)
	//	{
	//		b[numInx++]=a[i];
	//	}
	//	b[numInx]='\0';

//	char str[10];
	signed char i;
	char numInx=0;
	for (i=numID-1;i>=0;i--)
	{
		*(str+(numInx++))=a[i];
//		numInx++;
	}
	*(str+numInx)='\0';

}

// hai lôi trong cu phap, (se bao loi doi voi compiler C99 stric
//  + return con tro vao bien cuc bo
//  + dung index cho array co kieu la char
void int2num(int num,char *str)
{
	unsigned char numNeg,numID;
	long numDem;
	long numInt;

	numID=0;

	if (num<0)
	{
		num=-num;
		numNeg=1;
	}
	else
		numNeg=0;



	char a[10];

	//	numInt=num*100;

	//	a[numID++]=numInt%10+48;
	//	a[numID++]=numInt/10%10+48;
	//	a[numID++]='.';

	numInt=num;
	//	if(num<10)
	//	{
	//		a[numID++]=numInt+48;
	//	}
	//	else
	//	{
	a[numID++]=numInt%10+48;

	numDem=10;
	while(num>=numDem)
	{
		numInt=num/numDem;
		numDem*=10;
		a[numID++]=numInt%10+48;
	}
	//	}

	if(numNeg==1)
		a[numID++]='-';
	a[numID]='\0';


	signed char i;
	char numInx=0;
	for (i=0;i<numID;i++)
	{
		*(str+(numInx++))=a[numID-1-i];
	}
	*(str+numInx)='\0';
}

// send float number: xxx.xx
// using float type is less precession than double type, for example 2.35 -> display as 2.34
void UART_float_display(uint32_t ui32Base ,float num, bool terminator)
{
	unsigned short k;
	unsigned char a;

	num*=100.0;
	if (num <0)
	{
		UARTCharPut(ui32Base,'-');
		k = - num;
	}
	else
		k=num;

	a=k/10000 + 48;
	UARTCharPut(ui32Base,a);

	a=(k%10000)/1000+48;
	UARTCharPut(ui32Base,a);

	a=(k%10000)%1000/100+48;
	UARTCharPut(ui32Base,a);

	UARTCharPut(ui32Base,'.');

	a=(k%10000)%1000%100/10+48;
	UARTCharPut(ui32Base,a);

	a=(k%10000)%1000%100%10+48;
	UARTCharPut(ui32Base,a);

	if (terminator==true)
		UARTCharPut(ui32Base,TERMINATOR_);
}


// send integer number: xxxxx
void UART_int_display(uint32_t ui32Base,int16_t num, bool terminator )
{
	unsigned short k;
	unsigned char a;
	if (num <0)
	{
		UARTCharPut(ui32Base,'-');
		k = - num;
	}
	else
		k=num;

	a=k/10000 + 48;
	UARTCharPut(ui32Base,a);

	a=(k%10000)/1000+48;
	UARTCharPut(ui32Base,a);

	a=(k%10000)%1000/100+48;
	UARTCharPut(ui32Base,a);

	a=(k%10000)%1000%100/10+48;
	UARTCharPut(ui32Base,a);

	a=(k%10000)%1000%100%10+48;
	UARTCharPut(ui32Base,a);

	if (terminator==true)
		UARTCharPut(ui32Base,TERMINATOR_);

}

void UART_uint32_display(uint32_t ui32Base,uint32_t num, bool terminator )
{
	uint32_t k;
	unsigned char a;
	k=num;
	a=k/10000 + 48;
	UARTCharPut(ui32Base,a);

	a=(k%10000)/1000+48;
	UARTCharPut(ui32Base,a);

	a=(k%10000)%1000/100+48;
	UARTCharPut(ui32Base,a);

	a=(k%10000)%1000%100/10+48;
	UARTCharPut(ui32Base,a);

	a=(k%10000)%1000%100%10+48;
	UARTCharPut(ui32Base,a);

	if (terminator==true)
		UARTCharPut(ui32Base,TERMINATOR_);

}

// send 3 float number: xxx.xxaxxx.xxaxxx.xx|terminator|
void UART_3_float_display(uint32_t ui32Base,float num1, float num2, float num3 )
{
	UART_float_display(ui32Base,num1, false);
	UARTCharPut(ui32Base,'a');
	UART_float_display(ui32Base,num2, false);
	UARTCharPut(ui32Base,'a');
	UART_float_display(ui32Base,num3, false);
	UARTCharPut(ui32Base,TERMINATOR_);
}

// send 3 integer number xxxxxaxxxxxaxxxxx|terminator|
void UART_3_int_display(uint32_t ui32Base,int16_t num1, int16_t num2, int16_t num3 )
{
	UART_int_display(ui32Base,num1, false);
	UARTCharPut(ui32Base,'a');
	UART_int_display(ui32Base,num2, false);
	UARTCharPut(ui32Base,'a');
	UART_int_display(ui32Base,num3, false);
	UARTCharPut(ui32Base,TERMINATOR_);
}

void UART_3_uint32_display(uint32_t ui32Base,uint32_t num1, uint32_t num2, uint32_t num3 )
{
	UART_uint32_display(ui32Base,num1, false);
	UARTCharPut(ui32Base,'a');
	UART_uint32_display(ui32Base,num2, false);
	UARTCharPut(ui32Base,'a');
	UART_uint32_display(ui32Base,num3, false);
	UARTCharPut(ui32Base,TERMINATOR_);
}

void UART_n_int_display(uint32_t ui32Base,uint32_t *num,uint8_t size)
{
	uint8_t i;
	UARTCharPut(ui32Base,'a');
	for (i=1;i<size;i++)
	{
		UART_int_display(ui32Base,*(num++), false);
		UARTCharPut(ui32Base,'a');
	}
	UART_int_display(ui32Base,*(num), false);
	UARTCharPut(ui32Base,TERMINATOR_);
}

void UART_n_float_display(uint32_t ui32Base,float *num,uint8_t size)
{
	uint8_t i;
	UARTCharPut(ui32Base,'a');
	for (i=1;i<size;i++)
	{
		UART_float_display(ui32Base,*(num++), false);
		UARTCharPut(ui32Base,'a');
	}
	UART_float_display(ui32Base,*(num), false);
	UARTCharPut(ui32Base,TERMINATOR_);
}

void set_float_value (char *string,float *kp)
{
	while (!(isdigit(*string) || *string == '-')) (string++);
	*kp=atof(string);
}

void set_int_value (char *string,int32_t *kp)
{
	while (! (isdigit(*string) || *string == '-')) (string++);
	//	while (! (isdigit(*string))) (string++);
	*kp=atoi(string);
}

void set_position (char *string, int16_t *position)
{
	//	while(!(*string=='x')) string++;
	//	*position++=atoi(string+1);
	////	while(!(*string=='y')) string++;
	////	*position++=atoi(string+1);
	//	while(!(*string=='z')) string++;
	//	*position++=atoi(string+1);
	while(*string!='\0')
	{

		switch(*string){
		case 'x':
			*position=atoi(string+1);
			break;
		case 'y':
			*(position+1)=atoi(string+1);
			break;
		case 'z':
			*(position+2)=atoi(string+1);
			break;
		default:
			break;
		}
		string++;
	}

}

void float2str(float num,char *kq)
{

	unsigned short k;
	unsigned char a;


	num*=100.0;
	if (num <0)
	{
		*kq++='-';
		k = - num;
	}
	else
		k=num;

	a=k/10000 + 48;
	*kq++=a;

	a=(k%10000)/1000+48;
	*kq++=a;

	a=(k%10000)%1000/100+48;
	*kq++=a;

	*kq++='.';

	a=(k%10000)%1000%100/10+48;
	*kq++=a;

	a=(k%10000)%1000%100%10+48;
	*kq++=a;
	*kq=0;
}

void int2str(int32_t num,char *kq)
{

	unsigned short k;
	unsigned char a;

	if (num <0)
	{
		*kq++='-';
		k = - num;
	}
	else
		k=num;

	a=k/10000 + 48;
	*kq++=a;

	a=(k%10000)/1000+48;
	*kq++=a;

	a=(k%10000)%1000/100+48;
	*kq++=a;

	a=(k%10000)%1000%100/10+48;
	*kq++=a;

	a=(k%10000)%1000%100%10+48;
	*kq++=a;
	*kq=0;
}



