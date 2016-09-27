/*
 *Version: 0.0: the first version can flight
 *Work time: July - august, 2015
 *Author: Pham Nhat Tan
 *Description: This version can flight only by remote control, yaw angle's still not stable.
 */

#include "config.h"

void controller();
void communication();
void display_com();
void task_100Hz();
void task_20Hz();
void task_IMU();

//void safe_check()
//{
//	while(RF_module.Channel_3<C3_LIMIT_STICK_TOP);
//	while(RF_module.Channel_3>C3_LIMIT_STICK_BOTTOM);
////	printf("Drone_ready!");
//	UartPutStr(UART_BASE,"Drone_ready!");
//}

void main(void) {

	/*
	 * ================== Hardware's initialization ==========================================
	 */
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	myIO_init();
	led(LED_RED,0);

	//	UART6_Init();
	//	UART1_Init();
	UART0_Init();

	//	myFIFO_init(&communicationFIFO);
	//	myFIFO_init(&kinectFIFO);

	//	I2C1_Init();
	//	while(MPU6050_Init());

	//	RF_init();
	//	PPM_init();
	Timer0_init();
	SysTick_Init();


	// timer - sensor - communication
	//		IntPrioritySet(INT_TIMER0A,0x00);// configurated in Timer0_init()
	//		IntPrioritySet(INT_GPIOE,0xA0);// configurated in MPU6050_INTpin_Init() and MaxSonar_init()
	//		IntPrioritySet(INT_GPIOD,0xC0);//configurated in RF_init()
	//		IntPrioritySet(INT_GPIOC,0xC0);//configurated in RF_init()
	//		IntPrioritySet(INT_UART1,0xE0);//configurated in UART1_init()

	/*
	 *================  software's initialization ====================================
	 */
	//
	//	while(Calib_Gyro()); //  errorr

	//	printf("config done!");
	//	UartPutStr(UART_BASE,"config done!");

	//	PID_init();
	GUI_init();
	//	IMU_init();
	//	safe_check();
	while(1)
	{
		communication();
		//		if(Flag_Safe.RF_module==0)
		//		{
		//			IMU.yaw_gyro=0;
		//			PID_yaw.set_point=0;
		//			PID_reset();
		//
		//			ESC_ppm(1,ESC_1_MIN);
		//			ESC_ppm(2,ESC_2_MIN);
		//			ESC_ppm(3,ESC_3_MIN);
		//			ESC_ppm(4,ESC_4_MIN);
		//			safe_check();
		//		}

		/* code for position control
		 *
		 */
//		task_20Hz();
		task_100Hz();





	}//end of while(1)
}//end of main

void task_20Hz(){
	if(FlagTimer.Hz_20)
	{
		FlagTimer.Hz_20=0;
		/*
		 * Your code begin from here
		 */
		toggle_led[2]^=1;
		led(LED_RED,toggle_led[2]);
		if(Flag.display)
			display_com();
	}
}


void task_100Hz(){
	if(FlagTimer.Hz_100)
	{
		FlagTimer.Hz_100=0;
		/*
		 * Your code begin from here
		 */
		toggle_led[2]^=1;
		led(LED_RED,toggle_led[2]);
		if(Flag.display)
			display_com();
	}
}



void display_com(){


	char buffer[20];
	toggle_led[0]^=1;
	led(LED_BLUE,toggle_led[0]);
	float2num(2.37,buffer);
	UARTCharPut(UART_BASE_USE ,CN_1_);
	UartPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

	int2num(-34.12,buffer);
	UARTCharPut(UART_BASE_USE ,CN_2_);
	UartPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

	int2num(-12343.23,buffer);
	UARTCharPut(UART_BASE_USE ,CN_3_);
	UartPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

	float2num(-65.23,buffer);
	UARTCharPut(UART_BASE_USE ,CN_4_);
	UartPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

	float2num(0.35,buffer);
	UARTCharPut(UART_BASE_USE ,CN_5_);
	UartPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

	float2num(-0.35,buffer);
	UARTCharPut(UART_BASE_USE ,CN_6_);
	UartPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

	int2num(0,buffer);
	UARTCharPut(UART_BASE_USE ,CN_7_);
	UartPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

	int2num(12343.23,buffer);
	UARTCharPut(UART_BASE_USE ,CN_8_);
	UartPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

	int2num(-12343.23,buffer);
	UARTCharPut(UART_BASE_USE ,CN_9_);
	UartPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

	float2num(-12343.23,buffer);
	UARTCharPut(UART_BASE_USE ,CN_10_);
	UartPutStrLn(UART_COM_2_CONTROLLER_,buffer);
	toggle_led[0]^=1;
	led(LED_BLUE,toggle_led[0]);
}

//void run()
//{
//	PID_roll.sampling_time=sampling_time_second;
//	PID_pitch.sampling_time=sampling_time_second;
//	PID_yaw.sampling_time=sampling_time_second;
//
//#ifdef SONAR_HOLD_ATTITUDE
//	if(PID_y.set_point<10)
//#else
//		if(Socket.input_throttle<20)
//#endif
//		{
//			if(Flag.PID_controller==1)
//			{
//				Flag.PID_controller=0;
//				PID_pitch.I_term=0;
//				PID_roll.I_term=0;
//				PID_yaw.I_term=0;
//			}
//			Flag.PD_controller=1;
//
//			PD_type_3(IMU.roll,&PID_roll);
//			PD_type_3(IMU.pitch,&PID_pitch);
//			PD_type_3(IMU.yaw_gyro,&PID_yaw);
//
//		}
//		else{
//			if(Flag.PD_controller==1)
//			{
//				Flag.PD_controller=0;
//				PID_pitch.I_term=0;
//				PID_roll.I_term=0;
//				PID_yaw.I_term=0;
//			}
//			Flag.PID_controller=1;
//
//			PID_type_3(IMU.roll,&PID_roll);
//			PID_type_3(IMU.pitch,&PID_pitch);
//			PID_type_3(IMU.yaw_gyro,&PID_yaw);
//		}
//
//
//	cos_roll=cos(IMU.roll*deg_to_rad);
//	cos_pitch=cos(IMU.pitch*deg_to_rad);
//	/*
//	 * height control
//	 */
//#ifdef SONAR_HOLD_ATTITUDE
//	Socket.throttle=Socket.throttle_offset+PID_y.output;
//	if ((cos_roll>0)&&(cos_pitch>0)&&(Socket.throttle>=0))
//	{
//		Socket.attitude_hold_throttle=sqrt(Socket.throttle/cos_roll/cos_pitch);
//		Socket.attitude_hold_throttle=Map_y(Socket.attitude_hold_throttle,0,10,0,1000);
//	}
//
//	if (Socket.attitude_hold_throttle<MIN_THROTTLE)
//		Socket.attitude_hold_throttle=MIN_THROTTLE;
//	else if(Socket.attitude_hold_throttle>MAX_THROTTLE)
//		Socket.attitude_hold_throttle=MAX_THROTTLE;
//
//
//#else
//	if ((cos_roll>0)&&(cos_pitch>0)&&(Socket.input_throttle>=0))
//	{
//		Socket.attitude_hold_throttle=sqrt(Socket.input_throttle/cos_roll/cos_pitch);
//		Socket.attitude_hold_throttle=Map_y(Socket.attitude_hold_throttle,0,10,0,1000);
//	}
//#endif
//
//	Socket.output_1=Socket.attitude_hold_throttle;
//	Socket.output_2=Socket.attitude_hold_throttle;
//	Socket.output_3=Socket.attitude_hold_throttle;
//	Socket.output_4=Socket.attitude_hold_throttle;
//
//	/*
//	 * pitch control
//	 */
//
//#ifdef QUADCOPTER_V_1
//	Socket.output_1 -= PID_pitch.output;
//	Socket.output_4 -= PID_pitch.output;
//	Socket.output_2 += PID_pitch.output;
//	Socket.output_3 += PID_pitch.output;
//#else
//#ifdef QUADCOPTER_V_0
//	Socket.output_1 += PID_pitch.output;
//	Socket.output_4 += PID_pitch.output;
//	Socket.output_2 -= PID_pitch.output;
//	Socket.output_3 -= PID_pitch.output;
//#endif
//#endif
//	/*
//	 * roll control
//	 */
//	Socket.output_1 += PID_roll.output;
//	Socket.output_2 += PID_roll.output;
//	Socket.output_3 -= PID_roll.output;
//	Socket.output_4 -= PID_roll.output;
//
//	/*
//	 * yaw control
//	 */
//	Socket.output_1 -= PID_yaw.output;
//	Socket.output_3 -= PID_yaw.output;
//	Socket.output_2 += PID_yaw.output;
//	Socket.output_4 += PID_yaw.output;
//
//	/*
//	 * Map system's output (controller + remote) signal to ESC's input  signal
//	 */
//
//	ESC.ppm_1=(int32_t)Map_y(Socket.output_1,0,1000,ESC_1_MIN,ESC_1_MAX);
//	ESC.ppm_2=(int32_t)Map_y(Socket.output_2,0,1000,ESC_2_MIN,ESC_2_MAX);
//	ESC.ppm_3=(int32_t)Map_y(Socket.output_3,0,1000,ESC_3_MIN,ESC_3_MAX);
//	ESC.ppm_4=(int32_t)Map_y(Socket.output_4,0,1000,ESC_4_MIN,ESC_4_MAX);
//
//
//	if(Flag.test_motor==1)
//	{
//		ESC_ppm(1, ESC.ppm_1);
//		ESC_ppm(2, ESC.ppm_2);
//		ESC_ppm(3, ESC.ppm_3);
//		ESC_ppm(4, ESC.ppm_4);
//	}
//	else
//	{
//		ESC_ppm(1,ESC_1_MIN);
//		ESC_ppm(2,ESC_2_MIN);
//		ESC_ppm(3,ESC_3_MIN);
//		ESC_ppm(4,ESC_4_MIN);
//	}
//}


//volatile uint16_t PositionSensor_count=0;
void communication()
{
	if(UartKinect.Flag_receive)
	{
		UartKinect.Flag_receive=0;
		switch(UartKinect.Command_Data[0])
		{
		default:
			break;
		}
	}

	if(Uart.Flag_receive)
	{

		Uart.Flag_receive=0;

		switch(Uart.Command_Data[0])
		{
		case COM2CTL_RUN:
			if(Uart.Command_Data[1]==COM2CTL_RUN_ON){
				Flag.run_controller=1;
				preMicroSecond_angle=getMicroSecond();
				preMicroSecond_position=preMicroSecond_angle;
				// reset_controller's variable
				Flag.test_motor=0;
			}
			else{
				Flag.run_controller=0;
			}
			break;

		case COM2CTL_DISPLAY:
			if(Uart.Command_Data[1]==COM2CTL_DISPLAY_ON)
			{
				Flag.display=1;
			}
			else
			{
				Flag.display=0;
			}
			break;

		case 'c':
			Flag.display=1;
			break;

		case 'd':
			Flag.display=0;
			break;

		case COM2CTL_TEST_MOTOR:
			if(Uart.Command_Data[1]==COM2CTL_TEST_MOTOR_ON)
			{
				Flag.test_motor=1;
				//Flag.run_controller=0;
			}
			else
			{
				Flag.test_motor=0;
			}
			break;
		default:
			break;
		}//end of switch case

		//		printf(Uart.Command_Data);
		UartPutStrLn(UART_BASE_USE,Uart.Command_Data);
	} //end of if(Uart.Flag_receive)
}
