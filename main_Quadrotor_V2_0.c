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
//void task_200Hz();
void task_100Hz();
void task_20Hz();
void task_IMU();
void task_RF();
void mode_selection();
void Stable_Controller();
void height_controller();


void main(void) {

	/*
	 * ================== Hardware's initialization ==========================================
	 */
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	myIO_init();
	led(LED_RED,0);
	led(LED_BLUE,1);

	UART6_Init();
	//	UART1_Init();
	//	UART0_Init();

	PPM_init();
	Timer0_init();
	SysTick_Init();
	I2C1_Init();
	while(MPU6050_Init());

	RF_init();
	Sonar_module_init();


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
	SerialPutStrLn(UART_COM_2_CONTROLLER_,"calib gyroscope offset ...");
	while(Calib_Gyro());
	SerialPutStrLn(UART_COM_2_CONTROLLER_,"calib accelerometer offset ...");
	while(Calib_Accelerometer_Amplitude());
	SerialPutStrLn(UART_COM_2_CONTROLLER_," leave sonar module under ground for calib!!!...");
	while(Calib_Sonar_module());
	led(LED_BLUE,0);

	SerialPutStrLn(UART_COM_2_CONTROLLER_,"config done!");
	PID_init();
	GUI_init();
	IMU_init();
	model_init();
//	quad_model.g=MPU6050.acc_amplitude_offset;
	safe_check();
	while(1)
	{
		communication();
		mode_selection();
		task_RF();
		task_IMU();
		height_controller();

		/* code for position control
		 *
		 */
		task_20Hz();
		task_100Hz();
		//		task_200Hz();


	}//end of while(1)
}//end of main


void height_controller(){
	//#ifdef SONAR_HOLD_ATTITUDE
	if(Flag.SonarHoldAttitudeMode){
		if((Sonar_module.flag_update==1)&&(Flag.run_controller==1))
		{

			Sonar_module.flag_update=0;
			Sonar_module.attitude=Sonar_module.distance*cos_roll*cos_pitch;
			/*
			 * Your code start from here
			 */
			uint32_t microSecond=0;
			microSecond=getMicroSecond();
			sampling_time_second=(float)(microSecond- preMicroSecond_sonar)/1000000.0;
			preMicroSecond_sonar=microSecond;

			/*
			 * Y_Controller
			 */
			PID_z.sampling_time=sampling_time_second;

			if(PID_z.set_point<30)
			{
				if(Flag.PID_z_controller==1)
				{
					Flag.PID_z_controller=0;
					PID_y.I_term=0;
				}
				Flag.PD_z_controller=1;
				PD_type_3(Sonar_module.attitude,&PID_z);
			}

			else{
				if(Flag.PD_z_controller==1)
				{
					Flag.PD_z_controller=0;
					PID_z.I_term=0;
				}
				Flag.PID_z_controller=1;
				PID_type_3(Sonar_module.attitude,&PID_z);
			}

		}
	}
}
void task_IMU()
{
	if(flag_MPU6050_INTpin==1)
	{
		flag_MPU6050_INTpin=0;
		/*
		 * Your code begin from here
		 */

		uint32_t microSecond=0;
		microSecond=getMicroSecond();
		sampling_time_second=(float)(microSecond- preMicroSecond_angle)/1000000.0;
		preMicroSecond_angle=microSecond;
		/*
		 * update sensor's datas
		 */
		while(MPU6050DataGetRaw(&MPU6050.accX_raw));// loop untill data is read

		/*
		 * sensor processing
		 */
		angle(sampling_time_second);
		update_accelerometer();

		/*
		 *  running controller
		 */
		if(Flag.run_controller)
			Stable_Controller();
	}// end of (flag_MPU6050_INTpin==1)
}

void Stable_Controller(){
	PID_roll.sampling_time=sampling_time_second;
	PID_pitch.sampling_time=sampling_time_second;
	PID_yaw.sampling_time=sampling_time_second;

	//#ifdef SONAR_HOLD_ATTITUDE
	//	//if(Flag.SonarHoldAttitudeMode)
	//	if(PID_y.set_point<10)
	//		//	else
	//#else
	//		if(Socket.input_throttle<20)
	//#endif

	if(RF_module.Channel_3<1400)
	{
		if(Flag.PID_o_controller==1)
		{
			Flag.PID_o_controller=0;
			PID_pitch.I_term=0;
			PID_roll.I_term=0;
			PID_yaw.I_term=0;

			PID_yaw.set_point=0;
			IMU.yaw_gyro=0;
		}
		Flag.PD_o_controller=1;

		PD_type_3(IMU.roll,&PID_roll);
		PD_type_3(IMU.pitch,&PID_pitch);
		PD_type_3(IMU.yaw_gyro,&PID_yaw);

	}
	else{
		if(Flag.PD_o_controller==1)
		{
			Flag.PD_o_controller=0;
			PID_pitch.I_term=0;
			PID_roll.I_term=0;
			PID_yaw.I_term=0;
		}
		Flag.PID_o_controller=1;

		PID_type_3(IMU.roll,&PID_roll);
		PID_type_3(IMU.pitch,&PID_pitch);
		PID_type_3(IMU.yaw_gyro,&PID_yaw);
	}
	cos_roll=cos(IMU.roll*deg_to_rad);
	cos_pitch=cos(IMU.pitch*deg_to_rad);
	/*
	 * height control
	 */
	//#ifdef SONAR_HOLD_ATTITUDE
	if(Flag.SonarHoldAttitudeMode){
		Socket.throttle=Socket.throttle_offset+PID_z.output;
		if ((cos_roll>0)&&(cos_pitch>0)&&(Socket.throttle>=0))
		{
			Socket.attitude_hold_throttle=sqrt(Socket.throttle/cos_roll/cos_pitch);
			Socket.attitude_hold_throttle=Map_y(Socket.attitude_hold_throttle,0,10,0,1000);
		}

		if (Socket.attitude_hold_throttle<MIN_THROTTLE)
			Socket.attitude_hold_throttle=MIN_THROTTLE;
		else if(Socket.attitude_hold_throttle>MAX_THROTTLE)
			Socket.attitude_hold_throttle=MAX_THROTTLE;
	}
	else{
		//#else
		if ((cos_roll>0)&&(cos_pitch>0)&&(Socket.input_throttle>=0))
		{
			Socket.attitude_hold_throttle=sqrt(Socket.input_throttle/cos_roll/cos_pitch);
			Socket.attitude_hold_throttle=Map_y(Socket.attitude_hold_throttle,0,10,0,1000);
		}
	}
	//#endif

	Socket.output_1=Socket.attitude_hold_throttle;
	Socket.output_2=Socket.attitude_hold_throttle;
	Socket.output_3=Socket.attitude_hold_throttle;
	Socket.output_4=Socket.attitude_hold_throttle;

	/*
	 * pitch control
	 */


	Socket.output_1 -= PID_pitch.output;
	Socket.output_4 -= PID_pitch.output;
	Socket.output_2 += PID_pitch.output;
	Socket.output_3 += PID_pitch.output;

	/*
	 * roll control
	 */
	Socket.output_1 += PID_roll.output;
	Socket.output_2 += PID_roll.output;
	Socket.output_3 -= PID_roll.output;
	Socket.output_4 -= PID_roll.output;

	/*
	 * yaw control
	 */
	Socket.output_1 -= PID_yaw.output;
	Socket.output_3 -= PID_yaw.output;
	Socket.output_2 += PID_yaw.output;
	Socket.output_4 += PID_yaw.output;

	/*
	 * Map system's output (controller + remote) signal to ESC's input  signal
	 */
	update_omega(Socket.output_1,Socket.output_2,Socket.output_3,Socket.output_4);

	ESC.ppm_1=(int32_t)Map_y(Socket.output_1,0,1000,ESC_1_MIN,ESC_1_MAX);
	ESC.ppm_2=(int32_t)Map_y(Socket.output_2,0,1000,ESC_2_MIN,ESC_2_MAX);
	ESC.ppm_3=(int32_t)Map_y(Socket.output_3,0,1000,ESC_3_MIN,ESC_3_MAX);
	ESC.ppm_4=(int32_t)Map_y(Socket.output_4,0,1000,ESC_4_MIN,ESC_4_MAX);


	if(Flag.test_motor==1)
	{
		ESC_ppm(1, ESC.ppm_1);
		ESC_ppm(2, ESC.ppm_2);
		ESC_ppm(3, ESC.ppm_3);
		ESC_ppm(4, ESC.ppm_4);
	}
	else
	{
		ESC_ppm(1,ESC_1_MIN);
		ESC_ppm(2,ESC_2_MIN);
		ESC_ppm(3,ESC_3_MIN);
		ESC_ppm(4,ESC_4_MIN);
	}
}
void task_RF()
{
	if(RF_module.flag_update==1)
	{
		RF_module.flag_update=0;
		/*
		 * Your code begin from here
		 */


		/*
		 * Map RF signals to user control(remote) signal
		 */
		//#ifdef SONAR_HOLD_ATTITUDE
		if(Flag.SonarHoldAttitudeMode){
			Socket.input_throttle=Map_y((float)RF_module.Channel_3,C3_MIN,C3_MAX,0,1000);
			if(Socket.input_throttle<0)
				Socket.input_throttle=0;
			else if(Socket.input_throttle>1000)
				Socket.input_throttle=1000;
			PID_z.set_point=Socket.input_throttle;
		}
		else{
			//#else
			Socket.input_throttle=Map_y((float)RF_module.Channel_3,C3_MIN,C3_MAX,0,81);
			if(Socket.input_throttle<0)
				Socket.input_throttle=0;
			else if(Socket.input_throttle>81)
				Socket.input_throttle=81;
		}
		//#endif

#ifdef USE_KINECT
		if((Kinect.flag_update==1)&&(Flag.run_controller==1))
		{
			Kinect.flag_update=0;
			/*
			 * code is began from here
			 */
			uint32_t microSecond=0;
			microSecond=getMicroSecond();
			sampling_time_second=(float)(microSecond- preMicroSecond_kinect)/1000000.0;
			preMicroSecond_kinect=microSecond;

			PID_x.sampling_time=sampling_time_second;
			PID_type_3(PositionFilted.X,&PID_x);

			if(PID_x.output>PID_X_RANGE)
				Socket.input_aileron = PID_X_RANGE;
			else if(PID_x.output <- PID_X_RANGE)
				Socket.input_aileron = -PID_X_RANGE;
			else
				Socket.input_aileron=PID_x.output;

			Socket.input_aileron = -Socket.input_aileron;


			PID_z.sampling_time=sampling_time_second;
			PID_type_3(PositionFilted.Z,&PID_z);

			if(PID_z.output>PID_Z_RANGE)
				Socket.input_elevator = PID_Z_RANGE;
			else if(PID_z.output<-PID_Z_RANGE)
				Socket.input_elevator = -PID_Z_RANGE;
			else
				Socket.input_elevator=PID_z.output;

			Socket.input_elevator=Socket.input_elevator;


			if(Flag.display)
			{
				writeByte(CTL2COM_DISPLAY);
				FIFO_3_int_display(PID_z.er/10,PID_z.output*100,Socket.output_1/10);
			}
		}

		//			Socket.input_aileron=Map_y((float)RF_module.Channel_1,C1_ZERO,C1_MAX,0,AILERON_RANGE);
		//			Socket.input_elevator=Map_y((float)RF_module.Channel_2,C2_ZERO,C2_MAX,0,ELEVATOR_RANGE);

#else
		Socket.input_elevator=Map_y((float)RF_module.Channel_2,C2_ZERO,C2_MAX,0,ELEVATOR_RANGE);
		Socket.input_aileron=Map_y((float)RF_module.Channel_1,C1_ZERO,C1_MAX,0,AILERON_RANGE);
#endif

		Socket.input_rudder=Map_y((float)RF_module.Channel_4,C4_ZERO,C4_MAX,0,RUDDER_MAX_VELO);
		if((Socket.input_rudder>5)|(Socket.input_rudder<-5))
			PID_yaw.set_point-=Socket.input_rudder*0.02;//50hZ

		//pitch==elevator=C6; roll==aileron==C5
		Socket.input_elevator_midpoint=Map_y((float)RF_module.Channel_6,C6_ZERO,C6_MAX,PITCH_SET_ANGLE,PITCH_SET_ANGLE+SET_ANGLE_RANGE);
		Socket.input_aileron_midpoint=Map_y((float)RF_module.Channel_5,C5_ZERO,C5_MAX,ROLL_SET_ANGLE,ROLL_SET_ANGLE+SET_ANGLE_RANGE);

		//			Socket.input_aileron_midpoint=ROLL_SET_ANGLE;
		//#ifdef SONAR_HOLD_ATTITUDE
		if(Flag.SonarHoldAttitudeMode)
			Socket.throttle_offset=Map_y((float)RF_module.Channel_8,C8_MIN,C8_MAX,0,49);
		//#endif

		PID_pitch.set_point = Socket.input_elevator+Socket.input_elevator_midpoint;
		PID_roll.set_point = Socket.input_aileron+Socket.input_aileron_midpoint;
	}

}


void mode_selection()
{
	if(Flag_Safe.RF_module==0)
	{
		IMU.yaw_gyro=0;
		PID_yaw.set_point=0;
		PID_reset();

		ESC_ppm(1,ESC_1_MIN);
		ESC_ppm(2,ESC_2_MIN);
		ESC_ppm(3,ESC_3_MIN);
		ESC_ppm(4,ESC_4_MIN);

		safe_check();
		if(Flag_SonarHoldAttitudeMode==1)
		{
			Flag.SonarHoldAttitudeMode=1;
			SerialPutStrLn(UART_COM_2_CONTROLLER_,"auto_mode!");
			led(LED_RED,1);
		}
		else
		{
			Flag.SonarHoldAttitudeMode=0;
			SerialPutStrLn(UART_COM_2_CONTROLLER_,"manual_mode!");
			led(LED_RED,0);
		}

		//		if(Flag.SonarHoldAttitudeMode)
		//			led(LED_RED,1);
		//		else
		//			led(LED_RED,0);

		Flag.run_controller=1;
		Flag.test_motor=1;
	}
}

void task_20Hz(){
	if(FlagTimer.Hz_20)
	{
		FlagTimer.Hz_20=0;
		/*
		 * Your code begin from here
		 */
	}
}



void task_100Hz(){
	if(FlagTimer.Hz_100)
	{
		FlagTimer.Hz_100=0;
		/*
		 * Your code begin from here
		 */
		if(Flag.display)
			display_com();

	}
}



void display_com(){
	char buffer[20];
	float2num(MPU6050.z1_dot_dot,buffer);
	SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_1_);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

	float2num(quad_model.z1_dot_dot,buffer);
	SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_2_);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);

	float2num(Sonar_module.distance,buffer);
	SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_3_);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	//
	float2num(MPU6050.accY,buffer);
	SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_4_);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	//
	float2num(MPU6050.accZ,buffer);
	SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_5_);
	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	//
	//	int2num(RF_module.Channel_6,buffer);
	//	SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_6_);
	//	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	//	//
	//	int2num(RF_module.Channel_7,buffer);
	//	SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_7_);
	//	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	//
	//	int2num(RF_module.Channel_8,buffer);
	//	SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_8_);
	//	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	//
	//	int2num(-12343.23,buffer);
	//	SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_9_);
	//	SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,buffer);
	//
	//	float2num(-12343.23,buffer);
	//	SerialPutChar(UART_COM_2_CONTROLLER_ ,CN_10_);
	//	SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

	SerialTerminator(UART_COM_2_CONTROLLER_);
}


//volatile uint16_t PositionSensor_count=0;
void communication()
{
	if(Uart.Flag_receive)
	{
		Uart.Flag_receive=0;
		switch(Uart.Command_Data[0])
		{
		case COM2CTL_MOTOR_:
			switch(Uart.Command_Data[1])
			{
			char buffer[10];
			case TEST_MOTOR_1_:
				ESC.ppm_1=atoi(&Uart.Command_Data[2]);

				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"motor_1: ");
				int2num(ESC.ppm_1,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

				ESC_ppm(1,ESC.ppm_1);
				break;
			case TEST_MOTOR_2_:
				ESC.ppm_2=atoi(&Uart.Command_Data[2]);

				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"motor_2: ");
				int2num(ESC.ppm_2,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

				ESC_ppm(2,ESC.ppm_2);
				break;
			case TEST_MOTOR_3_:
				ESC.ppm_3=atoi(&Uart.Command_Data[2]);

				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"motor_3: ");
				int2num(ESC.ppm_1,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

				ESC_ppm(3,ESC.ppm_3);
				break;
			case TEST_MOTOR_4_:
				ESC.ppm_4=atoi(&Uart.Command_Data[2]);

				SerialPutStr_NonTer(UART_COM_2_CONTROLLER_,"motor_4: ");
				int2num(ESC.ppm_4,buffer);
				SerialPutStrLn(UART_COM_2_CONTROLLER_,buffer);

				ESC_ppm(4,ESC.ppm_4);
				break;
			case MOTOR_STOP_:
				Motor_stop();
				break;
			default:
				break;
			}
			break;

			case COM2CTL_DISPLAY_ON_:
				Flag.display=1;
				break;

			case COM2CTL_DISPLAY_OFF_:
				Flag.display=0;
				break;

			default:
				break;
		}//end of switch case

		//		printf(Uart.Command_Data);
		SerialPutStrLn(UART_COM_2_CONTROLLER_,Uart.Command_Data);
	} //end of if(Uart.Flag_receive)
}
