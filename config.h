/*
 * config.h
 *
 *  Created on: Sep 14, 2015
 *      Author: NhatTan
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
//#include "utils/uartstdio.h"
#include "driverlib/systick.h"
//#include "utils/ringbuf.h"
//#include "myFIFO.h"
//#include "mpu6050.h"
//#include "myI2C.h"
#include "myTimer.h"
#include "myIO.h"
//#include "IMU.h"
#include "serial.h"
//#include "RF.h"
//#include "PPM.h"
//#include "PID.h"
//#include "MaxSonar.h"
//#include "Kinect.h"
#include "config_hardware.h"



#ifndef CONFIG_H_
#define CONFIG_H_

#define DEBUG_UART
//#define SONAR_HOLD_ATTITUDE
//#define USE_KINECT

//----------DEFINE PARAMEETER---------------
#define PID_X_RANGE		1.2 //AILERON_RANGE
#define PID_Z_RANGE		1.2 //ELEVATOR_SET_ANGLE

#define ELEVATOR_RANGE	15.0
#define AILERON_RANGE	15.0
//#define ELEVATOR_SET_ANGLE	-1.3
//#define AILERON_SET_ANGLE	1.6

#define RUDDER_YAW_OFSET_INTEGRAL 30
#define RUDDER_INTEGRAL 0.05
#define RUDDER_SET_ANGLE	0
#define RUDDER_RANGE		25



#ifdef QUADCOPTER_V_1
#define ROLL_SET_ANGLE		-0.1
#define PITCH_SET_ANGLE		-0.9
#define SET_ANGLE_RANGE		2.0
#define RUDDER_MAX_VELO		45

#else

#ifdef QUADCOPTER_V_0
#define ROLL_SET_ANGLE		-1.22
#define PITCH_SET_ANGLE		-1.25
#define SET_ANGLE_RANGE		2.0
#define RUDDER_MAX_VELO		30
#endif

#endif

#define X_MAX 		1000
#define X_MIN		-1000
#define POSITION_FILTER_FACTOR		0.6

float cos_roll=1;
float cos_pitch=1;

//#define LIFT_THROTTLE		700

#ifdef QUADCOPTER_V_1
#define MAX_THROTTLE		900
#define MIN_THROTTLE		550
#define THROTTLE_OFFSET		24
#else
#ifdef QUADCOPTER_V_0
#define THROTTLE_OFFSET		45
#define MAX_THROTTLE		1000
#define MIN_THROTTLE		500
#endif
#endif
/*
 * ========define variables============
 */

volatile struct{
	unsigned char RF_module:1;
	unsigned char MaxSonar:1;
	unsigned char System:1;
} Flag_Safe;
uint32_t preMicroSecond_angle=0;
uint32_t preMicroSecond_position=0;
uint32_t preMicroSecond_sonar=0;
uint32_t preMicroSecond_kinect=0;
float sampling_time_second=0;



union{
	struct{
		int16_t X;
		int16_t Y;
		int16_t Z;
	}axis;
	int16_t data[3];
}Position;

struct{
	float X;
	float Y;
	float Z;
}PositionFilted; //after filter



struct{
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

/*
 * ========declare variables============
 */
extern volatile uint8_t safe_flag; // declare a variable, which is defined in Timer.c
extern volatile int32_t RF_pulse_width[];// declare a variable, which is defined in RF.c
extern volatile uint8_t flag_MPU6050_INTpin;//declare a variable, which is defined in mpu6050.c

extern volatile struct{
	int32_t Channel_1;
	int32_t Channel_2;
	int32_t Channel_3;
	int32_t Channel_4;
	int32_t Channel_5;
	int32_t Channel_6;
	unsigned char flag_update;
	uint16_t signal_count;
}RF_module;

volatile struct{
	uint32_t rise_time;
	uint32_t pulse_width;
	uint32_t pre_pulse_width;
	int32_t d_pulse_width;
	unsigned char flag_update;
	unsigned char fail_signal_count;
	float distance;
	float attitude;
	float pulse_width_offset;
}MaxSonar;



extern struct{
	float roll_acc;
	float pitch_acc;
	float roll_gyro;
	float pitch_gyro;
	float yaw_gyro;
	float roll;
	float pitch;
	float yaw;
	float yaw_ofset;
} IMU;

struct{
	float output_1; //output to motor1
	float output_2; //
	float output_3;
	float output_4;
	float input_throttle; //lift force, up-down
	float input_elevator; //bank forward - backward
	float input_rudder;	  //rotate left - right
	float input_aileron;  //bank sideways left - right
	float input_elevator_midpoint;
	float input_aileron_midpoint;
	float attitude_hold_throttle; //throttle to maintain the attitude
	float attitude_hold_velosity;
	float throttle_offset;
	float throttle;
}Socket;

void GUI_init(){
	Flag.run_controller=0;
	Flag.test_motor=0;
	Flag.display=0;
	Flag.set_parameter_roll=0;
	Flag.set_parameter_pitch=0;
	Flag.set_parameter_yaw=0;

	Flag_Safe.System=1;
}
void Position_init()
{

}

#ifdef QUADCOPTER_V_1

#else
#ifdef QUADCOPTER_V_0
void PID_init()
{
	Socket.throttle_offset=45;

	PID_roll.set_point=1.66;
	PID_roll.KP=7;
	PID_roll.KI=10;
	PID_roll.KD=1.8;
	PID_roll.output=0;
	PID_roll.er=0;
	PID_roll.pre_er=0;
	PID_roll.pre_pre_er=0;

	PID_pitch.set_point=-1.3;
	PID_pitch.KP=7;
	PID_pitch.KI=10;
	PID_pitch.KD=1.8;
	PID_pitch.output=0;
	PID_pitch.er=0;
	PID_pitch.pre_er=0;
	PID_pitch.pre_pre_er=0;

	PID_yaw.set_point=0;
	PID_yaw.KP=3;
	PID_yaw.KI=2.08;
	PID_yaw.KD=1.15;
	PID_yaw.output=0;
	PID_yaw.er=0;
	PID_yaw.pre_er=0;
	PID_yaw.pre_pre_er=0;

	PID_y.set_point=0;
	PID_y.KP=0.03;
	PID_y.KI=0.02;
	PID_y.KD=0.05;
	PID_y.output=0;
	PID_y.er=0;
	PID_y.pre_er=0;
	PID_y.pre_pre_er=0;

	PID_x.set_point=0;
	PID_x.KP=0.0002;
	PID_x.KI=0.0003;
	PID_x.KD=0.0035;
	PID_x.output=0;
	PID_x.er=0;
	PID_x.pre_er=0;
	PID_x.pre_pre_er=0;

	PID_z.set_point=1400;
	PID_z.KP=0.0002;
	PID_z.KI=0.0003;
	PID_z.KD=0.0035;
	PID_z.output=0;
	PID_z.er=0;
	PID_z.pre_er=0;
	PID_z.pre_pre_er=0;

	/*
	 * pid type_1
	 */
	PID_roll.I_term=0;
	PID_roll.I_limit=300;

	PID_pitch.I_term=0;
	PID_pitch.I_limit=300;

	PID_yaw.I_term=0;
	PID_yaw.I_limit=500;

	PID_y.I_term=0;
	PID_y.I_limit=30;

	PID_x.I_term=0;
	PID_x.I_limit=0.7;

	PID_z.I_term=0;
	PID_z.I_limit=0.7;
	/*
	 * pid type 3
	 */
	PID_pitch.pre_fb=0;
	PID_pitch.pre_pre_fb=0;

	PID_roll.pre_fb=0;
	PID_roll.pre_pre_fb=0;

	PID_yaw.pre_fb=0;
	PID_yaw.pre_pre_fb=0;

	PID_y.pre_fb=0;
	PID_y.pre_pre_fb=0;

	PID_x.pre_fb=0;
	PID_x.pre_pre_fb=0;

	PID_z.pre_fb=0;
	PID_z.pre_pre_fb=0;

	/*
	 * state
	 */
	Flag.PD_controller=0;
	Flag.PID_controller=0;
	Flag.PD_y_controller=0;
	Flag.PID_y_controller=0;
}
#endif
#endif

void PID_reset()
{
}


#endif /* CONFIG_H_ */
