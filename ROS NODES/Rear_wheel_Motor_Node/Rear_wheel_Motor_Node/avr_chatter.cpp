/*
 * avr_chatter.cpp
 *
 * Created: 1/17/2020 7:02:19 PM
 *  Author: abdelrhman
 */ 

#include <stdint.h>
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "ros.h"
#include "ros_lib/std_msgs/UInt16.h"

// Include C headers (ie, non C++ headers) in this block
extern "C"
{
	#define F_CPU 12000000UL
	#include <util/delay.h>
	#include "Interrupt.h"
	#include "std_macros.h"
	#include "Timer.h"
	#include "DIO_int.h"
	#include "PWM_int.h"
}

#define NUMBER_OVF 5
#define Init_ticks 108
#define Time_Interval 0.1
#define PPR 360.0
#define KP 15.547
#define KI 30.778
#define KD 0
#define Max_Volt_out 12000
#define Max_PWM_out 1023
#define PWM_VOLT_RATIO 0.08525


volatile uint16_t pulse_count = 0;
volatile uint16_t time_count = 0;

int16_t Last_Error = 0;
uint32_t speed_rpm ;
uint16_t PWM_Value = 0;
int32_t Volt_Out = 0 ;
uint16_t GOAL = 0;

// Needed for AVR to use virtual functions
extern "C" void __cxa_pure_virtual(void);
void __cxa_pure_virtual(void) {}
	
uint16_t Set_Motor_Speed (uint16_t Req_Speed, uint16_t Motor_Speed);

void SpeedCb(const std_msgs::UInt16& msg)
{
	GOAL = msg.data;
}

ros::NodeHandle nh;
std_msgs::UInt16 uint_msg;
ros::Publisher chatter("motor_speed", &uint_msg);
ros::Subscriber<std_msgs::UInt16> sub("MPC_velocity", &SpeedCb );



int main()
{
	/* Set Interrupt pin to input */
	DIO_VidSetPinDirection(PORTD, PIN2, INPUT);
	/* active pull up resistor for encoder pin */
	DIO_VidSetPinValue(PORTD, PIN2, HIGH);
	DIO_VidSetPinDirection(PORTD, PIN5, OUTPUT);

		
	/* INT0 initialization for rising edge */
	INT0_Init(Rising_Edge);

	/* set timer_0 prescaler to 1024 and init ticks to 28 to have 0.5 s sampling time (OVF = 0.0218 s) */
	TIM0_OVF_Init(TIMER_PRESCALER_1024, Init_ticks);
	
	// Initialize ROS
	uint32_t lasttime = 0UL;
	nh.initNode();
	nh.advertise(chatter);
	nh.subscribe(sub);



	while(1)
	{
	// Send the message every second
	if(avr_time_now() - lasttime > 100)
	{
		uint_msg.data = speed_rpm;
		chatter.publish(&uint_msg);
		lasttime = avr_time_now();
	}
	nh.spinOnce();

	}

	return 0;
}




ISR(INT0_vect)
{
	pulse_count++;
}


ISR(TIMER0_OVF_vect)
{
	time_count++;
	
	if (time_count == NUMBER_OVF)
	{
		speed_rpm = ((pulse_count/Time_Interval)/PPR)*60.0;   // speed equation
		TCNT0 = Init_ticks ;
		pulse_count = 0;
		time_count = 0 ;

		PWM_Value = Set_Motor_Speed(GOAL,speed_rpm);
		Fast_PWM1_init(TIMER_PRESCALER_256, PWM_Value, OC1A);


	}
}


uint16_t Set_Motor_Speed (uint16_t Req_Speed , uint16_t Motor_Speed  )
{
	int16_t Error = 0 ;
	uint16_t PWM = 0 ;
	
	Error = Req_Speed - Motor_Speed ;
	
	int32_t PID = (KP * Error) + ( KI * (Error + Last_Error) * (Time_Interval/2)) + ((KD * (Error - Last_Error)) / Time_Interval);

	Volt_Out = Volt_Out + PID ;

	Last_Error = Error;

	if (Volt_Out >= Max_Volt_out)
	{
		Volt_Out = Max_Volt_out ;
	}
	else if (Volt_Out <=0)
	{
		Volt_Out = 0;
		// apply break
	}
	
	PWM = Volt_Out * PWM_VOLT_RATIO;
	
	return PWM ;
}
