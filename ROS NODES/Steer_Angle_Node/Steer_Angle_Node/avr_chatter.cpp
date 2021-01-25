/*
 * avr_chatter.cpp
 *
 * Created: 1/17/2020 7:02:19 PM
 *  Author: abdelrhman
 */


#include "ros.h"
#include "ros_lib/std_msgs/Int32.h"
#include "std_macros.h"


#define NUMBER_OVF 5
#define Init_ticks 108
#define Time_Interval 0.1
#define PPR 8000
#define KP 400
#define KI 250
#define KD 400
#define Max_Volt_out 12000
#define Min_Volt_out -12000
#define Max_PWM_out 1023
#define PWM_VOLT_RATIO 0.08525
#define LEFT_DIR 1
#define RIGHT_DIR 0
#define EEPROM_ADDRESS 0x00


// Include C headers (ie, non C++ headers) in this block
extern "C"
{
	#define F_CPU 12000000UL
	#include <avr/interrupt.h>
	#include <math.h>
	#include <util/delay.h>
	#include "Interrupt.h"
	#include "Fun.h"
	#include "DIO_int.h"
	#include "Timer.h"
	#include "PWM_int.h"
	#include "EEPROM.h"

}

volatile int32_t pulse_count = 0 ;
volatile int8_t theta ;
volatile uint16_t time_count = 0;
volatile uint8_t eeprom_count = 0;


int8_t init_theta ;
int8_t Last_theta = 0;
int16_t Last_Error = 0;
uint16_t PWM_Value = 0;
int16_t Volt_Out = 0 ;
int8_t GOAL = 0;

uint8_t state = 0;


// Needed for AVR to use virtual functions
extern "C" void __cxa_pure_virtual(void);
void __cxa_pure_virtual(void) {}

static void Set_Steer_Postion(int16_t Req_Speed , int16_t Steer_Pos );
static void Motor_Dirction(uint8_t dir, uint16_t pwm);


void AngleCallback(const std_msgs::Int32& msg)
{
	GOAL = msg.data ;
}

ros::NodeHandle nh;
std_msgs::Int32 angle_msg;
ros::Publisher chatter("steer_angle", &angle_msg);
ros::Subscriber<std_msgs::Int32> sub("MPC_SteerAngle", &AngleCallback );


int main()
{

	DIO_VidSetPinDirection(PORTD, PIN4, OUTPUT);
	DIO_VidSetPinDirection(PORTD, PIN5, OUTPUT);
	DIO_VidSetPinDirection(PORTD, PIN6, OUTPUT);




	TIM0_OVF_Init(TIMER_PRESCALER_1024, Init_ticks);
	FN();

	/* both interrupts on rising edge (3) */
	INT0_Init(Logical_Change);
	INT1_Init(Logical_Change);
	
	// EEPROM_Write(EEPROM_ADDRESS,0);
		
	if(EEPROM_Read(EEPROM_ADDRESS) > 50)
	{
		init_theta = EEPROM_Read(EEPROM_ADDRESS) - 256;
	}
	else
	{
		init_theta = EEPROM_Read(EEPROM_ADDRESS);
	}
		
		
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
		angle_msg.data = theta;
		chatter.publish(&angle_msg);
	    lasttime = avr_time_now();
    }
	
	if (eeprom_count == 10)
	{
		if (theta != Last_theta)
		{
			EEPROM_Write(EEPROM_ADDRESS,theta);
		}
		Last_theta = theta ;
		eeprom_count = 0 ;
	}
	
    nh.spinOnce();

  }

  return 0;
}


ISR(INT0_vect)
{
	
	uint8_t s = state & 0b00000011;
	if (((PIND&0b00000100)==4))
	{
		s |= 4;
	}
	if (((PIND&0b00001000)==8))
	{
		s |= 8;
	}
	
	switch (s)
	{
		case 0: case 5: case 10: case 15:
		break;
		case 1: case 7: case 8: case 14:
		pulse_count ++; break;
		case 2: case 4: case 11: case 13:
		pulse_count --; break;
		case 3: case 12:
		pulse_count += 2; break;
		default:
		pulse_count -= 2; break;
	}
	
	state = (s >> 2);
	
}


ISR(INT1_vect)
{
	
	uint8_t s = state & 0b00000011;
	if (((PIND&0b00000100)==4))
	{
		s |= 4;
	}
	if (((PIND&0b00001000)==8))
	{
		s |= 8;
	}
	switch (s)
	{
		case 0: case 5: case 10: case 15:
		break;
		case 1: case 7: case 8: case 14:
		pulse_count ++; break;
		case 2: case 4: case 11: case 13:
		pulse_count --; break;
		case 3: case 12:
		pulse_count += 2; break;
		default:
		pulse_count -= 2; break;
		
		
	}
	state = (s >> 2);
	
}

ISR(TIMER0_OVF_vect)
{
	time_count++;

	if (time_count == NUMBER_OVF)
	{
		theta = init_theta + ((pulse_count*360)/PPR);

		Set_Steer_Postion(GOAL,theta);

		TCNT0 = Init_ticks ;
		time_count = 0;
		eeprom_count++;


	}
}

static void Set_Steer_Postion(int16_t Req_Pos , int16_t Steer_Pos )
{
	int16_t Error = 0 ;
	uint16_t PWM = 0 ;

	Error = Req_Pos - Steer_Pos ;

	int32_t PID = (KP * Error) + ( KI * (Error + Last_Error) * (Time_Interval/2)) + ((KD * (Error - Last_Error)) / Time_Interval) ; ;

	Volt_Out = Volt_Out + PID ;

	Last_Error = Error;

	if (Volt_Out >= Max_Volt_out)
	{
		Volt_Out = Max_Volt_out ;
	}
	else if (Volt_Out < Min_Volt_out)
	{
		Volt_Out = Min_Volt_out ;
	}

	PWM = abs(Volt_Out) * PWM_VOLT_RATIO;

	if (Volt_Out > 0)
	{
		Motor_Dirction(LEFT_DIR, PWM);
	}
	else if (Volt_Out < 0)
	{
		Motor_Dirction(RIGHT_DIR,PWM);
	}


}

void Motor_Dirction(uint8_t dir, uint16_t pwm)
{
	if (dir == LEFT_DIR)
	{
		DIO_VidSetPinValue(PORTD, PIN4, HIGH);
		DIO_VidSetPinValue(PORTD, PIN6, LOW);
	}
	else if (dir == RIGHT_DIR)
	{
		DIO_VidSetPinValue(PORTD, PIN4, LOW);
		DIO_VidSetPinValue(PORTD, PIN6, HIGH);
	}
	else
	{
		/* do nothing */
	}
	Fast_PWM1_init(TIMER_PRESCALER_256, pwm, OC1A);
	
}

