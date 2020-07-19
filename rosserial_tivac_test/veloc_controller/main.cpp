#include <stdbool.h>
#include <stdint.h>

//
// TivaC specific includes
//
extern "C" 
{
	#include <inc/tm4c123gh6pm.h>
	#include <inc/hw_memmap.h>
	#include <inc/hw_types.h>
	#include <driverlib/sysctl.h>
	#include <driverlib/pin_map.h>
	#include <driverlib/interrupt.h>
	#include <driverlib/gpio.h>
	#include <driverlib/timer.h>
	#include <driverlib/qei.h>

	#include "Drivers/tn_pwm.h"
	#include "Drivers/tn_timer.h"
	#include "Drivers/tn_pid.h"
	#include "Drivers/tn_uart.h"
	#include "Drivers/tn_qei.h"
	#include "Drivers/tn_motor_controller.h"
}

//
// ROS includes
//
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

void VelCallBack(const geometry_msgs::Twist& vel)
{
	if ((vel.linear.x == 0.5) && (vel.angular.z == 0))
	{
		MotorControl(LEFT, 50);
		MotorControl(RIGHT, 50);
	}
	else if ((vel.linear.x == -0.5) && (vel.angular.z == 0))
	{
		MotorControl(LEFT, -50);
		MotorControl(RIGHT, -50);
	}
	else if ((vel.linear.x == 0) && (vel.angular.z == 1))
	{
		MotorControl(LEFT, -30);
		MotorControl(RIGHT, 30);
	}
	else if ((vel.linear.x == 0) && (vel.angular.z == -1))
	{
		MotorControl(LEFT, 30);
		MotorControl(RIGHT, -30);
	}
	else if ((vel.linear.x == 0) && (vel.angular.z == 0))
	{
		MotorControl(LEFT, 0);
		MotorControl(RIGHT, 0);
	}
}

ros::NodeHandle nh; 
ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", &VelCallBack);

int main(int argc, char **argv)
{
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL
										 | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	TimerInit();
//	UARTConfig();

//	UARTPutInt(0);
//	UARTPutStr("\n");

	MotorInit();

	MotorControl(LEFT, 0);
	MotorControl(RIGHT, 0);


  nh.initNode();
  nh.subscribe(vel_sub);

  while(1)
  {
    nh.spinOnce();

    nh.getHardware()->delay(10);
  }
}
