#include <stdbool.h>
#include <stdint.h>
// TivaC specific includes
extern "C"
{
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
}
// ROS includes
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

void VelCallBack(const geometry_msgs::Twist& vel)
{
	
}

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", &VelCallBack);

int main(int argc, char **argv)
{
	// TivaC application specific code
  MAP_FPUEnable();
  MAP_FPULazyStackingEnable();
  // TivaC system clock configuration. Set to 80MHz.
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	nh.initNode();
	nh.subscribe(vel_sub);

	while(1)
	{
		nh.spinOnce();

		nh.getHardware()->delay(1000);
	}
}
