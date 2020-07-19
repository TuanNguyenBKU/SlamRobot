cmake_minimum_required(VERSION 3.0.2)                                          
project(veloc_controller)                                                              
                                                                     
# Include rosserial libraries for TivaC 
include_directories(${ROS_LIB_DIR})                                                          
                                                                                
# Per project based definitions and options                                      
add_definitions(-DLED_HEARTBEAT)                                                             
add_definitions(-DLED_COMM)                                                                  
add_definitions(-DROSSERIAL_BAUDRATE=57600)                                                  
add_definitions(-DTX_BUFFER_SIZE=256)                                                        
add_definitions(-DRX_BUFFER_SIZE=256)                                                        
                                                                                             
# Generate target for TivaC                                                                  
generate_tivac_firmware(              
  STARTUP startup_gcc.c
	SRCS 
		main.cpp 
  	Drivers/tn_motor_controller.c
		Drivers/tn_pid.c
		Drivers/tn_pwm.c
		Drivers/tn_qei.c
		Drivers/tn_timer.c
		Drivers/tn_uart.c
	BOARD tm4c123gxl                                                                           
)                                      

