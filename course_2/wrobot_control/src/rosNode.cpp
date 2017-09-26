
#include <ros/ros.h>
#include "wrobot_comm/WheelVelocity.h"
#include "wrobot_comm/RobotPosition.h"
#include "wrobot_control/controller.hpp"

static constexpr char LEFT_WHEEL[] = "/wheeled_robot/left_wheel";
static constexpr char RIGHT_WHEEL[] = "/wheeled_robot/right_wheel";
static constexpr char ROBOT_POSITION[] = "/wheeled_robot/robot_position";

controller::CController *g_controllerHandler;

void getConstantParams(ros::NodeHandle &node);
void rob_posCallback(const wrobot_comm::RobotPosition &msg);

#define CONTROL_RATE		(30) //Hz

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrobot_controller");
    ros::NodeHandle nh;

    /* get params */
    ros::NodeHandle n("~");
    getConstantParams(n);

    /*		control frequency	*/
	ros::Rate loop_rate(CONTROL_RATE);



	g_controllerHandler->new_reference(....);



    while (ros::ok())
    {
		ros::spinOnce(); 
		// callback will obtain the robot position and reference
		g_controllerHandler->getNewVelocity(....);



	    loop_rate.sleep();
    }   
    return 0;
}

void getConstantParams(ros::NodeHandle &node)
{

	//get params from YAML
	g_controllerHandler = new controller::CController(...);
}

void rob_posCallback(const wrobot_comm::RobotPosition &msg)
{
	
}
