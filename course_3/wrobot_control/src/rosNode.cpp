
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

    /*  Subscriber  */
    ros::Subscriber robot_position = nh.subscribe(ROBOT_POSITION, 1,  rob_posCallback);


    /* Publisher */
    ros::Publisher left_wheel = nh.advertise<wrobot_comm::WheelVelocity>(LEFT_WHEEL, 1);
    ros::Publisher right_wheel = nh.advertise<wrobot_comm::WheelVelocity>(RIGHT_WHEEL, 1);

    /* get params */
    ros::NodeHandle n("~");
    getConstantParams(n);

    /*		control frequency	*/
	ros::Rate loop_rate(CONTROL_RATE);

	g_controllerHandler->new_reference(1.0,7.0);

    double vw_left;
    double vw_right;

    wrobot_comm::WheelVelocity msg1;
    wrobot_comm::WheelVelocity msg2;


    while (ros::ok())
    {
		ros::spinOnce(); 
		// callback will obtain the robot position and reference
		g_controllerHandler->getNewVelocity(vw_left, vw_right);

        msg1.velocity.data = vw_left;
        msg2.velocity.data = vw_right;

        msg1.header.stamp = ros::Time::now();
        msg2.header.stamp = ros::Time::now();

        left_wheel.publish(msg1);
        right_wheel.publish(msg2);

	    loop_rate.sleep();
    }   
    return 0;
}

void getConstantParams(ros::NodeHandle &node)
{
    double K[3];

    if(!node.getParam("P", K[0]))
    {
        K[0] = 0.1;
    }
    if(!node.getParam("I", K[1]))
    {
        K[1] = 0.1;
    }
    if(!node.getParam("D", K[2]))
    {
        K[2] = 0.1;
    }
	g_controllerHandler = new controller::CController(K);
}

void rob_posCallback(const wrobot_comm::RobotPosition &msg)
{
    double x_pos = msg.pose.x;
    double y_pos = msg.pose.y;
	g_controllerHandler->new_worldPos(x_pos , y_pos);
}
