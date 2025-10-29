
#include "FrankaSubscriber.h"

FrankaSubscriber::FrankaSubscriber(int argc, char **argv, double* posTargetPtr)
{
    this->posTargetPtr = posTargetPtr;
    ros::init(argc, argv, "franka_listener");
    ros::NodeHandle nh;
    this->sub = nh.subscribe("franka_pos", 1000, &FrankaSubscriber::SubscriberCallback, this);
    ros::spin();  // Don't need ros::spin() because we are waiting for the thread to join in the main loop anyway and this is blocking
}

FrankaSubscriber::~FrankaSubscriber()
{
}

void FrankaSubscriber::SubscriberCallback(const geometry_msgs::Point::ConstPtr& msg)
{
   this->posTargetPtr[0] = msg->x;
   this->posTargetPtr[1] = msg->y;
   this->posTargetPtr[2] = msg->z;
}