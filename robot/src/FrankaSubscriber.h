#ifndef FRANKASUBCRIBER_H
#define FRANKASUBCRIBER_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

class FrankaSubscriber
{
private:
    /* data */
    double *posTargetPtr;
    ros::Subscriber sub;   
public:
    FrankaSubscriber(int argc, char **argv, double* posTargetPtr);
    void SubscriberCallback(const geometry_msgs::Point::ConstPtr&);
    ~FrankaSubscriber();
};

#endif