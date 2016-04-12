#ifndef PLANNER_EE_H_
#define PLANNER_EE_H_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "basePlanner.h"
#include "gmmPlanner.h"

namespace planners{

class GMR_ee_action {

public:

    GMR_ee_action(GMR_EE_Planner& gmr_ee, ros::NodeHandle& nh);

    //virtual bool execute_CB(asrv::alib_server& as_,asrv::alib_feedback& feedback,const asrv::cptrGoal& goal);

private:

    GMR_EE_Planner&   gmr_ee;
    double            default_speed;
    double            dt;
    double            reachingThreshold;
    double            orientationThreshold;


};

}

#endif
