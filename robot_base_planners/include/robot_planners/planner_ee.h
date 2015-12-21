#ifndef PLANNER_EE_H_
#define PLANNER_EE_H_

#include <ros/ros.h>
#include <kuka_action_server/action_server.h>
#include <kuka_action_server/base_ee_action.h>
#include <kuka_common_action_server/action_initialiser.h>
#include <std_msgs/Float64MultiArray.h>

#include "basePlanner.h"

namespace planners{

class Planner_ee : public asrv::Base_ee_action, public asrv::Base_action_server {
/*
public:

    Planner_ee(Base_EE_Planner& ee_planner,ros::NodeHandle& nh, const asrv::Action_ee_initialiser &action_ee_init);

    virtual bool execute_CB(asrv::alib_server& as_,asrv::alib_feedback& feedback,const asrv::cptrGoal& goal);

private:

    Base_EE_Planner& ee_planner;

    tf::Pose        des_ee_pose;    /// desired end-effector position


    double          dt;
    double          default_speed;
    double          reachingThreshold;
    double          orientationThreshold;

    bool            initial_config;
    std::size_t     tf_count;
*/
};

}

#endif
