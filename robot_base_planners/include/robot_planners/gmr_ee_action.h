#ifndef PLANNER_EE_H_
#define PLANNER_EE_H_

#include <ros/ros.h>
#include <kuka_action_server/action_server.h>
#include <kuka_action_server/base_ee_action.h>
#include <kuka_common_action_server/action_initialiser.h>
#include <std_msgs/Float64MultiArray.h>

#include "basePlanner.h"
#include "gmmPlanner.h"

namespace planners{

class GMR_ee_action : public asrv::Base_ee_action, public asrv::Base_action_server {

public:

    GMR_ee_action(GMR_EE_Planner& gmr_ee,const tf::Transform& trans,ros::NodeHandle& nh, const asrv::Action_ee_initialiser &action_ee_init);

    virtual bool execute_CB(asrv::alib_server& as_,asrv::alib_feedback& feedback,const asrv::cptrGoal& goal);

private:

    GMR_EE_Planner&   gmr_ee;
    tf::Pose          des_ee_pose;    /// desired end-effector position
    tf::Transform     trans;
    double            default_speed;
    double            dt;
    double            reachingThreshold;
    double            orientationThreshold;


};

}

#endif
