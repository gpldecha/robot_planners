#include "robot_planners/planner_ee.h"

namespace planners {

Planner_ee::Planner_ee(Base_EE_Planner& ee_planner, ros::NodeHandle &nh, const asrv::Action_ee_initialiser &action_ee_init):
    ee_planner(ee_planner),
    Base_ee_action(nh,
                   action_ee_init.ee_state_pos_topic,
                   action_ee_init.ee_cmd_pos_topic,
                   action_ee_init.ee_cmd_ft_topic)
{

}


bool Planner_ee::execute_CB(asrv::alib_server& as_,asrv::alib_feedback& feedback,const asrv::cptrGoal& goal){

}



}
