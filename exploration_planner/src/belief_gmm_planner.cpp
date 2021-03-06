#include "exploration_planner/belief_gmm_planner.h"
#include <optitrack_rviz/listener.h>
#include <optitrack_rviz/type_conversion.h>
#include <optitrack_rviz/debug.h>

const static bool   debug_flag      = true;
const static double time_throttle   = 0.1;

namespace belief{


Gmm_planner::Gmm_planner(ros::NodeHandle&  nh,Gmm_planner_initialiser init):
  world_frame(init.world_frame)
{

    ROS_INFO("INITIALISING BELIEF GMM PLANNER");

    // topics

    belief_info_sub         = nh.subscribe(init.bel_feature_topic,1,&Gmm_planner::belief_state_callback,this);
    service_server          = nh.advertiseService("bel_simple_planner_cmd",&Gmm_planner::service_callback,this);


    tf::StampedTransform trans;
    opti_rviz::Listener::get_tf_once(world_frame,"link_socket",trans);

    opti_rviz::type_conv::tf2mat(trans.getBasis(),Rt);
    opti_rviz::type_conv::tf2vec(trans.getOrigin(),T);

    Rt          = Rt.st();
    bPause      = true;

    std::string path_to_param_folders   = init.path_parameters;
    std::string model_name              = "qem_xhu_seprate";


    ///      Load GMMs
    gmr_planner  = planners::GMR_EE_Planner(path_to_param_folders + model_name);

  //  gmap_planner = planners::GMAPlanner(path_gmm_search);

    ///      Initialise belief vector
    belief_state.zeros(gmr_planner.get_in().size());
    belief_state_SF = belief_state;
    conditional_type = GMR;


    /// test
    {
        belief_state_SF(0) =  0.025;
        belief_state_SF(1) = -0.03;
        belief_state_SF(2) =  0;
        belief_state_SF(3) = -11.913400215435388;

        gmr_planner.gmm.print(0);
        gmr_planner.gmr(belief_state_SF);
        gmr_planner.get_ee_linear_velocity(direction);
        gmr_planner.gmm_c.gmm_c.print(0);
        direction.print("test direction");
    }

    std::cout<< "finished gmm_planner contructor [belief_gmm_planner.cpp]" << std::endl;
}


void Gmm_planner::print() const{
    std::cout<< "=== GMM BELIEF PLANNER ===" << std::endl;
    switch(conditional_type)
    {
    case GMR:
    {
        std::cout<< "conditional type: GMR" << std::endl;

        break;
    }
    case GMA:
    {
        std::cout<< "conditional type: GMA" << std::endl;
        break;
    }
    }
    const std::vector<std::size_t>&  in = gmap_planner.get_in();
    std::cout<< "in[" << in.size() << "]: ";
    for(std::size_t i = 0; i < in.size();i++){
        std::cout<< in[i] << " ";
    }
    std::cout<<std::endl;
    const std::vector<std::size_t>&  out = gmap_planner.get_out();
    std::cout<< "out[" << out.size() << "]: ";
    for(std::size_t i = 0; i < out.size();i++){
        std::cout<< out[i] << " ";
    }
    std::cout<<std::endl;
}


void Gmm_planner::get_linear_velocity(arma::colvec3& velocity){

    switch(conditional_type)
    {
    case GMR:
    {

        if(debug_flag){
            ROS_INFO_STREAM_THROTTLE(time_throttle,"GMR");
            ROS_INFO_STREAM_THROTTLE(time_throttle,"bel_SF: " << belief_state_SF(0) << " " << belief_state_SF(1) << " " << belief_state_SF(2));
        }
        gmr_planner.gmr(belief_state_SF);
        gmr_planner.get_ee_linear_velocity(direction);
        if(debug_flag){
            ROS_INFO_STREAM_THROTTLE(time_throttle,"direction: " << direction(0) << " " << direction(1) << " " << direction(2));
        }

        break;
    }
    case GMA:
    {
       // std::cout<< "GMA" << std::endl;

        gmap_planner.gmc(belief_state_SF,direction_tmp);
        gmap_planner.print();
        gmap_planner.get_ee_linear_velocity(direction);
        break;
    }
    }

    velocity      = direction;
    direction_tmp = direction;
}

void Gmm_planner::belief_state_callback(const std_msgs::Float64MultiArrayConstPtr &msg){
   // std::cout<< "belief_state_callback" << std::endl;

    if(msg->data.size() == belief_state.n_elem){
        // std::cout<< "msg->data.size(): " << msg->data.size() << std::endl;
        for(std::size_t i = 0; i < msg->data.size();i++){

            belief_state(i) = msg->data[i];
        }

        pos_tmp(0) = belief_state(0);
        pos_tmp(1) = belief_state(1);
        pos_tmp(2) = belief_state(2);

        pos_tmp = Rt * pos_tmp - Rt * T;


        belief_state_SF(0) = pos_tmp(0);
        belief_state_SF(1) = pos_tmp(1);
        belief_state_SF(2) = pos_tmp(2);
        belief_state_SF(3) = belief_state(3);

        opti_rviz::debug::tf_debuf(pos_tmp,"mode_SF_1/pos_tmp");


      //  ROS_INFO_STREAM_THROTTLE(1.0,"Gmm_planner belief_state: " << belief_state(0) << " " << belief_state(1) << " "
      //                           << belief_state(2));
    }else{
        ROS_WARN_STREAM_THROTTLE(1.0,"msg->data.size() != belief_state.n_elem [Gmm_planner::belief_state_callback]");
    }
}

bool Gmm_planner::service_callback(exploration_planner::String_cmd::Request& req,
                                      exploration_planner::String_cmd::Response& res)
{
    std::string cmd = req.cmd;
    if(cmd == "pause"){
        bPause = !bPause;
        if(!bPause){
            ROS_INFO("simple planner is paused");
        }else{
            ROS_INFO("simple planner is running");
        }
        return true;
    }else{
        res.res = "cmd [" + cmd  + "] does not exist [Simple_planner::service_callback]";
        return false;
    }
}



}
