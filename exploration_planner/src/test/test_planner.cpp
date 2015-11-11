#include "exploration_planner/belief_gmm_planner.h"
#include "robot_planners/gmmPlanner.h"

int main(int argc,char** argv){


    std::cout<< "=== test exploration planners === " << std::endl;

    ros::init(argc, argv, "test_exploration");
    ros::NodeHandle nh;

    std::string sensor_topic_name = "sensor_topic";
    std::string fixed_frame       = "world_frame";

    asrv::Action_ee_initialiser init_ee;
    init_ee.action_name = "gmm_exploration";



    std::string path_parameters = "/home/guillaume/MatlabWorkSpace/peg_in_hole_RL/PolicyModelSaved/PolicyModel_txt/gmm_xhu";


    GMM gmm;//(path_parameters);
    //gmm.load(path_parameters);

   // planners::GMAPlanner    gmap_planner;
   // gmap_planner.load(path_parameters);


    belief::Gmm_planner bel_gmm_planner(nh,sensor_topic_name,fixed_frame,path_parameters,init_ee);




    std::cout<< "=== finished text exploration planners === " << std::endl;


    return 0;
}
