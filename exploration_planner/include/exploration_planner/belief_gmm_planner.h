#ifndef BELIEF_GMM_EXPLORATION_PLANNER_H_
#define BELIEF_GMM_EXPLORATION_PLANNER_H_

/**
       Simple belief state

       Forms a plan by considering a single hypothetical state (x,y,z) from the belief p(x).
       The belief is in terms of the position of the end-effector and the beliefs
       parameterisation is not important for this implemenation.

       - The planner subscribs to a topic which provides a plossible true hypothetical
         state of the location of the end-effector.

       - Tipical belief information pip-line:
             particle_filter -> feature extraction -> hypothetical position (x,y,z)

  **/

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <visualise/vis_points.h>
#include <exploration_planner/String_cmd.h>
#include <optitrack_rviz/listener.h>

#include <armadillo>

#include "statistics/distributions/gmm.h"
#include "robot_planners/gmmPlanner.h"
#include <optitrack_rviz/listener.h>

#include "peg_sensor/listener.h"

/**
 *  GMM belief space planner.
 *
 *  Takes as input, the current belief state feature vector and outputs a desired direction.
 */


namespace belief{

typedef struct Gmm_planner_initialiser{
    Gmm_planner_initialiser(){
        world_frame         = "/world";
        sensor_topic        = "";
        ft_classifier_topic = "";
        bel_feature_topic   = "";
        path_parameters     = "";
        belief_state_size   = 4;
    }

    std::string world_frame, sensor_topic,ft_classifier_topic,bel_feature_topic,path_parameters;
    std::size_t belief_state_size;
} Gmm_planner_initialiser;

class Gmm_planner{

public:

    typedef enum CONDITIONAL_TYPE{
        GMR,
        GMA
    }CONDITIONAL_TYPE;

public:

    Gmm_planner(ros::NodeHandle&   nh,Gmm_planner_initialiser init);

    void get_linear_velocity(arma::colvec3& velocity);

    //virtual bool execute_CB(asrv::alib_server& as_,asrv::alib_feedback& feedback,const asrv::cptrGoal& goal);

    void print() const;

private:


    bool service_callback(exploration_planner::String_cmd::Request& req,exploration_planner::String_cmd::Response& res);

    void belief_state_callback(const std_msgs::Float64MultiArrayConstPtr& msg);

    inline double bell_velocity(double x,double beta,double off){
        return 1.0 - exp(-beta * (x+off) * (x+off));
    }

    inline double rescale(double x,double a,double b,double c,double d){
        return -(-b*c + a*d)/(-a + b) + (-c + d)*x/(-a + b);
    }

public:

    arma::colvec             belief_state,belief_state_SF;


private:

    planners::GMAPlanner     gmap_planner;
    planners::GMR_EE_Planner gmr_planner;

    stats::Load_param::scale        scale_;

    arma::colvec3            direction,direction_tmp;

    ros::Subscriber          belief_info_sub; /// subscriber to
    arma::colvec             tmp_bel_state;

    // Transformation between world FR and peg_link FR
    arma::colvec3            T;
    arma::mat33              Rt;
    arma::colvec3            pos_tmp;
    CONDITIONAL_TYPE         conditional_type;


    ros::ServiceServer       service_server;

    std::string              world_frame;

    bool                     bPause;

};

}

#endif
