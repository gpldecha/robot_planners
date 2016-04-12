#include "exploration_planner/simple_exploration.h"
#include <optitrack_rviz/listener.h>

namespace belief{


Simple_planner::Simple_planner(ros::NodeHandle&   nh,
                               const std::string& world_frame)
    :world_frame(world_frame)
{

    // topics

    //belief_info_sub         = nh.subscribe(topic_name,1,&Simple_planner::topic_callback,this);
   // service_server          = nh.advertiseService("bel_simple_planner_cmd",&Simple_planner::service_callback,this);

   // reachingThreshold       = 0.0001;//action_ee_init.reachingThreshold;     // [m]
   // orientationThreshold    = action_ee_init.orientationThreshold;  // [rad]

    dt                      = 1.0/100.0;
    default_speed           = 0.002; // [m/s]
    bSimulation             = true;
    bPause                  = true;
}

void Simple_planner::topic_callback(const std_msgs::Float64MultiArrayConstPtr &msg){
    hx_current_origin.setX(msg->data[0]);
    hx_current_origin.setY(msg->data[1]);
    hx_current_origin.setZ(msg->data[2]);
}

bool Simple_planner::service_callback(exploration_planner::String_cmd::Request& req,
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

//bool Simple_planner::execute_CB(asrv::alib_server& as_,asrv::alib_feedback& feedback,const asrv::cptrGoal& goal){
/*
    tf::Transform trans_att;

    trans_att.setRotation(tf::Quaternion(goal->attractor_frame.rotation.x,goal->attractor_frame.rotation.y,
                                         goal->attractor_frame.rotation.z,goal->attractor_frame.rotation.w));
    trans_att.setOrigin(tf::Vector3(goal->attractor_frame.translation.x, goal->attractor_frame.translation.y,
                                    goal->attractor_frame.translation.z));


    ros::Rate wait(1);

    tf::Transform trans_ee_belief,trans_ee_goal;

   // tf::Vector3     current_origin = ee_pose.getOrigin();
    tf::Quaternion  current_orient = ee_pose.getRotation();

    trans_ee_goal = trans_att;


    tf::Vector3    target_pos    = trans_att.getOrigin();
    tf::Quaternion target_orient = trans_att.getRotation();


    opti_rviz::Listener listener("world_frame","peg_link");
    tf::Vector3 peg_origin;
    tf::Matrix3x3 peg_orient;
    listener.update(peg_origin,peg_orient);



    tf::Vector3    velocity;

    double rate;
    if(goal->dt > 0){
        rate = 1.0/(goal->dt);
    }else{
        rate = 1.0/dt;
    }

    double max_speed =  default_speed; // [ms]
    double speed;
    double dist_targ_origin = (target_pos - hx_current_origin).length();
    double dist_targ_peg    = (target_pos - peg_origin).length();
    double dist_targ_target = current_orient.dot(target_orient);
    double max_dist         = dist_targ_origin;
    double slerp_t          = 0.5;
    double beta             = (1.0/10.0);
    double offset           = 0;
    double prev_orient_error  = 0.0; //[rad]

    ROS_INFO("distance_origin_target %f",dist_targ_origin);
    ROS_INFO("distance_orient_target %f",dist_targ_target);
    ROS_INFO("max_dist %f",max_dist);
    ROS_INFO("max_speed %f [m/s]",max_speed);
    ROS_INFO("rate %f",rate);


    virtual_end_effector.setOrigin(hx_current_origin);
    virtual_end_effector.setRotation(current_orient);


    bool bUseTruePosition = false;

*/
/*
    static tf::TransformBroadcaster br;
    tf::Vector3 true_peg_pos;
    arma::colvec3 velocity_tmp;
    double dist_bel_true = 1;

    ros::Rate loop_rate(rate);*/
//    while(ros::ok() /*&& bBaseRun*/) {
/*
        br.sendTransform(tf::StampedTransform(trans_att, ros::Time::now(), world_frame, "ee_final"));

        current_orient = ee_pose.getRotation();
        true_peg_pos   = ee_pose.getOrigin();

        if((true_peg_pos - trans_att.getOrigin()).length() < 0.045){
            bUseTruePosition = true;
            std::cout<< "use true (true cloes enougth)" << std::endl;
        }

        velocity_tmp(0) = hx_current_origin.getX();
        velocity_tmp(1) = hx_current_origin.getY();
        velocity_tmp(2) = hx_current_origin.getZ();


        if(std::isnan(dist_bel_true)){
            dist_bel_true = 10;
        }

        if(velocity_tmp.has_nan()){
            std::cout<< "hx_current_origin has nan" << std::endl;

        }else{

            if( dist_bel_true  < 0.01 ){
                bUseTruePosition = true;
                std::cout<< "use true (bel close): " << dist_bel_true << std::endl;
            }
        }




        if(bUseTruePosition){
            velocity = (trans_att.getOrigin() - true_peg_pos);
        }else{
            // Linear velocity between start position and target
            velocity = (trans_att.getOrigin() - hx_current_origin);
        }




        velocity_tmp(0) = velocity.getX();
        velocity_tmp(1) = velocity.getY();
        velocity_tmp(2) = velocity.getZ();

        if(velocity_tmp.has_nan()){
            std::cout<< "velocity is nan" << std::endl;
              velocity = (trans_att.getOrigin() - true_peg_pos);
        }

        dist_bel_true = velocity.length();


        listener.update(peg_origin,peg_orient);
        dist_targ_peg    = (target_pos - peg_origin).length();


        // compute desired speed (function of distance to goal, bell shapped velocity curve)
        dist_targ_origin = velocity.length(); // [meters]
        speed            = (max_speed * bell_velocity(dist_targ_origin * 100.0,beta,offset));    // convert [m] -> [cm]
        velocity         = speed * velocity.normalize();

        ROS_INFO("x : %f %f %f",ee_pose.getOrigin().x(),ee_pose.getOrigin().y(),ee_pose.getOrigin().z());
        ROS_INFO("hx: %f %f %f",hx_current_origin.x(),hx_current_origin.y(),hx_current_origin.z());
        ROS_INFO("ve: %f %f %f",velocity.x(),velocity.y(),velocity.z());
        ROS_INFO("speed: %f",velocity.length());

        des_ee_pose.setOrigin(velocity + ee_pose.getOrigin());
        des_ee_pose.setRotation( current_orient.slerp(target_orient, slerp_t)      );

        dist_targ_target = acos(abs(target_orient.dot(current_orient)));

        if(!bPause){
            sendPose(des_ee_pose);
        }

        listener.update(peg_origin,peg_orient);

        feedback.progress = 0;
        as_.publishFeedback(feedback);
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Preempted");
            as_.setPreempted();
           // bBaseRun = false;
            break;
        }

        float diff_ori_err = abs(dist_targ_target - prev_orient_error);
      //  ROS_INFO_STREAM("Distance to target origin: " << dist_targ_origin << " reachingThreshold: " << reachingThreshold );
      //  ROS_INFO_STREAM("Distance to target orient: " << dist_targ_target << " orientationsThreshold: " << orientationThreshold );
      //  ROS_INFO_STREAM("Difference in Orientation error: " << diff_ori_err);

        if (( dist_targ_origin < reachingThreshold) && (dist_targ_target < orientationThreshold || std::isnan(dist_targ_target) || diff_ori_err < 0.001) ){
            ROS_INFO("reached goal");
            break;
        }

        prev_orient_error = dist_targ_target; //[rad]

        trans_ee_belief.setOrigin(hx_current_origin);
        trans_ee_belief.setRotation(current_orient);

        br.sendTransform(tf::StampedTransform(trans_ee_belief, ros::Time::now(), world_frame, "ee_belief"));
        br.sendTransform(tf::StampedTransform(trans_ee_goal, ros::Time::now(), world_frame, "ee_goal"));

        loop_rate.sleep();

    }*/

  /*  if(!bBaseRun){
        return false;
    }else{
        return true;
    }*/
  //  return true;
//}


}
