#include "robot_planners/gmr_ee_action.h"
#include "optitrack_rviz/type_conversion.h"

namespace planners {

GMR_ee_action::GMR_ee_action(GMR_EE_Planner& gmr_ee,const tf::Transform& trans,ros::NodeHandle &nh, const asrv::Action_ee_initialiser &action_ee_init):
    gmr_ee(gmr_ee),
    trans(trans),
    Base_ee_action(nh,
                   action_ee_init.ee_state_pos_topic,
                   action_ee_init.ee_cmd_pos_topic,
                   action_ee_init.ee_cmd_ft_topic)
{

    reachingThreshold       = action_ee_init.reachingThreshold;     // [m]
    orientationThreshold    = action_ee_init.orientationThreshold;  // [rad]

    dt                      = 1.0/100.0;
    default_speed           = 0.001; // [m/s]


}


bool GMR_ee_action::execute_CB(asrv::alib_server& as_,asrv::alib_feedback& feedback,const asrv::cptrGoal& goal){

    tf::Transform trans_att;

    trans_att.setRotation(tf::Quaternion(goal->attractor_frame.rotation.x,goal->attractor_frame.rotation.y,
                                         goal->attractor_frame.rotation.z,goal->attractor_frame.rotation.w));
    trans_att.setOrigin(tf::Vector3(goal->attractor_frame.translation.x, goal->attractor_frame.translation.y,
                                    goal->attractor_frame.translation.z));

    ros::Rate wait(1);

    arma::colvec3 current_origin, des_origin;
    opti_rviz::type_conv::tf2vec(ee_pose.getOrigin(),current_origin);


    tf::Vector3     tf_current_origin   = ee_pose.getOrigin();
    tf::Vector3     tf_curr_origin_sock_fr;
    tf::Vector3     tf_des_origin       = ee_pose.getOrigin();
    tf::Quaternion  tf_current_orient   = ee_pose.getRotation();

    tf::Vector3    tf_target_pos        = trans_att.getOrigin();
    tf::Quaternion tf_target_orient     = trans_att.getRotation();



   // ROS_INFO("current_origin (%f %f %f)",current_origin(0),current_origin(1),current_origin(2));
   // ROS_INFO("current_target (%f %f %f)",tf_target_pos.x(),tf_target_pos.y(),tf_target_pos.z());
    tf::Vector3   T    = trans.getOrigin();
    tf::Quaternion tmp = trans.getRotation();
    tf::Matrix3x3 R; R.setRotation(tmp);

    tf_curr_origin_sock_fr = R * tf_current_origin + T;

    ROS_INFO("T (%f %f %f)",T.getX(),T.getY(),T.getZ());
    ROS_INFO("Q (%f %f %f %f)",tmp.getX(),tmp.getY(),tmp.getZ(),tmp.getW());



    double rate;
    if(goal->dt > 0){
        rate = 1.0/(goal->dt);
    }else{
        rate = 1.0/dt;
    }

    double max_speed =  default_speed; // [ms]
    double speed;
    double dist_targ_origin   = (tf_target_pos - ee_pose.getOrigin()).length();
    double dist_targ_target   = tf_current_orient.dot(tf_target_orient);
    double max_dist           = dist_targ_origin;
    double slerp_t            = 0.5;
    double beta               = (1.0/10.0);
    double offset             = 0;
    double prev_orient_error  = 0.0; //[rad]

 /*   ROS_INFO("distance_origin_target %f",dist_targ_origin);
    ROS_INFO("distance_orient_target %f",dist_targ_target);
    ROS_INFO("max_dist %f",max_dist);
    ROS_INFO("max_speed %f [m/s]",max_speed);
    ROS_INFO("rate %f",rate);*/

    arma::colvec in; in.resize(3);
    arma::colvec3    linear_velocity;
    linear_velocity.zeros();

    static tf::TransformBroadcaster br;
    ros::Rate loop_rate(rate);

    bool success = true;
    while(ros::ok()) {

        tf_current_origin = ee_pose.getOrigin();
        tf_current_orient = ee_pose.getRotation();

        /// put current position in the frame of reference of the socket
        tf_curr_origin_sock_fr = R * tf_current_origin + T;


        in(0) = tf_curr_origin_sock_fr.getX();
        in(1) = tf_curr_origin_sock_fr.getY();
        in(2) = tf_curr_origin_sock_fr.getZ();

        opti_rviz::type_conv::tf2vec(tf_current_origin,current_origin);


        gmr_ee.condition(in);
        gmr_ee.get_ee_linear_velocity(linear_velocity);
        linear_velocity = linear_velocity / arma::norm(linear_velocity);
        linear_velocity = 0.01 * linear_velocity;
        linear_velocity.zeros();
        des_origin      = current_origin + linear_velocity;

        des_ee_pose.setRotation(tf_current_orient.slerp(tf_target_orient, slerp_t)      );
        dist_targ_target = acos(abs(tf_target_orient.dot(tf_current_orient)));

        opti_rviz::type_conv::vec2tf(des_origin,tf_des_origin);
        des_ee_pose.setOrigin(tf_des_origin);


        sendPose(des_ee_pose);

        feedback.progress = 0;//dist_targ_target;
        as_.publishFeedback(feedback);
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Preempted");
            as_.setPreempted();
            success = false;
            break;
        }

        float diff_ori_err = abs(dist_targ_target - prev_orient_error);
        /*
        ROS_INFO_STREAM("Distance to target origin: " << dist_targ_origin << " reachingThreshold: " << reachingThreshold );
        ROS_INFO_STREAM("Distance to target orient: " << dist_targ_target << " orientationsThreshold: " << orientationThreshold );
        ROS_INFO_STREAM("Difference in Orientation error: " << diff_ori_err);
*/
        if (( dist_targ_origin < reachingThreshold) && (dist_targ_target < orientationThreshold || std::isnan(dist_targ_target) || diff_ori_err < 0.001) ){
            ROS_INFO("reached goal");
            break;
        }

        prev_orient_error = dist_targ_target; //[rad]
        loop_rate.sleep();
    }

    return success;

}



}
